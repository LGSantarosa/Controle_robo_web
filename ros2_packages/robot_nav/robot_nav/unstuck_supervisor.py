"""Watchdog de desencalhe do Nav2 — unstuck_supervisor.

Se o robô NÃO SE DESLOCA (>stuck_radius) por mais de `stuck_timeout` segundos
enquanto o nav2 está comandando, este nó assume o controle por um canal do
twist_mux que FURA o collision monitor (`unstuck_vel`, prioridade acima do
`nav_vel`) e dá RÉ; depois solta e o nav2 replaneja e desvia.

Decisões de campo (2026-06-10, validadas em teste ao vivo):
- Gatilho por DESLOCAMENTO, não velocidade: o robô "tentando girar" sem sair
  do lugar (RotationShim, recoveries do nav2) mexe uns mm e enganava o gatilho
  por velocidade. Não se deslocou = travado, ponto.
- SEM GIRO: o giro a baixa velocidade não vence o atrito do skid-steer (parecia
  "não fez nada"). A manobra é SEMPRE ré. Traseira bloqueada no /scan → espera
  e re-tenta quando liberar.
- Só age com goal ativo: nav2 comandou há <nav_latch (latch tolera os gaps de
  ~1-2s do ciclo de abort do progress_checker).

O collision monitor e a curva (RotationShim/DWB) NÃO são tocados. Ver
`docs/superpowers/specs/2026-06-10-unstuck-supervisor-design.md`.

A lógica de decisão é pura (sem ROS) pra ser testável offline; o nó embaixo é
só a cola de I/O.
"""

import math
from dataclasses import dataclass
from typing import NamedTuple, Optional, Tuple


# ---- lógica pura -----------------------------------------------------------

def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def rear_blocked(ranges, angle_min: float, angle_increment: float,
                 sector_deg: float, clearance: float) -> bool:
    """True se houver retorno do /scan a menos de `clearance` no setor traseiro.

    Traseira = ângulo ~pi (180°). O setor é pi ± sector_deg.
    """
    if angle_increment == 0.0:
        return False
    half = math.radians(sector_deg)
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue
        if r >= clearance:
            continue
        a = angle_min + i * angle_increment
        if abs(_norm_angle(a - math.pi)) <= half:
            return True
    return False


@dataclass
class UnstuckConfig:
    stuck_timeout: float = 10.0
    stuck_radius: float = 0.05     # deslocou menos que isso = "parado"
    reverse_distance: float = 0.30
    reverse_speed: float = 0.25
    reverse_time_cap: float = 6.0
    grace: float = 2.0
    nav_latch: float = 15.0        # nav2 conta como "com goal" se comandou há <=15s


class Command(NamedTuple):
    lin: float
    ang: float
    active: bool


_IDLE = Command(0.0, 0.0, False)

# estados
_MONITORING = "monitoring"
_REVERSING = "reversing"
_GRACE = "grace"


@dataclass
class UnstuckSupervisor:
    cfg: UnstuckConfig
    state: str = _MONITORING
    anchor: Optional[Tuple[float, float]] = None  # última posição "nova"
    anchor_t: float = 0.0
    maneuver_start_t: float = 0.0
    maneuver_start_pos: Tuple[float, float] = (0.0, 0.0)
    grace_start: float = 0.0
    last_nav_t: Optional[float] = None

    def update(self, now: float, *, nav_wants_move: bool,
               position: Tuple[float, float], rear_blocked: bool) -> Command:
        if nav_wants_move:
            self.last_nav_t = now
        if self.state == _MONITORING:
            return self._monitoring(now, position, rear_blocked)
        if self.state == _REVERSING:
            return self._reversing(now, position)
        if self.state == _GRACE:
            return self._grace(now)
        return _IDLE

    # -- estados --

    def _monitoring(self, now, position, rear_blk) -> Command:
        nav_recent = (self.last_nav_t is not None
                      and now - self.last_nav_t <= self.cfg.nav_latch)
        if not nav_recent:
            # sem goal ativo: parado aqui é normal (goal atingido/cancelado)
            self.anchor = None
            return _IDLE
        # âncora de deslocamento: só re-ancora quando o robô REALMENTE sai do
        # raio — micro-mexidas (tentando girar, ruído de odom) não resetam.
        if self.anchor is None or self._dist(position, self.anchor) > self.cfg.stuck_radius:
            self.anchor = position
            self.anchor_t = now
            return _IDLE
        if now - self.anchor_t < self.cfg.stuck_timeout:
            return _IDLE
        if rear_blk:
            # traseira bloqueada: NÃO gira (giro removido), segura e re-tenta
            # no próximo tick — dispara assim que o /scan liberar atrás.
            return _IDLE
        self.state = _REVERSING
        self.maneuver_start_t = now
        self.maneuver_start_pos = position
        return Command(-self.cfg.reverse_speed, 0.0, True)

    def _reversing(self, now, position) -> Command:
        dist = self._dist(position, self.maneuver_start_pos)
        if (dist >= self.cfg.reverse_distance
                or now - self.maneuver_start_t >= self.cfg.reverse_time_cap):
            self.state = _GRACE
            self.grace_start = now
            return _IDLE
        return Command(-self.cfg.reverse_speed, 0.0, True)

    def _grace(self, now) -> Command:
        if now - self.grace_start >= self.cfg.grace:
            self.state = _MONITORING
            self.anchor = None  # re-ancora na posição pós-manobra
        return _IDLE

    @staticmethod
    def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])


# ---- nó ROS (cola de I/O) --------------------------------------------------

def main(args=None):  # pragma: no cover - I/O glue, validado na bancada
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    try:
        from nav2_msgs.msg import CollisionMonitorState
    except ImportError:
        CollisionMonitorState = None

    # STOP = 1 no enum do collision monitor — só pra LOG (não é mais gatilho)
    STOP_ACTION = 1

    class UnstuckSupervisorNode(Node):
        def __init__(self):
            super().__init__("unstuck_supervisor")
            p = self.declare_parameters("", [
                ("stuck_timeout", 10.0),
                ("stuck_radius", 0.05),
                ("reverse_distance", 0.30),
                ("reverse_speed", 0.25),
                ("reverse_time_cap", 6.0),
                ("grace", 2.0),
                ("nav_latch", 15.0),
                ("rear_clearance", 0.35),
                ("rear_sector_deg", 30.0),
                ("nav_move_lin", 0.01),
                ("nav_move_ang", 0.05),
                ("rate_hz", 10.0),
            ])
            g = {n.name: n.value for n in p}
            self.cfg = UnstuckConfig(
                stuck_timeout=g["stuck_timeout"],
                stuck_radius=g["stuck_radius"],
                reverse_distance=g["reverse_distance"],
                reverse_speed=g["reverse_speed"],
                reverse_time_cap=g["reverse_time_cap"],
                grace=g["grace"],
                nav_latch=g["nav_latch"],
            )
            self.rear_clearance = g["rear_clearance"]
            self.rear_sector_deg = g["rear_sector_deg"]
            self.nav_move_lin = g["nav_move_lin"]
            self.nav_move_ang = g["nav_move_ang"]

            self.sup = UnstuckSupervisor(self.cfg)

            self._nav_wants_move = False
            self._position = (0.0, 0.0)
            self._rear_blocked = False
            self._stop_active = False  # só pra log
            self._last_state = self.sup.state

            be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)

            self.pub = self.create_publisher(Twist, "unstuck_vel", 10)
            self.create_subscription(Odometry, "odom", self._on_odom, 10)
            self.create_subscription(Twist, "nav_vel_raw", self._on_nav_raw, 10)
            self.create_subscription(LaserScan, "scan", self._on_scan, be)
            if CollisionMonitorState is not None:
                self.create_subscription(
                    CollisionMonitorState, "collision_monitor_state",
                    self._on_collision, 10)

            self.create_timer(1.0 / g["rate_hz"], self._tick)
            self.get_logger().info(
                "unstuck_supervisor ativo (sem-deslocamento %.0fs -> ré %.2fm; "
                "giro desativado)" % (
                    self.cfg.stuck_timeout, self.cfg.reverse_distance))

        def _on_odom(self, msg):
            self._position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        def _on_nav_raw(self, msg):
            self._nav_wants_move = (abs(msg.linear.x) > self.nav_move_lin
                                    or abs(msg.angular.z) > self.nav_move_ang)

        def _on_scan(self, msg):
            self._rear_blocked = rear_blocked(
                list(msg.ranges), msg.angle_min, msg.angle_increment,
                self.rear_sector_deg, self.rear_clearance)

        def _on_collision(self, msg):
            self._stop_active = (getattr(msg, "action_type", 0) == STOP_ACTION)

        def _tick(self):
            now = self.get_clock().now().nanoseconds * 1e-9
            cmd = self.sup.update(
                now, nav_wants_move=self._nav_wants_move,
                position=self._position, rear_blocked=self._rear_blocked)
            if self.sup.state != self._last_state:
                self.get_logger().warn(
                    "unstuck: %s -> %s (pos=%.2f,%.2f stop=%s rear=%s)" % (
                        self._last_state, self.sup.state,
                        self._position[0], self._position[1],
                        self._stop_active, self._rear_blocked))
                self._last_state = self.sup.state
            if cmd.active:
                t = Twist()
                t.linear.x = cmd.lin
                t.angular.z = cmd.ang
                self.pub.publish(t)

    rclpy.init(args=args)
    node = UnstuckSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
