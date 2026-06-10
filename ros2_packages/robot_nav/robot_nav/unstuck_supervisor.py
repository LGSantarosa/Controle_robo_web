"""Watchdog de desencalhe do Nav2 — unstuck_supervisor.

Quando o robô fica congelado pelo collision monitor por mais de
`stuck_timeout` segundos, este nó assume o controle por um canal do twist_mux
que FURA o collision monitor (`unstuck_vel`, prioridade acima do `nav_vel`),
executa uma manobra de desencalhe (ré, ou giro se a traseira estiver bloqueada)
e devolve o controle pro nav2, que replaneja e desvia.

O collision monitor e a curva (RotationShim/DWB) NÃO são tocados: o nó só age
DEPOIS que o robô já travou. Ver
`docs/superpowers/specs/2026-06-10-unstuck-supervisor-design.md`.

A lógica de decisão é pura (sem ROS) pra ser testável offline; o nó embaixo é só
a cola de I/O.
"""

import math
from dataclasses import dataclass, field
from typing import List, NamedTuple, Optional, Tuple


# ---- lógica pura -----------------------------------------------------------

def is_frozen(lin: float, ang: float, zero_lin: float, zero_ang: float) -> bool:
    """True se a velocidade medida está abaixo dos limiares (robô parado)."""
    return abs(lin) < zero_lin and abs(ang) < zero_ang


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
    reverse_distance: float = 0.30
    reverse_speed: float = 0.15
    reverse_time_cap: float = 3.0
    spin_speed: float = 0.5
    spin_angle: float = 1.0
    escalate_after: int = 3
    same_spot_radius: float = 0.5
    escalate_window: float = 60.0
    grace: float = 2.0


class Command(NamedTuple):
    lin: float
    ang: float
    active: bool


_IDLE = Command(0.0, 0.0, False)

# estados
_MONITORING = "monitoring"
_REVERSING = "reversing"
_SPINNING = "spinning"
_GRACE = "grace"


@dataclass
class UnstuckSupervisor:
    cfg: UnstuckConfig
    state: str = _MONITORING
    stuck_since: Optional[float] = None
    maneuver_start_t: float = 0.0
    maneuver_start_pos: Tuple[float, float] = (0.0, 0.0)
    grace_start: float = 0.0
    history: List[Tuple[float, Tuple[float, float]]] = field(default_factory=list)

    def update(self, now: float, *, stop_active: bool, frozen: bool,
               nav_wants_move: bool, position: Tuple[float, float],
               rear_blocked: bool) -> Command:
        if self.state == _MONITORING:
            return self._monitoring(now, stop_active, frozen, nav_wants_move,
                                    position, rear_blocked)
        if self.state == _REVERSING:
            return self._reversing(now, position)
        if self.state == _SPINNING:
            return self._spinning(now)
        if self.state == _GRACE:
            return self._grace(now)
        return _IDLE

    # -- estados --

    def _monitoring(self, now, stop_active, frozen, nav_wants_move, position,
                    rear_blk) -> Command:
        stuck = stop_active and frozen and nav_wants_move
        if not stuck:
            self.stuck_since = None
            return _IDLE
        if self.stuck_since is None:
            self.stuck_since = now
            return _IDLE
        if now - self.stuck_since < self.cfg.stuck_timeout:
            return _IDLE
        return self._begin_maneuver(now, position, rear_blk)

    def _begin_maneuver(self, now, position, rear_blk) -> Command:
        # registra o evento e conta quantos houve perto deste ponto na janela
        self._prune(now)
        self.history.append((now, position))
        nearby = sum(
            1 for (_, p) in self.history
            if math.hypot(p[0] - position[0], p[1] - position[1])
            <= self.cfg.same_spot_radius
        )
        force_spin = nearby >= self.cfg.escalate_after

        self.maneuver_start_t = now
        self.maneuver_start_pos = position
        if rear_blk or force_spin:
            self.state = _SPINNING
            return Command(0.0, self.cfg.spin_speed, True)
        self.state = _REVERSING
        return Command(-self.cfg.reverse_speed, 0.0, True)

    def _reversing(self, now, position) -> Command:
        dist = math.hypot(position[0] - self.maneuver_start_pos[0],
                          position[1] - self.maneuver_start_pos[1])
        if (dist >= self.cfg.reverse_distance
                or now - self.maneuver_start_t >= self.cfg.reverse_time_cap):
            return self._enter_grace(now)
        return Command(-self.cfg.reverse_speed, 0.0, True)

    def _spinning(self, now) -> Command:
        spin_time = (self.cfg.spin_angle / self.cfg.spin_speed
                     if self.cfg.spin_speed else 0.0)
        if now - self.maneuver_start_t >= spin_time:
            return self._enter_grace(now)
        return Command(0.0, self.cfg.spin_speed, True)

    def _enter_grace(self, now) -> Command:
        self.state = _GRACE
        self.grace_start = now
        return _IDLE

    def _grace(self, now) -> Command:
        if now - self.grace_start >= self.cfg.grace:
            self.state = _MONITORING
            self.stuck_since = None
        return _IDLE

    def _prune(self, now) -> None:
        cutoff = now - self.cfg.escalate_window
        self.history = [(t, p) for (t, p) in self.history if t >= cutoff]


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
    except ImportError:  # nome pode variar entre distros
        CollisionMonitorState = None

    # STOP = 1 no enum do collision monitor (DO_NOTHING=0, STOP=1, SLOWDOWN=2, ...)
    STOP_ACTION = 1

    class UnstuckSupervisorNode(Node):
        def __init__(self):
            super().__init__("unstuck_supervisor")
            p = self.declare_parameters("", [
                ("stuck_timeout", 10.0),
                ("reverse_distance", 0.30),
                ("reverse_speed", 0.15),
                ("reverse_time_cap", 3.0),
                ("spin_speed", 0.5),
                ("spin_angle", 1.0),
                ("escalate_after", 3),
                ("same_spot_radius", 0.5),
                ("escalate_window", 60.0),
                ("grace", 2.0),
                ("rear_clearance", 0.35),
                ("rear_sector_deg", 30.0),
                ("odom_zero_lin", 0.02),
                ("odom_zero_ang", 0.05),
                ("nav_move_lin", 0.01),
                ("nav_move_ang", 0.05),
                ("rate_hz", 10.0),
            ])
            g = {n.name: n.value for n in p}
            self.cfg = UnstuckConfig(
                stuck_timeout=g["stuck_timeout"],
                reverse_distance=g["reverse_distance"],
                reverse_speed=g["reverse_speed"],
                reverse_time_cap=g["reverse_time_cap"],
                spin_speed=g["spin_speed"],
                spin_angle=g["spin_angle"],
                escalate_after=int(g["escalate_after"]),
                same_spot_radius=g["same_spot_radius"],
                escalate_window=g["escalate_window"],
                grace=g["grace"],
            )
            self.rear_clearance = g["rear_clearance"]
            self.rear_sector_deg = g["rear_sector_deg"]
            self.odom_zero_lin = g["odom_zero_lin"]
            self.odom_zero_ang = g["odom_zero_ang"]
            self.nav_move_lin = g["nav_move_lin"]
            self.nav_move_ang = g["nav_move_ang"]

            self.sup = UnstuckSupervisor(self.cfg)

            self._stop_active = False
            self._frozen = False
            self._nav_wants_move = False
            self._position = (0.0, 0.0)
            self._rear_blocked = False
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
            else:
                self.get_logger().warn(
                    "CollisionMonitorState indisponível — supervisor inativo")

            self.create_timer(1.0 / g["rate_hz"], self._tick)
            self.get_logger().info(
                "unstuck_supervisor ativo (gatilho %.0fs, ré %.2fm)" % (
                    self.cfg.stuck_timeout, self.cfg.reverse_distance))

        def _on_odom(self, msg):
            self._position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self._frozen = is_frozen(
                msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                self.odom_zero_lin, self.odom_zero_ang)

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
                now, stop_active=self._stop_active, frozen=self._frozen,
                nav_wants_move=self._nav_wants_move, position=self._position,
                rear_blocked=self._rear_blocked)
            if self.sup.state != self._last_state:
                self.get_logger().warn(
                    "unstuck: %s -> %s" % (self._last_state, self.sup.state))
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
