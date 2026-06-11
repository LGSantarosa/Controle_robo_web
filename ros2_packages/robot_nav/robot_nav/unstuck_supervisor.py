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
  "não fez nada"). A manobra é SEMPRE ré.
- Ré com OLHO NO VÃO (batida de 2026-06-11: a checagem antiga por setor
  angular media do LiDAR e era cega pra quina — o robô recuou em cima de um
  obstáculo atrás): `rear_min_gap` mede em METROS o vão real entre o
  para-choque traseiro e o /scan num corredor retangular da largura do robô.
  Sem vão útil → espera; vão curto → ré PARCIAL (recua o que dá); vão some
  no MEIO da manobra → STOP imediato.
- Só age com goal ativo: o gate primário é o STATUS do action server do
  bt_navigator (ACCEPTED/EXECUTING/CANCELING = ativo) — autoritativo, mata a
  "ré póstuma" pós-cancel e cobre o BT em recovery com o controller mudo.
  Sem status visto ainda, cai no fallback: nav2 comandou há <nav_latch
  (latch tolera os gaps de ~1-2s do ciclo de abort do progress_checker).
- Fim da ré publica um Twist ZERO explícito (cmd_vel_to_wheels segura o último
  comando) e /scan velho >scan_stale trata a traseira como bloqueada.

O collision monitor e a curva (RotationShim/DWB) NÃO são tocados. Ver
`docs/superpowers/specs/2026-06-10-unstuck-supervisor-design.md`.

A lógica de decisão é pura (sem ROS) pra ser testável offline; o nó embaixo é
só a cola de I/O.
"""

import math
from dataclasses import dataclass, field
from typing import List, NamedTuple, Optional, Tuple


# ---- lógica pura -----------------------------------------------------------

def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def rear_min_gap(ranges, angle_min: float, angle_increment: float,
                 lidar_x: float, tail_x: float, half_width: float) -> float:
    """Menor vão livre (m) entre o PARA-CHOQUE traseiro e o que o /scan vê
    no corredor que o corpo varre dando ré. inf = nada atrás.

    Substitui o setor angular de 2026-06-10, que causou a batida de ré de
    2026-06-11 por 3 vias: media a folga a partir do LIDAR (que fica
    `lidar_x` à FRENTE do centro — 0.35m de "folga" = obstáculo ENCOSTADO
    no para-choque), o cone de ±30° era mais estreito que o robô (quina
    traseira a 35° passava despercebida), e era um bool sem noção de
    quanto espaço existe. Aqui cada ponto vira (x,y) no frame base_link e
    conta se cai no retângulo atrás do robô: x < tail_x, |y| <= half_width.
    """
    if angle_increment == 0.0:
        return math.inf
    gap = math.inf
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue
        a = angle_min + i * angle_increment
        x = lidar_x + r * math.cos(a)
        y = r * math.sin(a)
        if x < tail_x and abs(y) <= half_width:
            gap = min(gap, tail_x - x)
    return gap


def freer_side(ranges, angle_min: float, angle_increment: float) -> int:
    """+1 se o setor frontal ESQUERDO (20°..90°) tem mais espaço, -1 se o direito.

    Usado pra escolher pra que lado a ré em arco vira o nariz.
    """
    if angle_increment == 0.0:
        return 1
    lo, hi = math.radians(20.0), math.radians(90.0)
    best = {1: math.inf, -1: math.inf}
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue
        a = _norm_angle(angle_min + i * angle_increment)
        if lo <= a <= hi:
            best[1] = min(best[1], r)
        elif -hi <= a <= -lo:
            best[-1] = min(best[-1], r)
    return 1 if best[1] >= best[-1] else -1


@dataclass
class UnstuckConfig:
    stuck_timeout: float = 10.0
    stuck_radius: float = 0.05     # deslocou menos que isso = "parado"
    reverse_distance: float = 0.30
    reverse_speed: float = 0.25
    reverse_time_cap: float = 6.0
    grace: float = 2.0
    nav_latch: float = 15.0        # nav2 conta como "com goal" se comandou há <=15s
    # Escalada (pedido 2026-06-10: "limite de 3 tentativas até pensar em virar"):
    # ré reta repetida no MESMO ponto não resolve -> a partir da 3ª, depois da
    # ré vem um GIRO FORTE no lugar. Forte porque giro fraco não vence o atrito
    # do skid-steer (0.5 falhou em campo; o RotationShim gira o robô a 3.67).
    # Arco durante a ré também falhou (30cm não muda heading).
    escalate_after: int = 3        # tentativas no mesmo ponto antes do giro
    same_spot_radius: float = 0.5  # raio que define "mesmo ponto"
    escalate_window: float = 120.0  # esquece travamentos mais velhos que isso
    spin_speed: float = 3.0        # rad/s do giro pós-ré (precisa vencer atrito)
    # Giro em MALHA FECHADA no yaw (campo: comanda 30° e a roda patinando
    # entrega 5°): gira até o yaw MEDIDO (IMU, confiável mesmo patinando)
    # acumular spin_angle; spin_time_cap é o teto se nem patinar resolver.
    spin_angle: float = 0.44       # alvo de virada REAL (~25°)
    spin_time_cap: float = 4.0     # teto de tempo do giro
    # As rodas pegam pior girando pra ESQUERDA -> boost de FORÇA nesse lado.
    spin_left_boost: float = 1.4   # velocidade do giro à esquerda x1.4
    # Segurança da ré (batida de 2026-06-11: ré em cima de obstáculo atrás).
    # A ré só sai se houver vão útil, recua NO MÁXIMO (vão - margem) e aborta
    # na hora se o vão cair abaixo da margem durante a manobra.
    rear_stop_margin: float = 0.10  # nunca chega a menos disso do obstáculo
    reverse_min: float = 0.10       # vão útil mínimo pra valer a pena dar ré


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
    anchor: Optional[Tuple[float, float]] = None  # última posição "nova"
    anchor_t: float = 0.0
    maneuver_start_t: float = 0.0
    maneuver_start_pos: Tuple[float, float] = (0.0, 0.0)
    reverse_target: float = 0.0    # quanto recuar NESTA manobra (<= reverse_distance)
    grace_start: float = 0.0
    last_nav_t: Optional[float] = None
    escalated: bool = False    # esta manobra termina em giro forte?
    spin_side: int = 1         # +1 esq / -1 dir
    spin_start_t: float = 0.0
    spin_start_yaw: float = 0.0
    history: List[Tuple[float, Tuple[float, float]]] = field(default_factory=list)

    def update(self, now: float, *, nav_wants_move: bool,
               position: Tuple[float, float], rear_gap: float = math.inf,
               goal_active: Optional[bool] = None,
               open_side: int = 1, yaw: float = 0.0) -> Command:
        if nav_wants_move:
            self.last_nav_t = now
        if self.state == _MONITORING:
            return self._monitoring(now, position, rear_gap, goal_active,
                                    open_side)
        if self.state == _REVERSING:
            return self._reversing(now, position, yaw, rear_gap)
        if self.state == _SPINNING:
            return self._spinning(now, yaw)
        if self.state == _GRACE:
            return self._grace(now)
        return _IDLE

    # -- estados --

    def _monitoring(self, now, position, rear_gap, goal_active,
                    open_side) -> Command:
        if goal_active is not None:
            # status do action server do nav2 disponível: é AUTORITATIVO.
            # Mata a "ré póstuma" (goal cancelado mas flag de nav_vel_raw
            # parado em True) e cobre o BT em recovery (controller mudo
            # mas goal seguindo ativo).
            nav_gate = goal_active
        else:
            # fallback sem status: nav2 comandou há <nav_latch (tolera os
            # gaps de ~1-2s do ciclo de abort do progress_checker)
            nav_gate = (self.last_nav_t is not None
                        and now - self.last_nav_t <= self.cfg.nav_latch)
        if not nav_gate:
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
        # Quanto dá pra recuar SEM bater: o vão medido menos a margem. Vão
        # apertado vira ré PARCIAL (encurralado recua o que dá); sem vão
        # útil, segura e re-tenta no próximo tick (dispara quando liberar).
        target = min(self.cfg.reverse_distance,
                     rear_gap - self.cfg.rear_stop_margin)
        if target < self.cfg.reverse_min:
            return _IDLE
        # escalada: conta travamentos recentes perto DESTE ponto; na 3ª
        # tentativa no mesmo lugar a ré reta não resolveu -> ré + GIRO FORTE
        self.history = [(t, p) for (t, p) in self.history
                        if now - t <= self.cfg.escalate_window]
        self.history.append((now, position))
        nearby = sum(
            1 for (_, p) in self.history
            if self._dist(p, position) <= self.cfg.same_spot_radius)
        self.escalated = nearby >= self.cfg.escalate_after
        self.spin_side = 1 if open_side >= 0 else -1
        self.state = _REVERSING
        self.maneuver_start_t = now
        self.maneuver_start_pos = position
        self.reverse_target = target
        return Command(-self.cfg.reverse_speed, 0.0, True)

    def _spin_cmd(self) -> Command:
        speed = self.cfg.spin_speed
        if self.spin_side > 0:
            speed *= self.cfg.spin_left_boost  # esquerda escorrega: + força
        return Command(0.0, self.spin_side * speed, True)

    def _reversing(self, now, position, yaw, rear_gap) -> Command:
        if rear_gap <= self.cfg.rear_stop_margin:
            # Algo apareceu/entrou atrás DURANTE a ré (batida de 2026-06-11:
            # a checagem era só no disparo). STOP imediato e SEM giro — com
            # coisa colada atrás, girar varre as quinas pra cima dela.
            self.state = _GRACE
            self.grace_start = now
            return Command(0.0, 0.0, True)
        dist = self._dist(position, self.maneuver_start_pos)
        if (dist >= self.reverse_target
                or now - self.maneuver_start_t >= self.cfg.reverse_time_cap):
            if self.escalated:
                # recuou: agora GIRO FORTE no lugar pro lado mais livre —
                # muda o heading de verdade (arco em 30cm não virava)
                self.state = _SPINNING
                self.spin_start_t = now
                self.spin_start_yaw = yaw
                return self._spin_cmd()
            self.state = _GRACE
            self.grace_start = now
            # STOP explícito: o cmd_vel_to_wheels segura o último comando;
            # sem este zero o robô continuaria de ré até o nav2 publicar
            # de novo (que pode estar mudo, abortado/replanejando).
            return Command(0.0, 0.0, True)
        return Command(-self.cfg.reverse_speed, 0.0, True)

    def _spinning(self, now, yaw) -> Command:
        # MALHA FECHADA: para pelo yaw MEDIDO, não por tempo — roda patinando
        # comanda 30° e entrega 5°; a IMU vê a virada real.
        turned = abs(_norm_angle(yaw - self.spin_start_yaw))
        if (turned >= self.cfg.spin_angle
                or now - self.spin_start_t >= self.cfg.spin_time_cap):
            self.state = _GRACE
            self.grace_start = now
            return Command(0.0, 0.0, True)  # STOP explícito
        return self._spin_cmd()

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
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    try:
        from nav2_msgs.msg import CollisionMonitorState
    except ImportError:
        CollisionMonitorState = None

    # STOP = 1 no enum do collision monitor — só pra LOG (não é mais gatilho)
    STOP_ACTION = 1
    # GoalStatus: ACCEPTED=1, EXECUTING=2, CANCELING=3 = goal ainda vivo
    ACTIVE_STATUSES = (1, 2, 3)

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
                ("escalate_after", 3),
                ("same_spot_radius", 0.5),
                ("escalate_window", 120.0),
                ("spin_speed", 3.0),
                ("spin_angle", 0.44),
                ("spin_time_cap", 4.0),
                ("spin_left_boost", 1.4),
                # Geometria da ré (frame base_link): LiDAR fica à FRENTE do
                # centro; o vão é medido do PARA-CHOQUE traseiro (tail_x).
                ("rear_lidar_x", 0.10),
                ("rear_tail_x", -0.25),
                ("rear_half_width", 0.30),
                ("rear_stop_margin", 0.10),
                ("reverse_min", 0.10),
                ("scan_stale", 2.0),
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
                escalate_after=int(g["escalate_after"]),
                same_spot_radius=g["same_spot_radius"],
                escalate_window=g["escalate_window"],
                spin_speed=g["spin_speed"],
                spin_angle=g["spin_angle"],
                spin_time_cap=g["spin_time_cap"],
                spin_left_boost=g["spin_left_boost"],
                rear_stop_margin=g["rear_stop_margin"],
                reverse_min=g["reverse_min"],
            )
            self.rear_lidar_x = g["rear_lidar_x"]
            self.rear_tail_x = g["rear_tail_x"]
            self.rear_half_width = g["rear_half_width"]
            self.scan_stale = g["scan_stale"]
            self.nav_move_lin = g["nav_move_lin"]
            self.nav_move_ang = g["nav_move_ang"]

            self.sup = UnstuckSupervisor(self.cfg)

            self._nav_wants_move = False
            self._position = (0.0, 0.0)
            self._yaw = 0.0
            self._rear_gap = math.inf
            self._open_side = 1  # +1 esq / -1 dir (lado mais livre na frente)
            self._scan_t = None  # quando o último /scan chegou
            self._goal_active = {}  # por tópico de status; None até a 1ª msg
            self._stop_active = False  # só pra log
            self._last_state = self.sup.state

            be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)

            self.pub = self.create_publisher(Twist, "unstuck_vel", 10)
            self.create_subscription(Odometry, "odom", self._on_odom, 10)
            self.create_subscription(Twist, "nav_vel_raw", self._on_nav_raw, 10)
            self.create_subscription(LaserScan, "scan", self._on_scan, be)
            # Status dos goals do bt_navigator: gate AUTORITATIVO (mata a
            # "ré póstuma" após cancel e cobre o BT em recovery c/ controller
            # mudo). Sem msg ainda -> cai no fallback por nav_latch.
            for topic in ("navigate_to_pose/_action/status",
                          "navigate_through_poses/_action/status"):
                self.create_subscription(
                    GoalStatusArray, topic,
                    lambda m, t=topic: self._on_goal_status(t, m), 10)
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
            q = msg.pose.pose.orientation
            self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        def _on_nav_raw(self, msg):
            self._nav_wants_move = (abs(msg.linear.x) > self.nav_move_lin
                                    or abs(msg.angular.z) > self.nav_move_ang)

        def _on_scan(self, msg):
            self._scan_t = self.get_clock().now().nanoseconds * 1e-9
            ranges = list(msg.ranges)
            self._rear_gap = rear_min_gap(
                ranges, msg.angle_min, msg.angle_increment,
                self.rear_lidar_x, self.rear_tail_x, self.rear_half_width)
            self._open_side = freer_side(
                ranges, msg.angle_min, msg.angle_increment)

        def _on_goal_status(self, topic, msg):
            self._goal_active[topic] = any(
                s.status in ACTIVE_STATUSES for s in msg.status_list)

        def _on_collision(self, msg):
            self._stop_active = (getattr(msg, "action_type", 0) == STOP_ACTION)

        def _tick(self):
            now = self.get_clock().now().nanoseconds * 1e-9
            # scan velho (LiDAR caiu?) -> trata traseira como BLOQUEADA
            # (vão zero): melhor segurar a ré do que dar ré cego.
            scan_fresh = (self._scan_t is not None
                          and now - self._scan_t <= self.scan_stale)
            gap = self._rear_gap if scan_fresh else 0.0
            # status visto em algum tópico? OR entre eles; nunca visto -> None
            goal_active = (any(self._goal_active.values())
                           if self._goal_active else None)
            cmd = self.sup.update(
                now, nav_wants_move=self._nav_wants_move,
                position=self._position, rear_gap=gap,
                goal_active=goal_active, open_side=self._open_side,
                yaw=self._yaw)
            if self.sup.state != self._last_state:
                self.get_logger().warn(
                    "unstuck: %s -> %s (pos=%.2f,%.2f stop=%s vao_re=%.2f)" % (
                        self._last_state, self.sup.state,
                        self._position[0], self._position[1],
                        self._stop_active, gap))
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
