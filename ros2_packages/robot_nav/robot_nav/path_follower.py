#!/usr/bin/env python3
"""path_follower — seguidor decisivo "reto + giro no lugar" p/ skid-steer que NÃO arqueia.

Por que existe (2026-06-25): o arc_calib provou que este robô não faz arco (vira
~3% do comando a 0.5 rad/s; só gira de verdade no lugar). Os controladores de
prateleira do nav2 brigam com isso:
  - DWB/arco  -> tenta seguir a CURVA do plano arqueando -> não arqueia ->
                 deriva RETO pra cima do obstáculo (espera o unstuck);
  - DWB reto + RotationShim -> zigue-zague de micro-giros, e o lookahead do shim
                 pula o canto -> gira pro LADO ERRADO.

Este nó ignora o *tracking* do controller_server e segue o /plan (idealmente do
Theta*, que dá RETAS com cantos) com lógica DETERMINÍSTICA:
  1. acha o próximo CANTO do plano (onde a direção muda);
  2. se o robô não está de frente pro canto -> GIRA no lugar pelo MENOR ângulo
     (nunca pro lado errado), autoridade alta (acima da zona-morta 1.7);
  3. se está de frente -> anda RETO (wz=0) até o canto -> re-mira o próximo;
  4. chegou no goal -> gira pra encarar o yaw final -> para.

Publica em `follow_vel`. No twist_mux entra em prioridade 15: > nav_vel (10, o
controller_server fica ignorado) e < door (20) < unstuck (30) — então o
door_crossing assume a porta e o unstuck resgata, como antes.

A LÓGICA é pura (funções + classe DecisiveFollower) p/ testar sem ROS; o main()
é só a cola de I/O (TF, /plan, status do goal, publisher, timer).
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

Pt = Tuple[float, float]


def wrap(a: float) -> float:
    """normaliza ângulo p/ (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def closest_index(path: List[Pt], x: float, y: float) -> int:
    """índice do ponto do caminho mais próximo de (x,y)."""
    best_i, best_d = 0, float('inf')
    for i, (px, py) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def _dir_at(path: List[Pt], i: int, window: int) -> Optional[float]:
    """direção (rad) do caminho em i, olhando `window` pontos à frente (suaviza ruído)."""
    j = min(i + window, len(path) - 1)
    if j <= i:
        return None
    return math.atan2(path[j][1] - path[i][1], path[j][0] - path[i][0])


def next_corner_index(path: List[Pt], i0: int, corner_tol: float,
                      window: int) -> int:
    """A partir de i0, anda pelo caminho e devolve o índice do PRÓXIMO CANTO:
    o primeiro ponto onde a direção do caminho desvia mais que `corner_tol` (rad)
    da direção inicial. Se não houver canto até o fim, devolve o último índice
    (= o goal). Assim o alvo é sempre 'o fim da reta atual'."""
    n = len(path)
    if i0 >= n - 1:
        return n - 1
    base = _dir_at(path, i0, window)
    if base is None:
        return n - 1
    k = i0 + 1
    while k < n - 1:
        d = _dir_at(path, k, window)
        if d is not None and abs(wrap(d - base)) > corner_tol:
            return k
        k += 1
    return n - 1


@dataclass
class FollowConfig:
    forward_speed: float = 0.25     # m/s no trecho reto
    turn_tol: float = 0.17          # rad (~10°) — acima disso GIRA antes de andar
    arrive_tol: float = 0.20        # m — chegou no canto-alvo (re-mira o próximo)
    goal_xy_tol: float = 0.15       # m — chegou no goal (casa c/ goal_checker do nav2)
    goal_yaw_tol: float = 0.17      # rad (~10°) — encarou o yaw do goal
    corner_tol: float = 0.35        # rad (~20°) — desvio que conta como CANTO
    dir_window: int = 6             # pontos à frente p/ medir direção do caminho
    rot_k: float = 3.0              # ganho P do giro (rad/s por rad)
    rot_min: float = 2.0            # rad/s — piso do giro (vence a zona-morta 1.7)
    rot_max: float = 4.5            # rad/s — teto do giro
    slow_radius: float = 0.4        # m — começa a frear o avanço ao chegar no alvo
    min_speed: float = 0.10         # m/s — avanço mínimo (não rastejar)


@dataclass
class Cmd:
    vx: float
    wz: float
    state: str          # idle | turning | driving | goal_turn | arrived


class DecisiveFollower:
    """Estado mínimo: só guarda o último estado p/ histerese leve do giro."""

    def __init__(self, cfg: FollowConfig):
        self.cfg = cfg
        self.state = 'idle'

    def _turn_cmd(self, herr: float) -> float:
        """giro no lugar pelo MENOR ângulo: sinal = sinal do erro; magnitude P
        saturada entre rot_min e rot_max."""
        c = self.cfg
        mag = min(c.rot_max, max(c.rot_min, abs(herr) * c.rot_k))
        return math.copysign(mag, herr)

    def update(self, pose: Optional[Tuple[float, float, float]],
               path: Optional[List[Pt]], goal_active: bool,
               goal_yaw: Optional[float]) -> Cmd:
        c = self.cfg
        if pose is None or not goal_active or not path or len(path) < 2:
            self.state = 'idle'
            return Cmd(0.0, 0.0, 'idle')

        x, y, yaw = pose
        gx, gy = path[-1]
        dist_goal = math.hypot(gx - x, gy - y)

        # 1) chegou no goal (xy) -> encara o yaw do goal, depois para
        if dist_goal <= c.goal_xy_tol:
            if goal_yaw is not None:
                yerr = wrap(goal_yaw - yaw)
                if abs(yerr) > c.goal_yaw_tol:
                    self.state = 'goal_turn'
                    return Cmd(0.0, self._turn_cmd(yerr), 'goal_turn')
            self.state = 'arrived'
            return Cmd(0.0, 0.0, 'arrived')

        # 2) alvo = próximo canto do plano (fim da reta atual)
        i0 = closest_index(path, x, y)
        ci = next_corner_index(path, i0, c.corner_tol, c.dir_window)
        ax, ay = path[ci]
        # se o canto-alvo está colado, mira direto o goal (evita jitter no fim)
        if math.hypot(ax - x, ay - y) < c.arrive_tol:
            ax, ay = gx, gy

        bearing = math.atan2(ay - y, ax - x)
        herr = wrap(bearing - yaw)

        # 3) não está de frente -> GIRA no lugar (reto só depois de alinhar)
        if abs(herr) > c.turn_tol:
            self.state = 'turning'
            return Cmd(0.0, self._turn_cmd(herr), 'turning')

        # 4) de frente -> anda RETO (wz=0). Freia perto do alvo.
        dist_aim = math.hypot(ax - x, ay - y)
        speed = c.forward_speed
        if dist_aim < c.slow_radius:
            speed = max(c.min_speed, c.forward_speed * dist_aim / c.slow_radius)
        self.state = 'driving'
        return Cmd(speed, 0.0, 'driving')


def main(args=None):  # pragma: no cover - cola de I/O, validar no sim/bancada
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Path
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    from .utils import quat_to_yaw, spin_node

    ACTIVE = {1, 2, 3}  # ACCEPTED, EXECUTING, CANCELING (igual door/unstuck)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class PathFollowerNode(Node):
        def __init__(self):
            super().__init__('path_follower')
            g = {}
            for name, default in (
                ('forward_speed', 0.25), ('turn_tol_deg', 10.0),
                ('arrive_tol', 0.20), ('goal_xy_tol', 0.15),
                ('goal_yaw_tol_deg', 10.0), ('corner_tol_deg', 20.0),
                ('dir_window', 6), ('rot_k', 3.0), ('rot_min', 2.0),
                ('rot_max', 4.5), ('slow_radius', 0.4), ('min_speed', 0.10),
                ('rate_hz', 20.0),
            ):
                self.declare_parameter(name, default)
                g[name] = self.get_parameter(name).value

            self.cfg = FollowConfig(
                forward_speed=g['forward_speed'],
                turn_tol=math.radians(g['turn_tol_deg']),
                arrive_tol=g['arrive_tol'], goal_xy_tol=g['goal_xy_tol'],
                goal_yaw_tol=math.radians(g['goal_yaw_tol_deg']),
                corner_tol=math.radians(g['corner_tol_deg']),
                dir_window=int(g['dir_window']), rot_k=g['rot_k'],
                rot_min=g['rot_min'], rot_max=g['rot_max'],
                slow_radius=g['slow_radius'], min_speed=g['min_speed'])
            self.fol = DecisiveFollower(self.cfg)

            self._path: Optional[List[Pt]] = None
            self._goal_yaw: Optional[float] = None
            self._goal_active = {}

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.pub = self.create_publisher(Twist, 'follow_vel', 10)
            self.pub_state = self.create_publisher(String, 'follow_state', latched)

            self.create_subscription(Path, 'plan', self._on_plan,
                                     qos_profile_sensor_data)
            for topic in ('navigate_to_pose/_action/status',
                          'navigate_through_poses/_action/status'):
                self.create_subscription(
                    GoalStatusArray, topic,
                    lambda m, t=topic: self._on_status(t, m), 10)

            self._last_state = None
            self.create_timer(1.0 / g['rate_hz'], self._tick)
            self.get_logger().info(
                'path_follower ativo: reto %.2fm/s, gira |err|>%.0f°, '
                'canto>%.0f°, giro %.1f–%.1f rad/s' % (
                    self.cfg.forward_speed, g['turn_tol_deg'],
                    g['corner_tol_deg'], self.cfg.rot_min, self.cfg.rot_max))

        def _on_plan(self, msg: Path):
            self._path = [(p.pose.position.x, p.pose.position.y)
                          for p in msg.poses]
            if msg.poses:
                q = msg.poses[-1].pose.orientation
                self._goal_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        def _on_status(self, topic, msg):
            self._goal_active[topic] = any(st.status in ACTIVE
                                           for st in msg.status_list)

        def _pose_map(self):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
            except TransformException:
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))

        def _tick(self):
            goal = any(self._goal_active.values()) if self._goal_active else False
            pose = self._pose_map()
            cmd = self.fol.update(pose, self._path, goal, self._goal_yaw)
            # só publica quando há comando ativo; idle/arrived -> não publica
            # (deixa o mux cair pra baixo e o robô parar/ceder).
            if cmd.state not in ('idle', 'arrived'):
                m = Twist()
                m.linear.x = cmd.vx
                m.angular.z = cmd.wz
                self.pub.publish(m)
            if cmd.state != self._last_state:
                self._last_state = cmd.state
                self.pub_state.publish(String(data=cmd.state))

    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
