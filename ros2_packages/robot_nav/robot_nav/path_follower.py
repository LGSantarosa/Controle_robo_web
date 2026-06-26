#!/usr/bin/env python3
"""path_follower — seguidor decisivo "reto + giro no lugar" p/ skid-steer que NÃO arqueia.

Por que existe (2026-06-25): o arc_calib provou que este robô não faz arco (vira
~3% do comando a 0.5 rad/s; só gira de verdade no lugar). Os controladores de
prateleira do nav2 brigam com isso (DWB arqueia e deriva pro obstáculo; o
RotationShim micro-gira em limite-ciclo = zigue-zague).

Este nó ignora o *tracking* do controller_server e segue o /plan (Theta*, retas
com cantos) com lógica DETERMINÍSTICA + 2 truques que faltavam:

  1. CARROT no caminho: mira um ponto ~`lookahead` m À FRENTE NO PLANO (não o
     goal lá no fim). Assim ele segue a FORMA do plano (o desvio pro vão), em vez
     de cortar reto pro destino e raspar o obstáculo.
  2. HISTERESE: começa a girar quando o erro de heading passa de `turn_enter`,
     mas só PARA de girar quando cai abaixo de `turn_exit` (bem menor). Sem isso
     ele girava e parava no MESMO limiar -> limite-ciclo = "pulinhos". Com
     histerese: gira decidido até alinhar, anda comprometido, re-gira só no canto.

Estados: idle | turning | driving | goal_turn | arrived.
Publica `follow_vel` (twist_mux prio 15 > nav_vel 10 = ignora controller_server;
< door 20 < unstuck 30). Enquanto há goal ativo SEMPRE publica (segura o mux).

A LÓGICA é pura (funções + DecisiveFollower) p/ testar sem ROS; o main() é a cola
de I/O (TF, /plan, status do goal, publisher, timer, CSV de diagnóstico).
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


def carrot_point(path: List[Pt], i0: int, lookahead: float) -> Tuple[int, Pt]:
    """Ponto do caminho a ~`lookahead` m À FRENTE (arc-length) do índice i0.
    Devolve (índice, ponto). Se o caminho acaba antes, devolve o último (goal).
    É o 'carrot' do pure-pursuit — mas aqui ele só dá a DIREÇÃO pra reto/giro."""
    acc = 0.0
    n = len(path)
    for k in range(i0, n - 1):
        acc += math.hypot(path[k + 1][0] - path[k][0],
                          path[k + 1][1] - path[k][1])
        if acc >= lookahead:
            return k + 1, path[k + 1]
    return n - 1, path[-1]


@dataclass
class FollowConfig:
    forward_speed: float = 0.25     # m/s no trecho reto
    lookahead: float = 0.6          # m — distância do carrot à frente no plano
                                    # (1.0 cortava por dentro do arco e raspava o
                                    # obstáculo; 0.6 cola na linha do plano)
    turn_enter: float = 0.21        # rad (~12°) — acima disso COMEÇA a girar
    turn_exit: float = 0.05         # rad (~3°)  — abaixo disso PARA de girar (histerese)
    goal_xy_tol: float = 0.15       # m — chegou no goal (casa c/ goal_checker do nav2)
    goal_yaw_tol: float = 0.10      # rad (~6°) — encarou o yaw do goal
    rot_k: float = 3.0              # ganho P do giro (rad/s por rad)
    rot_min: float = 2.0            # rad/s — piso do giro (vence a zona-morta 1.7)
    rot_max: float = 4.5            # rad/s — teto do giro
    slow_radius: float = 0.4        # m — começa a frear o avanço perto do goal
    # 2026-06-26: 0.10 -> 0.22. CAMPO: perto do goal o ramp baixava p/ ~0.10-0.11
    # m/s, ABAIXO da zona-morta linear do robô real (pesado) -> mandava 0.11 e NÃO
    # ANDAVA = congelava sem finalizar nenhum ponto (precisava empurrar no controle).
    # 0.11 trava, 0.25 (cruise) anda -> a zona-morta tá no meio; 0.22 fica bem acima.
    # Overshoot a 20Hz ~1cm, irrelevante vs goal_xy_tol 0.15. Se AINDA rastejar/
    # travar, subir p/ 0.25; se passar do ponto, descer. (Zona-morta linear NUNCA
    # foi medida — só a do giro=1.7; o sim_actuator_model tb só modela o giro, por
    # isso o sim não pegava esse trava.)
    min_speed: float = 0.22         # m/s — avanço mínimo (acima da zona-morta)


@dataclass
class Cmd:
    vx: float
    wz: float
    state: str          # idle | turning | driving | goal_turn | arrived


class DecisiveFollower:
    def __init__(self, cfg: FollowConfig):
        self.cfg = cfg
        self.state = 'idle'
        self.dbg = {}        # diagnóstico do último update (logado pelo nó)

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
                    self.dbg = {'i0': len(path) - 1, 'ci': len(path) - 1,
                                'n': len(path), 'ax': gx, 'ay': gy,
                                'herr_deg': math.degrees(yerr), 'dist_aim': 0.0,
                                'dist_goal': dist_goal}
                    return Cmd(0.0, self._turn_cmd(yerr), 'goal_turn')
            self.state = 'arrived'
            return Cmd(0.0, 0.0, 'arrived')

        # 2) CARROT no plano a ~lookahead à frente (segue a FORMA do caminho)
        i0 = closest_index(path, x, y)
        ci, (ax, ay) = carrot_point(path, i0, c.lookahead)
        bearing = math.atan2(ay - y, ax - x)
        herr = wrap(bearing - yaw)
        dist_aim = math.hypot(ax - x, ay - y)
        self.dbg = {'i0': i0, 'ci': ci, 'n': len(path), 'ax': ax, 'ay': ay,
                    'herr_deg': math.degrees(herr), 'dist_aim': dist_aim,
                    'dist_goal': dist_goal}

        # 3) HISTERESE: girando -> só sai quando alinha BEM; senão -> entra em giro
        #    quando o erro passa do turn_enter. Quebra o limite-ciclo (pulinho).
        if self.state == 'turning':
            if abs(herr) <= c.turn_exit:
                self.state = 'driving'
        else:
            if abs(herr) >= c.turn_enter:
                self.state = 'turning'
            else:
                self.state = 'driving'

        if self.state == 'turning':
            return Cmd(0.0, self._turn_cmd(herr), 'turning')

        # 4) de frente -> anda RETO (wz=0). Freia perto do goal.
        speed = c.forward_speed
        if dist_goal < c.slow_radius:
            speed = max(c.min_speed, c.forward_speed * dist_goal / c.slow_radius)
        return Cmd(speed, 0.0, 'driving')


def main(args=None):  # pragma: no cover - cola de I/O, validar no sim/bancada
    import csv as _csv
    import os as _os
    import time as _time

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
                ('forward_speed', 0.25), ('lookahead', 0.6),
                ('turn_enter_deg', 12.0), ('turn_exit_deg', 3.0),
                ('goal_xy_tol', 0.15), ('goal_yaw_tol_deg', 6.0),
                ('rot_k', 3.0), ('rot_min', 2.0), ('rot_max', 4.5),
                ('slow_radius', 0.4), ('min_speed', 0.22), ('rate_hz', 20.0),
            ):
                self.declare_parameter(name, default)
                g[name] = self.get_parameter(name).value

            self.cfg = FollowConfig(
                forward_speed=g['forward_speed'], lookahead=g['lookahead'],
                turn_enter=math.radians(g['turn_enter_deg']),
                turn_exit=math.radians(g['turn_exit_deg']),
                goal_xy_tol=g['goal_xy_tol'],
                goal_yaw_tol=math.radians(g['goal_yaw_tol_deg']),
                rot_k=g['rot_k'], rot_min=g['rot_min'], rot_max=g['rot_max'],
                slow_radius=g['slow_radius'], min_speed=g['min_speed'])
            self.fol = DecisiveFollower(self.cfg)

            self._path: Optional[List[Pt]] = None
            self._goal_yaw: Optional[float] = None
            self._goal_active = {}

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            # publica follow_vel DIRETO no twist_mux (prio 15). 2026-06-26: tentei
            # rotear pelo collision (follow_vel_raw -> collision -> follow_vel) p/ o
            # reflexo proteger o seguidor, mas isso criou PONTO ÚNICO DE FALHA: quando
            # o bringup do Nav2 aborta antes de ativar o collision_monitor (bond
            # timeout do velocity_smoother na Pi lenta), o follow_vel não era
            # republicado e a NAV INTEIRA MORRIA (robô só andava no controle/unstuck).
            # Revertido: o seguidor não depende do collision pra dirigir.
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
            d = 'controle_web/logs'
            _os.makedirs(d, exist_ok=True)
            self._csv_f = open(_os.path.join(d, 'follow_debug.csv'), 'w', newline='')
            self._csv = _csv.writer(self._csv_f)
            self._csv.writerow(['t', 'state', 'x', 'y', 'yaw_deg', 'i0', 'ci', 'n',
                                'aim_x', 'aim_y', 'herr_deg', 'dist_aim',
                                'dist_goal', 'vx', 'wz'])
            self._plan_path = _os.path.join(d, 'follow_plan_last.csv')
            # Snapshot do PRIMEIRO plano longo de cada goal (a FORMA do contorno —
            # o last.csv vira stub coladinho no goal). Resetado quando um novo goal
            # fica ativo (_on_status).
            self._plan_first_path = _os.path.join(d, 'follow_plan_first.csv')
            self._plan_snapped = False
            self._goal_active_any = False
            self._time = _time
            self.create_timer(1.0 / g['rate_hz'], self._tick)
            self.get_logger().info(
                'path_follower ativo: reto %.2fm/s, carrot %.1fm, gira>%.0f° '
                'até <%.0f°, giro %.1f–%.1f rad/s' % (
                    self.cfg.forward_speed, self.cfg.lookahead,
                    g['turn_enter_deg'], g['turn_exit_deg'],
                    self.cfg.rot_min, self.cfg.rot_max))

        def _on_plan(self, msg: Path):
            self._path = [(p.pose.position.x, p.pose.position.y)
                          for p in msg.poses]
            if msg.poses:
                q = msg.poses[-1].pose.orientation
                self._goal_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                # dump do plano (sobrescreve) p/ eu inspecionar a FORMA depois
                try:
                    with open(self._plan_path, 'w', newline='') as f:
                        w = _csv.writer(f)
                        w.writerow(['x', 'y'])
                        w.writerows(self._path)
                except OSError:
                    pass
                # primeiro plano LONGO do goal (>=0.5 m) -> grava 1x a forma do
                # contorno num arquivo que NÃO é sobrescrito pelo stub final
                if not self._plan_snapped and len(self._path) >= 2:
                    plen = sum(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
                               for a, b in zip(self._path, self._path[1:]))
                    if plen >= 0.5:
                        try:
                            with open(self._plan_first_path, 'w', newline='') as f:
                                w = _csv.writer(f)
                                w.writerow(['x', 'y'])
                                w.writerows(self._path)
                            self._plan_snapped = True
                        except OSError:
                            pass

        def _on_status(self, topic, msg):
            self._goal_active[topic] = any(st.status in ACTIVE
                                           for st in msg.status_list)
            active = any(self._goal_active.values())
            if active and not self._goal_active_any:
                # novo goal -> libera o snapshot do primeiro plano longo
                self._plan_snapped = False
            self._goal_active_any = active

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

            # SEGURA O MUX: com goal ativo SEMPRE publica (prio 15 não expira ->
            # o controller_server prio 10 nunca assume e briga). Sem goal -> cala.
            if goal:
                m = Twist()
                m.linear.x = cmd.vx
                m.angular.z = cmd.wz
                self.pub.publish(m)
            if cmd.state != self._last_state:
                self._last_state = cmd.state
                self.pub_state.publish(String(data=cmd.state))

            if goal:
                d = self.fol.dbg
                x = pose[0] if pose else float('nan')
                y = pose[1] if pose else float('nan')
                yd = math.degrees(pose[2]) if pose else float('nan')
                self._csv.writerow([
                    round(self._time.time(), 3), cmd.state, round(x, 3),
                    round(y, 3), round(yd, 1), d.get('i0', ''), d.get('ci', ''),
                    d.get('n', ''), round(d.get('ax', float('nan')), 3),
                    round(d.get('ay', float('nan')), 3),
                    round(d.get('herr_deg', float('nan')), 1),
                    round(d.get('dist_aim', float('nan')), 3),
                    round(d.get('dist_goal', float('nan')), 3),
                    round(cmd.vx, 3), round(cmd.wz, 3)])
                self._csv_f.flush()

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
