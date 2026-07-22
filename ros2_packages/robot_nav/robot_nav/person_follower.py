"""person_follower — modo 'seguir pessoa' (tap-to-track por lidar).

Lógica PURA (classe PersonFollower/FollowConfig) testável sem ROS, no molde
do motion_guard. O main() é só cola ROS (# pragma: no cover), validado no sim.
Frame de trabalho = odom: clusters e alvo em (cx,cy); o controle converte pra
bearing/dist relativo usando a pose. A velocidade de saída é DESEJO — a
segurança (guard/collision/unstuck/E-stop) é aplicada a JUSANTE no pipeline
(follow_person_vel -> twist_mux_auto -> motion_guard -> collision_monitor).
"""
import json
import math
import os
from collections import namedtuple
from dataclasses import dataclass

Target = namedtuple('Target', 'cx cy')


@dataclass
class FollowConfig:
    stop_dist: float = 1.5
    stop_hyst: float = 0.2
    vx_max: float = 0.25
    wz_cap: float = 2.4
    wz_kp: float = 2.0            # ganho do giro (rad/s por rad de erro), antes do cap
    face_deadband_deg: float = 8.0
    drive_align_deg: float = 20.0
    acquire_cone_deg: float = 60.0
    acquire_range: float = 3.0
    assoc_gate: float = 0.6
    lost_grace: float = 2.0        # tolera piscada do alvo (guard só vê movimento)
    lost_timeout: float = 12.0


def _wrap_rad(rad: float) -> float:
    return (rad + math.pi) % (2 * math.pi) - math.pi


def _rel(cx: float, cy: float, pose):
    """(dist, bearing_deg) do ponto odom (cx,cy) relativo ao robô.
    bearing 0 = frente, + = esquerda."""
    rx, ry, ryaw = pose
    dx, dy = cx - rx, cy - ry
    dist = math.hypot(dx, dy)
    bearing = math.degrees(_wrap_rad(math.atan2(dy, dx) - ryaw))
    return dist, bearing


class PersonFollower:
    def __init__(self, cfg: FollowConfig):
        self.cfg = cfg
        self.state = 'idle'
        self.target = None
        self._driving = False
        self._start_req = False
        self.just_spoke = None      # 'start'|'lost'|None — evento de fala (consumido pelo nó)
        self.no_target = False      # start pedido mas ninguém no cone
        self._last_seen = 0.0
        self._lost_since = 0.0

    def acquire(self, clusters, pose):
        """Trava o cluster MÓVEL mais próximo dentro do alcance e do cone
        frontal. Clusters chegam como (cx, cy[, movendo]); parede estática
        (movendo=0) NÃO vira alvo. Sem o flag = trata como móvel."""
        cfg = self.cfg
        best, best_d = None, math.inf
        for c in clusters:
            cx, cy = c[0], c[1]
            moving = c[2] if len(c) > 2 else 1.0
            if not moving:
                continue
            d, b = _rel(cx, cy, pose)
            if d <= cfg.acquire_range and abs(b) <= cfg.acquire_cone_deg / 2 and d < best_d:
                best, best_d = Target(cx, cy), d
        return best

    def associate(self, clusters):
        """Casa self.target com o cluster mais próximo dentro do gate (odom)."""
        if self.target is None:
            return None
        tx, ty = self.target
        best, best_d = None, self.cfg.assoc_gate
        for c in clusters:
            cx, cy = c[0], c[1]
            d = math.hypot(cx - tx, cy - ty)
            if d <= best_d:
                best, best_d = Target(cx, cy), d
        if best is not None:
            self.target = best
        return best

    def control(self, dist, bearing_deg):
        """(vx, wz) desejados pra encarar e manter stop_dist. vx >= 0 (não recua)."""
        cfg = self.cfg
        # --- giro: encara o alvo ---
        if abs(bearing_deg) < cfg.face_deadband_deg:
            wz = 0.0
        else:
            wz = math.radians(bearing_deg) * cfg.wz_kp
            wz = max(-cfg.wz_cap, min(cfg.wz_cap, wz))
        # --- avanço: mantém stop_dist, com histerese p/ não pulsar ---
        if self._driving:
            if dist <= cfg.stop_dist:
                self._driving = False
        else:
            if dist > cfg.stop_dist + cfg.stop_hyst:
                self._driving = True
        aligned = abs(bearing_deg) < cfg.drive_align_deg
        if self._driving and aligned:
            vx = min(cfg.vx_max, max(0.0, dist - cfg.stop_dist))
        else:
            vx = 0.0
        return vx, wz

    # --- máquina de estados ---
    def start(self):
        self._start_req = True
        self.no_target = False

    def stop(self):
        if self.state in ('following', 'lost'):
            self.state = 'ending'

    def reset(self):
        self.state = 'idle'
        self.target = None
        self._start_req = False
        self._driving = False
        self.just_spoke = None
        self.no_target = False

    def tick(self, t, clusters, pose):
        """Avança a máquina UMA vez com o relógio `t` (s, travado na fonte).
        Retorna (vx, wz) — não-zero só em following com alvo casado."""
        if self.state == 'idle':
            if self._start_req:
                self._start_req = False
                tgt = self.acquire(clusters, pose)
                if tgt is not None:
                    self.target = tgt
                    self.state = 'following'
                    self.just_spoke = 'start'
                    self._last_seen = t
                else:
                    self.no_target = True
            return 0.0, 0.0

        if self.state == 'following':
            m = self.associate(clusters)
            if m is not None:
                self._last_seen = t
                dist, bearing = _rel(m.cx, m.cy, pose)
                return self.control(dist, bearing)
            # sem match: o alvo pisca (guard só vê movimento). Dentro do grace,
            # MANTÉM o rumo pro último alvo conhecido (posição fixa no odom) em
            # vez de congelar — seguimento liso, sem flap following<->lost.
            if t - self._last_seen > self.cfg.lost_grace:
                self.state = 'lost'
                self._lost_since = t
                self.just_spoke = 'lost'
                return 0.0, 0.0
            if self.target is not None:
                dist, bearing = _rel(self.target.cx, self.target.cy, pose)
                return self.control(dist, bearing)
            return 0.0, 0.0

        if self.state == 'lost':
            m = self.associate(clusters)
            if m is not None:
                self.state = 'following'
                self._last_seen = t
                dist, bearing = _rel(m.cx, m.cy, pose)
                return self.control(dist, bearing)
            if t - self._lost_since > self.cfg.lost_timeout:
                self.state = 'ending'
            return 0.0, 0.0

        # ending
        return 0.0, 0.0


class FollowFaceFile:
    """Estado do seguir pro face_web (iPad). JSON minúsculo em tmpfs, atômico
    (tmp + os.replace), ≤5Hz p/ o estado periódico; a fala (speak) grava na
    hora. I/O NUNCA propaga (a cara é decorativa; o follower não cai por ela)."""

    def __init__(self, path: str = '/tmp/person_follow_face.json',
                 min_period: float = 0.2):
        self.path = path
        self.min_period = min_period
        self.last_error = None
        self._last_write_t = -math.inf

    def update(self, t, state, speak=None, bearing_deg=None) -> bool:
        if speak is None and t - self._last_write_t < self.min_period:
            return False
        try:
            tmp = self.path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'ts': round(t, 3), 'follow_state': state,
                           'speak': speak, 'cbear_deg': bearing_deg}, f)
            os.replace(tmp, self.path)
        except OSError as e:
            self.last_error = str(e)
            return False
        self._last_write_t = t
        return True


# knobs afináveis ao vivo (mesma disciplina do motion_guard): mutam a MESMA
# ref de cfg que o tick lê -> `ros2 param set /person_follower <x> <v>` pega
# no tick seguinte. Exceção: lost_grace/lost_timeout também são lidos ao vivo.
_CFG_PARAMS = ('stop_dist', 'stop_hyst', 'vx_max', 'wz_cap', 'wz_kp',
               'face_deadband_deg', 'drive_align_deg', 'acquire_cone_deg',
               'acquire_range', 'assoc_gate', 'lost_grace', 'lost_timeout')


def main(args=None):  # pragma: no cover - cola de I/O, validar no sim
    import rclpy
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32MultiArray, String

    from .utils import quat_to_yaw, spin_node

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class PersonFollowerNode(Node):
        def __init__(self):
            super().__init__('person_follower')
            self.declare_parameter('follow_enabled', False)
            self.enabled = self.get_parameter('follow_enabled').value
            cfg = FollowConfig()
            for name in _CFG_PARAMS:
                self.declare_parameter(name, getattr(cfg, name))
                setattr(cfg, name, self.get_parameter(name).value)
            self.pf = PersonFollower(cfg)
            self.add_on_set_parameters_callback(self._on_set_params)
            self.face = FollowFaceFile()

            self._targets = []
            self._pose = (0.0, 0.0, 0.0)
            self.pub = self.create_publisher(Twist, 'follow_person_vel', 10)
            self.pub_state = self.create_publisher(
                String, 'follow_person_state', latched)
            self.create_subscription(Float32MultiArray, 'follow_person_targets',
                                     self._on_targets, 10)
            self.create_subscription(String, 'follow_cmd', self._on_cmd, 10)
            self.create_subscription(Odometry, 'odom', self._on_odom,
                                     qos_profile_sensor_data)
            self.create_timer(0.1, self._tick)     # 10 Hz
            self._publish_state()

        def _on_set_params(self, params):
            for p in params:
                if p.name == 'follow_enabled':
                    self.enabled = bool(p.value)
                elif p.name in _CFG_PARAMS:
                    setattr(self.pf.cfg, p.name, p.value)
            return SetParametersResult(successful=True)

        def _on_targets(self, msg):
            d = list(msg.data)
            # triplas (cx, cy, movendo)
            self._targets = [(d[i], d[i + 1], d[i + 2])
                             for i in range(0, len(d) - 2, 3)]

        def _on_cmd(self, msg):
            if not self.enabled:
                return
            if msg.data == 'START':
                self.pf.start()
            elif msg.data == 'STOP':
                self.pf.stop()

        def _on_odom(self, msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            self._pose = (p.x, p.y, yaw)

        def _tick(self):
            t = self.get_clock().now().nanoseconds * 1e-9
            vx, wz = self.pf.tick(t, self._targets, self._pose)
            if self.pf.state == 'following':
                m = Twist()
                m.linear.x = float(vx)
                m.angular.z = float(wz)
                self.pub.publish(m)
            speak = self.pf.just_spoke
            self.pf.just_spoke = None
            if self.pf.target is not None:
                _, bearing = _rel(self.pf.target.cx, self.pf.target.cy, self._pose)
            else:
                bearing = None
            self.face.update(t, self.pf.state, speak=speak, bearing_deg=bearing)
            self._publish_state()
            if self.pf.state == 'ending':
                self.pf.reset()

        def _publish_state(self):
            self.pub_state.publish(String(data=self.pf.state))

    rclpy.init(args=args)
    node = PersonFollowerNode()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
