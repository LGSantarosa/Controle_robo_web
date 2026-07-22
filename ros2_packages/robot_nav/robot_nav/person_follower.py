"""person_follower — modo 'seguir pessoa' (tap-to-track por lidar).

Lógica PURA (classe PersonFollower/FollowConfig) testável sem ROS, no molde
do motion_guard. O main() é só cola ROS (# pragma: no cover), validado no sim.
Frame de trabalho = odom: clusters e alvo em (cx,cy); o controle converte pra
bearing/dist relativo usando a pose. A velocidade de saída é DESEJO — a
segurança (guard/collision/unstuck/E-stop) é aplicada a JUSANTE no pipeline
(follow_person_vel -> twist_mux_auto -> motion_guard -> collision_monitor).
"""
import math
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
    lost_grace: float = 1.0
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
        """Trava o cluster mais PRÓXIMO dentro do alcance e do cone frontal."""
        cfg = self.cfg
        best, best_d = None, math.inf
        for cx, cy in clusters:
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
        for cx, cy in clusters:
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
            if t - self._last_seen > self.cfg.lost_grace:
                self.state = 'lost'
                self._lost_since = t
                self.just_spoke = 'lost'
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
