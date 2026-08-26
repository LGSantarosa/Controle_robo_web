#!/usr/bin/env python3
"""Uma corrida do diagnostico de associacao: carrega uma rota, da PLAY, mede.

METRICA DE ERRO FINAL — e' ancorada no CONE FISICO, de proposito. O waypoint
final foi gravado em `odom`, um frame que ja estava derivado na hora da
gravacao; comparar a pose final com ele mediria a minha deriva de gravacao
junto. O que a rota REALMENTE pede e' "pare a (dx,dy) do cone" — esse offset e'
invariante. Entao:

    alvo_verdadeiro = cone_REAL_do_mundo + (waypoint_gravado - cone_gravado)

O cone real vem do .sdf (verdade-terreno), nao da odom. Irmao do gt_trekking.py
e da licao de 2026-08-24 (converter odom->mundo por premissa mente).
"""
import argparse, json, math, re, subprocess, sys, time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

_POSE_RE = re.compile(r'- Pose[^\n]*\n\s*\[([-\d.e ]+)\]\s*\n\s*\[([-\d.e ]+)\]')


def gt(modelo='sim_robot'):
    """(x, y, yaw) verdadeiros do Gazebo, ou None."""
    for _ in range(4):
        try:
            out = subprocess.run(['gz', 'model', '-m', modelo, '-p'],
                                 capture_output=True, text=True, timeout=6).stdout
        except Exception:
            out = ''
        m = _POSE_RE.search(out)
        if m:
            xyz = [float(v) for v in m.group(1).split()]
            rpy = [float(v) for v in m.group(2).split()]
            return xyz[0], xyz[1], rpy[2]
        time.sleep(0.3)
    return None


def cones_do_mundo(path):
    """[(x, y), ...] dos cones do .sdf — verdade-terreno, nao odometria."""
    txt = open(path).read()
    out = []
    for m in re.finditer(r'<model name="cone_\d+">\s*<static>[^<]*</static>'
                         r'\s*<pose>([-\d.e ]+)</pose>', txt):
        v = [float(t) for t in m.group(1).split()]
        out.append((v[0], v[1]))
    return out


class T(Node):
    def __init__(self):
        super().__init__('diag_assoc')
        self.pub = self.create_publisher(String, 'trekking/cmd', 10)
        self.st = None
        self.odom = None
        self.create_subscription(String, 'trekking/state', self._s, 10)
        self.create_subscription(PoseStamped, 'trekking/pose', self._p, 10)

    def _s(self, m):
        try:
            self.st = json.loads(m.data)
        except Exception:
            pass

    def _p(self, m):
        self.odom = (m.pose.position.x, m.pose.position.y)

    def cmd(self, **kw):
        self.pub.publish(String(data=json.dumps(kw)))


def spin(n, s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.05)


ap = argparse.ArgumentParser()
ap.add_argument('--rota', default='maps/routes/trekking/rota2.json')
ap.add_argument('--world', default='worlds/trekking.sdf')
ap.add_argument('--spawn', default='2.0,2.5')
ap.add_argument('--timeout', type=float, default=120.0)
a = ap.parse_args()

rota = json.load(open(a.rota))
wps = rota['waypoints']
trail = rota.get('trail') or []
spawn = [float(v) for v in a.spawn.split(',')]
mundo = cones_do_mundo(a.world)

rclpy.init()
n = T()
spin(n, 3.0)
if n.st is None:
    print(json.dumps({'fim': 'SEM_STATE'}))
    sys.exit(1)

p0 = gt()
n.cmd(cmd='load_waypoints', waypoints=wps, trail=trail)
spin(n, 1.5)
n.cmd(cmd='play')
spin(n, 0.5)

t0 = time.time()
fim = 'TIMEOUT'
while time.time() - t0 < a.timeout:
    spin(n, 0.3)
    if n.st and n.st.get('mode') == 'idle':
        fim = 'concluiu'
        break
spin(n, 1.5)
p1 = gt()
n.cmd(cmd='stop')
dur = time.time() - t0

# Alvo verdadeiro do ultimo waypoint COM cone (ver docstring).
alvo = None
erro = None
for w in reversed(wps):
    if not w.get('has_cone'):
        continue
    # qual cone do mundo e' esse? o mais proximo da posicao gravada + spawn
    est = (w['cone_x'] + spawn[0], w['cone_y'] + spawn[1])
    if not mundo:
        break
    real = min(mundo, key=lambda c: math.hypot(c[0] - est[0], c[1] - est[1]))
    dist_id = math.hypot(real[0] - est[0], real[1] - est[1])
    if dist_id > 1.5:      # nao consegui identificar com confianca
        break
    alvo = (real[0] + (w['x'] - w['cone_x']), real[1] + (w['y'] - w['cone_y']))
    if p1:
        erro = math.hypot(p1[0] - alvo[0], p1[1] - alvo[1])
    break

print(json.dumps({
    'fim': fim, 'dur': round(dur, 1),
    'x': round(p1[0], 3) if p1 else None,
    'y': round(p1[1], 3) if p1 else None,
    'yaw': round(math.degrees(p1[2]), 1) if p1 else None,
    'x0': round(p0[0], 3) if p0 else None,
    'y0': round(p0[1], 3) if p0 else None,
    'odom_x': round(n.odom[0], 3) if n.odom else None,
    'odom_y': round(n.odom[1], 3) if n.odom else None,
    'alvo_x': round(alvo[0], 3) if alvo else None,
    'alvo_y': round(alvo[1], 3) if alvo else None,
    'erro_final_cm': round(100 * erro, 1) if erro is not None else None,
}))
n.destroy_node()
rclpy.try_shutdown()
