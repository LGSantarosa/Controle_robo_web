#!/usr/bin/env python3
"""Um trial: carrega 1 waypoint, da PLAY, espera acabar, imprime a pose REAL."""
import argparse, json, math, re, subprocess, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def gt():
    for _ in range(4):
        out = subprocess.run(['gz','model','-m','sim_robot','-p'],
                             capture_output=True, text=True, timeout=6).stdout
        m = re.search(r'- Pose[^\n]*\n\s*\[([-\d.e ]+)\]\s*\n\s*\[([-\d.e ]+)\]', out)
        if m:
            xyz=[float(v) for v in m.group(1).split()]
            rpy=[float(v) for v in m.group(2).split()]
            return xyz[0], xyz[1], rpy[2]
        time.sleep(0.3)
    return None

class T(Node):
    def __init__(self):
        super().__init__('ponto_unico')
        self.pub = self.create_publisher(String, 'trekking/cmd', 10)
        self.st = None; self.odom = None
        self.create_subscription(String, 'trekking/state', self._s, 10)
        self.create_subscription(PoseStamped, 'trekking/pose', self._p, 10)
    def _s(self, m):
        try: self.st = json.loads(m.data)
        except Exception: pass
    def _p(self, m):
        self.odom = (m.pose.position.x, m.pose.position.y)
    def cmd(self, **kw):
        self.pub.publish(String(data=json.dumps(kw)))

def spin(n, s):
    t = time.time()
    while time.time()-t < s: rclpy.spin_once(n, timeout_sec=0.05)

ap = argparse.ArgumentParser()
ap.add_argument('--standoff', type=float, default=1.2)
ap.add_argument('--cone-odom', default='4.0,0.0')
ap.add_argument('--sem-cone', action='store_true')
ap.add_argument('--timeout', type=float, default=60.0)
a = ap.parse_args()

cx, cy = [float(v) for v in a.cone_odom.split(',')]
# waypoint = standoff metros ANTES do cone, na linha de aproximacao (+x)
wx, wy = cx - a.standoff, cy
wp = {'x': wx, 'y': wy, 'yaw': 0.0, 'has_cone': not a.sem_cone,
      'cone_x': cx if not a.sem_cone else 0.0,
      'cone_y': cy if not a.sem_cone else 0.0,
      'cone_bearing': 0.0}

rclpy.init(); n = T(); spin(n, 2.5)
if n.st is None:
    print('ERRO: sem /trekking/state'); sys.exit(1)
p0 = gt()
n.cmd(cmd='load_waypoints', waypoints=[wp]); spin(n, 1.0)
n.cmd(cmd='play'); spin(n, 0.5)
t0 = time.time(); fim = None
while time.time()-t0 < a.timeout:
    spin(n, 0.3)
    if n.st and n.st.get('mode') == 'idle':
        fim = 'concluiu'; break
else:
    fim = 'TIMEOUT'
spin(n, 1.5)
p1 = gt()
n.cmd(cmd='stop')
dur = time.time()-t0
if p0 and p1:
    print(json.dumps({'fim': fim, 'dur': round(dur,1),
                      'x': round(p1[0],3), 'y': round(p1[1],3),
                      'yaw': round(math.degrees(p1[2]),1),
                      'odom_x': round(n.odom[0],3) if n.odom else None,
                      'odom_y': round(n.odom[1],3) if n.odom else None,
                      'wp_odom': [round(wx,3), round(wy,3)]}))
n.destroy_node(); rclpy.try_shutdown()
