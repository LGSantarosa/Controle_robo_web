#!/usr/bin/env python3
"""freeze_capture — coletor de diagnóstico do "robô burro / congela perto do goal".

Por que existe (2026-06-24): perto do ponto / diante de obstáculo o robô vai RETO
mesmo com o planner mandando contornar, gira só no lugar, e só para porque o
collision manda. Grava DOIS CSVs (controle_web/logs/) pra eu (assistente) ler
DEPOIS — nunca ao vivo:

1) freeze_capture.csv — a CADEIA de velocidade + odom, 1 linha por msg:
     t_wall, topic, vx, wz, px, py
     cmd_vel_nav : o que o controller (DWB/RotationShim) QUER (pré-smoother)
     nav_vel     : o que sobra DEPOIS do collision_monitor
     cmd_vel     : o que vai pro motor (pós twist_mux / modelo de giro no sim)
     odom        : o que o robô FAZ (twist) + pose (px,py)

2) freeze_diag.csv — métricas DERIVADAS a ~5 Hz pra provar planner-vs-controller:
     t_wall, px, py, yaw_deg, plan_rel_deg, front_obst_m, cmd_nav_vx, cmd_nav_wz
     yaw_deg      : heading do robô no frame map (via TF map→base_link)
     plan_rel_deg : ângulo do /plan a ~0.5 m à frente RELATIVO ao heading do robô
                    (+ = planner quer virar à esquerda, − à direita; ~0 = seguindo).
                    Grande + robô indo reto = "planner grita contornar, robô força reto".
     front_obst_m : obstáculo mais próximo num setor ±15° à frente (do /scan)
     cmd_nav_vx/wz: último comando do controller (o que ele tenta fazer)

Sobe sozinho no nav2.launch.py. Read-only: só assina + grava arquivo.
"""
import os
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class FreezeCapture(Node):
    def __init__(self):
        super().__init__('freeze_capture')
        out_dir = self.declare_parameter(
            'out_dir', 'controle_web/logs').get_parameter_value().string_value
        self.lookahead = self.declare_parameter('plan_lookahead', 0.5).value
        self.front_sector_deg = self.declare_parameter('front_sector_deg', 15.0).value
        p_chain, p_diag = self._open_csvs(out_dir)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        for topic in ('cmd_vel_nav', 'nav_vel', 'cmd_vel'):
            self.create_subscription(Twist, topic, self._mk_twist(topic), qos)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos)
        self.create_subscription(Path, 'plan', self._on_plan, qos)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # estado p/ o diag derivado
        self._plan = []           # [(x,y), ...] no frame map
        self._front = math.inf    # obstáculo à frente (m)
        self._cmd_nav = (0.0, 0.0)  # último cmd_vel_nav (vx, wz)

        self.create_timer(0.2, self._diag_tick)   # 5 Hz
        self.create_timer(2.0, self._flush)
        self.get_logger().info(f'freeze_capture: chain={p_chain} diag={p_diag}')

    def _open_csvs(self, out_dir):
        try:
            os.makedirs(out_dir, exist_ok=True)
            base = out_dir
        except OSError:
            base = '/tmp'
        self._f = open(os.path.join(base, 'freeze_capture.csv'), 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(['t_wall', 'topic', 'vx', 'wz', 'px', 'py'])
        self._fd = open(os.path.join(base, 'freeze_diag.csv'), 'w', newline='')
        self._wd = csv.writer(self._fd)
        self._wd.writerow(['t_wall', 'px', 'py', 'yaw_deg', 'plan_rel_deg',
                           'front_obst_m', 'cmd_nav_vx', 'cmd_nav_wz'])
        return (os.path.join(base, 'freeze_capture.csv'),
                os.path.join(base, 'freeze_diag.csv'))

    # ---- cadeia de velocidade (CSV 1) ----
    def _mk_twist(self, topic):
        def cb(m):
            if topic == 'cmd_vel_nav':
                self._cmd_nav = (m.linear.x, m.angular.z)
            self._w.writerow([f'{time.time():.3f}', topic,
                              f'{m.linear.x:.4f}', f'{m.angular.z:.4f}', '', ''])
        return cb

    def _on_odom(self, m):
        t = m.twist.twist
        p = m.pose.pose.position
        self._w.writerow([f'{time.time():.3f}', 'odom',
                          f'{t.linear.x:.4f}', f'{t.angular.z:.4f}',
                          f'{p.x:.3f}', f'{p.y:.3f}'])

    # ---- entradas p/ o diag (CSV 2) ----
    def _on_plan(self, m):
        self._plan = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]

    def _on_scan(self, m):
        lim = math.radians(self.front_sector_deg)
        best = math.inf
        a = m.angle_min
        for r in m.ranges:
            if -lim <= a <= lim and m.range_min < r < m.range_max and math.isfinite(r):
                if r < best:
                    best = r
            a += m.angle_increment
        self._front = best

    def _plan_rel(self, px, py, yaw):
        pts = self._plan
        if len(pts) < 2:
            return math.nan
        di = min(range(len(pts)), key=lambda i: (pts[i][0] - px) ** 2 + (pts[i][1] - py) ** 2)
        acc = 0.0
        j = di
        while j + 1 < len(pts) and acc < self.lookahead:
            acc += math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1])
            j += 1
        tx, ty = pts[j]
        if math.hypot(tx - px, ty - py) < 1e-3:
            return math.nan
        return _norm(math.atan2(ty - py, tx - px) - yaw)

    def _diag_tick(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        px = tf.transform.translation.x
        py = tf.transform.translation.y
        yaw = _yaw_from_quat(tf.transform.rotation)
        rel = self._plan_rel(px, py, yaw)
        front = '' if math.isinf(self._front) else f'{self._front:.3f}'
        rel_s = '' if (rel is None or math.isnan(rel)) else f'{math.degrees(rel):.1f}'
        self._wd.writerow([f'{time.time():.3f}', f'{px:.3f}', f'{py:.3f}',
                           f'{math.degrees(yaw):.1f}', rel_s, front,
                           f'{self._cmd_nav[0]:.4f}', f'{self._cmd_nav[1]:.4f}'])

    def _flush(self):
        self._f.flush()
        self._fd.flush()

    def destroy_node(self):
        for f in (getattr(self, '_f', None), getattr(self, '_fd', None)):
            try:
                if f:
                    f.flush()
                    f.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = FreezeCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
