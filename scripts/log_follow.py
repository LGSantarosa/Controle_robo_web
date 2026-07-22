#!/usr/bin/env python3
"""Logger do teste de SEGUIR PESSOA no sim — grava um CSV que o assistente lê.

Assina o que o person_follower produz e a odom, e escreve 1 linha por mudança
relevante (ou a cada 0.5s) em /tmp/follow_test.csv:
  t, state, vx, wz, tgt_dist, tgt_bearing_deg, n_targets

Uso: python3 scripts/log_follow.py   (Ctrl-C encerra)
"""
import csv
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import Float32MultiArray, String

OUT = '/tmp/follow_test.csv'


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Logger(Node):
    def __init__(self):
        super().__init__('log_follow')
        self.state = 'idle'
        self.vx = self.wz = 0.0
        self.pose = (0.0, 0.0, 0.0)
        self.targets = []
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'follow_person_state',
                                 self._on_state, latched)
        self.create_subscription(Twist, 'follow_person_vel', self._on_vel, 10)
        self.create_subscription(Float32MultiArray, 'follow_person_targets',
                                 self._on_tgt, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom,
                                 qos_profile_sensor_data)
        self.f = open(OUT, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'state', 'vx', 'wz', 'tgt_dist',
                         'tgt_bearing_deg', 'n_targets'])
        self.t0 = time.monotonic()
        self.create_timer(0.5, self._tick)
        self.get_logger().info('logando em ' + OUT)

    def _on_state(self, m):
        if m.data != self.state:
            self.state = m.data
            self._write()             # transição = linha na hora

    def _on_vel(self, m):
        self.vx, self.wz = m.linear.x, m.angular.z

    def _on_tgt(self, m):
        d = list(m.data)
        self.targets = [(d[i], d[i + 1]) for i in range(0, len(d) - 1, 2)]

    def _on_odom(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y, quat_yaw(m.pose.pose.orientation))

    def _nearest(self):
        rx, ry, ryaw = self.pose
        best = None
        for cx, cy in self.targets:
            d = math.hypot(cx - rx, cy - ry)
            if best is None or d < best[0]:
                b = math.degrees((math.atan2(cy - ry, cx - rx) - ryaw
                                  + math.pi) % (2 * math.pi) - math.pi)
                best = (d, b)
        return best if best else ('', '')

    def _write(self):
        d, b = self._nearest()
        self.w.writerow([round(time.monotonic() - self.t0, 2), self.state,
                         round(self.vx, 3), round(self.wz, 3),
                         round(d, 2) if d != '' else '',
                         round(b, 1) if b != '' else '', len(self.targets)])
        self.f.flush()

    def _tick(self):
        self._write()


def main():
    rclpy.init()
    n = Logger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.f.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
