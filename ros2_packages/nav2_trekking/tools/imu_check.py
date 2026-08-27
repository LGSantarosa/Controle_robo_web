#!/usr/bin/env python3
"""Monitor de bancada IMU/odom — valida sinais da MPU6050 antes de SLAM/nav2.

Throwaway. Rodar com ROS sourced:  python3 /tmp/imu_check.py

Lê:
  /odom      → x, y, yaw (acumulado, JÁ com imu_yaw_sign aplicado), vyaw
  /imu/data  → gz BRUTO do sensor (sem correção de sinal — pra ver o cru)

Como ler:
  - PARADO:        gz_bruto ~ 0 (±pequeno) e vyaw ~ 0  → bias do giro OK
  - FRENTE:        x sobe, yaw ~constante
  - GIRA P/ ESQ.:  yaw SOBE e vyaw POSITIVO  (se descer, troque imu_yaw_sign)
  - GIRA 90° real: yaw muda ~90°  (valida a MAGNITUDE — o bug do nav2/slam)
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def yaw_deg(z, w):
    return math.degrees(2.0 * math.atan2(z, w))


class Mon(Node):
    def __init__(self):
        super().__init__('imu_check')
        self.x = self.y = self.yaw = self.vyaw = 0.0
        self.gz_raw = 0.0
        self.acc = 0.0          # yaw acumulado (desenrolado), graus
        self._last_yaw = 0.0
        self.got_odom = self.got_imu = False
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # /imu/data é BEST_EFFORT (sensor_data) — casar QoS ou nada chega.
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.2, self.tick)   # 5 Hz, legível a olho

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        cur = yaw_deg(q.z, q.w)
        if self.got_odom:
            d = cur - self._last_yaw          # desenrola o salto de ±180
            if d > 180.0:
                d -= 360.0
            elif d < -180.0:
                d += 360.0
            self.acc += d
        self._last_yaw = cur
        self.yaw = cur
        self.vyaw = math.degrees(m.twist.twist.angular.z)
        self.got_odom = True

    def on_imu(self, m):
        self.gz_raw = math.degrees(m.angular_velocity.z)
        self.got_imu = True

    def tick(self):
        odom = 'ok ' if self.got_odom else 'SEM /odom'
        imu = 'ok ' if self.got_imu else 'SEM /imu/data'
        print(f"[odom:{odom} imu:{imu}] "
              f"x={self.x:+.3f}m y={self.y:+.3f}m | yaw={self.yaw:+7.1f}deg "
              f"acc={self.acc:+8.1f}deg | "
              f"vyaw(odom)={self.vyaw:+7.1f}deg/s | gz_bruto(/imu)={self.gz_raw:+7.1f}deg/s",
              flush=True)


def main():
    rclpy.init()
    n = Mon()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
