#!/usr/bin/env python3
"""Monitor de bancada da BNO055 (IMU #2) — valida sinal, mag e âncora de heading.

Ferramenta de bancada, irmã do imu_check.py. Rodar com ROS sourced, com a stack
do robô no ar:

    python3 ros2_packages/robot_nav/tools/imu2_check.py

Lê:
  /imu/data        → gz da IMU #1 (MPU), BRUTO (sem imu_yaw_sign)
  /imu2/data       → gz da IMU #2 (BNO055) BRUTO + yaw ABSOLUTO do quaternion
  /imu2/mag        → campo magnético (módulo em µT)
  /imu2/calib      → calibração do chip (sys/gyro/accel/mag, 0..3)
  /odom            → yaw fundido que saiu de tudo isso
  /trekking/health → o que o pose_estimator está fazendo com a BNO055

O QUE CONFERIR, na ordem:

1) SINAL (o mais importante — é o parâmetro imu2_yaw_sign):
   gire o robô PRA ESQUERDA. Os dois `gz` impressos aqui já vêm CORRIGIDOS
   (gz1*imu_yaw_sign e gz2*imu2_yaw_sign, os mesmos sinais que o pose_estimator
   aplica) e têm que ter o MESMO sinal. Saíram opostos → inverta `imu2_yaw_sign`.

   NÃO compare os valores crus dos tópicos: o `/imu/data` é o MPU sem correção
   de montagem, e neste robô ele está DE PONTA-CABEÇA (imu_yaw_sign=-1.0). Os
   crus saem opostos justamente quando as duas IMUs estão CERTAS — medido no
   robô em 2026-09-03, com pico gz1=+1.268 contra gz2=-1.328 rad/s no mesmo
   giro, e o parâmetro correto sendo o default +1.0.

   Enquanto discordarem de verdade, o pose_estimator IGNORA a BNO055 e loga erro
   (proposital: com o sinal trocado a média das duas daria zero e o robô giraria
   sem girar no mapa).

2) MAGNITUDE: gire 90° reais; `yaw_abs` tem que andar ~90° (e no mesmo sentido
   do `yaw_odom`). Se andar 45° ou 180°, a montagem não é plana.

3) CALIBRAÇÃO do mag: `mag` sobe pra 3 depois de mover o robô em ∞ (oito) no ar
   por uns 20 s. Enquanto estiver <2 o heading absoluto NÃO é usado — é o
   esperado, não é bug. `|B|` deve ficar perto de ~25-65 µT (campo da Terra);
   valor muito acima/instável = ferro perto do sensor ou EMI dos motores.

4) ÂNCORA: com `anchor=True`, ande em linha reta pra lá e pra cá por uns
   minutos e veja `corr` — é a deriva sendo comida (rad/tick, minúsculo por
   projeto). `mag` caindo pra <2 com os motores ligados = EMI: considere
   `use_imu2_heading:=false` nesse ambiente.
"""
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String


def yaw_deg(z, w):
    return math.degrees(2.0 * math.atan2(z, w))


class Mon(Node):
    def __init__(self):
        super().__init__('imu2_check')
        self.gz1 = self.gz2 = 0.0
        self.yaw_abs = 0.0
        self.yaw_odom = 0.0
        self.bmag = 0.0
        self.calib = {}
        self.health = {}
        self.got1 = self.got2 = False
        # Sinais de montagem, os MESMOS que o pose_estimator aplica. Sem eles a
        # comparação é entre um sensor corrigido e outro cru — que é como a
        # regra "mesmo sinal" passou a mentir neste robô (MPU de ponta-cabeça).
        self.declare_parameter('imu_yaw_sign', -1.0)
        self.declare_parameter('imu2_yaw_sign', 1.0)
        self.s1 = float(self.get_parameter('imu_yaw_sign').value)
        self.s2 = float(self.get_parameter('imu2_yaw_sign').value)

        # /imu/data e /imu2/data são BEST_EFFORT (sensor_data) — casar o QoS ou
        # nada chega (o clássico "robô sem IMU").
        self.create_subscription(Imu, '/imu/data', self.on_imu1, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu2/data', self.on_imu2, qos_profile_sensor_data)
        self.create_subscription(MagneticField, '/imu2/mag', self.on_mag, qos_profile_sensor_data)
        self.create_subscription(String, '/imu2/calib', self.on_calib, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/trekking/health', self.on_health, 10)
        self.create_timer(0.2, self.tick)   # 5 Hz, legível a olho

    def on_imu1(self, m):
        self.gz1 = math.degrees(m.angular_velocity.z)
        self.got1 = True

    def on_imu2(self, m):
        self.gz2 = math.degrees(m.angular_velocity.z)
        q = m.orientation
        self.yaw_abs = yaw_deg(q.z, q.w)
        self.got2 = True

    def on_mag(self, m):
        b = m.magnetic_field
        # tesla → µT
        self.bmag = math.sqrt(b.x**2 + b.y**2 + b.z**2) * 1e6

    def on_calib(self, m):
        try:
            self.calib = json.loads(m.data)
        except ValueError:
            pass

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.yaw_odom = yaw_deg(q.z, q.w)

    def on_health(self, m):
        try:
            self.health = json.loads(m.data)
        except ValueError:
            pass

    def tick(self):
        s1 = 'ok' if self.got1 else 'SEM /imu/data'
        s2 = 'ok' if self.got2 else 'SEM /imu2/data'
        # Sinais opostos com giro REAL nos dois = imu2_yaw_sign errado. Mesma
        # regra do blend_yaw_rate (0.15 rad/s ≈ 8.6°/s), pra bater com o que o
        # pose_estimator decide.
        c1 = self.gz1 * self.s1
        c2 = self.gz2 * self.s2
        flag = ''
        if abs(c1) > 8.6 and abs(c2) > 8.6 and (c1 > 0) != (c2 > 0):
            flag = '  <<< SINAIS OPOSTOS: inverta imu2_yaw_sign'
        cal = self.calib or {}
        print(
            f"[imu1:{s1} imu2:{s2}] "
            f"gz1={c1:+7.1f} gz2={c2:+7.1f} deg/s (sinais {self.s1:+.0f}/{self.s2:+.0f}) | "
            f"yaw_abs={self.yaw_abs:+7.1f} yaw_odom={self.yaw_odom:+7.1f} deg | "
            f"|B|={self.bmag:5.1f}uT calib(sys/g/a/mag)="
            f"{cal.get('sys','-')}/{cal.get('gyro','-')}/{cal.get('accel','-')}/{cal.get('mag','-')} | "
            f"anchor={self.health.get('heading_anchored','?')} "
            f"corr={self.health.get('heading_corr','?')} "
            f"src={self.health.get('yaw_source','?')}"
            f"{flag}",
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
