#!/usr/bin/env python3
"""freeze_capture — coletor de diagnóstico do "robô congela perto do goal".

Por que existe (2026-06-24): perto do ponto o robô para, dá ré do unstuck,
volta e repete. O log mostrou que nas janelas em que o NAV está no controle a
POSE não muda (ele congela), não é o unstuck criando o problema — o unstuck só
empurra. Pergunta a responder: o nav COMANDA movimento e o robô não anda
(zona-morta/collision congela), ou o nav comanda ~0 (goal inalcançável)?

Grava a CADEIA inteira de velocidade + odom num CSV pra eu (assistente) ler
DEPOIS — nunca ao vivo. Cadeia (ver nav2_params_pi.yaml):
  controller -> cmd_vel_nav -> smoother -> nav_vel_raw -> [collision_monitor]
             -> nav_vel -> twist_mux(unstuck prio30) -> cmd_vel -> rodas

CSV (controle_web/logs/freeze_capture.csv), 1 linha por msg:
  t_wall, topic, vx, wz, px, py
    cmd_vel_nav : o que o controller (DWB/RotationShim) QUER
    nav_vel     : o que sobra DEPOIS do collision_monitor
    cmd_vel     : o que vai pro motor (pós twist_mux)
    odom        : o que o robô FAZ (twist) + pose (px,py) p/ achar a janela

Sobe sozinho no nav2.launch.py. Read-only (só assina + grava arquivo): não
publica nada, não interfere na navegação.
"""
import os
import csv
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class FreezeCapture(Node):
    def __init__(self):
        super().__init__('freeze_capture')
        # out_dir relativo resolve a partir do cwd do launch (raiz do repo);
        # absoluto também é aceito. Cria a pasta e cai pra /tmp se falhar.
        out_dir = self.declare_parameter(
            'out_dir', 'controle_web/logs').get_parameter_value().string_value
        path = self._open_csv(out_dir)

        # BEST_EFFORT recebe de publisher reliable E best_effort.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        for topic in ('cmd_vel_nav', 'nav_vel', 'cmd_vel'):
            self.create_subscription(Twist, topic, self._mk_twist(topic), qos)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos)
        self.create_timer(2.0, lambda: self._f.flush())
        self.get_logger().info(f'freeze_capture gravando em {path}')

    def _open_csv(self, out_dir):
        try:
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(out_dir, 'freeze_capture.csv')
            self._f = open(path, 'w', newline='')
        except OSError:
            path = '/tmp/freeze_capture.csv'
            self._f = open(path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(['t_wall', 'topic', 'vx', 'wz', 'px', 'py'])
        return path

    def _mk_twist(self, topic):
        def cb(m):
            self._w.writerow([f'{time.time():.3f}', topic,
                              f'{m.linear.x:.4f}', f'{m.angular.z:.4f}', '', ''])
        return cb

    def _on_odom(self, m):
        t = m.twist.twist
        p = m.pose.pose.position
        self._w.writerow([f'{time.time():.3f}', 'odom',
                          f'{t.linear.x:.4f}', f'{t.angular.z:.4f}',
                          f'{p.x:.3f}', f'{p.y:.3f}'])

    def destroy_node(self):
        try:
            self._f.flush()
            self._f.close()
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
