#!/usr/bin/env python3
"""sim_trekking_pose — cola SÓ DO SIM: /odom (Gazebo) -> /trekking/pose.

POR QUE EXISTE: no robô real quem publica `/trekking/pose` é o `pose_estimator`
(fusão MPU6050 + RPM das 4 rodas), que sobe no `robot.launch.py`. O `--sim` não
usa o `robot.launch.py` — usa o `sim.launch.py` — e o Gazebo não tem nem
`/imu/data` nem `/hoverboard/wheel_velocities`. Resultado: `/trekking/pose`
ficava com ZERO publishers e o `trekking_runner` voltava em
`if not have_pose: return`. Ou seja, `--sim --trekking` nunca andou.

FIDELIDADE — este aviso estava ERRADO e foi corrigido em 2026-08-25. Dizia
que a `/odom` do Gazebo era "quase-ground-truth, SEM drift". Não é: o plugin
DiffDrive integra a rotação das RODAS, e num skid-steer a roda patina no
point-turn. MEDIDO na pista, comparando com `gz model -p`: **48,4 cm de deriva**
numa rota de 7 m com 5 cantos (e +13,4 cm por curva à direita contra -3,1 à
esquerda, no banco mínimo). O drift existe, é físico, vem do modelo de atrito —
não precisa ser injetado.

Consequência prática: por meses este nó jogou fora a correção do cone-âncora,
que é a razão de existir do LiDAR no trekking. O runner publicava
`/trekking/pose_fix` e ninguém escutava, então no sim o mecanismo mais
importante da prova nunca foi medido nem a favor nem contra.

Nunca sobe no real: o `launch.sh` só passa `sim_pose_from_odom:=true` em `--sim`.
"""
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .cone_pose_fix import apply_pose_fix
from .utils import spin_node


class SimTrekkingPose(Node):

    def __init__(self):
        super().__init__('sim_trekking_pose')
        # Mesmos números do pose_estimator real (pose_estimator.py:136) — o
        # ponto é o sim exercitar a MESMA regra, não uma parecida.
        self.gain = self.declare_parameter('pose_fix_gain', 0.5).value
        self.max_mag = self.declare_parameter('pose_fix_max', 0.6).value
        # Desligável pro A/B: com e sem âncora, mesma rota, verdade-terreno do
        # Gazebo do lado. É a única forma de responder se a âncora AJUDA.
        self.enabled = self.declare_parameter('enable_pose_fix', True).value

        # Correção ACUMULADA. O real corrige a pose fundida, que ele mesmo
        # integra; aqui a pose vem pronta do Gazebo a cada mensagem, então a
        # correção tem que viver num offset, senão o próximo /odom a apagaria.
        self._off_x = 0.0
        self._off_y = 0.0
        self._odom = None
        self._lock = threading.Lock()

        self.pub = self.create_publisher(PoseStamped, 'trekking/pose', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(Vector3Stamped, 'trekking/pose_fix',
                                 self._on_pose_fix, 10)
        self.get_logger().info(
            f'SIM: /trekking/pose = /odom do Gazebo (que DERIVA — as rodas '
            f'patinam no pivô) + correção do cone-âncora '
            f'{"LIGADA" if self.enabled else "DESLIGADA"} '
            f'(ganho {self.gain}, rejeita acima de {self.max_mag} m)')

    def _on_odom(self, msg):
        with self._lock:
            self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            ox, oy = self._off_x, self._off_y
        p = PoseStamped()
        p.header = msg.header
        p.pose = msg.pose.pose
        p.pose.position.x += ox
        p.pose.position.y += oy
        self.pub.publish(p)

    def _on_pose_fix(self, msg):
        """Mesma regra do robô real: empurra x/y pela deriva medida no cone,
        com ganho parcial, e rejeita teleporte (associação suspeita). Yaw nunca
        é tocado — no real quem manda no yaw é a IMU."""
        if not self.enabled:
            return
        dx, dy = float(msg.vector.x), float(msg.vector.y)
        with self._lock:
            if self._odom is None:
                return
            cx = self._odom[0] + self._off_x
            cy = self._odom[1] + self._off_y
            nx, ny, ok = apply_pose_fix(cx, cy, dx, dy, self.gain, self.max_mag)
            if ok:
                self._off_x += nx - cx
                self._off_y += ny - cy
                off = (self._off_x, self._off_y)
        if ok:
            self.get_logger().info(
                f'pose_fix aplicado: Δ=({dx:+.2f}, {dy:+.2f}) m, ganho '
                f'{self.gain} -> correção acumulada ({off[0]:+.2f}, {off[1]:+.2f}) m')
        else:
            self.get_logger().warn(
                f'pose_fix REJEITADO: Δ=({dx:+.2f}, {dy:+.2f}) m acima de '
                f'{self.max_mag} m — associação suspeita')


def main(args=None):
    rclpy.init(args=args)
    node = SimTrekkingPose()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
