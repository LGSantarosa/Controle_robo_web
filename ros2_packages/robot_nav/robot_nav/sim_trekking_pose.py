#!/usr/bin/env python3
"""sim_trekking_pose — cola SÓ DO SIM: /odom (Gazebo) -> /trekking/pose.

POR QUE EXISTE: no robô real quem publica `/trekking/pose` é o `pose_estimator`
(fusão MPU6050 + RPM das 4 rodas), que sobe no `robot.launch.py`. O `--sim` não
usa o `robot.launch.py` — usa o `sim.launch.py` — e o Gazebo não tem nem
`/imu/data` nem `/hoverboard/wheel_velocities`. Resultado: `/trekking/pose`
ficava com ZERO publishers e o `trekking_runner` voltava em
`if not have_pose: return`. Ou seja, `--sim --trekking` nunca andou.

FIDELIDADE — ler antes de tirar conclusão: a `/odom` do Gazebo é
quase-ground-truth. Ela NÃO tem o drift de odometria que o snap-to-cone existe
pra corrigir. Então este relay serve pra validar a LEI DE MOVIMENTO (gira?
anda? converge no waypoint?), e NÃO a precisão do cone-âncora. Precisão só no
robô real, ou no sim depois de injetar drift de propósito.

Nunca sobe no real: o `launch.sh` só passa `sim_pose_from_odom:=true` em `--sim`.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .utils import spin_node


class SimTrekkingPose(Node):

    def __init__(self):
        super().__init__('sim_trekking_pose')
        self.pub = self.create_publisher(PoseStamped, 'trekking/pose', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.get_logger().warn(
            'SIM: publicando /trekking/pose a partir da /odom do Gazebo '
            '(quase-ground-truth, SEM drift — não vale pra aferir snap-to-cone)')

    def _on_odom(self, msg):
        p = PoseStamped()
        p.header = msg.header
        p.pose = msg.pose.pose
        self.pub.publish(p)


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
