#!/usr/bin/env python3
"""
Odometria do robô 4 rodas (skid-steer, 2 placas de hoverboard).

Subscreve as 4 RPMs publicadas pelo `mega_bridge`:
  /hoverboard/front/left/velocity
  /hoverboard/front/right/velocity
  /hoverboard/rear/left/velocity
  /hoverboard/rear/right/velocity

Calcula a média de cada lado (mais robusto a derrapagem de uma roda só) e
aplica cinemática diff-drive:
    v        = (v_left + v_right) / 2
    omega    = (v_right - v_left) / wheel_base

Publica nav_msgs/Odometry e o TF odom -> base_link.

`left_wheel_sign` / `right_wheel_sign` são os mesmos sinais usados antes
(no driver antigo eles eram −1 / 1 para compensar a fiação invertida).
Mantemos como parâmetros pra calibração final no robô.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')

        self.declare_parameter('wheel_radius', 0.085)
        self.declare_parameter('wheel_base', 0.50)        # bitola: distância entre os centros das rodas L-R
        self.declare_parameter('rpm_to_rads', 2.0 * math.pi / 60.0)
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.rpm_to_rads = float(self.get_parameter('rpm_to_rads').value)
        self.left_sign = float(self.get_parameter('left_wheel_sign').value)
        self.right_sign = float(self.get_parameter('right_wheel_sign').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Estado da pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Velocidades das 4 rodas em m/s
        self.v_fl = 0.0
        self.v_fr = 0.0
        self.v_rl = 0.0
        self.v_rr = 0.0

        self.create_subscription(Float64, 'hoverboard/front/left/velocity',
                                 lambda m: self._set_wheel('fl', m), 10)
        self.create_subscription(Float64, 'hoverboard/front/right/velocity',
                                 lambda m: self._set_wheel('fr', m), 10)
        self.create_subscription(Float64, 'hoverboard/rear/left/velocity',
                                 lambda m: self._set_wheel('rl', m), 10)
        self.create_subscription(Float64, 'hoverboard/rear/right/velocity',
                                 lambda m: self._set_wheel('rr', m), 10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.05, self._publish_odom)

        self.get_logger().info(
            f'OdomPublisher (4 rodas) | wheel_radius={self.wheel_radius}m '
            f'| wheel_base={self.wheel_base}m'
        )

    def _rpm_to_ms(self, rpm: float) -> float:
        return rpm * self.rpm_to_rads * self.wheel_radius

    def _set_wheel(self, which: str, msg: Float64):
        # Aplica sinal por lado (calibração de polaridade) e converte RPM → m/s
        sign = self.left_sign if which in ('fl', 'rl') else self.right_sign
        v = self._rpm_to_ms(msg.data * sign)
        if which == 'fl':
            self.v_fl = v
        elif which == 'fr':
            self.v_fr = v
        elif which == 'rl':
            self.v_rl = v
        elif which == 'rr':
            self.v_rr = v

    def _publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        # Média das duas rodas de cada lado — reduz erro quando uma derrapa
        v_left = (self.v_fl + self.v_rl) / 2.0
        v_right = (self.v_fr + self.v_rr) / 2.0

        linear = (v_right + v_left) / 2.0
        angular = (v_right - v_left) / self.wheel_base

        self.x += linear * math.cos(self.theta) * dt
        self.y += linear * math.sin(self.theta) * dt
        self.theta += angular * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = q_z
        tf.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        # Covariâncias finitas: sem isso o AMCL/EKF trata a odom como
        # infinitamente confiável (zeros) ou ignora (NaN). Valores
        # razoáveis para skid-steer com média das 4 rodas.
        odom.pose.covariance[0]  = 0.05   # var(x)
        odom.pose.covariance[7]  = 0.05   # var(y)
        odom.pose.covariance[35] = 0.10   # var(yaw)
        odom.twist.covariance[0]  = 0.01  # var(vx)
        odom.twist.covariance[35] = 0.05  # var(vyaw)
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
