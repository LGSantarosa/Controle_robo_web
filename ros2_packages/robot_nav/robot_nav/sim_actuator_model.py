#!/usr/bin/env python3
"""sim_actuator_model — faz o GIRO do sim sofrer a mesma limitação do robô real.

Por que existe (2026-06-24, BO "deixar o sim igual ao real"): o plugin DiffDrive
do Gazebo é um motor IDEAL — entrega exatamente o cmd_vel.angular.z comandado.
O robô real NÃO: por patinagem do skid-steer + zona-morta dos hoverboards, o
`spin_calib.py` mediu (com as fitas nas rodas):

    giro_real ≈ 0.6 · (|cmd| − 1.7),  satura ~2.5 rad/s,  NÃO gira se |cmd| < 1.7
    (direita gira um pouco mais que a esquerda: ~30% a 2 rad/s, ~3% a 4–6 rad/s)

Sem modelar isso, o sim gira com qualquer comandinho e o "congela perto do goal"
(RotationShim comandando giro pequeno que o real não executa → nunca alinha)
JAMAIS reproduz no sim. Este nó fica ENTRE o twist_mux e o DiffDrive:

    twist_mux → /cmd_vel_raw → [sim_actuator_model] → /cmd_vel → bridge → DiffDrive

Aplica a curva no angular.z; linear.x passa direto (não há curva medida do linear
— a dinâmica linear fica por conta do max_linear_acceleration do DiffDrive). Tudo
parametrizado pra calibrar fino sem reflashar nada.

Convenção: angular.z > 0 = girar à ESQUERDA (CCW); < 0 = DIREITA.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimActuatorModel(Node):
    def __init__(self):
        super().__init__('sim_actuator_model')
        # Curva do giro real (spin_calib 2026-06-19, fitas nas rodas).
        self.deadzone = self.declare_parameter('theta_deadzone', 1.7).value
        self.gain = self.declare_parameter('theta_gain', 0.6).value
        self.sat = self.declare_parameter('theta_saturation', 2.5).value
        # Assimetria: direita (cmd<0) entrega um tico a mais. Default leve;
        # a curva real é dependente de velocidade (30%@2 rad/s, 3%@4–6) — aqui
        # fica um fator único aproximado, ajustável.
        self.right_factor = self.declare_parameter('right_factor', 1.05).value
        self.left_factor = self.declare_parameter('left_factor', 1.0).value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self._on_cmd, 10)
        self.get_logger().info(
            f'sim_actuator_model: giro deadzone={self.deadzone} gain={self.gain} '
            f'sat={self.sat} (R={self.right_factor} L={self.left_factor})')

    def _model_theta(self, w):
        aw = abs(w)
        if aw < self.deadzone:
            return 0.0
        out = self.gain * (aw - self.deadzone)
        if out > self.sat:
            out = self.sat
        out *= self.right_factor if w < 0.0 else self.left_factor
        return out if w > 0.0 else -out

    def _on_cmd(self, msg):
        out = Twist()
        out.linear.x = msg.linear.x       # passa direto (sem curva medida)
        out.linear.y = msg.linear.y
        out.angular.z = self._model_theta(msg.angular.z)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SimActuatorModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
