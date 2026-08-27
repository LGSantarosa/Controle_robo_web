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

Aplica a curva no angular.z e uma ZONA-MORTA no linear.x. Tudo parametrizado
pra calibrar fino sem reflashar nada.

Zona-morta linear (2026-06-26, BO "sim não modela zona-morta linear"): o robô
real é pesado e NÃO anda com comando linear pequeno — medido indiretamente no
"congela perto do goal": o ramp do path_follower baixava p/ ~0.11 m/s e o robô
TRAVAVA (manda 0.11, não anda); o fix foi subir o min_speed p/ 0.22. O valor
exato da zona-morta nunca foi medido (só a do giro=1.7), então fica entre 0.11
(trava) e 0.25 (cruza), default 0.15 e parametrizável. Sem isso o sim anda com
qualquer comandinho linear e NÃO reproduz o congelamento no goal. DiffDrive é
ideal no linear também, por isso o modelo mora aqui:

    twist_mux → /cmd_vel_raw → [sim_actuator_model] → /cmd_vel → bridge → DiffDrive

Convenção: angular.z > 0 = girar à ESQUERDA (CCW); < 0 = DIREITA.
"""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist


def model_theta(w, deadzone, gain, sat, right_factor, left_factor):
    """Curva do giro real (spin_calib 2026-06-19, fitas nas rodas):
    giro ≈ gain·(|cmd|−deadzone), satura em sat, não gira se |cmd|<deadzone.
    Assimetria: direita (w<0) entrega um tico a mais (right_factor)."""
    aw = abs(w)
    if aw < deadzone:
        return 0.0
    out = gain * (aw - deadzone)
    if out > sat:
        out = sat
    out *= right_factor if w < 0.0 else left_factor
    return out if w > 0.0 else -out


def model_linear(v, deadzone):
    """Zona-morta linear: abaixo do limiar o robô pesado não anda (vira 0);
    acima passa direto (a 0.25 m/s o real cruza normal). Sem curva medida acima
    do limiar — a dinâmica fica por conta do max_linear_acceleration do DiffDrive."""
    if abs(v) < deadzone:
        return 0.0
    return v


def entrada_secou(idade_sim, timeout_sim, idade_wall, timeout_wall):
    """A entrada secou? Dois relogios, e QUALQUER um dos dois basta.

    O DiffDrive do Gazebo NAO tem timeout: ele mantem a ultima velocidade de
    roda indefinidamente. Quando todas as entradas do twist_mux expiram, o mux
    simplesmente PARA de publicar — e o robo continua andando/girando sozinho.
    Medido em 2026-08-24: /cmd_vel sem nenhuma mensagem e o robo a +58,8 graus/s,
    1,5 s depois de soltar o comando. Era o BO "ele gira do nada" do dono.
    O robo real para quando o comando cessa (watchdog do firmware), entao este
    zero explicito tambem e FIDELIDADE sim=real.

    POR QUE DOIS RELOGIOS (2026-08-25): o relogio de SIMULACAO e o
    semanticamente certo — com RTF 0.3 o publicador tambem fica 3x mais lento, e
    um prazo de parede dispararia sozinho no meio do movimento, inventando uma
    parada que ninguem pediu. Mas ele tem um ponto cego fatal: se o /clock
    EMPACA (o BO do orfao de parameter_bridge, duas fontes de /clock), a idade
    nunca cresce e o watchdog nunca dispara — justamente quando mais precisa.
    Dai o segundo relogio, de PAREDE, com prazo folgado: ele nao opina no caso
    normal, so pega o caso em que o tempo do sim parou de andar.

    `timeout <= 0` desliga aquele relogio.
    """
    if timeout_sim > 0.0 and idade_sim > timeout_sim:
        return True
    if timeout_wall > 0.0 and idade_wall > timeout_wall:
        return True
    return False


def deve_mandar_zero(secou, ja_enviados, rajada):
    """Publicar mais um zero agora?

    Era UM zero so, e a parada do robo inteira ficava pendurada nesse unico
    pacote: se ele se perdesse (bridge reconectando, congestionamento, o proprio
    DDS), o DiffDrive segurava a ultima velocidade PARA SEMPRE e nada no sistema
    tentava de novo — robo girando constante, travado, que foi o BO relatado
    pelo dono em 2026-08-25 ("girando devagarzinho e travado pra direita,
    constante, depois de ter parado").

    Agora e uma RAJADA curta. A razao original de nao repetir continua valendo e
    esta respeitada: nao martelar zero a 20 Hz no barramento pra sempre depois
    de parado — a rajada acaba.
    """
    return secou and ja_enviados < rajada


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
        # Zona-morta linear. 2026-07-06 (A/B do bolsão): 0.15 -> 0.10. O real ANDA
        # a 0.105 (aproximação mansa do slowdown 0.35*0.3, vista em todo run de
        # campo); com 0.15 o sim ZERAVA esse crawl e travava em vão de 0.9m que o
        # real cruza ("no sim ele é lento demais" — dono). O caso medido oposto
        # ("parte do repouso a 0.11 e trava", atrito estático) fica no limiar.
        self.lin_deadzone = self.declare_parameter('linear_deadzone', 0.10).value
        # Watchdog: o DiffDrive do Gazebo trava o ultimo comando. Ver
        # entrada_secou / deve_mandar_zero. 0 desliga cada relogio.
        self.input_timeout = self.declare_parameter('input_timeout', 0.3).value
        # Prazo de PAREDE, folgado: so pega /clock empacado (ver entrada_secou).
        self.input_timeout_wall = self.declare_parameter(
            'input_timeout_wall', 2.0).value
        # Rajada de zeros. 10 a 20 Hz = 0,5 s insistindo que e pra parar.
        self.stop_burst = self.declare_parameter('stop_burst', 10).value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self._on_cmd, 10)
        self._last_rx = self._agora()
        self._last_rx_wall = time.monotonic()
        # Comeca "ja parado": no boot ninguem mandou nada, nao ha o que zerar.
        self._zeros = self.stop_burst
        if self.input_timeout > 0.0 or self.input_timeout_wall > 0.0:
            # Relogio FIRME de proposito: o timer padrao segue o relogio do no,
            # que com use_sim_time e o /clock. Se o /clock empaca, o timer nem
            # dispara — e a checagem de parede aqui dentro nunca rodaria. O
            # vigia nao pode depender do relogio que ele esta vigiando.
            self.create_timer(0.05, self._watchdog,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'sim_actuator_model: giro deadzone={self.deadzone} gain={self.gain} '
            f'sat={self.sat} (R={self.right_factor} L={self.left_factor}); '
            f'linear deadzone={self.lin_deadzone}')

    def _agora(self):
        # relogio do NO: com use_sim_time isto segue o /clock do Gazebo
        return self.get_clock().now().nanoseconds * 1e-9

    def _watchdog(self):
        secou = entrada_secou(self._agora() - self._last_rx, self.input_timeout,
                              time.monotonic() - self._last_rx_wall,
                              self.input_timeout_wall)
        if not deve_mandar_zero(secou, self._zeros, self.stop_burst):
            return
        self.pub.publish(Twist())
        self._zeros += 1
        if self._zeros == 1:
            self.get_logger().info(
                'entrada secou -> rajada de %d zeros; sem isto o DiffDrive '
                'seguraria a ultima velocidade pra sempre' % self.stop_burst,
                throttle_duration_sec=10.0)

    def _on_cmd(self, msg):
        out = Twist()
        out.linear.x = model_linear(msg.linear.x, self.lin_deadzone)
        out.linear.y = msg.linear.y
        out.angular.z = model_theta(
            msg.angular.z, self.deadzone, self.gain, self.sat,
            self.right_factor, self.left_factor)
        self.pub.publish(out)
        self._last_rx = self._agora()
        self._last_rx_wall = time.monotonic()
        parado = (out.linear.x == 0.0 and out.angular.z == 0.0
                  and out.linear.y == 0.0)
        # Comando com movimento REARMA a rajada: e o que garante que a proxima
        # secagem seja insistida de novo, e nao uma vez so na vida do no.
        self._zeros = self.stop_burst if parado else 0


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
