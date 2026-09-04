#!/usr/bin/env python3
"""Teleop de teclado com dead-man — WASD -> /key_vel (prio 90 no twist_mux).

Por que existe (2026-08-06, o robô BATEU): o `teleop_twist_keyboard` padrão
republica o último comando PRA SEMPRE e não tem dead-man. Uma ré ficou presa e o
robô seguiu até bater. O `timeout: 0.5` do `key_vel` no twist_mux não protege
nesse caso — ele só corta quando a fonte PARA de publicar, e o nó padrão nunca
para. Terminal perdendo foco ou SSH engasgando = robô andando sozinho.

Além disso o nó padrão sobe com speed=0.5 m/s e turn=1.0 rad/s, enquanto este
chassi usa 0.30 e 6.0 (BASE_LINEAR_SPEED/BASE_ANGULAR_SPEED em
controle_web/controllers/robot_controller.py). Resultado: andava rápido demais e
quase não girava — as 4 rodas patinam e giro abaixo de ~6 rad/s não roda o robô
(mesma razão do scale_angular do teleop_ps4.yaml).

COMO O DEAD-MAN FUNCIONA, e a limitação honesta: terminal não entrega evento de
"tecla solta", só o auto-repeat enquanto ela fica pressionada. Então o dead-man é
por TIMEOUT: se não chega tecla há `hold_s`, o comando zera. O piso do `hold_s` é
o atraso do auto-repeat do terminal (tipicamente ~0,5 s antes da primeira
repetição) — abaixo disso o robô andaria aos soquinhos. Por isso o default é
0,6 s: é o menor valor que não engasga. Não é um dead-man de verdade como o L1 do
PS4 (`enable_button` + `require_enable_button`), é um limitador de deriva: em vez
de seguir infinito, o robô para ~0,6 s depois da última tecla. ESPAÇO zera na
hora — é o freio.

Ao expirar, publica zero por `zero_s` e então PARA de publicar, liberando o
twist_mux pras fontes de prioridade menor (autonomia) em vez de segurar a saída.
"""
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from robot_nav.utils import spin_node

# (linear, angular) em múltiplos de speed/turn. Aceita WASD e o padrão
# i/,/j/l do teleop_twist_keyboard, pra não brigar com o costume de quem já usa.
#
# q/z NÃO são movimento: no teleop_twist_keyboard eles ajustam velocidade, e
# quem tem o costume aperta q esperando acelerar. Na 1ª versão q era diagonal
# (frente+esquerda) e o robô saía andando em vez de acelerar. Diagonais ficam
# só em u/o, que são as do próprio teleop_twist_keyboard.
BINDINGS = {
    'w': (1.0, 0.0), 'i': (1.0, 0.0),
    's': (-1.0, 0.0), ',': (-1.0, 0.0),
    'a': (0.0, 1.0), 'j': (0.0, 1.0),
    'd': (0.0, -1.0), 'l': (0.0, -1.0),
    'u': (1.0, 1.0),      # frente + esquerda
    'o': (1.0, -1.0),     # frente + direita
}
FASTER_KEYS = ('q', '+', '=')   # '=' porque '+' exige shift
SLOWER_KEYS = ('z', '-', '_')
STOP_KEYS = (' ', 'k', '\x03')  # espaço, k, Ctrl+C — freio imediato

HELP = """
Teleop de teclado (dead-man por timeout)

   u  w  o        w/i frente    s/, ré
   a  s  d        a/j gira esq  d/l gira dir
                  u/o diagonais

   ESPAÇO ou k = PARA na hora
   q / z       = velocidade (x1.25 / x0.8)   [+ / - também]
   Ctrl+C      = sai (zera antes)

Segure a tecla pra andar. Soltou, o robô para em ~{hold:.1f}s.
speed={speed:.2f} m/s   turn={turn:.2f} rad/s (piso {floor:.2f} — abaixo disso
o chassi patina e NÃO gira, então baixar a velocidade só afeta o andar)
"""


class KeyTeleop(Node):
    def __init__(self):
        super().__init__('key_teleop')
        # Defaults alinhados ao robô real, não aos do teleop_twist_keyboard.
        self.declare_parameter('speed', 0.30)
        self.declare_parameter('turn', 6.0)
        # Piso do giro. O multiplicador de velocidade NÃO pode derrubar o
        # angular abaixo disso: o chassi de 4 rodas patina e comando angular
        # baixo simplesmente não roda o robô (mesma razão do scale_angular=6.0
        # do teleop_ps4.yaml). Na 1ª versão o mult escalava o angular junto, e
        # bastava apertar 'menos' umas vezes pra o robô parar de girar de vez
        # enquanto continuava andando — parecia defeito mecânico.
        self.declare_parameter('turn_floor', 6.0)
        self.declare_parameter('hold_s', 0.6)
        self.declare_parameter('zero_s', 0.5)
        self.declare_parameter('rate_hz', 20.0)

        self.speed = self.get_parameter('speed').value
        self.turn = self.get_parameter('turn').value
        self.turn_floor = self.get_parameter('turn_floor').value
        self.hold_s = self.get_parameter('hold_s').value
        self.zero_s = self.get_parameter('zero_s').value
        rate = self.get_parameter('rate_hz').value

        self.pub = self.create_publisher(Twist, 'key_vel', 10)
        self.mult = 1.0
        self.cmd = (0.0, 0.0)
        self.last_key_t = None   # None = nada pressionado desde o último zero
        self.zero_until = None

        # Publica a ~20 Hz: bem acima do timeout de 0.5 s do twist_mux, senão o
        # mux dropava a fonte no meio de uma tecla segurada.
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _publish(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)

    def stop_now(self):
        """Freio: zera o comando e agenda o pulso de zeros."""
        self.cmd = (0.0, 0.0)
        self.last_key_t = None
        self.zero_until = self.get_clock().now().nanoseconds * 1e-9 + self.zero_s
        self._publish(0.0, 0.0)

    def turn_scale(self):
        """Angular efetivo. Acelerar sobe o giro junto; desacelerar NÃO o
        derruba abaixo do piso — senão o robô só anda e não vira."""
        return max(self.turn * self.mult, self.turn_floor)

    def _log_speeds(self):
        self.get_logger().info(
            f'velocidade x{self.mult:.2f} — '
            f'{self.speed * self.mult:.2f} m/s, {self.turn_scale():.2f} rad/s')

    def on_key(self, key):
        if key in STOP_KEYS:
            self.stop_now()
            return key != '\x03'
        if key in FASTER_KEYS:
            self.mult = min(self.mult * 1.25, 3.0)
            self._log_speeds()
            return True
        if key in SLOWER_KEYS:
            self.mult = max(self.mult * 0.8, 0.1)
            self._log_speeds()
            return True
        binding = BINDINGS.get(key.lower())
        if binding is not None:
            self.cmd = binding
            self.last_key_t = self.get_clock().now().nanoseconds * 1e-9
            self.zero_until = None
        return True

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_key_t is not None:
            if now - self.last_key_t <= self.hold_s:
                lin, ang = self.cmd
                self._publish(lin * self.speed * self.mult,
                              ang * self.turn_scale())
                return
            # Dead-man estourou: soltou a tecla, ou o terminal/ssh engasgou.
            self.stop_now()
            return
        if self.zero_until is not None:
            if now <= self.zero_until:
                self._publish(0.0, 0.0)
            else:
                # Solta o mux: sem publisher ativo o twist_mux cai pra próxima
                # prioridade em 0.5 s, em vez de ficar preso num zero eterno.
                self.zero_until = None


def main(args=None):
    rclpy.init(args=args)
    node = KeyTeleop()
    print(HELP.format(hold=node.hold_s, speed=node.speed, turn=node.turn,
                      floor=node.turn_floor))

    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        node.get_logger().error(
            'stdin não é um terminal — rode em SSH/tmux interativo, '
            'não em background nem com pipe.')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)

        def read_keys():
            # Drena TUDO que o auto-repeat empilhou: só a última tecla importa,
            # senão uma rajada vira fila de comandos velhos.
            while select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if not key or not node.on_key(key):
                    raise KeyboardInterrupt

        node.create_timer(0.02, read_keys)
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Zera na saída: nunca deixar o robô com comando pendurado.
        try:
            node.stop_now()
            for _ in range(5):
                node._publish(0.0, 0.0)
        except Exception:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
