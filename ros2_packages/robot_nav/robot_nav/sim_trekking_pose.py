#!/usr/bin/env python3
"""sim_trekking_pose — pose do SIM: rodas + IMU -> /trekking/pose.

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

YAW PELA IMU (2026-08-26) — o buraco mais caro da simulação. Até esta data o
sim NÃO TINHA IMU: este nó repassava a pose da `/odom`, cujo yaw o DiffDrive
integra das RODAS. Num skid-steer a roda patina no pivô, e MEDIDO contra
verdade-terreno o erro é um PULSO no arranque de cada giro: a odom girou 7,8°
enquanto o robô girou 0,5°, e depois o giro rastreava perfeito. Com 4-5 giros
por rota isso vira ~10° de rumo errado e ~0,5 m de desvio lateral.

O robô REAL não tem esse erro: o `pose_estimator` usa a IMU (`yaw_source='imu'`)
e só cai pra roda quando a IMU morre. Ou seja, o sim rodava permanentemente no
MODO DEGRADADO do real, justamente no point-turn — a manobra em que o trekking
inteiro se apoia. Tudo que foi "provado" no sim sobre giro antes desta data
precisa ser relido com isso em mente.

Agora o nó espelha o real: **yaw integrado da IMU, x/y integrados com a
velocidade das RODAS naquele yaw**. Não basta trocar só o yaw publicado — o x/y
do DiffDrive foi integrado com o yaw de roda e carrega o mesmo erro embutido.

`use_imu_yaw:=false` volta exatamente ao comportamento antigo — é o A/B
roda-vs-IMU, e é a única forma de medir quanto isto valeu.

Nunca sobe no real: o `launch.sh` só passa `sim_pose_from_odom:=true` em `--sim`.
"""
import math
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .cone_pose_fix import apply_pose_fix
from .utils import quat_to_yaw as _quat_to_yaw
from .utils import spin_node, wrap_pi


def integra_pose(x, y, yaw, v, yaw_rate, dt):
    """Um passo de dead-reckoning: rodas dão a VELOCIDADE, a IMU dá o RUMO.

    Espelha o que o `pose_estimator` faz no robô real. `dt` fora de (0, 0.5]
    devolve o estado intocado: dt<=0 é carimbo repetido ou relógio pra trás
    (o BO do órfão de /clock, 2026-07-20), e dt grande é buraco de mensagens —
    integrar por cima de um buraco inventa deslocamento que ninguém andou.
    """
    if not (0.0 < dt <= 0.5):
        return x, y, yaw
    novo_yaw = wrap_pi(yaw + yaw_rate * dt)
    # meio-passo no yaw: o robô girou DURANTE o deslocamento, não antes dele
    meio = wrap_pi(yaw + 0.5 * yaw_rate * dt)
    return x + v * math.cos(meio) * dt, y + v * math.sin(meio) * dt, novo_yaw


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
        # A/B roda-vs-IMU. false = comportamento de ANTES de 2026-08-26 (yaw da
        # /odom, integrado das rodas). É o que mede o quanto a IMU valeu.
        self.use_imu_yaw = bool(
            self.declare_parameter('use_imu_yaw', True).value)
        # IMU considerada morta com este atraso; cai pra roda e avisa.
        self.imu_timeout = float(
            self.declare_parameter('imu_timeout', 0.5).value)

        # Correção ACUMULADA num offset. Em modo IMU a pose é integrada aqui e
        # daria pra corrigir direto; o offset é mantido porque no FALLBACK de
        # roda a pose é relida do /odom a cada mensagem e uma correção aplicada
        # no estado seria apagada no tick seguinte. Um caminho só pros dois.
        self._off_x = 0.0
        self._off_y = 0.0
        self._odom = None
        self._lock = threading.Lock()

        # Estado do dead-reckoning (só usado com use_imu_yaw)
        self._x = None          # None = ainda não semeado pelo 1º /odom
        self._y = 0.0
        self._yaw = 0.0
        self._t_odom = None     # carimbo do /odom anterior (tempo de SIM)
        self._imu_rate = 0.0
        self._imu_t = None
        self._avisou_sem_imu = False

        self.pub = self.create_publisher(PoseStamped, 'trekking/pose', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(Imu, 'imu', self._on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped, 'trekking/pose_fix',
                                 self._on_pose_fix, 10)
        fonte = ('yaw da IMU + velocidade das rodas (espelha o robô real)'
                 if self.use_imu_yaw
                 else 'yaw das RODAS — modo DEGRADADO, use_imu_yaw=false')
        self.get_logger().info(
            f'SIM: /trekking/pose = {fonte}; correção do cone-âncora '
            f'{"LIGADA" if self.enabled else "DESLIGADA"} '
            f'(ganho {self.gain}, rejeita acima de {self.max_mag} m)')

    @staticmethod
    def _stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def _on_imu(self, msg):
        with self._lock:
            self._imu_rate = float(msg.angular_velocity.z)
            self._imu_t = self._stamp(msg.header)

    def _on_odom(self, msg):
        t = self._stamp(msg.header)
        q = msg.pose.pose.orientation
        yaw_roda = _quat_to_yaw(q.x, q.y, q.z, q.w)

        with self._lock:
            if self._x is None:      # semeia no 1º /odom (pose do spawn)
                self._x = msg.pose.pose.position.x
                self._y = msg.pose.pose.position.y
                self._yaw = yaw_roda
            imu_viva = (self.use_imu_yaw and self._imu_t is not None
                        and (t - self._imu_t) <= self.imu_timeout)
            if imu_viva:
                dt = 0.0 if self._t_odom is None else (t - self._t_odom)
                self._x, self._y, self._yaw = integra_pose(
                    self._x, self._y, self._yaw,
                    float(msg.twist.twist.linear.x), self._imu_rate, dt)
            else:
                # Fallback: exatamente o comportamento de antes (yaw de roda).
                self._x = msg.pose.pose.position.x
                self._y = msg.pose.pose.position.y
                self._yaw = yaw_roda
            self._t_odom = t
            x, y, yaw = self._x, self._y, self._yaw
            self._odom = (x, y)
            ox, oy = self._off_x, self._off_y

        if self.use_imu_yaw and not imu_viva and not self._avisou_sem_imu:
            self._avisou_sem_imu = True
            self.get_logger().warn(
                'SEM /imu — caindo pro yaw das RODAS, que patina no pivô. '
                'O mundo tem o plugin gz-sim-imu-system e o sim_robot.sdf o '
                'sensor imu? (ver comentários dos dois arquivos)')

        p = PoseStamped()
        p.header = msg.header
        p.pose.position.x = x + ox
        p.pose.position.y = y + oy
        p.pose.orientation.z = math.sin(0.5 * yaw)
        p.pose.orientation.w = math.cos(0.5 * yaw)
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
