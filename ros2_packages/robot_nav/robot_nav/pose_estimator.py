#!/usr/bin/env python3
"""
Estimador de pose pro modo TREKKING.

Funde 4 fontes em (x, y, yaw) no frame `odom`:
  - MPU6050 (/imu/data)       → taxa de yaw (giro); yaw INTEGRADO (sem mag)
  - BNO055  (/imu2/data)      → 2ª taxa de yaw (entra na média com a #1) E o
                                heading ABSOLUTO (9 eixos, fusão no chip): âncora
                                magnética que tira a deriva do yaw integrado,
                                aplicada devagar e com teto. Gateada pela
                                calibração do mag (/imu2/calib) — mag cru mente
                                com confiança. Principal ganho no TREKKING, onde
                                o percurso é longo e não há LiDAR pra reancorar.
  - PMW3901 (/optical_flow)   → velocidade no chão em (vx, vy) corpo. DORMENTE:
                                o sensor foi removido do robô em 2026-07-01
                                (0 Hz é o normal); o caminho fica pronto pra um
                                breakout melhor no futuro.
  - Encoders (4 RPMs)         → velocidade no corpo; hoje é a fonte de translação

Saídas:
  /odom            nav_msgs/Odometry          (frame: odom→base_link) + TF
  /trekking/pose   geometry_msgs/PoseStamped  (frame: odom)
  /trekking/odom   nav_msgs/Odometry          (com twist no body frame)
  /trekking/slip   std_msgs/Float32           (divergência roda↔flow em m/s; NaN
                                              quando não há referência — ver
                                              slip_estimate no fused_odom)

É o nó único de odometria agora: /odom + TF `odom→base_link` alimentam SLAM/
AMCL/Nav2. O trekking_runner e o cone_detector consomem /trekking/pose direto
— sem TF no caminho crítico.

Fusão:
  vx_body = α·vx_flow + (1-α)·vx_roda
  vy_body = α·vy_flow + (1-α)·0           (skid-steer cega à lateral)
  α       = sigmoid((quality - q_mid) / q_slope)   ∈ [0, 1]

Quando |vx_roda - vx_flow| > slip_threshold, /trekking/slip recebe a diferença
e o logger emite warn — útil pra UI marcar derrapagem. SEM referência viva de
translação (hoje é o caso: o PMW3901 saiu do robô em 2026-07-01) o tópico
publica NaN, não zero — a detecção de derrapagem está SEM FONTE, e dizer isso
em voz alta é o ponto.
"""
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from tf2_ros import TransformBroadcaster

from .fused_odom import (
    FusedOdom, flow_alpha, flow_plausible, flow_tick_velocity, flow_yaw_gate,
    slip_estimate,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64, Float64MultiArray, String


from .utils import quat_to_yaw as _quat_to_yaw   # heading absoluto da BNO055
from .utils import spin_node, wrap_pi
from .cone_pose_fix import apply_pose_fix


def _build_odom(stamp, odom_frame, base_frame, x, y, qz, qw, vx, vy, yaw_rate):
    """Monta um nav_msgs/Odometry 2D (sem covariâncias — quem precisa seta depois).

    Fonte única pros dois publishers (/trekking/odom e /odom): evita atualizar um
    bloco e esquecer o outro.
    """
    od = Odometry()
    od.header.stamp = stamp
    od.header.frame_id = odom_frame
    od.child_frame_id = base_frame
    od.pose.pose.position.x = x
    od.pose.pose.position.y = y
    od.pose.pose.orientation.z = qz
    od.pose.pose.orientation.w = qw
    od.twist.twist.linear.x = vx
    od.twist.twist.linear.y = vy
    od.twist.twist.angular.z = yaw_rate
    return od


class PoseEstimator(Node):

    def __init__(self):
        super().__init__('pose_estimator')

        # --- Geometria das rodas (espelha odom_publisher) ---
        # wheel_radius CALIBRADO 2026-06-08: 3 cursos retos de 2,00 m davam
        # +3,7% longo com 0,085 (lia 2,04-2,12 m) → 0,082 centra em ~0%.
        self.declare_parameter('wheel_radius', 0.082)
        self.declare_parameter('wheel_base', 0.50)
        self.declare_parameter('rpm_to_rads', 2.0 * math.pi / 60.0)
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)

        # --- PMW3901 ---
        # Escala: m/contagem = flow_height · tan(rad_per_count). O rad_per_count é
        # CALIBRADO empiricamente — NÃO derivado de FoV/Npix. O modelo antigo
        # "1 count = 1 pixel" (h·tan(42°/35)) dava 2,51 mm/count e errava a escala
        # por ~12,8× porque o PMW3901 interpola subpixel (~445 counts no FoV, não
        # 35). Calibração 2026-06-08: 5 cursos medidos de 2,00 m no chão →
        # Σ|dy| = 10107/10248/9734/10084/9698 counts (média 9974), 0 lixo,
        # SQUAL~130 → m/count ≈ 0,200 mm @ h=0,12 m → rad_per_count ≈ 1,67e-3.
        # Espalhamento ~±5% é o ruído natural do óptico (entra na fusão como tal).
        # rad_per_count independe da altura (propriedade da óptica); m/count
        # escala linear com flow_height, então remontou mais alto → só ajustar h.
        self.declare_parameter('flow_height', 0.12)
        self.declare_parameter('flow_rad_per_count', 0.00167)
        # Eixos do PMW3901 vs body frame do robô. Default: x_sensor = forward,
        # y_sensor = lateral à esquerda. Ajustar via launch se montar girado.
        self.declare_parameter('flow_x_sign', 1.0)
        self.declare_parameter('flow_y_sign', 1.0)
        self.declare_parameter('flow_swap_xy', False)
        # Quality é 0..245 (PMW3901). q_mid no meio, q_slope controla a transição.
        self.declare_parameter('flow_quality_mid', 80.0)
        self.declare_parameter('flow_quality_slope', 20.0)
        # Watchdog do flow: se passar tempo demais sem mensagem, peso vai a zero
        self.declare_parameter('flow_timeout', 0.5)
        # Gate por taxa de giro (rad/s): em rotação rápida o PMW3901 (no centro)
        # vê o chão girando → dx/dy espúrio, + derrapagem real do spin → o α é
        # zerado. Passa inteiro abaixo de _lo, ignora acima de _hi. ω limpo da IMU.
        # 0.4 rad/s ≈ 23°/s (curva mansa, flow vale); 1.2 ≈ 69°/s (giro, corta).
        self.declare_parameter('flow_yaw_gate_lo', 0.4)
        self.declare_parameter('flow_yaw_gate_hi', 1.2)
        # Liga/desliga a CONTRIBUIÇÃO do flow na fusão de translação. Default
        # False desde 2026-08-26: o PMW3901 foi ARRANCADO do robô em 2026-07-01
        # (commit 33647e4) — não há o que fundir, e a odometria hoje é roda +
        # 2 IMUs (+ âncora de cone por LiDAR no trekking). O nó segue assinando
        # /optical_flow, então religar é só use_flow:=true quando o sensor
        # voltar; o histórico de por que ele saiu (EMI do motor, lixo com
        # quality alta) está em project_pmw3901_emi_motor e nos gates abaixo,
        # que continuam válidos.
        self.declare_parameter('use_flow', False)
        # Gate de plausibilidade: EMI do motor faz o PMW3901 cuspir velocidades
        # impossíveis (medido -10,6 m/s parado) com quality ALTA — o gate de
        # qualidade não pega. Acima de flow_v_max (m/s) a amostra é descartada
        # (α→0 no tick, cai pra roda+IMU). 0.8 ≈ 2,3× a v_max do chassi (0,35),
        # então nunca corta movimento real, só lixo. Ver project_pmw3901_emi_motor.
        self.declare_parameter('flow_v_max', 0.8)

        # --- Detecção de slip ---
        self.declare_parameter('slip_threshold', 0.15)  # m/s

        # --- Correção de pose por cone-âncora (trekking_runner publica pose_fix) ---
        self.declare_parameter('pose_fix_gain', 0.5)   # fração do delta aplicada
        self.declare_parameter('pose_fix_max', 0.6)    # m — acima disso, rejeita

        # --- Saída ---
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_timeout', 0.3)   # s — IMU a 50 Hz; >0.3 = ausente
        # s — rodas (/hoverboard/wheel_velocities) a 50 Hz; >0.3 = stream da MEGA
        # parou. CRÍTICO: sem isso, se a MEGA trava (I2C lockup) com o robô
        # girando, o diferencial de roda congelado integra um giro fantasma
        # infinito no mapa. Ver project_mega_i2c_hang + fused_odom (wheel_fresh).
        self.declare_parameter('wheel_timeout', 0.3)
        # Sinal da taxa de yaw da IMU (gyro Z). 2026-07-01: voltou o MPU6050
        # antigo, montado de PONTA-CABEÇA (Z pra baixo) → -1.0. (Era +1.0 com o
        # MPU6500 montado plano, devolvido por vir sem magnetômetro.) O launch
        # passa este valor; se na bancada girar pro lado errado no odom, troque o
        # sinal (não precisa reflashear a MEGA). Ver project_imu_mpu9250.
        self.declare_parameter('imu_yaw_sign', -1.0)

        # --- IMU #2: BNO055 (9 eixos, /imu2/data) ---
        # Entra como TERCEIRA fonte de giro e de direção, ao lado do MPU e das
        # rodas. Dois caminhos independentes, ligáveis/desligáveis à parte:
        #   (a) TAXA de yaw   → média ponderada com o MPU (imu2_rate_weight)
        #   (b) yaw ABSOLUTO  → âncora magnética que remove a deriva integrada
        # use_imu2=False corta os dois e a pose volta a ser EXATAMENTE a de antes.
        self.declare_parameter('use_imu2', True)
        # Sinal da taxa de yaw da BNO055 (gyro Z + heading), pra casar a
        # montagem — mesma ideia do imu_yaw_sign. Confira na bancada com
        # tools/imu2_check.py: girando pra ESQUERDA, gz das duas IMUs tem que
        # ter o MESMO sinal. Se saírem opostos, inverta ESTE parâmetro.
        self.declare_parameter('imu2_yaw_sign', 1.0)
        self.declare_parameter('imu2_timeout', 0.3)   # s — BNO055 a 50 Hz
        # Peso da BNO055 na taxa fundida quando as DUAS IMUs estão frescas.
        # 0.8, e NÃO 0.5 (média simples), porque as duas não são equivalentes:
        #  - o que faz o yaw derivar é BIAS, não ruído branco. O bias do MPU é
        #    estimado UMA VEZ no boot (calibrateGyro_) e depois passeia com a
        #    temperatura; a BNO055 recalibra o dela continuamente (é o campo
        #    'gyro' do calib). Média 50/50 IMPORTA metade do bias do MPU pro
        #    yaw — sujar o sinal bom pra depois a âncora magnética limpar é
        #    trabalho inventado.
        #  - pelas covariâncias que o mega_bridge declara (0.0025 do MPU contra
        #    0.0012 da BNO055), o peso ótimo por inverso da variância já daria
        #    ~0.68. 0.8 fica um degrau acima disso, coerente com a vantagem de
        #    bias que a variância sozinha não captura.
        # Não vai a 1.0 de propósito: o MPU seguir na conta é o que mantém o
        # cross-check vivo (gate de discordância) e a troca de fonte instantânea
        # se a BNO055 cair no meio de uma manobra.
        self.declare_parameter('imu2_rate_weight', 0.8)
        # Piso de calibração do GIRO da BNO055 (0..3) pra ela valer o peso
        # cheio. Abaixo disso o chip ainda está estimando o próprio bias (a
        # janela dos primeiros segundos depois do boot, tipicamente) — aí a
        # vantagem que justifica o 0.8 não existe ainda e o peso cai pra 0.5.
        self.declare_parameter('imu2_gyro_calib_min', 2)
        # --- âncora de heading absoluto ---
        self.declare_parameter('use_imu2_heading', True)
        # Ganho da correção (1/s). 0.2 → constante de tempo ~5 s: tira 1° de
        # deriva em ~5 s, invisível pro controlador e pro scan-matcher.
        self.declare_parameter('heading_gain', 0.2)
        # Teto da correção (rad/s). 0.15 rad/s ≈ 8,6°/s: mesmo um erro de 180°
        # (mag maluco por EMI) vira um giro lento e visível, nunca um salto.
        self.declare_parameter('heading_max_rate', 0.15)
        # Calibração MÍNIMA do magnetômetro (campo 'mag' de /imu2/calib, 0..3)
        # pra aceitar o heading. 2 = "razoável" pela Bosch; com 0/1 a BNO055
        # ainda entrega quaternion, só que apontando pra um norte inventado —
        # aceitar isso arrastaria o robô pro lado errado devagar, que é bem
        # pior de diagnosticar do que não ter correção nenhuma.
        self.declare_parameter('mag_calib_min', 2)

        self.wheel_radius   = float(self.get_parameter('wheel_radius').value)
        self.wheel_base     = float(self.get_parameter('wheel_base').value)
        self.rpm_to_rads    = float(self.get_parameter('rpm_to_rads').value)
        self.left_sign      = float(self.get_parameter('left_wheel_sign').value)
        self.right_sign     = float(self.get_parameter('right_wheel_sign').value)

        self.flow_height        = float(self.get_parameter('flow_height').value)
        self.flow_rad_per_count = float(self.get_parameter('flow_rad_per_count').value)
        self.m_per_count        = self.flow_height * math.tan(self.flow_rad_per_count)
        self.flow_x_sign    = float(self.get_parameter('flow_x_sign').value)
        self.flow_y_sign    = float(self.get_parameter('flow_y_sign').value)
        self.flow_swap_xy   = bool(self.get_parameter('flow_swap_xy').value)
        self.q_mid          = float(self.get_parameter('flow_quality_mid').value)
        self.q_slope        = float(self.get_parameter('flow_quality_slope').value)
        self.flow_timeout   = float(self.get_parameter('flow_timeout').value)
        self.flow_yaw_gate_lo = float(self.get_parameter('flow_yaw_gate_lo').value)
        self.flow_yaw_gate_hi = float(self.get_parameter('flow_yaw_gate_hi').value)
        self.use_flow       = bool(self.get_parameter('use_flow').value)
        self.flow_v_max     = float(self.get_parameter('flow_v_max').value)

        self.slip_threshold = float(self.get_parameter('slip_threshold').value)
        self.pose_fix_gain  = float(self.get_parameter('pose_fix_gain').value)
        self.pose_fix_max   = float(self.get_parameter('pose_fix_max').value)
        rate                = float(self.get_parameter('publish_rate').value)
        self.odom_frame     = self.get_parameter('odom_frame').value
        self.base_frame     = self.get_parameter('base_frame').value
        self.imu_timeout    = float(self.get_parameter('imu_timeout').value)
        self.wheel_timeout  = float(self.get_parameter('wheel_timeout').value)
        self.imu_yaw_sign   = float(self.get_parameter('imu_yaw_sign').value)
        self.use_imu2       = bool(self.get_parameter('use_imu2').value)
        self.imu2_yaw_sign  = float(self.get_parameter('imu2_yaw_sign').value)
        self.imu2_timeout   = float(self.get_parameter('imu2_timeout').value)
        self.imu2_rate_weight = float(self.get_parameter('imu2_rate_weight').value)
        self.imu2_gyro_calib_min = int(self.get_parameter('imu2_gyro_calib_min').value)
        self.use_imu2_heading = bool(self.get_parameter('use_imu2_heading').value)
        self.heading_gain   = float(self.get_parameter('heading_gain').value)
        self.heading_max_rate = float(self.get_parameter('heading_max_rate').value)
        self.mag_calib_min  = int(self.get_parameter('mag_calib_min').value)

        # --- Estado ---
        self._lock = threading.Lock()
        # A pose (x, y, yaw) vive no núcleo puro FusedOdom.
        self._fused = FusedOdom(self.wheel_base)
        # Última leitura da IMU (None = nunca chegou). MPU6050 só dá taxa de
        # yaw (gyro Z); não há yaw absoluto.
        self._imu_yaw_rate = 0.0
        # Freshness por time.monotonic() (float, imune a NTP): criar
        # rclpy.time.Time via rcl em todo callback custava ~200 objetos/s
        # (P3 da AUDITORIA_2026-06-11). Stamps PUBLICADOS seguem no clock ROS.
        self._last_imu_wall = None    # time.monotonic()

        # --- IMU #2 (BNO055) ---
        self._imu2_yaw_rate = 0.0     # rad/s, já com imu2_yaw_sign
        self._imu2_abs_yaw = None     # rad — heading do quaternion, com o sinal
        self._last_imu2_wall = None   # time.monotonic()
        self._imu2_mag_calib = 0      # 0..3; 0 até /imu2/calib chegar (seguro)
        self._imu2_gyro_calib = 0     # 0..3; idem — segura o peso em 0.5 até saber
        # Alinhamento entre o "norte" da BNO055 e a origem de yaw do frame odom
        # (que é arbitrária — é onde o robô estava quando o nó subiu). Latcheado
        # na PRIMEIRA amostra aceita e depois só muda em yaw_fix. Sem este
        # offset, a primeira correção giraria o robô inteiro pro norte magnético
        # — teleporte de heading no mapa, exatamente o que não queremos.
        self._mag_yaw_offset = None
        self._imu2_was_stale = False
        self._disagree_logged = False

        # Velocidades nas rodas (m/s, lado)
        self.v_fl = 0.0; self.v_fr = 0.0
        self.v_rl = 0.0; self.v_rr = 0.0
        self._last_wheel_wall = None  # time.monotonic() — última /hoverboard/wheel_velocities
        self._wheels_was_stale = False

        # Deslocamento body-frame do flow ACUMULADO desde o último tick (m). O
        # tick converte em velocidade (accum/dt_tick) — ver flow_tick_velocity:
        # NÃO se calcula velocidade pelo intervalo de chegada (rajada inflava a
        # pose ~2×). Drenado a cada tick.
        self._flow_dx_accum = 0.0
        self._flow_dy_accum = 0.0
        self.flow_quality = 0.0
        self._last_flow_stamp = None  # rclpy.time.Time
        self._last_flow_wall = None   # time.monotonic() de chegada

        # Última fusão (pra publicar twist)
        self.vx_body = 0.0
        self.vy_body = 0.0
        self.v_wheel_body = 0.0       # cache pra detecção de slip

        self.last_pub_time = self.get_clock().now()
        # Diagnóstico do flow: combinado com C5 (PMW3901 sem SQUAL → quality=0
        # sempre → alpha ≈ 0), o nó silenciosamente ignora o flow. Marcadores
        # aqui permitem warns throttled e publish do /trekking/health.
        self._alpha_low_since = None     # rclpy.time.Time — primeiro tick com α<0.05
        self._flow_was_stale = False     # estado anterior do flow_age > timeout
        self._last_alpha = 0.0
        self._last_flow_age = float('inf')

        # --- Subscribers ---
        # IMU e flow são publicados pelo mega_bridge como BEST_EFFORT
        # (qos_profile_sensor_data). Assinar com QoS default (RELIABLE) é
        # INCOMPATÍVEL → nenhuma mensagem chega. Casar o profile sensor_data.
        self.create_subscription(Imu, 'imu/data', self._on_imu, qos_profile_sensor_data)
        # IMU #2 (BNO055) — mesmo QoS BEST_EFFORT do resto do stream de sensor.
        # A calibração vai num tópico à parte porque muda raramente (RELIABLE,
        # ~0 tráfego) e porque precisa ser explícita: é ela que autoriza a
        # correção de heading.
        self.create_subscription(Imu, 'imu2/data', self._on_imu2, qos_profile_sensor_data)
        self.create_subscription(String, 'imu2/calib', self._on_imu2_calib, 10)
        self.create_subscription(Vector3Stamped, 'optical_flow', self._on_flow, qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped, 'trekking/pose_fix', self._on_pose_fix, 10)
        # Correção manual de DIREÇÃO (yaw). data = delta em rad a aplicar no
        # ponteiro. Usado pela web no SLAM (robô sem IMU): gira o yaw integrado
        # da roda e deixa o scan-matcher do slam re-convergir — sem tocar o mapa.
        self.create_subscription(Float64, 'trekking/yaw_fix', self._on_yaw_fix, 10)
        # 4 rodas num tópico só (Float64MultiArray, ordem [FL,FR,RL,RR], RPM já
        # normalizado pro referencial do robô pelo mega_bridge). Era 4 subs Float64
        # separadas = 4 wakeups/ciclo do executor; 1 sub = 1 wakeup, mesmo dado.
        # sensor_data: TEM que casar com o pub do mega_bridge (P4).
        self.create_subscription(Float64MultiArray, 'hoverboard/wheel_velocities',
                                 self._on_wheels, qos_profile_sensor_data)

        # --- Publishers ---
        self.pub_pose = self.create_publisher(PoseStamped, 'trekking/pose', 10)
        self.pub_odom = self.create_publisher(Odometry, 'trekking/odom', 10)
        self.pub_slip = self.create_publisher(Float32, 'trekking/slip', 10)
        # A detecção de derrapagem compara roda contra FLOW. Sem PMW3901 no
        # robô ela não tem com o que comparar — avisa UMA vez no boot, porque
        # descobrir isso no meio de um teste de campo custa caro.
        if not self.use_flow:
            self.get_logger().warn(
                'detecção de derrapagem SEM FONTE: use_flow=false, então /trekking/slip '
                'publica NaN e nada vai avisar se as rodas patinarem. A translação é '
                '100% roda. Meça com trena, ou traga o LiDAR pra conferir.'
            )
        self.pub_health = self.create_publisher(String, 'trekking/health', 10)

        # /odom + TF odom->base_link: o que SLAM/AMCL/Nav2 consomem. Este nó é o
        # ÚNICO dono desse TF agora (odom_publisher saiu dos launches).
        self.pub_odom_std = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1.0 / rate, self._tick)

        if not self.use_imu2:
            imu2_desc = 'BNO055 DESLIGADA (use_imu2:=false)'
        else:
            imu2_desc = (f'BNO055 peso_giro={self.imu2_rate_weight:.2f} '
                         f'(0.50 até calib gyro>={self.imu2_gyro_calib_min}) '
                         f'sinal={self.imu2_yaw_sign:+.0f} heading='
                         + (f'ganho {self.heading_gain:.2f}/s, teto '
                            f'{self.heading_max_rate:.2f} rad/s, mag>={self.mag_calib_min}'
                            if self.use_imu2_heading else 'OFF'))
        self.get_logger().info(
            f'pose_estimator: m/contagem flow = {self.m_per_count*1000:.2f} mm '
            f'(h={self.flow_height:.3f} m), rate={rate:.0f} Hz | {imu2_desc}'
        )

    # ------------------------------------------------------------------
    def _on_imu(self, msg: Imu):
        with self._lock:
            # MPU6050 (6 eixos): SEM magnetômetro → só a taxa de yaw do giro (z),
            # yaw integrado. O imu_yaw_sign casa a montagem (de ponta-cabeça,
            # Z pra baixo → -1.0). Yaw absoluto (mag) não existe neste chip.
            # Ver project_imu_mpu9250.
            self._imu_yaw_rate = msg.angular_velocity.z * self.imu_yaw_sign
            self._last_imu_wall = time.monotonic()

    def _on_imu2(self, msg: Imu):
        # BNO055 (9 eixos): taxa de yaw COMO a #1, mais o heading ABSOLUTO que
        # vem do quaternion fundido no chip. O imu2_yaw_sign casa a montagem —
        # aplicado nos dois (giro e heading), porque uma montagem espelhada
        # inverte o sentido de rotação nas duas leituras.
        yaw_abs = _quat_to_yaw(msg.orientation.x, msg.orientation.y,
                               msg.orientation.z, msg.orientation.w)
        with self._lock:
            self._imu2_yaw_rate = msg.angular_velocity.z * self.imu2_yaw_sign
            self._imu2_abs_yaw = wrap_pi(yaw_abs * self.imu2_yaw_sign)
            self._last_imu2_wall = time.monotonic()

    def _on_imu2_calib(self, msg: String):
        # {"sys":0..3,"gyro":0..3,"accel":0..3,"mag":0..3} — publicado pelo
        # mega_bridge só quando muda. Usamos dois campos: 'mag' decide se o
        # heading absoluto vale, 'gyro' decide se a BNO055 merece o peso cheio
        # na taxa. ('accel' só afetaria roll/pitch, que este robô plano ignora.)
        try:
            cal = json.loads(msg.data)
            mag = int(cal.get('mag', 0))
            gyro = int(cal.get('gyro', 0))
        except (ValueError, TypeError):
            return
        with self._lock:
            prev = self._imu2_mag_calib
            self._imu2_mag_calib = mag
            # 'gyro' modula o PESO da taxa (não o heading): enquanto o chip não
            # tiver o próprio bias estimado, ela não é melhor que o MPU.
            self._imu2_gyro_calib = gyro
        if (prev >= self.mag_calib_min) != (mag >= self.mag_calib_min):
            if mag >= self.mag_calib_min:
                self.get_logger().info(
                    f'BNO055: mag calibrado ({mag}/3) — heading absoluto ATIVO')
            else:
                self.get_logger().warn(
                    f'BNO055: mag caiu pra {mag}/3 (< {self.mag_calib_min}) — '
                    f'heading absoluto SUSPENSO, yaw volta a ser só integrado')

    def _on_flow(self, msg: Vector3Stamped):
        # dx, dy são contagens acumuladas desde a última mensagem. Convertemos em
        # DESLOCAMENTO (metros) e ACUMULAMOS — o tick fecha em velocidade dividindo
        # pelo dt do tick. NÃO calculamos velocidade pelo intervalo de chegada:
        # o PMW3901 chega em rajada e d/dt_chegada segurado/re-integrado dobrava a
        # pose (ver flow_tick_velocity). Amostra EMI vem NULA (quality=0 → α≈0;
        # AUDITORIA_2026-05-29 A2), então só soma ~0 no acumulador, sem furo.
        dx = msg.vector.x
        dy = msg.vector.y
        quality = msg.vector.z

        with self._lock:
            self._last_flow_wall = time.monotonic()  # p/ freshness/timeout no tick
            # Converte contagens → metros e aplica sinais/swap
            if self.flow_swap_xy:
                dx, dy = dy, dx
            self._flow_dx_accum += dx * self.flow_x_sign * self.m_per_count
            self._flow_dy_accum += dy * self.flow_y_sign * self.m_per_count
            self.flow_quality = quality

    def _on_pose_fix(self, msg: Vector3Stamped):
        # Empurra x/y pela deriva medida no cone-âncora. Rejeita teleportes
        # (associação suspeita) e aplica suave. Yaw nunca é tocado (só IMU).
        dx = float(msg.vector.x)
        dy = float(msg.vector.y)
        with self._lock:
            nx, ny, ok = apply_pose_fix(
                self._fused.x, self._fused.y, dx, dy,
                self.pose_fix_gain, self.pose_fix_max,
            )
            if ok:
                self._fused.x = nx
                self._fused.y = ny
        if ok:
            self.get_logger().info(
                f'pose_fix aplicado: Δ=({dx:+.2f}, {dy:+.2f}) m '
                f'(ganho {self.pose_fix_gain:.2f})'
            )
        else:
            self.get_logger().warn(
                f'pose_fix REJEITADO: |Δ|={math.hypot(dx, dy):.2f} m '
                f'> {self.pose_fix_max:.2f} m — associação de cone suspeita'
            )

    def _on_yaw_fix(self, msg: Float64):
        # Gira o ponteiro de direção por `delta` rad. O yaw é sempre integrado
        # (FusedOdom) — tanto da roda quanto do giro da MPU6050 (taxa, não
        # absoluto) — então setá-lo aqui GRUDA: os passos seguintes integram a
        # partir do novo valor, com ou sem IMU. (Com o BNO055 antigo, o yaw
        # absoluto sobrescrevia isto a cada tick; não é mais o caso.)
        delta = float(msg.data)
        with self._lock:
            self._fused.yaw = wrap_pi(self._fused.yaw + delta)
            new_yaw = self._fused.yaw
            # O offset do heading absoluto anda JUNTO. Sem isto, a âncora
            # magnética passaria os próximos segundos desfazendo a correção que
            # o operador acabou de fazer na web — o robô "voltaria sozinho" pro
            # ponteiro errado, e a UI pareceria quebrada.
            if self._mag_yaw_offset is not None:
                self._mag_yaw_offset = wrap_pi(self._mag_yaw_offset + delta)
        self.get_logger().info(
            f'yaw_fix: ponteiro girado {delta:+.3f} rad → yaw(odom)={new_yaw:+.3f}'
        )

    def _on_wheels(self, msg: Float64MultiArray):
        # data = [FL, FR, RL, RR] em RPM normalizado (ordem fixada pelo mega_bridge).
        # Aplica sinal por lado (polaridade) + RPM→m/s, idêntico ao _set_wheel antigo.
        if len(msg.data) != 4:
            return
        fl, fr, rl, rr = msg.data
        k = self.rpm_to_rads * self.wheel_radius
        with self._lock:
            self.v_fl = fl * self.left_sign  * k
            self.v_fr = fr * self.right_sign * k
            self.v_rl = rl * self.left_sign  * k
            self.v_rr = rr * self.right_sign * k
            self._last_wheel_wall = time.monotonic()

    # ------------------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_pub_time).nanoseconds / 1e9
        self.last_pub_time = now
        if dt <= 0.0 or dt > 0.5:
            # Salto de tempo (drift do clock ou pausa). Não integra — e DRENA o
            # acumulador do flow: o deslocamento da janela perdida re-integrado
            # no próximo tick (dt≈0,02s) viraria velocidade ~25× a real
            # (B2 da AUDITORIA_2026-06-11).
            with self._lock:
                self._flow_dx_accum = 0.0
                self._flow_dy_accum = 0.0
            return

        mono = time.monotonic()
        with self._lock:
            # Freshness da IMU
            if self._last_imu_wall is None:
                imu_age = float('inf')
            else:
                imu_age = mono - self._last_imu_wall
            imu_fresh = imu_age <= self.imu_timeout

            # Freshness da IMU #2 (BNO055) — mesma lógica; use_imu2=False a
            # trata como permanentemente ausente (fusão idêntica à de antes).
            if self._last_imu2_wall is None:
                imu2_age = float('inf')
            else:
                imu2_age = mono - self._last_imu2_wall
            imu2_fresh = self.use_imu2 and imu2_age <= self.imu2_timeout

            # Peso da BNO055 na taxa: cheio só com o giro dela calibrado. Na
            # janela de boot (chip ainda estimando o próprio bias) ela não é
            # superior ao MPU, então cai pra média simples em vez de mandar.
            rate_weight = self.imu2_rate_weight
            if self._imu2_gyro_calib < self.imu2_gyro_calib_min:
                rate_weight = min(rate_weight, 0.5)

            # Heading absoluto: só entra com IMU #2 fresca, correção ligada e
            # magnetômetro calibrado o bastante. Qualquer um faltando → None, e
            # o yaw fica sendo o integrado puro (comportamento de sempre).
            abs_yaw = None
            mag_ok = self._imu2_mag_calib >= self.mag_calib_min
            if (imu2_fresh and self.use_imu2_heading and mag_ok
                    and self._imu2_abs_yaw is not None):
                if self._mag_yaw_offset is None:
                    # Latch: alinha o norte da BNO055 com o yaw que a odometria
                    # já tem AGORA. A correção nasce, portanto, valendo zero —
                    # daí em diante ela só combate a DERIVA, sem nunca girar o
                    # robô pro norte magnético de verdade.
                    self._mag_yaw_offset = wrap_pi(
                        self._fused.yaw - self._imu2_abs_yaw)
                    self.get_logger().info(
                        f'BNO055: heading absoluto ancorado '
                        f'(offset={self._mag_yaw_offset:+.3f} rad, mag={self._imu2_mag_calib}/3)')
                abs_yaw = wrap_pi(self._imu2_abs_yaw + self._mag_yaw_offset)

            # Freshness das rodas: se a MEGA parou de mandar frames, v_fl..v_rr
            # estão CONGELADAS. wheel_fresh=False faz o FusedOdom zerar a
            # contribuição das rodas (anti-giro-fantasma) — ver project_mega_i2c_hang.
            if self._last_wheel_wall is None:
                wheel_age = float('inf')
            else:
                wheel_age = mono - self._last_wheel_wall
            wheel_fresh = wheel_age <= self.wheel_timeout

            # Idade + peso do flow
            flow_age = float('inf')
            if self._last_flow_wall is not None:
                flow_age = mono - self._last_flow_wall
            alpha = flow_alpha(self.flow_quality, self.q_mid, self.q_slope,
                               flow_age, self.flow_timeout)
            # Flow desligado (EMI do PMW3901): zera o peso → translação só de roda.
            if not self.use_flow:
                alpha = 0.0
            # Gate por giro: em rotação rápida o flow vê o chão girando (dx/dy
            # espúrio) + derrapagem do spin → corta o peso com o ω limpo da IMU.
            # ω pro gate: a IMU #1 se estiver fresca, senão a #2. Antes das duas
            # IMUs isto era sempre _imu_yaw_rate — que, com o MPU mudo, ficava
            # CONGELADO no último valor e o gate julgava o flow por um giro que
            # já tinha acabado.
            gate_rate = self._imu_yaw_rate if imu_fresh else (
                self._imu2_yaw_rate if imu2_fresh else 0.0)
            alpha *= flow_yaw_gate(gate_rate,
                                   self.flow_yaw_gate_lo, self.flow_yaw_gate_hi)
            # Deslocamento acumulado desde o último tick → velocidade pela janela
            # do TICK (não pelo intervalo de chegada). Drena o acumulador.
            flow_vx_tick, flow_vy_tick = flow_tick_velocity(
                self._flow_dx_accum, self._flow_dy_accum, dt)
            self._flow_dx_accum = 0.0
            self._flow_dy_accum = 0.0
            # Gate de plausibilidade: pico de EMI (velocidade impossível com
            # quality alta) → descarta o flow neste tick (só roda+IMU), pra não
            # teleportar a pose e perder a localização na manobra.
            if not flow_plausible(flow_vx_tick, flow_vy_tick, self.flow_v_max):
                alpha = 0.0
                self.get_logger().warn(
                    f'flow IMPLAUSÍVEL (vx={flow_vx_tick:+.1f}, vy={flow_vy_tick:+.1f} '
                    f'm/s > {self.flow_v_max:.1f}) — EMI, descartado neste tick',
                    throttle_duration_sec=2.0,
                )
            flow_stale = flow_age > self.flow_timeout
            flow_vx = 0.0 if flow_stale else flow_vx_tick
            flow_vy = 0.0 if flow_stale else flow_vy_tick

            self._last_alpha = alpha
            self._last_flow_age = flow_age

            # Passo de fusão (núcleo puro)
            res = self._fused.step(
                dt,
                self.v_fl, self.v_fr, self.v_rl, self.v_rr,
                imu_fresh, self._imu_yaw_rate,
                flow_vx, flow_vy, alpha,
                wheel_fresh=wheel_fresh,
                imu2_fresh=imu2_fresh,
                imu2_yaw_rate=self._imu2_yaw_rate,
                imu2_rate_weight=rate_weight,
                abs_yaw=abs_yaw,
                heading_gain=self.heading_gain,
                heading_max_rate=self.heading_max_rate,
            )

            # Cache pra slip / twist
            vx_wheel = (self.v_fl + self.v_rl + self.v_fr + self.v_rr) / 4.0
            self.v_wheel_body = vx_wheel
            self.vx_body = res.vx_body
            self.vy_body = res.vy_body

            # Detecta slip (só log/publish). NaN quando não há referência de
            # translação viva — ver slip_estimate(): zero seria mentira.
            slip = slip_estimate(vx_wheel, flow_vx, alpha)
            if alpha > 0.3 and abs(slip) > self.slip_threshold:
                self.get_logger().warn(
                    f'slip detectado: roda={vx_wheel:+.2f} m/s vs flow={flow_vx:+.2f} m/s '
                    f'(α={alpha:.2f}, q={self.flow_quality:.0f})',
                    throttle_duration_sec=1.0,
                )

            x = res.x
            y = res.y
            yaw = res.yaw
            yaw_rate = res.yaw_rate
            yaw_source = res.yaw_source
            vx_out = res.vx_body
            vy_out = res.vy_body
            slip_out = slip
            quality_out = self.flow_quality
            heading_corr = res.heading_corr
            rate_disagree = res.rate_disagree
            mag_calib_out = self._imu2_mag_calib
            rate_weight_out = rate_weight

        # ----- diagnóstico do flow -----
        if flow_stale and not self._flow_was_stale:
            self.get_logger().warn(
                f'flow stale (age={flow_age:.2f} s > {self.flow_timeout:.2f} s) — '
                f'pose_estimator usando só rodas',
                throttle_duration_sec=60.0,
            )
        elif not flow_stale and self._flow_was_stale:
            self.get_logger().info('flow voltou')
        self._flow_was_stale = flow_stale

        # ----- diagnóstico da IMU #2 (BNO055) -----
        # Só reclama se ela chegou a funcionar: robô montado sem BNO055 não
        # merece warn nenhum (o sensor é opcional por projeto).
        imu2_stale = self.use_imu2 and imu2_age > self.imu2_timeout
        if self._last_imu2_wall is not None:
            if imu2_stale and not self._imu2_was_stale:
                self.get_logger().warn(
                    f'BNO055 stale (age={imu2_age:.2f} s > {self.imu2_timeout:.2f} s) — '
                    f'yaw volta pro MPU sozinho (e sem âncora magnética)',
                    throttle_duration_sec=30.0,
                )
            elif not imu2_stale and self._imu2_was_stale:
                self.get_logger().info('BNO055 voltou')
        self._imu2_was_stale = imu2_stale

        # Discordância de SINAL entre as duas IMUs: quase sempre imu2_yaw_sign
        # errado (BNO055 montada girada). O núcleo já ignorou a #2 neste tick;
        # aqui a gente grita, porque o sintoma no campo (yaw meio certo) é sutil
        # demais pra alguém desconfiar do parâmetro sozinho.
        if rate_disagree:
            self.get_logger().error(
                'IMUs DISCORDAM no sinal do giro — BNO055 ignorada. '
                'Provável imu2_yaw_sign invertido: rode tools/imu2_check.py e, '
                'girando pra esquerda, confira se os dois gz têm o mesmo sinal.',
                throttle_duration_sec=10.0,
            )
            self._disagree_logged = True
        elif self._disagree_logged and abs(yaw_rate) > 0.15:
            # "Sem discordância" também é o que se vê com o robô PARADO (o gate
            # ignora |ω| pequeno), então só declaramos reconciliação com giro
            # real acontecendo — senão cada trecho em linha reta logaria isto.
            self.get_logger().info('IMUs voltaram a concordar no sinal do giro')
            self._disagree_logged = False

        # ----- diagnóstico do stream das rodas (MEGA viva?) -----
        # Rodas stale = a MEGA parou de mandar frames (provável I2C lockup do
        # firmware — ver project_mega_i2c_hang). A pose CONGELA (não integra
        # lixo); este WARN denuncia a causa raiz no campo em vez de deixar o
        # robô "girar no mapa" sem explicação.
        wheels_stale = not wheel_fresh
        if wheels_stale and not self._wheels_was_stale:
            self.get_logger().error(
                f'RODAS stale (age={wheel_age:.2f} s > {self.wheel_timeout:.2f} s) — '
                f'stream da MEGA parou! Pose CONGELADA (anti-giro-fantasma). '
                f'Cheque a MEGA (LED ON aceso + TX apagado = firmware travado).',
                throttle_duration_sec=5.0,
            )
        elif not wheels_stale and self._wheels_was_stale:
            self.get_logger().info('rodas voltaram — stream da MEGA restabelecido')
        self._wheels_was_stale = wheels_stale

        if alpha < 0.05:
            if self._alpha_low_since is None:
                self._alpha_low_since = now
            else:
                low_dt = (now - self._alpha_low_since).nanoseconds / 1e9
                if low_dt > 2.0:
                    self.get_logger().warn(
                        f'alpha={alpha:.3f} (quality={quality_out:.0f}) há {low_dt:.1f} s — '
                        f'flow contribuindo ~0 na fusão',
                        throttle_duration_sec=60.0,
                    )
        else:
            self._alpha_low_since = None

        # ----- publica -----
        stamp = now.to_msg()
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # /odom padrão (consumido por SLAM/AMCL/Nav2/nav_metrics) + TF: SEMPRE
        # têm consumidor → publica incondicional.
        od_std = _build_odom(stamp, self.odom_frame, self.base_frame,
                             x, y, qz, qw, vx_out, vy_out, yaw_rate)
        od_std.pose.covariance[0] = 0.05    # var(x)
        od_std.pose.covariance[7] = 0.05    # var(y)
        # Confiança no yaw, do melhor pro pior caso (AMCL/Nav2 pesam com isto):
        #   0.05 — giro de IMU + âncora magnética: a deriva não cresce com o tempo
        #   0.10 — giro de IMU integrado: bom, mas deriva devagar
        #   0.50 — fallback de roda: derrapagem do skid-steer, o pior heading
        if yaw_source == 'wheel':
            od_std.pose.covariance[35] = 0.5
        elif abs_yaw is not None:
            od_std.pose.covariance[35] = 0.05
        else:
            od_std.pose.covariance[35] = 0.10
        od_std.twist.covariance[0] = 0.01   # var(vx)
        od_std.twist.covariance[7] = 0.05   # var(vy) — flow publica vy não-nulo
        od_std.twist.covariance[35] = 0.05  # var(vyaw)
        self.pub_odom_std.publish(od_std)

        # TF odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        # /trekking/* só interessam ao modo trekking (cone_detector/trekking_runner)
        # e a ferramentas manuais (flow_check). No modo nav2 NINGUÉM assina → o
        # get_subscription_count() == 0 pula a construção/json.dumps/serialização
        # inteiras. Quando alguém assina, volta a publicar sozinho. Zero diferença
        # de comportamento, só não trabalha pra plateia vazia.
        if self.pub_pose.get_subscription_count() > 0:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.odom_frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            self.pub_pose.publish(ps)

        if self.pub_odom.get_subscription_count() > 0:
            od = _build_odom(stamp, self.odom_frame, self.base_frame,
                             x, y, qz, qw, vx_out, vy_out, yaw_rate)
            self.pub_odom.publish(od)

        if self.pub_slip.get_subscription_count() > 0:
            self.pub_slip.publish(Float32(data=float(slip_out)))

        if self.pub_health.get_subscription_count() > 0:
            health = {
                'flow_stale': bool(flow_stale),
                'flow_age':   round(flow_age, 3) if flow_age != float('inf') else None,
                'alpha':      round(alpha, 3),
                # de onde sai a referência do slip neste tick — None = detecção
                # de derrapagem SEM FONTE (o /trekking/slip está publicando NaN)
                'slip_source': 'flow' if alpha > 0.1 else None,
                'quality':    int(quality_out),
                'yaw_source': yaw_source,
                # IMU #2 (BNO055): dá pra ver da web/CLI se ela está entrando na
                # fusão e se a âncora magnética está de fato corrigindo algo.
                'imu2_stale':   bool(imu2_stale),
                'imu2_age':     round(imu2_age, 3) if imu2_age != float('inf') else None,
                'mag_calib':    int(mag_calib_out),
                # peso EFETIVO da BNO055 na taxa neste tick (cai pra 0.5 na
                # janela de boot, quando o giro dela ainda não se calibrou)
                'imu2_weight':  round(rate_weight_out, 2),
                'heading_anchored': abs_yaw is not None,
                # rad aplicados NESTE tick (µrad na prática); o sinal diz pra que
                # lado a deriva estava indo.
                'heading_corr': round(heading_corr, 6),
            }
            self.pub_health.publish(String(data=json.dumps(health, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
