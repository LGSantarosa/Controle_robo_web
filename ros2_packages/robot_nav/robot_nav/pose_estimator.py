#!/usr/bin/env python3
"""
Estimador de pose pro modo TREKKING.

Funde 3 fontes em (x, y, yaw) no frame `odom`:
  - MPU9250 (/imu/data)       → taxa de yaw (giro); yaw INTEGRADO (mag/absoluto: TODO)
  - PMW3901 (/optical_flow)   → velocidade no chão em (vx, vy) corpo
  - Encoders (4 RPMs)         → velocidade no corpo, fallback quando o flow é ruim

Saídas:
  /odom            nav_msgs/Odometry          (frame: odom→base_link) + TF
  /trekking/pose   geometry_msgs/PoseStamped  (frame: odom)
  /trekking/odom   nav_msgs/Odometry          (com twist no body frame)
  /trekking/slip   std_msgs/Float32           (módulo da divergência roda↔flow, m/s)

É o nó único de odometria agora: /odom + TF `odom→base_link` alimentam SLAM/
AMCL/Nav2. O trekking_runner e o cone_detector consomem /trekking/pose direto
— sem TF no caminho crítico.

Fusão:
  vx_body = α·vx_flow + (1-α)·vx_roda
  vy_body = α·vy_flow + (1-α)·0           (skid-steer cega à lateral)
  α       = sigmoid((quality - q_mid) / q_slope)   ∈ [0, 1]

Quando |vx_roda - vx_flow| > slip_threshold, /trekking/slip recebe a diferença
e o logger emite warn — útil pra UI marcar derrapagem.
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
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64, Float64MultiArray, String


from .utils import quat_to_yaw as _quat_to_yaw  # noqa: F401
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
        # Liga/desliga a CONTRIBUIÇÃO do flow na fusão de translação. O PMW3901
        # cospe lixo por EMI do motor ao dirigir (ver project_pmw3901_emi_motor);
        # com use_flow=False o α é forçado a 0 → translação = só roda (+ IMU no
        # yaw). Mantém o nó assinando /optical_flow (diagnóstico) sem deixá-lo
        # corromper a pose. Religar quando o HW do shifter for corrigido.
        self.declare_parameter('use_flow', True)
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
        self.pub_health = self.create_publisher(String, 'trekking/health', 10)

        # /odom + TF odom->base_link: o que SLAM/AMCL/Nav2 consomem. Este nó é o
        # ÚNICO dono desse TF agora (odom_publisher saiu dos launches).
        self.pub_odom_std = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'pose_estimator: m/contagem flow = {self.m_per_count*1000:.2f} mm '
            f'(h={self.flow_height:.3f} m), rate={rate:.0f} Hz'
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
            alpha *= flow_yaw_gate(self._imu_yaw_rate,
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
            )

            # Cache pra slip / twist
            vx_wheel = (self.v_fl + self.v_rl + self.v_fr + self.v_rr) / 4.0
            self.v_wheel_body = vx_wheel
            self.vx_body = res.vx_body
            self.vy_body = res.vy_body

            # Detecta slip (só log/publish)
            slip = vx_wheel - flow_vx if alpha > 0.1 else 0.0
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
        # yaw menos confiável no fallback de roda → AMCL/Nav confiam menos
        od_std.pose.covariance[35] = 0.10 if yaw_source == 'imu' else 0.5
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
                'quality':    int(quality_out),
                'yaw_source': yaw_source,
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
