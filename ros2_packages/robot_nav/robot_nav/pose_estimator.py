#!/usr/bin/env python3
"""
Estimador de pose pro modo TREKKING.

Funde 3 fontes em (x, y, yaw) no frame `odom`:
  - MPU6050 (/imu/data)       → taxa de yaw (giro); yaw INTEGRADO, sem absoluto
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

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from tf2_ros import TransformBroadcaster

from .fused_odom import FusedOdom, flow_alpha
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64, String


from .utils import quat_to_yaw as _quat_to_yaw  # noqa: F401
from .utils import wrap_pi
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
        self.declare_parameter('wheel_radius', 0.085)
        self.declare_parameter('wheel_base', 0.50)
        self.declare_parameter('rpm_to_rads', 2.0 * math.pi / 60.0)
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)

        # --- PMW3901 ---
        # FoV 42°, matriz 35×35 → rad/pix ≈ 0.021. m/contagem = h · tan(rad/pix).
        # Altura nominal do sensor ao chão (centro do robô): 12 cm.
        self.declare_parameter('flow_height', 0.12)
        self.declare_parameter('flow_fov_deg', 42.0)
        self.declare_parameter('flow_pixels', 35)
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
        # Liga/desliga a CONTRIBUIÇÃO do flow na fusão de translação. O PMW3901
        # cospe lixo por EMI do motor ao dirigir (ver project_pmw3901_emi_motor);
        # com use_flow=False o α é forçado a 0 → translação = só roda (+ IMU no
        # yaw). Mantém o nó assinando /optical_flow (diagnóstico) sem deixá-lo
        # corromper a pose. Religar quando o HW do shifter for corrigido.
        self.declare_parameter('use_flow', True)

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
        # Sinal da taxa de yaw da IMU (gyro Z). A MPU6050 está montada DE
        # PONTA-CABEÇA (Z aponta pra baixo) → giro vem invertido, default -1.0.
        # Se na bancada o robô girar pro lado errado no odom, troque pra +1.0
        # (não precisa reflashear a MEGA). Ver project_imu_mpu6050_mounting.
        self.declare_parameter('imu_yaw_sign', -1.0)

        self.wheel_radius   = float(self.get_parameter('wheel_radius').value)
        self.wheel_base     = float(self.get_parameter('wheel_base').value)
        self.rpm_to_rads    = float(self.get_parameter('rpm_to_rads').value)
        self.left_sign      = float(self.get_parameter('left_wheel_sign').value)
        self.right_sign     = float(self.get_parameter('right_wheel_sign').value)

        self.flow_height    = float(self.get_parameter('flow_height').value)
        fov_rad             = math.radians(float(self.get_parameter('flow_fov_deg').value))
        n_pix               = int(self.get_parameter('flow_pixels').value)
        rad_per_pix         = fov_rad / max(1, n_pix)
        self.m_per_count    = self.flow_height * math.tan(rad_per_pix)
        self.flow_x_sign    = float(self.get_parameter('flow_x_sign').value)
        self.flow_y_sign    = float(self.get_parameter('flow_y_sign').value)
        self.flow_swap_xy   = bool(self.get_parameter('flow_swap_xy').value)
        self.q_mid          = float(self.get_parameter('flow_quality_mid').value)
        self.q_slope        = float(self.get_parameter('flow_quality_slope').value)
        self.flow_timeout   = float(self.get_parameter('flow_timeout').value)
        self.use_flow       = bool(self.get_parameter('use_flow').value)

        self.slip_threshold = float(self.get_parameter('slip_threshold').value)
        self.pose_fix_gain  = float(self.get_parameter('pose_fix_gain').value)
        self.pose_fix_max   = float(self.get_parameter('pose_fix_max').value)
        rate                = float(self.get_parameter('publish_rate').value)
        self.odom_frame     = self.get_parameter('odom_frame').value
        self.base_frame     = self.get_parameter('base_frame').value
        self.imu_timeout    = float(self.get_parameter('imu_timeout').value)
        self.imu_yaw_sign   = float(self.get_parameter('imu_yaw_sign').value)

        # --- Estado ---
        self._lock = threading.Lock()
        # A pose (x, y, yaw) vive no núcleo puro FusedOdom.
        self._fused = FusedOdom(self.wheel_base)
        # Última leitura da IMU (None = nunca chegou). MPU6050 só dá taxa de
        # yaw (gyro Z); não há yaw absoluto.
        self._imu_yaw_rate = 0.0
        self._last_imu_wall = None    # rclpy.time.Time

        # Velocidades nas rodas (m/s, lado)
        self.v_fl = 0.0; self.v_fr = 0.0
        self.v_rl = 0.0; self.v_rr = 0.0

        # Velocidade body-frame do flow (m/s)
        self.flow_vx = 0.0
        self.flow_vy = 0.0
        self.flow_quality = 0.0
        self._last_flow_stamp = None  # rclpy.time.Time
        self._last_flow_wall = None   # tempo de chegada

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
        self.create_subscription(Float64, 'hoverboard/front/left/velocity',
                                 lambda m: self._set_wheel('fl', m), 10)
        self.create_subscription(Float64, 'hoverboard/front/right/velocity',
                                 lambda m: self._set_wheel('fr', m), 10)
        self.create_subscription(Float64, 'hoverboard/rear/left/velocity',
                                 lambda m: self._set_wheel('rl', m), 10)
        self.create_subscription(Float64, 'hoverboard/rear/right/velocity',
                                 lambda m: self._set_wheel('rr', m), 10)

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
            # MPU6050 (6 eixos): sem orientação absoluta (não tem magnetômetro).
            # Usamos só a taxa de yaw do giro (z); imu_yaw_sign corrige a
            # montagem de ponta-cabeça (Z pra baixo → sinal invertido). O yaw é
            # integrado no FusedOdom. Ver project_imu_mpu6050_mounting.
            self._imu_yaw_rate = msg.angular_velocity.z * self.imu_yaw_sign
            self._last_imu_wall = self.get_clock().now()

    def _on_flow(self, msg: Vector3Stamped):
        # dx, dy são contagens acumuladas desde a última mensagem.
        # Velocidade = dist / dt entre mensagens consecutivas.
        # O dt é medido por chegada (não pelo stamp do firmware), o que só é
        # correto se a cadência do flow for regular. O firmware garante isso
        # (AUDITORIA_2026-05-29 A2): amostra rejeitada por EMI é publicada NULA
        # (quality=0 → α≈0 aqui) em vez de suprimida, então não abre buraco no
        # dt e a amostra boa seguinte não fica subestimada.
        now = self.get_clock().now()
        dx = msg.vector.x
        dy = msg.vector.y
        quality = msg.vector.z

        with self._lock:
            if self._last_flow_wall is None:
                self._last_flow_wall = now
                self.flow_quality = quality
                return
            dt = (now - self._last_flow_wall).nanoseconds / 1e9
            self._last_flow_wall = now
            if dt <= 1e-4:
                return

            # Converte contagens → metros e aplica sinais/swap
            if self.flow_swap_xy:
                dx, dy = dy, dx
            d_body_x = dx * self.flow_x_sign * self.m_per_count
            d_body_y = dy * self.flow_y_sign * self.m_per_count

            self.flow_vx = d_body_x / dt
            self.flow_vy = d_body_y / dt
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

    def _set_wheel(self, which: str, msg: Float64):
        sign = self.left_sign if which in ('fl', 'rl') else self.right_sign
        v = msg.data * sign * self.rpm_to_rads * self.wheel_radius
        with self._lock:
            if   which == 'fl': self.v_fl = v
            elif which == 'fr': self.v_fr = v
            elif which == 'rl': self.v_rl = v
            elif which == 'rr': self.v_rr = v

    # ------------------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_pub_time).nanoseconds / 1e9
        self.last_pub_time = now
        if dt <= 0.0 or dt > 0.5:
            # Salto de tempo (drift do clock ou pausa). Não integra.
            return

        with self._lock:
            # Freshness da IMU
            if self._last_imu_wall is None:
                imu_age = float('inf')
            else:
                imu_age = (now - self._last_imu_wall).nanoseconds / 1e9
            imu_fresh = imu_age <= self.imu_timeout

            # Idade + peso do flow
            flow_age = float('inf')
            if self._last_flow_wall is not None:
                flow_age = (now - self._last_flow_wall).nanoseconds / 1e9
            alpha = flow_alpha(self.flow_quality, self.q_mid, self.q_slope,
                               flow_age, self.flow_timeout)
            # Flow desligado (EMI do PMW3901): zera o peso → translação só de roda.
            if not self.use_flow:
                alpha = 0.0
            flow_stale = flow_age > self.flow_timeout
            flow_vx = 0.0 if flow_stale else self.flow_vx
            flow_vy = 0.0 if flow_stale else self.flow_vy

            self._last_alpha = alpha
            self._last_flow_age = flow_age

            # Passo de fusão (núcleo puro)
            res = self._fused.step(
                dt,
                self.v_fl, self.v_fr, self.v_rl, self.v_rr,
                imu_fresh, self._imu_yaw_rate,
                flow_vx, flow_vy, alpha,
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

        # /trekking/pose (frame odom)
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.odom_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pub_pose.publish(ps)

        # /trekking/odom (twist no body frame)
        od = _build_odom(stamp, self.odom_frame, self.base_frame,
                         x, y, qz, qw, vx_out, vy_out, yaw_rate)
        self.pub_odom.publish(od)

        # /odom padrão (consumido por SLAM/AMCL/Nav2) + covariâncias
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

        self.pub_slip.publish(Float32(data=float(slip_out)))

        # /trekking/health
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
