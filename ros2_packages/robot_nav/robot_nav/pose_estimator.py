#!/usr/bin/env python3
"""
Estimador de pose pro modo TREKKING.

Funde 3 fontes em (x, y, yaw) no frame `odom`:
  - BNO055 (/imu/data)        → yaw absoluto (quaternion fundido)
  - PMW3901 (/optical_flow)   → velocidade no chão em (vx, vy) corpo
  - Encoders (4 RPMs)         → velocidade no corpo, fallback quando o flow é ruim

Saídas:
  /trekking/pose   geometry_msgs/PoseStamped  (frame: odom)
  /trekking/odom   nav_msgs/Odometry          (com twist no body frame)
  /trekking/slip   std_msgs/Float32           (módulo da divergência roda↔flow, m/s)

NÃO publica TF (`odom→base_link` continua sendo do odom_publisher). O
trekking_runner e o cone_detector consomem /trekking/pose direto — sem TF
no caminho crítico.

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
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64, String


from .utils import quat_to_yaw as _quat_to_yaw  # noqa: F401


def _sigmoid(x: float) -> float:
    if x >= 0:
        z = math.exp(-x)
        return 1.0 / (1.0 + z)
    z = math.exp(x)
    return z / (1.0 + z)


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

        # --- Detecção de slip ---
        self.declare_parameter('slip_threshold', 0.15)  # m/s

        # --- Saída ---
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

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

        self.slip_threshold = float(self.get_parameter('slip_threshold').value)
        rate                = float(self.get_parameter('publish_rate').value)
        self.odom_frame     = self.get_parameter('odom_frame').value
        self.base_frame     = self.get_parameter('base_frame').value

        # --- Estado ---
        self._lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0          # do BNO055 (rad/s)
        self.have_yaw = False

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
        self.create_subscription(Imu, 'imu/data', self._on_imu, 20)
        self.create_subscription(Vector3Stamped, 'optical_flow', self._on_flow, 20)
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

        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'pose_estimator: m/contagem flow = {self.m_per_count*1000:.2f} mm '
            f'(h={self.flow_height:.3f} m), rate={rate:.0f} Hz'
        )

    # ------------------------------------------------------------------
    def _on_imu(self, msg: Imu):
        with self._lock:
            self.yaw = _quat_to_yaw(
                msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w,
            )
            self.yaw_rate = msg.angular_velocity.z
            self.have_yaw = True

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
            if not self.have_yaw:
                return

            # Velocidade das rodas no body frame
            v_left  = (self.v_fl + self.v_rl) / 2.0
            v_right = (self.v_fr + self.v_rr) / 2.0
            vx_wheel = (v_left + v_right) / 2.0
            self.v_wheel_body = vx_wheel

            # Idade do flow — peso vai a 0 se ficou velho demais
            flow_age = float('inf')
            if self._last_flow_wall is not None:
                flow_age = (now - self._last_flow_wall).nanoseconds / 1e9

            if flow_age > self.flow_timeout:
                alpha = 0.0
                flow_vx = 0.0
                flow_vy = 0.0
                flow_stale = True
            else:
                alpha = _sigmoid((self.flow_quality - self.q_mid) / max(self.q_slope, 1e-3))
                flow_vx = self.flow_vx
                flow_vy = self.flow_vy
                flow_stale = False

            self._last_alpha = alpha
            self._last_flow_age = flow_age

            vx_body = alpha * flow_vx + (1.0 - alpha) * vx_wheel
            vy_body = alpha * flow_vy                # roda contribui 0 em vy

            # Detecta slip (só log/publish, não corrige aqui)
            slip = vx_wheel - flow_vx if alpha > 0.1 else 0.0
            if alpha > 0.3 and abs(slip) > self.slip_threshold:
                self.get_logger().warn(
                    f'slip detectado: roda={vx_wheel:+.2f} m/s vs flow={flow_vx:+.2f} m/s '
                    f'(α={alpha:.2f}, q={self.flow_quality:.0f})',
                    throttle_duration_sec=1.0,
                )

            # Integra no mundo
            cy = math.cos(self.yaw); sy = math.sin(self.yaw)
            self.x += (vx_body * cy - vy_body * sy) * dt
            self.y += (vx_body * sy + vy_body * cy) * dt

            self.vx_body = vx_body
            self.vy_body = vy_body

            yaw = self.yaw
            yaw_rate = self.yaw_rate
            x = self.x; y = self.y
            slip_out = slip
            quality_out = self.flow_quality

        # ----- diagnóstico do flow -----
        # Edge "ficou stale": loga uma vez. Edge "voltou": loga info.
        if flow_stale and not self._flow_was_stale:
            self.get_logger().warn(
                f'flow stale (age={flow_age:.2f} s > {self.flow_timeout:.2f} s) — '
                f'pose_estimator usando só rodas',
                throttle_duration_sec=60.0,
            )
        elif not flow_stale and self._flow_was_stale:
            self.get_logger().info('flow voltou')
        self._flow_was_stale = flow_stale

        # alpha persistentemente baixo (>2 s seguidos) → flow ativo mas inútil.
        # Sinaliza diferente de "stale" pra UI poder mostrar mensagem específica.
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

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.odom_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pub_pose.publish(ps)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.vx_body
        od.twist.twist.linear.y = self.vy_body
        od.twist.twist.angular.z = yaw_rate
        self.pub_odom.publish(od)

        self.pub_slip.publish(Float32(data=float(slip_out)))

        # /trekking/health: estado da fusão (UI exibe ícones).
        health = {
            'flow_stale': bool(flow_stale),
            'flow_age':   round(flow_age, 3) if flow_age != float('inf') else None,
            'alpha':      round(alpha, 3),
            'quality':    int(quality_out),
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
