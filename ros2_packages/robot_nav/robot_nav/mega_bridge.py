#!/usr/bin/env python3
"""
Ponte ROS2 ↔ Arduino MEGA 2560.

Substitui o `ros2-hoverboard-driver` antigo (que falava direto com uma única
placa). Agora a MEGA agrega:
  - 2 placas de hoverboard (frontal Serial1, traseira Serial2)
  - BNO055 IMU (I²C)
  - PMW3901 optical flow (SPI)
  - anel WS2812, relé da luz, LED de marco, botão de partida

Protocolo (frames) — ver firmware/mega_bridge/include/protocol.h.

Tópicos publicados:
  /hoverboard/front/{left,right}/velocity  (std_msgs/Float64, RPM)
  /hoverboard/rear/{left,right}/velocity   (std_msgs/Float64, RPM)
  /imu/data                                (sensor_msgs/Imu)
  /optical_flow                            (geometry_msgs/Vector3Stamped, x=dx, y=dy, z=quality)
  /battery/{front,rear}                    (sensor_msgs/BatteryState, V)
  /start_button                            (std_msgs/Bool)

Tópicos consumidos:
  /wheel_vel_setpoints                     (wheel_msgs/WheelSpeeds, mesmo formato do cmd_vel_to_wheels)
  /leds/color                              (std_msgs/ColorRGBA)
  /light/cmd                               (std_msgs/Bool)
"""
import math
import struct
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, ColorRGBA, Float64
from wheel_msgs.msg import WheelSpeeds

START0 = 0xAA
START1 = 0x55

FT_SET_SPEED = 0x01
FT_LEDS      = 0x02
FT_RELAY     = 0x03
FT_STATE     = 0x81
FT_IMU       = 0x82
FT_FLOW      = 0x83


def _xor8(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x & 0xFF


def _build_frame(ft: int, payload: bytes) -> bytes:
    header = bytes([ft, len(payload)])
    chk = _xor8(header + payload)
    return bytes([START0, START1]) + header + payload + bytes([chk])


class _Decoder:
    """Decodificador de frames vindos da MEGA (estado de máquina simples)."""

    def __init__(self):
        self._st = 0
        self._type = 0
        self._len = 0
        self._got = 0
        self._buf = bytearray(64)

    def feed(self, b: int):
        """Alimenta um byte. Retorna (type, payload_bytes) quando completa um frame."""
        if self._st == 0:
            if b == START0:
                self._st = 1
            return None
        if self._st == 1:
            # Resync: depois de um 0xAA, 0x55 fecha o header. Outro 0xAA
            # mantém em S1 (header novo começou); qualquer outra coisa
            # volta a S0. Sem isso, 0xAA 0xAA 0x55 perdia frame.
            if b == START1:
                self._st = 2
            elif b == START0:
                self._st = 1
            else:
                self._st = 0
            return None
        if self._st == 2:
            self._type = b
            self._st = 3
            return None
        if self._st == 3:
            self._len = b
            self._got = 0
            if self._len > 64:
                self._st = 0
                return None
            self._st = 5 if self._len == 0 else 4
            return None
        if self._st == 4:
            self._buf[self._got] = b
            self._got += 1
            if self._got >= self._len:
                self._st = 5
            return None
        if self._st == 5:
            self._st = 0
            expected = (self._type ^ self._len) & 0xFF
            for i in range(self._len):
                expected ^= self._buf[i]
            if expected == b:
                return self._type, bytes(self._buf[: self._len])
            return None
        return None


class MegaBridge(Node):

    def __init__(self):
        super().__init__('mega_bridge')

        self.declare_parameter('port', '/dev/mega')
        self.declare_parameter('baud', 230400)
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('flow_frame', 'flow_link')
        # Reescala wheel_msgs.WheelSpeeds (unidades do hoverboard) caso
        # cmd_vel_to_wheels ainda esteja saindo nas unidades originais.
        self.declare_parameter('wheel_scale', 1.0)

        self._port = self.get_parameter('port').value
        self._baud = int(self.get_parameter('baud').value)
        self._imu_frame = self.get_parameter('imu_frame').value
        self._flow_frame = self.get_parameter('flow_frame').value
        self._wheel_scale = float(self.get_parameter('wheel_scale').value)

        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.05)
        except Exception as e:
            self.get_logger().error(f'falha ao abrir {self._port}: {e}')
            raise

        self._tx_lock = threading.Lock()
        self._decoder = _Decoder()

        # QoS por tipo de dado:
        # - sensor_data (BEST_EFFORT, depth=5) para IMU 50 Hz e flow 100 Hz —
        #   sob jitter, RELIABLE força reenvio e empilha latência; melhor
        #   perder uma amostra do que receber tudo atrasado.
        # - RELIABLE depth=10 para setpoints/comandos (perder pode parar o robô).
        qos_cmd = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self._pub_rpm = {
            ('front', 'left'):  self.create_publisher(Float64, 'hoverboard/front/left/velocity', qos_cmd),
            ('front', 'right'): self.create_publisher(Float64, 'hoverboard/front/right/velocity', qos_cmd),
            ('rear',  'left'):  self.create_publisher(Float64, 'hoverboard/rear/left/velocity', qos_cmd),
            ('rear',  'right'): self.create_publisher(Float64, 'hoverboard/rear/right/velocity', qos_cmd),
        }
        self._pub_imu = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self._pub_flow = self.create_publisher(Vector3Stamped, 'optical_flow', qos_profile_sensor_data)
        self._pub_bat_front = self.create_publisher(BatteryState, 'battery/front', qos_cmd)
        self._pub_bat_rear = self.create_publisher(BatteryState, 'battery/rear', qos_cmd)
        self._pub_button = self.create_publisher(Bool, 'start_button', qos_cmd)

        # Subscribers
        self.create_subscription(WheelSpeeds, 'wheel_vel_setpoints', self._on_setpoint, qos_cmd)
        self.create_subscription(ColorRGBA, 'leds/color', self._on_leds, qos_cmd)
        self.create_subscription(Bool, 'light/cmd', self._on_light, qos_cmd)

        # Thread de leitura — bloqueante em ser.read, fora do executor pra não atrapalhar callbacks.
        self._stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name='mega_rx')
        self._rx_thread.start()

        self.get_logger().info(
            f'MegaBridge: {self._port}@{self._baud} | wheel_scale={self._wheel_scale}'
        )

    # ------------------------------------------------------------------
    # Saída pra MEGA (setpoints, LEDs, relé)
    # ------------------------------------------------------------------

    def _wheelspeeds_to_steer_speed(self, left: float, right: float):
        """Converte (left, right) → (steer, speed) com saturação int16.

        Convenção do firmware hoverboard (NiklasFauth fork):
            speedL = speed + steer
            speedR = speed - steer
        Logo:
            speed = (left + right) / 2
            steer = (left - right) / 2
        """
        L = left * self._wheel_scale
        R = right * self._wheel_scale
        speed = int(round((L + R) / 2.0))
        steer = int(round((L - R) / 2.0))
        # Saturação int16
        speed = max(-32000, min(32000, speed))
        steer = max(-32000, min(32000, steer))
        return steer, speed

    def _on_setpoint(self, msg: WheelSpeeds):
        steer, speed = self._wheelspeeds_to_steer_speed(msg.left_wheel, msg.right_wheel)
        # Mesma referência pras duas placas — skid-steer 4 rodas.
        payload = struct.pack('<hhhh', steer, speed, steer, speed)
        self._send(FT_SET_SPEED, payload)

    def _on_leds(self, msg: ColorRGBA):
        r = max(0, min(255, int(msg.r * 255)))
        g = max(0, min(255, int(msg.g * 255)))
        b = max(0, min(255, int(msg.b * 255)))
        # `a` (alpha) é reaproveitado como modo: 0=fixo, 1=pisca, 2=rotação.
        mode = max(0, min(255, int(msg.a)))
        self._send(FT_LEDS, bytes([r, g, b, mode]))

    def _on_light(self, msg: Bool):
        self._send(FT_RELAY, bytes([1 if msg.data else 0, 0]))

    def _send(self, ft: int, payload: bytes):
        frame = _build_frame(ft, payload)
        with self._tx_lock:
            try:
                self._ser.write(frame)
            except Exception as e:
                self.get_logger().warn(f'serial write falhou: {e}')

    # ------------------------------------------------------------------
    # Entrada da MEGA (STATE/IMU/FLOW)
    # ------------------------------------------------------------------

    def _rx_loop(self):
        while not self._stop and rclpy.ok():
            try:
                chunk = self._ser.read(64)
            except Exception as e:
                self.get_logger().warn(f'serial read falhou: {e}')
                time.sleep(0.1)
                continue
            for b in chunk:
                frame = self._decoder.feed(b)
                if frame is None:
                    continue
                ft, payload = frame
                try:
                    if ft == FT_STATE:
                        self._handle_state(payload)
                    elif ft == FT_IMU:
                        self._handle_imu(payload)
                    elif ft == FT_FLOW:
                        self._handle_flow(payload)
                except Exception as e:
                    self.get_logger().warn(f'erro decodificando frame 0x{ft:02x}: {e}')

    def _handle_state(self, p: bytes):
        # 16 bytes: rpm_FL, rpm_FR, rpm_RL, rpm_RR, batF_x100, batR_x100, faultF, faultR, btn, _pad
        if len(p) != 16:
            return
        rpm_FL, rpm_FR, rpm_RL, rpm_RR, batF, batR = struct.unpack('<hhhhhh', p[:12])
        btn = p[14]

        for (board, side), value in (
            (('front', 'left'),  rpm_FL),
            (('front', 'right'), rpm_FR),
            (('rear',  'left'),  rpm_RL),
            (('rear',  'right'), rpm_RR),
        ):
            self._pub_rpm[(board, side)].publish(Float64(data=float(value)))

        for pub, raw in ((self._pub_bat_front, batF), (self._pub_bat_rear, batR)):
            b = BatteryState()
            b.header.stamp = self.get_clock().now().to_msg()
            b.voltage = raw / 100.0
            b.present = raw > 0
            pub.publish(b)

        self._pub_button.publish(Bool(data=bool(btn)))

    def _handle_imu(self, p: bytes):
        # 20 bytes: quat w,x,y,z (Q14); gyro x,y,z (rad/s ×1000); accel x,y,z (m/s²×1000)
        if len(p) != 20:
            return
        qw, qx, qy, qz, gx, gy, gz, ax, ay, az = struct.unpack('<hhhhhhhhhh', p)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame
        msg.orientation.w = qw / 16384.0
        msg.orientation.x = qx / 16384.0
        msg.orientation.y = qy / 16384.0
        msg.orientation.z = qz / 16384.0
        msg.angular_velocity.x = gx / 1000.0
        msg.angular_velocity.y = gy / 1000.0
        msg.angular_velocity.z = gz / 1000.0
        msg.linear_acceleration.x = ax / 1000.0
        msg.linear_acceleration.y = ay / 1000.0
        msg.linear_acceleration.z = az / 1000.0
        # Covariâncias: setamos a diagonal com valores razoáveis pro BNO055 calibrado.
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        msg.linear_acceleration_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
        self._pub_imu.publish(msg)

    def _handle_flow(self, p: bytes):
        # 5 bytes: dx, dy, quality
        if len(p) != 5:
            return
        dx, dy = struct.unpack('<hh', p[:4])
        quality = p[4]
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._flow_frame
        msg.vector.x = float(dx)
        msg.vector.y = float(dy)
        msg.vector.z = float(quality)
        self._pub_flow.publish(msg)

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._stop = True
        try:
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MegaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
