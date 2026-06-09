#!/usr/bin/env python3
"""
Ponte ROS2 ↔ Arduino MEGA 2560.

Substitui o `ros2-hoverboard-driver` antigo (que falava direto com uma única
placa). Agora a MEGA agrega:
  - 2 placas de hoverboard (frontal Serial1, traseira Serial2)
  - MPU6050 IMU (I²C, 6 eixos: giro + accel; sem yaw absoluto)
  - PMW3901 optical flow (SPI)
  - relé da luz, LED de marco, botão de partida
  - (anel WS2812 comentado no firmware — ver AUDITORIA_2026-05-29 A1)

Protocolo (frames) — ver firmware/mega_bridge/include/protocol.h.

Tópicos publicados:
  /hoverboard/front/{left,right}/velocity  (std_msgs/Float64, RPM)
  /hoverboard/rear/{left,right}/velocity   (std_msgs/Float64, RPM)
  /imu/data                                (sensor_msgs/Imu)
  /optical_flow                            (geometry_msgs/Vector3Stamped, x=dx, y=dy, z=quality)
  /battery/{front,rear}                    (sensor_msgs/BatteryState, V)
  /start_button                            (std_msgs/Bool)
  /system/health                           (std_msgs/String, JSON)

Tópicos consumidos:
  /wheel_vel_setpoints                     (wheel_msgs/WheelSpeeds, mesmo formato do cmd_vel_to_wheels)
  /leds/color                              (std_msgs/ColorRGBA)  NO-OP: anel comentado no firmware (A1); frame FT_LEDS é ignorado pela MEGA
  /light/cmd                               (std_msgs/Bool)    relé da luz
  /light/marker                            (std_msgs/Bool)    LED de marco
"""
import json
import math
import queue
import struct
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, ColorRGBA, Float64, String
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
        self.dropped = 0   # frames com len inválido (> 64) descartados

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
                self.dropped += 1
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
        # Inversão de polaridade da placa TRÁS: hardware do robô tem os dois
        # motores RL/RR ligados com fase invertida, então "frente" comanda
        # PWM positivo mas o motor gira pra trás. Aplicado antes de empacotar
        # o frame FT_SET_SPEED; default False pra não quebrar quem não tem
        # esse hardware.
        self.declare_parameter('rear_invert_speed', False)
        self.declare_parameter('rear_invert_steer', False)
        # Normalização do FEEDBACK das rodas para o referencial do robô
        # ("frente = +", topico bate com o lado fisico). Mapeamento medido em
        # bancada 2026-05-30 (rodas no ar) com comando reto e giro, cruzado com
        # observacao VISUAL:
        #   - FRENTE: roda direita le invertida (espelho L/R do hoverboard).
        #     pub front/left = +rawFL ; pub front/right = -rawFR.
        #   - TRAS: os cabos L/R da placa traseira estao TROCADOS (confirmado:
        #     no giro a frente e a tras batiam, mas o feedback da tras saia
        #     invertido). Entao o canal "RL" carrega a roda fisica da direita e
        #     vice-versa -> pub rear/left = +rawRR ; pub rear/right = -rawRL.
        # Andando reto o swap e' invisivel (RL==RR), por isso a 1a versao (so
        # sinal por roda, sem swap) acertava reto mas CANCELAVA o angular no
        # giro: v_left=(FL+RL)/2 e v_right=(FR+RR)/2 zeravam. Ver
        # AUDITORIA_2026-05-29b (A1) + execucao 2026-05-30 no fim do doc.
        # mapa: topico publicado -> (campo do struct STATE, sinal)
        self._fb_map = {
            ('front', 'left'):  ('FL',  1.0),
            ('front', 'right'): ('FR', -1.0),
            ('rear',  'left'):  ('RR',  1.0),   # cabos L/R trocados na tras
            ('rear',  'right'): ('RL', -1.0),
        }

        self._port = self.get_parameter('port').value
        self._baud = int(self.get_parameter('baud').value)
        self._imu_frame = self.get_parameter('imu_frame').value
        self._flow_frame = self.get_parameter('flow_frame').value
        self._wheel_scale = float(self.get_parameter('wheel_scale').value)
        self._rear_speed_sign = -1 if bool(self.get_parameter('rear_invert_speed').value) else 1
        self._rear_steer_sign = -1 if bool(self.get_parameter('rear_invert_steer').value) else 1

        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.05)
        except Exception as e:
            self.get_logger().error(f'falha ao abrir {self._port}: {e}')
            raise

        self._tx_lock = threading.Lock()
        self._decoder = _Decoder()
        self._dropped_logged = 0      # último valor de _decoder.dropped já logado
        self._dropped_log_ts = 0.0    # throttle do warn de frames descartados

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
        self._pub_health = self.create_publisher(String, 'system/health', qos_cmd)
        # Cache do último health para evitar reemitir JSON idêntico a 50 Hz.
        self._last_health_json: str = ''

        # Estado do FT_RELAY (1 byte relé + 1 byte marker). Guardamos os dois
        # bits aqui pra que comandos vindos em /light/cmd e /light/marker não
        # se sobrescrevam — cada callback atualiza só o próprio bit antes do
        # send.
        self._light_state = False
        self._marker_state = False

        # Subscribers
        self.create_subscription(WheelSpeeds, 'wheel_vel_setpoints', self._on_setpoint, qos_cmd)
        self.create_subscription(ColorRGBA, 'leds/color', self._on_leds, qos_cmd)
        self.create_subscription(Bool, 'light/cmd', self._on_light, qos_cmd)
        self.create_subscription(Bool, 'light/marker', self._on_marker, qos_cmd)

        # Thread de leitura — bloqueante em ser.read, fora do executor pra não atrapalhar callbacks.
        # A thread enfileira frames decodificados em `_rx_queue`; o timer ROS
        # `_drain_rx` os processa dentro de um callback do executor. Sem isso,
        # publicar direto da thread RX quebra com MultiThreadedExecutor e
        # dificulta o tracing de logs (ros2 não associa o publish ao nó).
        self._stop = False
        self._rx_queue: "queue.Queue[tuple[int, bytes]]" = queue.Queue(maxsize=256)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name='mega_rx')
        self._rx_thread.start()
        # 50 Hz: o drain esvazia a fila em LOTE (até 64 frames/tick, ~3200/s de
        # capacidade) — muito acima dos ~200 frames/s reais (flow 100 + IMU 50 +
        # STATE 50). A 200 Hz o executor single-thread do rclpy remontava o
        # wait-set sobre ~16 entidades 200×/s em Python puro e queimava ~70% de
        # um core (medido 2026-06-09: tempo no executor, não na serial/DDS).
        # 50 Hz corta as acordadas em 4× e adiciona ≤20 ms de latência (irrelevante
        # pra fusão de odom).
        self._drain_timer = self.create_timer(0.02, self._drain_rx)

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

        `_wheel_scale` amplia o valor antes do `int(round)` — comandos
        fracionários pequenos (ex.: 0.4 unidades) iam para 0 e criavam
        deadband artificial. Subir wheel_scale resolve sem mexer no
        cmd_vel_to_wheels. Default 1.0 mantém comportamento antigo.
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
        # TRÁS pode estar com polaridade dos motores invertida — aplica os
        # sinais antes de empacotar (frente fica sempre direto).
        steer_rear = self._rear_steer_sign * steer
        speed_rear = self._rear_speed_sign * speed
        payload = struct.pack('<hhhh', steer, speed, steer_rear, speed_rear)
        self._send(FT_SET_SPEED, payload)

    def _on_leds(self, msg: ColorRGBA):
        r = max(0, min(255, int(msg.r * 255)))
        g = max(0, min(255, int(msg.g * 255)))
        b = max(0, min(255, int(msg.b * 255)))
        # `a` (alpha) é reaproveitado como modo: 0=fixo, 1=pisca, 2=rotação.
        mode = max(0, min(255, int(msg.a)))
        self._send(FT_LEDS, bytes([r, g, b, mode]))

    def _on_light(self, msg: Bool):
        self._light_state = bool(msg.data)
        self._send_relay()

    def _on_marker(self, msg: Bool):
        self._marker_state = bool(msg.data)
        self._send_relay()

    def _send_relay(self):
        self._send(FT_RELAY, bytes([
            1 if self._light_state else 0,
            1 if self._marker_state else 0,
        ]))

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
                try:
                    # Drop o mais novo se a fila encheu — manter o mais antigo
                    # geraria latência crescente sem upside (publishers ROS
                    # também só consomem o mais recente em best-effort).
                    self._rx_queue.put_nowait(frame)
                except queue.Full:
                    pass

    def _drain_rx(self):
        # Processa todo o backlog acumulado desde o último tick. Limite alto
        # de iterações é só rede de segurança — em prática a fila fica vazia
        # rapidamente.
        dropped = self._decoder.dropped
        if dropped > self._dropped_logged:
            now = time.time()
            if now - self._dropped_log_ts > 5.0:
                self.get_logger().warn(
                    f'{dropped} frames descartados (len inválido) desde o boot')
                self._dropped_log_ts = now
                self._dropped_logged = dropped

        for _ in range(64):
            try:
                ft, payload = self._rx_queue.get_nowait()
            except queue.Empty:
                return
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
        # 16 bytes: rpm_FL/FR/RL/RR, batF_x100, batR_x100, faultF, faultR, btn, sensor_flags
        if len(p) != 16:
            return
        rpm_FL, rpm_FR, rpm_RL, rpm_RR, batF, batR = struct.unpack('<hhhhhh', p[:12])
        faultF = p[12]
        faultR = p[13]
        btn = p[14]
        sensor_flags = p[15]

        # Normaliza o feedback pro referencial do robô (frente = +) com o
        # mapa _fb_map (sinal por roda + swap L/R na traseira). Sem isto a odom
        # cancela: andando reto (só sinal) OU no giro (sem o swap). Ver __init__.
        raw = {'FL': rpm_FL, 'FR': rpm_FR, 'RL': rpm_RL, 'RR': rpm_RR}
        for (board, side), (src, sign) in self._fb_map.items():
            self._pub_rpm[(board, side)].publish(
                Float64(data=float(raw[src]) * sign)
            )

        # present = placa respondendo (não-stale), não "voltagem > 0". Assim
        # 0 V com present=True = curto/medida real; 0 V com present=False =
        # placa muda. faultF/faultR bit 0 = stale (ver firmware txState).
        for pub, raw, stale in (
            (self._pub_bat_front, batF, bool(faultF & 0x01)),
            (self._pub_bat_rear,  batR, bool(faultR & 0x01)),
        ):
            b = BatteryState()
            b.header.stamp = self.get_clock().now().to_msg()
            b.voltage = raw / 100.0
            b.present = not stale
            pub.publish(b)

        self._pub_button.publish(Bool(data=bool(btn)))

        # /system/health — JSON. Só emite quando muda, evita poluir tópico
        # a 50 Hz com a mesma string.
        health = {
            'front_stale': bool(faultF & 0x01),
            'rear_stale':  bool(faultR & 0x01),
            'imu_ok':      bool(sensor_flags & 0x01),
            'flow_ok':     bool(sensor_flags & 0x02),
        }
        as_json = json.dumps(health, sort_keys=True)
        if as_json != self._last_health_json:
            self._last_health_json = as_json
            self._pub_health.publish(String(data=as_json))

    def _handle_imu(self, p: bytes):
        # 12 bytes: gyro x,y,z (rad/s ×1000); accel x,y,z (m/s² ×1000).
        # MPU6050 (6 eixos): SEM orientação absoluta (não tem magnetômetro). O
        # yaw é integrado da taxa do giro no pose_estimator, que também corrige
        # a montagem de ponta-cabeça (Z pra baixo). Frame BRUTO do sensor.
        if len(p) != 12:
            return
        gx, gy, gz, ax, ay, az = struct.unpack('<hhhhhh', p)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame
        msg.angular_velocity.x = gx / 1000.0
        msg.angular_velocity.y = gy / 1000.0
        msg.angular_velocity.z = gz / 1000.0
        msg.linear_acceleration.x = ax / 1000.0
        msg.linear_acceleration.y = ay / 1000.0
        msg.linear_acceleration.z = az / 1000.0
        # Sem orientação absoluta: convenção sensor_msgs/Imu é marcar
        # orientation_covariance[0] = -1 (consumidores devem ignorar orientation).
        msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # MPU6050 é mais ruidoso que o BNO055 → diagonal um pouco maior.
        msg.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
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
