#!/usr/bin/env python3
"""Monitor de bancada FLOW vs RODA — separa as fontes que o pose_estimator funde.

Throwaway, mesmo estilo do imu_check.py. Rodar com ROS sourced na Pi:
    python3 ros2_packages/robot_nav/tools/flow_check.py

Por que existe: o pose_estimator publica só a translação JÁ FUNDIDA
(vx_body = α·flow + (1-α)·roda). Quando o ponteiro "anda mais do que deveria"
e o SLAM se perde, não dá pra saber pela /odom QUEM exagerou. Este monitor
reproduz as MESMAS conversões (m/count, swap, sinais; raio de roda) mas integra
flow-só e roda-só em trilhas SEPARADAS — então um curso reto medido a trena
diz, direto: flow leu X m, roda leu Y m, real = Z m.

Espelha os defaults do robot.launch.py + pose_estimator. Se mudar lá, passe por
argv:  flow_check.py rad_per_count=0.00167 height=0.12 wheel_radius=0.082

Como ler (curso RETO de comprimento medido, MOTOR LIGADO = condição que quebra):
  - flow_fwd vs roda_fwd vs trena → quem exagera a translação e quanto.
  - flow_lat (deveria ~0 num reto): mede a deriva lateral que a roda NÃO ancora.
  - alpha (de /trekking/health): confirma o peso real do flow (~0.92 esperado).
  - q=0 / spikes: amostras gateadas por EMI (PMW3901 vê lixo do motor).
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String
import json


def _arg(name, default):
    for a in sys.argv[1:]:
        if a.startswith(name + '='):
            return float(a.split('=', 1)[1])
    return default


class FlowCheck(Node):
    def __init__(self):
        super().__init__('flow_check')

        # --- constantes espelhadas do pose_estimator / launch ---
        self.height        = _arg('height', 0.12)
        self.rad_per_count = _arg('rad_per_count', 0.00167)
        self.m_per_count   = self.height * math.tan(self.rad_per_count)
        self.flow_swap_xy  = True
        self.flow_x_sign   = -1.0
        self.flow_y_sign   = 1.0
        self.wheel_radius  = _arg('wheel_radius', 0.082)
        self.rpm_to_rads   = 2.0 * math.pi / 60.0

        # --- trilhas separadas (body frame, curso reto) ---
        self.flow_fwd = 0.0   # ∫ d_body_x das contagens (canal limpo, robusto a dt)
        self.flow_lat = 0.0    # ∫ d_body_y  (deveria ~0 num reto)
        self.wheel_fwd = 0.0   # ∫ wheel_vx dt
        self.flow_path = 0.0   # ∫ |d_body| (caminho, p/ ver inflação)
        # Reproduz a integração REAL do pose_estimator: segura flow_vx e re-integra
        # a 50 Hz no tick. Se a cadência do flow tiver jitter/burst, isso VAZA vs o
        # flow_fwd limpo — é o over-travel que o slam fica corrigindo.
        self.flow_reint = 0.0   # ∫ flow_vx(segurado) · tick_dt
        # /odom REAL (o que o slam_toolbox consome) — net e caminho da pose publicada
        self.odom_x0 = self.odom_y0 = None
        self.odom_x = self.odom_y = 0.0
        self.odom_path = 0.0
        self._odom_lx = self._odom_ly = None

        # --- estado instantâneo ---
        self.flow_vx = self.flow_vy = 0.0
        self.v_fl = self.v_fr = self.v_rl = self.v_rr = 0.0
        self.gz_raw = 0.0
        self.alpha = self.quality = 0.0
        self._last_flow_wall = None

        # --- estatística de saúde do flow ---
        self.n_flow = 0
        self.n_q0 = 0          # amostras quality==0 (gateadas/EMI)
        self.n_spike = 0       # |dx|>500 ou |dy|>500 counts num tick (~0.1 m)
        self.q_min = 1e9
        self.q_sum = 0.0
        self.alpha_sum = 0.0
        self.n_alpha = 0

        self.got_flow = self.got_wheel = self.got_imu = False

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Vector3Stamped, '/optical_flow', self.on_flow,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, '/trekking/health', self.on_health, 10)
        self.create_subscription(Float64MultiArray, '/hoverboard/wheel_velocities',
                                 self._on_wheels, 10)

        # integra a roda no timer (velocidade contínua); flow integra na chegada
        self._last_tick = self.get_clock().now()
        self.create_timer(0.1, self.tick)        # 10 Hz integra roda + print 2 Hz
        self._print_div = 0

        print(f"flow_check: m/count={self.m_per_count*1000:.3f} mm "
              f"(h={self.height:.3f}, rad/count={self.rad_per_count:.5f}), "
              f"wheel_radius={self.wheel_radius:.3f}\n"
              f"ZERA os acumuladores no start. Ctrl-C imprime o resumo.\n", flush=True)

    def on_flow(self, msg):
        now = self.get_clock().now()
        dx = msg.vector.x
        dy = msg.vector.y
        q = msg.vector.z
        self.got_flow = True
        self.n_flow += 1
        self.quality = q
        self.q_sum += q
        self.q_min = min(self.q_min, q)
        if q <= 0.0:
            self.n_q0 += 1
        if abs(dx) > 500 or abs(dy) > 500:
            self.n_spike += 1

        if self._last_flow_wall is None:
            self._last_flow_wall = now
            return
        dt = (now - self._last_flow_wall).nanoseconds / 1e9
        self._last_flow_wall = now
        if dt <= 1e-4:
            return
        if self.flow_swap_xy:
            dx, dy = dy, dx
        d_body_x = dx * self.flow_x_sign * self.m_per_count
        d_body_y = dy * self.flow_y_sign * self.m_per_count
        self.flow_vx = d_body_x / dt
        self.flow_vy = d_body_y / dt
        # integra flow-só direto do deslocamento (não re-multiplica por dt)
        self.flow_fwd += d_body_x
        self.flow_lat += d_body_y
        self.flow_path += math.hypot(d_body_x, d_body_y)

    def on_odom(self, m):
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        if self.odom_x0 is None:
            self.odom_x0, self.odom_y0 = x, y
            self._odom_lx, self._odom_ly = x, y
        self.odom_path += math.hypot(x - self._odom_lx, y - self._odom_ly)
        self._odom_lx, self._odom_ly = x, y
        self.odom_x, self.odom_y = x, y

    def on_imu(self, m):
        self.gz_raw = math.degrees(m.angular_velocity.z)
        self.got_imu = True

    def on_health(self, m):
        try:
            h = json.loads(m.data)
            self.alpha = float(h.get('alpha', 0.0))
            self.alpha_sum += self.alpha
            self.n_alpha += 1
        except Exception:
            pass

    def _on_wheels(self, msg):
        # data = [FL, FR, RL, RR] em RPM normalizado (mega_bridge). Tool de display:
        # só converte RPM→m/s (sem sinal por-lado, igual à versão antiga).
        if len(msg.data) != 4:
            return
        k = self.rpm_to_rads * self.wheel_radius
        self.v_fl, self.v_fr, self.v_rl, self.v_rr = (x * k for x in msg.data)
        self.got_wheel = True

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds / 1e9
        self._last_tick = now
        if 0.0 < dt < 0.5:
            v_left = (self.v_fl + self.v_rl) / 2.0
            v_right = (self.v_fr + self.v_rr) / 2.0
            vx_wheel = (v_left + v_right) / 2.0
            self.wheel_fwd += vx_wheel * dt
            # mesma integração do pose_estimator: flow_vx SEGURADO re-integrado
            self.flow_reint += self.flow_vx * dt

        self._print_div += 1
        if self._print_div % 5:      # ~2 Hz
            return
        f = 'ok' if self.got_flow else '--'
        w = 'ok' if self.got_wheel else '--'
        print(f"[flow:{f} roda:{w}] "
              f"flow_fwd={self.flow_fwd:+.3f} flow_lat={self.flow_lat:+.3f} "
              f"roda_fwd={self.wheel_fwd:+.3f} m | "
              f"v_flow={self.flow_vx:+.2f} v_roda={(self.v_fl+self.v_fr+self.v_rl+self.v_rr)/4.0:+.2f} m/s | "
              f"alpha={self.alpha:.2f} q={self.quality:.0f} gz={self.gz_raw:+.0f}deg/s",
              flush=True)

    def resumo(self):
        q_mean = self.q_sum / self.n_flow if self.n_flow else 0.0
        a_mean = self.alpha_sum / self.n_alpha if self.n_alpha else 0.0
        ratio = (self.flow_fwd / self.wheel_fwd) if abs(self.wheel_fwd) > 1e-3 else float('nan')
        odom_net = (math.hypot(self.odom_x - self.odom_x0, self.odom_y - self.odom_y0)
                    if self.odom_x0 is not None else 0.0)
        leak = (self.flow_reint / self.flow_fwd) if abs(self.flow_fwd) > 1e-3 else float('nan')
        print("\n========== RESUMO flow_check ==========")
        print(f"  *** /odom REAL (o que o slam consome) ***")
        print(f"  odom_net  = {odom_net:+.3f} m   (deslocamento liquido da pose publicada)")
        print(f"  odom_path = {self.odom_path:+.3f} m   (caminho da pose — jitter inflaciona)")
        print(f"  --- canais que alimentam o /odom ---")
        print(f"  flow_fwd  = {self.flow_fwd:+.3f} m   (contagens limpas, robusto a dt)")
        print(f"  flow_reint= {self.flow_reint:+.3f} m   (flow_vx SEGURADO re-integrado a 50Hz = jeito do pose_estimator)")
        print(f"  reint/lim = {leak:.3f}            (>1 = a re-integração VAZA pra frente — over-travel)")
        print(f"  flow_lat  = {self.flow_lat:+.3f} m   (deriva lateral sem ancora)")
        print(f"  flow_path = {self.flow_path:+.3f} m   (caminho do flow limpo)")
        print(f"  roda_fwd  = {self.wheel_fwd:+.3f} m   (∫ wheel_vx dt)")
        print(f"  flow/roda = {ratio:.3f}")
        print(f"  alpha med = {a_mean:.3f}            (peso real do flow na fusão)")
        print(f"  quality   = med {q_mean:.0f} / min {self.q_min:.0f}")
        print(f"  amostras  = {self.n_flow}  | q=0 (gateadas): {self.n_q0}  | spikes>500cnt: {self.n_spike}")
        print("=======================================")
        print("odom_net vs TRENA = verdade. reint/lim>1 = bug da re-integração do flow.")


def main():
    rclpy.init()
    n = FlowCheck()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.resumo()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
