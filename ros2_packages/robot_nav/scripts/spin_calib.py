#!/usr/bin/env python3
"""spin_calib — calibração de giro no lugar (esquerda × direita).

Mede a SIMETRIA e a ESCALA do giro do skid-steer: a cada velocidade angular X,
gira no lugar pra ESQUERDA por `duration` s, mede pela POSE (/odom, já fundida
com a IMU) o quanto REALMENTE girou; repete pra DIREITA. Compara os dois lados
e mostra o rad/s efetivo — assim dá pra ver se os lados estão iguais e se baixar
a velocidade faz girar melhor (motivo: as fitas nas rodas deixaram a curva do
nav2 rápida/girando demais, 2026-06-19).

Pipeline (NÃO sobe nav2):
  este script -> Twist em key_vel -> twist_mux (prio 90, SEM collision) ->
  cmd_vel -> cmd_vel_to_wheels -> rodas.  Leitura do yaw: /odom (pose fundida).

PRÉ-REQUISITOS no robô (LIGADO, no chão, espaço livre, PS4 DESLIGADO):
  ros2 launch robot_nav robot.launch.py        # base: mega_bridge+pose+mux
  (PS4 off senão joy_vel prio 100 sobrepõe o key_vel)

USO:
  python3 spin_calib.py                         # varredura 6,4,3,2 rad/s
  python3 spin_calib.py --speeds 4,3,2,1 --duration 2.0
  python3 spin_calib.py --speeds 3              # uma só

O yaw é acumulado continuamente (soma de incrementos wrap) -> conta certo mesmo
passando de 360° (a 6 rad/s em 2 s dá ~2 voltas). Cruza com a taxa crua da
/imu/data como conferência. Ctrl-C PARA o robô.
"""
import argparse
import csv
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def _yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class SpinCalib(Node):
    def __init__(self, cmd_topic, odom_topic):
        super().__init__('spin_calib')
        # estado lido pelos callbacks (thread do executor) e pela sequência
        self._yaw_accum = 0.0       # yaw acumulado contínuo (rad) — conta voltas
        self._last_yaw = None
        self._have_odom = False
        self._imu_accum = 0.0       # integral do gyro Z (rad) — cross-check
        self._imu_t = None
        self._imu_wz = 0.0
        self._have_imu = False

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(Imu, 'imu/data', self._on_imu,
                                 qos_profile_sensor_data)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        if self._last_yaw is not None:
            self._yaw_accum += _wrap(yaw - self._last_yaw)
        self._last_yaw = yaw
        self._have_odom = True

    def _on_imu(self, msg):
        now = time.monotonic()
        wz = msg.angular_velocity.z
        if self._imu_t is not None:
            self._imu_accum += wz * (now - self._imu_t)
        self._imu_t = now
        self._imu_wz = wz
        self._have_imu = True

    # snapshots atômicos o suficiente (float read em CPython)
    def snap(self):
        return self._yaw_accum, self._imu_accum

    def publish_twist(self, wz):
        m = Twist()
        m.angular.z = float(wz)
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--speeds', default='6,4,3,2',
                    help='velocidades angulares rad/s (vírgula). Default 6,4,3,2')
    ap.add_argument('--duration', type=float, default=2.0, help='s por lado (2.0)')
    ap.add_argument('--settle', type=float, default=1.5,
                    help='s parado entre giros (1.5)')
    ap.add_argument('--rate', type=float, default=50.0, help='Hz do comando (50)')
    ap.add_argument('--cmd-topic', default='key_vel')
    ap.add_argument('--odom-topic', default='/odom')
    args = ap.parse_args()

    speeds = [float(s) for s in args.speeds.split(',') if s.strip()]
    dt = 1.0 / args.rate

    rclpy.init()
    node = SpinCalib(args.cmd_topic, args.odom_topic)
    # executor num thread em background; a sequência roda no main thread
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    log = node.get_logger().info

    def hold(wz, secs):
        """publica `wz` em key_vel a `rate` Hz por `secs` s (mantém fresco no mux)."""
        t_end = time.monotonic() + secs
        while time.monotonic() < t_end:
            node.publish_twist(wz)
            time.sleep(dt)

    def stop(secs):
        hold(0.0, secs)

    rows = []
    try:
        log('esperando /odom...')
        t0 = time.monotonic()
        while not node._have_odom and time.monotonic() - t0 < 10.0:
            time.sleep(0.1)
        if not node._have_odom:
            log('SEM /odom em 10s — o robot.launch.py está no ar? abortando.')
            return
        log('IMU presente: %s' % node._have_imu)
        log('ATENÇÃO: robô vai GIRAR no lugar. PS4 desligado? espaço livre?')
        stop(3.0)  # assenta e dá tempo de afastar

        for sp in speeds:
            res = {}
            for name, sign in (('ESQ(+)', +1.0), ('DIR(-)', -1.0)):
                stop(args.settle)
                y0, i0 = node.snap()
                hold(sign * sp, args.duration)          # GIRA
                y1, i1 = node.snap()                    # fim do comando (2s)
                stop(args.settle)                       # deixa parar (coast)
                y2, i2 = node.snap()
                d_cmd = math.degrees(y1 - y0)           # girou no comando
                d_tot = math.degrees(y2 - y0)           # + coast
                d_imu = math.degrees(i2 - i0)           # cross-check IMU
                eff = (y1 - y0) / args.duration         # rad/s efetivo no comando
                res[name] = (d_cmd, d_tot, d_imu, eff)
                log('  %4.1f rad/s %s: cmd=%+7.1f°  +coast=%+7.1f°  '
                    'IMU=%+7.1f°  efetivo=%+.2f rad/s'
                    % (sp, name, d_cmd, d_tot, d_imu, eff))
                rows.append({'speed': sp, 'side': name, 'deg_cmd': d_cmd,
                             'deg_total': d_tot, 'deg_imu': d_imu, 'eff_radps': eff})
            # simetria do par: |esq| vs |dir| (pelo total, inclui coast)
            le = abs(res['ESQ(+)'][1])
            ri = abs(res['DIR(-)'][1])
            big = max(le, ri) or 1.0
            asym = 100.0 * (le - ri) / big
            log('  %4.1f rad/s  SIMETRIA: esq=%.1f° dir=%.1f°  '
                'assimetria=%+.1f%% (%s gira mais)'
                % (sp, le, ri, asym, 'esq' if le > ri else 'dir'))
        stop(0.5)

        # CSV
        path = '/tmp/spin_calib_%s.csv' % time.strftime('%Y%m%d_%H%M%S')
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['speed', 'side', 'deg_cmd',
                                              'deg_total', 'deg_imu', 'eff_radps'])
            w.writeheader()
            w.writerows(rows)
        log('CSV salvo em %s' % path)
        log('PRONTO. Lados ~iguais = bom; diferença grande = ajustar o ganho '
            'por lado. rad/s efetivo << comando = patina/perde; ~igual = fiel.')
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(15):
            node.publish_twist(0.0)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
