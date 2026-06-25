#!/usr/bin/env python3
"""arc_calib — o robô ARQUEIA andando? (irmão do spin_calib).

PERGUNTA QUE RESPONDE: o skid-steer faz arco coordenado (andar pra frente +
virar suave ao mesmo tempo) ou só anda reto / só gira no lugar? O `spin_calib`
mediu giro PARADO; aqui medimos GIRO ANDANDO.

Comanda UM arco por execução: `vx` pra frente + `wz` de giro durante `duration`
segundos, mede pela POSE (/odom, já fundida com a IMU) quanto REALMENTE girou e
andou, e compara com o que foi comandado. Como o `cmd_vel_to_wheels` é cinemática
diferencial PURA (sem zona-morta no software), o que sair aqui é firmware+físico.

  wz efetivo ≈ comandado  -> ELE ARQUEIA (problema era a config do nav2)
  wz efetivo ≈ 0          -> NÃO ARQUEIA (reto + giro-no-lugar é o caminho)

Pipeline (NÃO sobe nav2):
  este script -> Twist em key_vel -> twist_mux (prio 90, SEM collision) ->
  cmd_vel -> cmd_vel_to_wheels -> rodas.  Leitura: /odom (pose fundida) + imu/data.

UM ARCO POR PLAY (precisa de MUITO espaço — o arco anda pra frente):
  você posiciona o robô com ~4 m livres -> roda o script numa wz -> ele faz o
  arco e PARA -> traz de volta -> roda de novo na próxima wz. Cada execução
  ANEXA uma linha no CSV mestre /tmp/arc_calib.csv (eu puxo no fim) e grava o
  traço cru completo em /tmp/arc_calib_raw_*.csv.

PRÉ-REQUISITOS no robô (LIGADO, no chão, ~4 m livres, PS4 DESLIGADO):
  ros2 launch robot_nav robot.launch.py        # base: mega_bridge+pose+mux
  (PS4 off senão joy_vel prio 100 sobrepõe o key_vel)

USO (uma wz por vez; vx fica fixo p/ comparar o raio entre as velocidades):
  python3 arc_calib.py --wz 0.5                 # 1 arco: vx=0.25, wz=+0.5 (esq)
  python3 arc_calib.py --wz 0.8 --vx 0.25
  python3 arc_calib.py --wz -0.8                # wz NEGATIVO = arco pra DIREITA
  # varredura sugerida (uma por play): 0.3, 0.5, 0.8, 1.2, 1.7, 2.5

wz positivo = arco pra ESQUERDA, negativo = pra DIREITA. Ctrl-C PARA o robô.
"""
import argparse
import csv
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


MASTER_CSV = '/tmp/arc_calib.csv'


def _yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ArcCalib(Node):
    def __init__(self, cmd_topic, odom_topic):
        super().__init__('arc_calib')
        # estado lido pelos callbacks (thread do executor) e pela sequência
        self._yaw_accum = 0.0       # yaw acumulado contínuo (rad) — conta voltas
        self._last_yaw = None
        self._x = 0.0               # pose atual (m), frame odom
        self._y = 0.0
        self._odom_vx = 0.0         # twist medido (m/s)
        self._odom_wz = 0.0         # twist medido (rad/s)
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
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_wz = msg.twist.twist.angular.z
        self._have_odom = True

    def _on_imu(self, msg):
        now = time.monotonic()
        wz = msg.angular_velocity.z
        if self._imu_t is not None:
            self._imu_accum += wz * (now - self._imu_t)
        self._imu_t = now
        self._imu_wz = wz
        self._have_imu = True

    # snapshot atômico o suficiente (float read em CPython)
    def snap(self):
        return (self._yaw_accum, self._imu_accum, self._x, self._y)

    def sample(self):
        return (self._odom_vx, self._odom_wz, self._imu_wz,
                self._yaw_accum, self._x, self._y)

    def publish_arc(self, vx, wz):
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--wz', type=float, default=0.5,
                    help='giro do arco rad/s (+esq / -dir). Default 0.5')
    ap.add_argument('--vx', type=float, default=0.25,
                    help='velocidade pra frente m/s (fixa entre runs). Default 0.25')
    ap.add_argument('--duration', type=float, default=2.0, help='s arqueando (2.0)')
    ap.add_argument('--settle', type=float, default=1.5,
                    help='s parado antes de arrancar — afasta-se (1.5)')
    ap.add_argument('--rate', type=float, default=50.0, help='Hz do comando (50)')
    ap.add_argument('--cmd-topic', default='key_vel')
    ap.add_argument('--odom-topic', default='/odom')
    args = ap.parse_args()

    dt = 1.0 / args.rate

    rclpy.init()
    node = ArcCalib(args.cmd_topic, args.odom_topic)
    # executor num thread em background; a sequência roda no main thread
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    log = node.get_logger().info
    raw_rows = []

    def hold(vx, wz, secs, record=False):
        """publica (vx,wz) em key_vel a `rate` Hz por `secs` s (mantém fresco no mux)."""
        t_end = time.monotonic() + secs
        t_start = time.monotonic()
        while time.monotonic() < t_end:
            node.publish_arc(vx, wz)
            if record:
                ovx, owz, iwz, ya, x, y = node.sample()
                raw_rows.append({
                    't': round(time.monotonic() - t_start, 3),
                    'cmd_vx': vx, 'cmd_wz': wz,
                    'odom_vx': round(ovx, 4), 'odom_wz': round(owz, 4),
                    'imu_wz': round(iwz, 4), 'yaw_deg': round(math.degrees(ya), 2),
                    'x': round(x, 4), 'y': round(y, 4),
                })
            time.sleep(dt)

    def stop(secs):
        hold(0.0, 0.0, secs)

    try:
        log('esperando /odom...')
        t0 = time.monotonic()
        while not node._have_odom and time.monotonic() - t0 < 10.0:
            time.sleep(0.1)
        if not node._have_odom:
            log('SEM /odom em 10s — o robot.launch.py está no ar? abortando.')
            return
        log('IMU presente: %s' % node._have_imu)
        side = 'ESQUERDA' if args.wz >= 0 else 'DIREITA'
        log('ATENÇÃO: arco pra %s — vx=%.2f m/s, wz=%+.2f rad/s por %.1fs. '
            'PS4 off? ~4 m livres À FRENTE?' % (side, args.vx, args.wz, args.duration))
        stop(args.settle)

        y0, i0, x0, py0 = node.snap()
        hold(args.vx, args.wz, args.duration, record=True)   # ARQUEIA
        y1, i1, x1, py1 = node.snap()                         # fim do comando
        stop(args.settle)                                     # deixa parar (coast)
        y2, i2, x2, py2 = node.snap()

        d_cmd = math.degrees(y1 - y0)            # girou no comando
        d_tot = math.degrees(y2 - y0)            # + coast
        d_imu = math.degrees(i2 - i0)            # cross-check IMU
        eff_wz = (y1 - y0) / args.duration       # rad/s efetivo
        ratio = eff_wz / args.wz if args.wz else 0.0
        chord = math.hypot(x1 - x0, py1 - py0)   # deslocamento reto (m)
        arc_len = args.vx * args.duration        # caminho comandado (m)
        dyaw_rad = abs(y1 - y0)
        r_eff = (arc_len / dyaw_rad) if dyaw_rad > 1e-3 else float('inf')
        r_cmd = abs(args.vx / args.wz) if args.wz else float('inf')

        log('=== RESULTADO ===')
        log('  comandado : vx=%.2f m/s  wz=%+.2f rad/s  raio=%.2f m'
            % (args.vx, args.wz, r_cmd))
        log('  efetivo   : wz=%+.2f rad/s (%.0f%% do comando)  girou=%+.1f° (+coast %+.1f°)'
            % (eff_wz, 100.0 * ratio, d_cmd, d_tot))
        log('  IMU check : girou=%+.1f°   |   andou(reto)=%.2f m   raio_efetivo=%.2f m'
            % (d_imu, chord, r_eff))
        if abs(ratio) < 0.25:
            log('  >> ARQUEOU QUASE NADA (%.0f%%): nessa wz ele NÃO faz arco.' % (100 * ratio))
        elif abs(ratio) > 0.7:
            log('  >> ARQUEOU BEM (%.0f%%): ele FAZ arco nessa wz.' % (100 * ratio))
        else:
            log('  >> arco PARCIAL (%.0f%%): vira menos do que pede.' % (100 * ratio))

        # CSV mestre (anexa 1 linha por run)
        new = not os.path.exists(MASTER_CSV)
        with open(MASTER_CSV, 'a', newline='') as f:
            cols = ['stamp', 'vx', 'wz_cmd', 'wz_eff', 'ratio', 'deg_cmd',
                    'deg_total', 'deg_imu', 'chord_m', 'r_cmd_m', 'r_eff_m']
            w = csv.DictWriter(f, fieldnames=cols)
            if new:
                w.writeheader()
            w.writerow({
                'stamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'vx': args.vx, 'wz_cmd': args.wz, 'wz_eff': round(eff_wz, 4),
                'ratio': round(ratio, 3), 'deg_cmd': round(d_cmd, 1),
                'deg_total': round(d_tot, 1), 'deg_imu': round(d_imu, 1),
                'chord_m': round(chord, 3),
                'r_cmd_m': round(r_cmd, 3) if math.isfinite(r_cmd) else -1,
                'r_eff_m': round(r_eff, 3) if math.isfinite(r_eff) else -1,
            })
        log('CSV mestre (1 linha/run) anexado: %s' % MASTER_CSV)

        # traço cru deste run
        raw_path = '/tmp/arc_calib_raw_wz%+.2f_%s.csv' % (
            args.wz, time.strftime('%Y%m%d_%H%M%S'))
        if raw_rows:
            with open(raw_path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(raw_rows[0].keys()))
                w.writeheader()
                w.writerows(raw_rows)
            log('traço cru gravado: %s' % raw_path)

        log('PRONTO. Traga o robô de volta e rode a próxima wz.')
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(15):
            node.publish_arc(0.0, 0.0)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
