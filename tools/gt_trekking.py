#!/usr/bin/env python3
"""gt_trekking — verdade-terreno do sim lado a lado com a odometria.

POR QUE EXISTE (BO de método, 2026-08-24): analisando as rotas de trekking eu
converti as coordenadas gravadas de `odom` pro mundo somando um offset FIXO
`+(spawn_x, spawn_y)`. Isso só vale se a odometria fosse perfeita. Medido
naquele mesmo instante contra o Gazebo: a odom tinha derivado **2,51 m e 27,1°**.
Resultado: cones gravados CERTOS apareciam como "fantasma a 2,3 m do cone real"
— o erro que eu media era a minha própria deriva, e o diagnóstico foi todo pro
lugar errado.

Este logger existe pra isso não se repetir: grava, no mesmo CSV e no mesmo
instante, a pose VERDADEIRA do Gazebo e a pose da odometria (`/trekking/pose`),
com a deriva já calculada. Com ele, converter odom->mundo vira medida, não
premissa.

Bônus: a deriva medida aqui é exatamente o que o snap-to-cone existe pra
corrigir. Se ela for grande, o sim é um banco de prova legítimo pra precisão.

USO (com o sim rodando):
  python3 tools/gt_trekking.py --dur 300 --out /tmp/gt.csv
  python3 tools/gt_trekking.py --mundo trekking --modelo sim_robot
"""
import argparse
import csv
import math
import re
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

_POSE_RE = re.compile(
    r'Name:\s*{modelo}\s*\n\s*- Pose[^\n]*\n\s*\[([-\d.e ]+)\]\s*\n\s*\[([-\d.e ]+)\]')


def pose_gazebo(mundo, modelo):
    """(x, y, yaw) verdadeiros, ou None. Usa o CLI do gz (1 chamada por amostra;
    a 2 Hz é irrelevante e evita ter que bridgear dynamic_pose/info)."""
    try:
        # NAO mexer no env: forcar GZ_PARTICAO='' quebra a descoberta e o
        # comando volta vazio (testado).
        out = subprocess.run(
            ['gz', 'model', '-m', modelo, '-p'],
            capture_output=True, text=True, timeout=5,
        ).stdout
    except Exception:
        return None
    m = re.search(r'- Pose[^\n]*\n\s*\[([-\d.e ]+)\]\s*\n\s*\[([-\d.e ]+)\]', out)
    if not m:
        return None
    xyz = [float(v) for v in m.group(1).split()]
    rpy = [float(v) for v in m.group(2).split()]
    return xyz[0], xyz[1], rpy[2]


class Odo(Node):
    def __init__(self):
        super().__init__('gt_trekking')
        self.p = None
        self.create_subscription(PoseStamped, 'trekking/pose', self._p, 10)

    def _p(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.p = (m.pose.position.x, m.pose.position.y, yaw)


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dur', type=float, default=300.0)
    ap.add_argument('--out', default='/tmp/gt_trekking.csv')
    ap.add_argument('--hz', type=float, default=2.0)
    ap.add_argument('--modelo', default='sim_robot')
    args = ap.parse_args()

    rclpy.init()
    n = Odo()
    t0 = time.time()
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw_deg',
                    'odom_x', 'odom_y', 'odom_yaw_deg',
                    'drift_xy', 'drift_yaw_deg'])
        prox = time.time()
        ref = None      # 1a amostra: ancora dos dois frames
        while time.time() - t0 < args.dur:
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.time() < prox:
                continue
            prox = time.time() + 1.0 / args.hz
            gt = pose_gazebo(None, args.modelo)
            if gt is None or n.p is None:
                continue
            gx, gy, gyaw = gt
            ox, oy, oyaw = n.p
            if ref is None:
                ref = (gx, gy, gyaw, ox, oy, oyaw)
            gx0, gy0, gyaw0, ox0, oy0, oyaw0 = ref
            # Deriva SEM assumir offset nenhum: compara o DESLOCAMENTO desde a
            # 1a amostra, cada um expresso no seu proprio frame de partida.
            # Se a odom fosse perfeita os dois vetores seriam identicos.
            def local(dx, dy, yaw0):
                c, s_ = math.cos(-yaw0), math.sin(-yaw0)
                return dx * c - dy * s_, dx * s_ + dy * c
            gdx, gdy = local(gx - gx0, gy - gy0, gyaw0)
            odx, ody = local(ox - ox0, oy - oy0, oyaw0)
            drift = math.hypot(gdx - odx, gdy - ody)
            w.writerow([round(time.time() - t0, 2),
                        round(gx, 3), round(gy, 3), round(math.degrees(gyaw), 1),
                        round(ox, 3), round(oy, 3), round(math.degrees(oyaw), 1),
                        round(drift, 3),
                        round(math.degrees(wrap((gyaw - gyaw0) - (oyaw - oyaw0))), 1)])
            f.flush()
    n.destroy_node()
    rclpy.try_shutdown()
    print('gravado em', args.out)


if __name__ == '__main__':
    main()
