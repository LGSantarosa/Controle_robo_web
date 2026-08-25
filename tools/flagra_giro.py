#!/usr/bin/env python3
"""Grava TUDO que manda o robô se mexer, pra achar giro que ninguem pediu.

POR QUE EXISTE (2026-08-25): o dono viu o robô "girando pra direita" no sim.
Quando fui olhar, ja tinha parado: nenhum topico publicando, /joy mudo, runner
em idle. Diagnostico feito DEPOIS do fato nao pega comando intermitente — este
logger fica ligado durante a direcao e registra, no mesmo instante:
  - o que cada FONTE do twist_mux mandou (auto/web/joy) e o que SAIU no /cmd_vel
  - a pose VERDADEIRA do Gazebo (nao a odometria, que pode estar mentindo)
Com isso da pra separar as duas causas: "alguem mandou girar" (aparece na fonte)
de "girou sem ninguem mandar" (cmd_vel zerado e yaw mudando).

USO: rodar ANTES de comecar a dirigir, Ctrl+C quando terminar.
  python3 tools/flagra_giro.py [--csv caminho.csv]
"""
import argparse, csv, math, re, subprocess, sys, threading, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

ap = argparse.ArgumentParser()
ap.add_argument('--csv', default='controle_web/logs/flagra_giro.csv')
ap.add_argument('--hz', type=float, default=10.0)
a = ap.parse_args()

FONTES = ['cmd_vel', 'auto_vel', 'web_vel', 'joy_vel']


class Flagra(Node):
    def __init__(self):
        super().__init__('flagra_giro')
        self.ult = {t: (0.0, 0.0, 0.0) for t in FONTES}   # (vx, wz, t_recebido)
        for t in FONTES:
            self.create_subscription(Twist, t,
                                     lambda m, tt=t: self._on(tt, m), 10)
        self.gt = None
        threading.Thread(target=self._gt_loop, daemon=True).start()

    def _on(self, topico, msg):
        self.ult[topico] = (msg.linear.x, msg.angular.z, time.time())

    def _gt_loop(self):
        """`gz model -p` e um subprocesso lento (~100 ms) — fora do executor
        pra nao engasgar as assinaturas."""
        while rclpy.ok():
            try:
                out = subprocess.run(['gz', 'model', '-m', 'sim_robot', '-p'],
                                     capture_output=True, text=True, timeout=6).stdout
                m = re.search(r'- Pose[^\n]*\n\s*\[([-\d.e ]+)\]\s*\n\s*\[([-\d.e ]+)\]', out)
                if m:
                    xyz = [float(v) for v in m.group(1).split()]
                    rpy = [float(v) for v in m.group(2).split()]
                    self.gt = (xyz[0], xyz[1], rpy[2])
            except Exception:
                pass
            time.sleep(0.1)


rclpy.init()
n = Flagra()
f = open(a.csv, 'w', newline='')
w = csv.writer(f)
w.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw_deg', 'dyaw_deg_s'] +
           [c for t in FONTES for c in (t + '_vx', t + '_wz', t + '_idade_s')])
t0 = time.time()
ult_yaw = None
ult_t = None
print(f'gravando em {a.csv} — pode dirigir. Ctrl+C pra fechar.')
try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=1.0 / a.hz)
        agora = time.time()
        if n.gt is None:
            continue
        x, y, yaw = n.gt
        dyaw = ''
        if ult_yaw is not None and agora - ult_t > 1e-3:
            d = (yaw - ult_yaw + math.pi) % (2 * math.pi) - math.pi
            dyaw = round(math.degrees(d) / (agora - ult_t), 1)
        ult_yaw, ult_t = yaw, agora
        linha = [round(agora - t0, 2), round(x, 3), round(y, 3),
                 round(math.degrees(yaw), 1), dyaw]
        for t in FONTES:
            vx, wz, quando = n.ult[t]
            # idade: ha quanto tempo essa fonte falou. Fonte MUDA com o robo
            # girando e a assinatura do bug "gira sem ninguem mandar".
            idade = round(agora - quando, 2) if quando else ''
            linha += [round(vx, 3), round(wz, 3), idade]
        w.writerow(linha)
        f.flush() if int((agora - t0) * a.hz) % 20 == 0 else None
except KeyboardInterrupt:
    pass
finally:
    f.flush(); f.close()
    n.destroy_node(); rclpy.try_shutdown()
    print(f'\nfechado: {a.csv}')
