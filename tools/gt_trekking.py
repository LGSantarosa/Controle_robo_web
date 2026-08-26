#!/usr/bin/env python3
"""gt_trekking — verdade-terreno do sim lado a lado com a odometria.

POR QUE EXISTE (BO de metodo, 2026-08-24): analisando as rotas de trekking eu
converti as coordenadas gravadas de `odom` pro mundo somando um offset FIXO
`+(spawn_x, spawn_y)`. Isso so vale se a odometria fosse perfeita. Medido
naquele mesmo instante contra o Gazebo: a odom tinha derivado **2,51 m e 27,1**.
Resultado: cones gravados CERTOS apareciam como "fantasma a 2,3 m do cone real"
-- o erro que eu media era a minha propria deriva.

CASAMENTO NO TEMPO (2026-08-26, 2o BO do mesmo instrumento): a versao antiga
chamava `gz model -p` (subprocesso, ~1 s) e comparava aquela pose VELHA com a
odom ATUAL. A 0,9 m/s isso sao 90 cm de defasagem; a 0,35 m/s, 35 cm. Eu reportei
a razao dessas defasagens (2,55) como se fosse deriva do robo -- ela era quase
exatamente a razao das velocidades (2,57). O instrumento estava se medindo.

O conserto: a pose verdadeira agora vem do STREAM `gz topic -e -t
/world/<mundo>/dynamic_pose/info`, que chega CARIMBADO em tempo de simulacao, e
a odom e' INTERPOLADA para esse mesmo carimbo. (O caminho "certo" -- bridgear
Pose_V pra tf2_msgs/TFMessage -- foi tentado e DESCARTADO: o ros_gz_bridge do
jazzy entrega os transforms com `child_frame_id` vazio e `stamp` zerado, ou
seja, sem nome de modelo e sem tempo, que era exatamente o que a gente foi
buscar. Testado em 2026-08-26, nao repetir.) Sobra a coluna `dt_match_ms`: o quao longe estavam as
amostras de odom usadas na interpolacao. Ela nao esconde o erro do instrumento
-- ela publica. Se `dt_match_ms` estiver grande, a linha nao vale.

USO (com o sim rodando):
  python3 tools/gt_trekking.py --dur 300 --out /tmp/gt.csv
  python3 tools/gt_trekking.py --mundo trekking --modelo sim_robot
  python3 tools/gt_trekking.py --modo cli      # volta ao metodo antigo (defasado)
"""
import argparse
import bisect
import csv
import math
import os
import re
import subprocess
import time

import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

_POSE_RE = re.compile(
    r'- Pose[^\n]*\n\s*\[([-\d.e ]+)\]\s*\n\s*\[([-\d.e ]+)\]')


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def interpola_odom(buf, alvo):
    """Odom no instante `alvo`, interpolada entre as amostras que o cercam.

    `buf` = [(t, x, y, yaw), ...] ordenado por t. Devolve (x, y, yaw, dt_ms) ou
    None se `alvo` cai fora do buffer -- fora e' extrapolacao, e extrapolar e'
    justamente o erro que este arquivo existe pra nao cometer de novo.

    `dt_ms` = distancia da amostra mais proxima usada. E' o residuo do
    casamento; quem le o CSV precisa ver quanto de folga sobrou.
    """
    if len(buf) < 2:
        return None
    ts = [b[0] for b in buf]
    if alvo < ts[0] or alvo > ts[-1]:
        return None
    i = bisect.bisect_left(ts, alvo)
    if i == 0:
        return buf[0][1], buf[0][2], buf[0][3], 0.0
    a, b = buf[i - 1], buf[i]
    span = b[0] - a[0]
    f = 0.0 if span <= 0 else (alvo - a[0]) / span
    x = a[1] + f * (b[1] - a[1])
    y = a[2] + f * (b[2] - a[2])
    yaw = a[3] + f * wrap(b[3] - a[3])      # pelo caminho curto
    dt_ms = 1000.0 * min(alvo - a[0], b[0] - alvo)
    return x, y, wrap(yaw), dt_ms


_CAMPO = r'%s:\s*(-?[\d.e+-]+)'


def _num(bloco, campo):
    m = re.search(_CAMPO % campo, bloco)
    return float(m.group(1)) if m else 0.0   # protobuf OMITE campos zerados


def parse_pose_v(texto, modelo):
    """(t_sim, x, y, yaw) do `modelo` numa mensagem Pose_V em texto, ou None.

    O formato de debug do protobuf OMITE campos com valor zero (um robo em
    x=0 simplesmente nao tem a linha `x:`), por isso `_num` devolve 0.0 quando
    nao acha em vez de falhar.
    """
    m = re.search(r'header\s*{\s*stamp\s*{([^}]*)}', texto)
    if not m:
        return None
    t = _num(m.group(1), 'sec') + _num(m.group(1), 'nsec') * 1e-9
    m = re.search(
        r'pose\s*{\s*name:\s*"%s"\s*id:\s*\d+\s*'
        r'position\s*{([^}]*)}\s*orientation\s*{([^}]*)}' % re.escape(modelo),
        texto)
    if not m:
        return None
    pos, ori = m.group(1), m.group(2)
    return (t, _num(pos, 'x'), _num(pos, 'y'),
            _yaw(_num(ori, 'x'), _num(ori, 'y'), _num(ori, 'z'),
                 _num(ori, 'w') or 1.0))


def nome_do_mundo(padrao='trekking'):
    """Nome do <world> dentro do .sdf apontado por SIM_WORLD (nao e' o do arquivo)."""
    caminho = os.environ.get('SIM_WORLD', '')
    if caminho and os.path.exists(caminho):
        m = re.search(r'<world name="([^"]+)"', open(caminho).read())
        if m:
            return m.group(1)
    return padrao


def pose_gazebo_cli(modelo):
    """Metodo ANTIGO (subprocesso ~1 s, DEFASADO). So' com --modo cli."""
    try:
        out = subprocess.run(['gz', 'model', '-m', modelo, '-p'],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return None
    m = _POSE_RE.search(out)
    if not m:
        return None
    xyz = [float(v) for v in m.group(1).split()]
    rpy = [float(v) for v in m.group(2).split()]
    return xyz[0], xyz[1], rpy[2]


class GT(Node):
    def __init__(self, modelo, janela):
        super().__init__('gt_trekking')
        self.modelo = modelo
        self.janela = janela
        self.odom = []          # [(t, x, y, yaw)] carimbado em tempo de SIM
        self.gt = None          # (t, x, y, yaw) da ultima verdade-terreno
        self._lock = threading.Lock()
        self.create_subscription(PoseStamped, 'trekking/pose', self._on_odom, 50)

    def le_stream(self, proc):
        """Consome o `gz topic -e` linha a linha (thread separada).

        Uma mensagem nova comeca quando aparece `header {` na coluna 0; ai a
        anterior esta completa e pode ser parseada.
        """
        buf = []
        for linha in iter(proc.stdout.readline, ''):
            if linha.startswith('header {') and buf:
                got = parse_pose_v(''.join(buf), self.modelo)
                if got:
                    with self._lock:
                        self.gt = got
                buf = []
            buf.append(linha)

    @staticmethod
    def _t(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def _on_odom(self, m):
        q = m.pose.orientation
        self.odom.append((self._t(m.header), m.pose.position.x,
                          m.pose.position.y, _yaw(q.x, q.y, q.z, q.w)))
        # buffer curto: so' precisa cercar o carimbo da verdade-terreno
        corte = self.odom[-1][0] - self.janela
        while len(self.odom) > 4 and self.odom[0][0] < corte:
            self.odom.pop(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dur', type=float, default=300.0)
    ap.add_argument('--out', default='/tmp/gt_trekking.csv')
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--modelo', default='sim_robot')
    ap.add_argument('--mundo', default=None)
    ap.add_argument('--modo', choices=['topico', 'cli'], default='topico')
    ap.add_argument('--janela', type=float, default=2.0,
                    help='s de odom guardados p/ interpolar')
    args = ap.parse_args()

    mundo = args.mundo or nome_do_mundo()
    rclpy.init()
    n = GT(args.modelo, args.janela)
    fluxo = None
    if args.modo == 'topico':
        # UM subprocesso pra corrida inteira, nao um por amostra: a latencia do
        # `gz model -p` (~1 s) era justamente o defeito que este arquivo conserta.
        fluxo = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', '/world/%s/dynamic_pose/info' % mundo],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
            bufsize=1)
        threading.Thread(target=n.le_stream, args=(fluxo,), daemon=True).start()
    t0 = time.time()
    linhas = semgt = semcasar = 0
    try:
        with open(args.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 't_sim', 'gt_x', 'gt_y', 'gt_yaw_deg',
                        'odom_x', 'odom_y', 'odom_yaw_deg',
                        'drift_xy', 'drift_yaw_deg', 'dt_match_ms'])
            prox = time.time()
            ref = None
            while time.time() - t0 < args.dur:
                rclpy.spin_once(n, timeout_sec=0.02)
                if time.time() < prox:
                    continue
                prox = time.time() + 1.0 / args.hz

                if args.modo == 'cli':
                    g = pose_gazebo_cli(args.modelo)
                    if g is None or not n.odom:
                        semgt += 1
                        continue
                    tsim = n.odom[-1][0]
                    gx, gy, gyaw = g
                    ox, oy, oyaw = n.odom[-1][1:]
                    dt_ms = -1.0            # metodo antigo: defasagem DESCONHECIDA
                else:
                    with n._lock:
                        atual = n.gt
                    if atual is None:
                        semgt += 1
                        continue
                    tsim, gx, gy, gyaw = atual
                    got = interpola_odom(n.odom, tsim)
                    if got is None:
                        semcasar += 1
                        continue
                    ox, oy, oyaw, dt_ms = got

                if ref is None:
                    ref = (gx, gy, gyaw, ox, oy, oyaw)
                gx0, gy0, gyaw0, ox0, oy0, oyaw0 = ref

                # Deriva SEM assumir offset nenhum: compara o DESLOCAMENTO desde
                # a 1a amostra, cada um no seu proprio frame de partida. Se a
                # odom fosse perfeita os dois vetores seriam identicos.
                def local(dx, dy, yaw0):
                    c, s_ = math.cos(-yaw0), math.sin(-yaw0)
                    return dx * c - dy * s_, dx * s_ + dy * c
                gdx, gdy = local(gx - gx0, gy - gy0, gyaw0)
                odx, ody = local(ox - ox0, oy - oy0, oyaw0)

                w.writerow([round(time.time() - t0, 2), round(tsim, 3),
                            round(gx, 3), round(gy, 3), round(math.degrees(gyaw), 1),
                            round(ox, 3), round(oy, 3), round(math.degrees(oyaw), 1),
                            round(math.hypot(gdx - odx, gdy - ody), 3),
                            round(math.degrees(wrap((gyaw - gyaw0) - (oyaw - oyaw0))), 1),
                            round(dt_ms, 1)])
                linhas += 1
    except KeyboardInterrupt:
        pass
    finally:
        print('gt_trekking: %d linhas | %d sem verdade-terreno | %d sem casar no tempo'
              % (linhas, semgt, semcasar))
        n.destroy_node()
        rclpy.try_shutdown()
        if fluxo:
            fluxo.terminate()
            try:
                fluxo.wait(timeout=5)
            except Exception:
                fluxo.kill()


if __name__ == '__main__':
    main()
