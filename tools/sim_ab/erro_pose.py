#!/usr/bin/env python3
"""Erro de pose AMCL x verdade-terreno do Gazebo, por volta do harness A/B.

As duas séries de uma volta não compartilham relógio:
  - `follow_debug.csv` (o seguidor) grava a pose do **AMCL** em relógio de parede;
  - `colisao.csv` (o oráculo) grava a pose do **Gazebo** em tempo relativo.

O alinhamento é ESTIMADO, não medido: varre o offset e fica com o que minimiza o
erro MEDIANO de yaw. É o mesmo método usado à mão na §2B.4 do DIARIO_ARENA.md
(lá o offset foi chutado em +3,45 s / +5,8 s); aqui ele é varrido, e o erro de
yaw residual é impresso junto para quem lê poder desconfiar do alinhamento.

Uso:  erro_pose.py <tag> [<tag> ...]     (tags em log/sim_ab/)
      erro_pose.py --autoteste
"""
import csv
import math
import os
import statistics as st
import sys

SP = os.environ.get('SIM_AB_DIR',
                    '/home/rbe-luis/Workspace/Controle_robo_web/log/sim_ab')
JANELA = 0.20      # s: casamento por vizinho mais próximo; mais longe = descarta
OFF_MAX = 25.0     # s: o boot do harness nunca passou disso
OFF_PASSO = 0.05


def _le(caminho, cols):
    L = []
    with open(caminho) as f:
        for l in csv.DictReader(f):
            L.append(tuple(float(l[c]) for c in cols))
    return L


def series(tag):
    """(gt, amcl) — as duas em (t, x, y, yaw_deg), amcl com t começando em 0."""
    gt = _le(f'{SP}/{tag}/colisao.csv', ('t', 'x', 'y', 'yaw_deg'))
    am = _le(f'{SP}/{tag}/follow_debug.csv', ('t', 'x', 'y', 'yaw_deg'))
    t0 = am[0][0]
    return gt, [(t - t0, x, y, yw) for t, x, y, yw in am]


def _vizinho(serie, t):
    lo, hi = 0, len(serie) - 1
    while lo < hi:
        m = (lo + hi) // 2
        if serie[m][0] < t:
            lo = m + 1
        else:
            hi = m
    cands = [serie[i] for i in (lo - 1, lo, lo + 1) if 0 <= i < len(serie)]
    melhor = min(cands, key=lambda s: abs(s[0] - t))
    return melhor if abs(melhor[0] - t) < JANELA else None


def _dyaw(a, b):
    return abs((a - b + 180) % 360 - 180)


def avalia(gt, am, off):
    """(erro mediano de yaw, lista de erros de posição) para um offset dado."""
    ey, ep = [], []
    for t, x, y, yw in am:
        s = _vizinho(gt, t + off)
        if s is None:
            continue
        ey.append(_dyaw(yw, s[3]))
        ep.append(math.hypot(x - s[1], y - s[2]))
    return (st.median(ey) if ey else float('inf')), ep


def mede(tag):
    gt, am = series(tag)
    melhor, off = float('inf'), 0.0
    o = 0.0
    while o <= OFF_MAX:
        my, _ = avalia(gt, am, o)
        if my < melhor:
            melhor, off = my, o
        o += OFF_PASSO
    yaw_med, ep = avalia(gt, am, off)
    ep.sort()
    q = lambda p: ep[min(len(ep) - 1, int(p * len(ep)))]
    return dict(tag=tag, offset=off, yaw_med=yaw_med, n=len(ep),
                mediana=q(0.5), p90=q(0.9), maximo=ep[-1])


def autoteste():
    """Série sintética com offset e erro CONHECIDOS.

    Sensível por construção (BO 72): além de afirmar que acha o offset certo,
    afirma que com o offset ERRADO (0 s) o erro medido é muito maior — senão o
    teste passaria numa implementação que ignora o alinhamento.
    """
    OFF, ERRO = 4.30, 0.12
    gt = [(i * 0.05, i * 0.02, 0.0, (i * 0.5) % 360) for i in range(2000)]
    am = [(t - OFF, x + ERRO, y, yw) for t, x, y, yw in gt if t >= OFF]
    achado, ep = None, None
    o, melhor = 0.0, float('inf')
    while o <= OFF_MAX:
        my, _ = avalia(gt, am, o)
        if my < melhor:
            melhor, achado = my, o
        o += OFF_PASSO
    assert abs(achado - OFF) <= OFF_PASSO, f'offset {achado} != {OFF}'
    _, ep = avalia(gt, am, achado)
    med = st.median(ep)
    assert abs(med - ERRO) < 0.005, f'erro {med} != {ERRO}'
    _, ep0 = avalia(gt, am, 0.0)
    assert st.median(ep0) > 10 * ERRO, (
        'com o offset errado o erro tinha que explodir — se não explode, o '
        'alinhamento não está sendo usado')
    print('[erro_pose] autoteste OK '
          f'(offset {achado:.2f}s, erro {med*100:.1f} cm, '
          f'desalinhado {st.median(ep0)*100:.0f} cm)')
    return 0


def main():
    if '--autoteste' in sys.argv:
        return autoteste()
    tags = [a for a in sys.argv[1:] if not a.startswith('-')]
    if not tags:
        print(__doc__)
        return 2
    for tag in tags:
        r = mede(tag)
        print(f"{r['tag']}: offset {r['offset']:.2f}s  "
              f"yaw_med {r['yaw_med']:.2f}deg  n={r['n']}  "
              f"erro_pos mediana {r['mediana']*100:.1f} cm  "
              f"p90 {r['p90']*100:.1f}  max {r['maximo']*100:.1f}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
