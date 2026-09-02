#!/usr/bin/env python3
"""Como o robô cruza a fresta A — as DUAS metades do orçamento lateral.

Por que existe (DIARIO_ARENA §2H.4): a §2G.10 concluiu que "nenhuma solução
map-relative passa 100%" comparando o orçamento lateral do vão (±0,20 m) com o
erro de pose do AMCL de **0,49 m**. Aquele 0,49 é o **máximo da volta inteira**,
e caiu no point-turn do goal 2 — NÃO na fresta. Este script mede o erro ONDE a
fresta está, e separa o que é localização do que é condução:

  (a) erro LATERAL do AMCL na boca da fresta  -> quanto a pose mente ali
  (b) verdade-terreno no plano dos batentes   -> onde e com que ângulo cruzou

O orçamento se gasta em três parcelas, e (b) mostra quais mandam:
    folga = 0,45 − |desvio lateral| − meia_largura(yaw)
com meia_largura(yaw) = 0,25·cos|yaw| + 0,25·sin|yaw| (o robô "engorda" torto).

Uso:  travessia_fresta.py <tag> [<tag> ...]     (tags em log/sim_ab/)
      travessia_fresta.py --autoteste
"""
import csv
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import erro_pose as E                                    # noqa: E402

SP = E.SP
# fresta A (tools/gera_arena_galpao.py:43): batentes (7,50;1,80) e (7,50;2,70).
JAMB_X, CY, MEIO_VAO = 7.50, 2.25, 0.45
X_BOCA = (6.40, 7.50)      # da zona do door_crossing (r=1,1) até o plano
Y_PERNA = (1.0, 3.5)       # só a perna que passa pela A
ROBO_MEIA = 0.25           # meia-largura (0,50 m roda-a-roda)


def meia_largura(yaw_deg):
    """Meia-largura EFETIVA de um 0,50×0,50 atravessando com |yaw| de erro."""
    a = math.radians(abs(yaw_deg))
    return ROBO_MEIA * math.cos(a) + ROBO_MEIA * math.sin(a)


def erro_lateral_na_boca(tag):
    """(n, |lat| mediana, p90, max) do AMCL contra o Gazebo na boca da fresta.
    Reusa o alinhamento de relógios do erro_pose.py (os dois CSVs não
    compartilham base de tempo)."""
    gt, am = E.series(tag)
    melhor, off, o = float('inf'), 0.0, 0.0
    while o <= E.OFF_MAX:
        my, _ = E.avalia(gt, am, o)
        if my < melhor:
            melhor, off = my, o
        o += E.OFF_PASSO
    lat = []
    for t, x, y, _yw in am:
        s = E._vizinho(gt, t + off)
        if s is None:
            continue
        if not (X_BOCA[0] <= s[1] <= X_BOCA[1]):
            continue
        if not (Y_PERNA[0] <= s[2] <= Y_PERNA[1]):
            continue
        lat.append(abs(y - s[2]))
    if not lat:
        return None
    lat.sort()
    return (len(lat), st.median(lat), lat[int(0.9 * len(lat))], lat[-1])


def _le_gt(tag):
    with open(f'{SP}/{tag}/colisao.csv') as f:
        return [(float(r['t']), float(r['x']), float(r['y']),
                 float(r['yaw_deg']), float(r['folga_min']), r['obj'], r['evento'])
                for r in csv.DictReader(f)]


def travessias(linhas):
    """Interpola o instante em que a verdade-terreno cruza o plano dos batentes.
    Devolve [(t, y, yaw_deg)] — vazio se a volta CONTORNOU."""
    out = []
    for i in range(1, len(linhas)):
        x0, x1 = linhas[i - 1][1], linhas[i][1]
        if not (Y_PERNA[0] <= linhas[i][2] <= Y_PERNA[1]):
            continue
        if (x0 - JAMB_X) * (x1 - JAMB_X) >= 0:
            continue
        f = (JAMB_X - x0) / (x1 - x0)
        y = linhas[i - 1][2] + f * (linhas[i][2] - linhas[i - 1][2])
        yaw = linhas[i - 1][3] + f * (linhas[i][3] - linhas[i - 1][3])
        out.append((linhas[i][0], y, yaw))
    return out


def eventos_na_fresta(linhas):
    return [(t, obj, ev, fol) for t, _x, _y, _yw, fol, obj, ev in linhas
            if ev and obj.startswith('A_fresta90')]


def folga(desvio, yaw_deg):
    return MEIO_VAO - abs(desvio) - meia_largura(yaw_deg)


def mede(tag):
    linhas = _le_gt(tag)
    cz = travessias(linhas)
    ev = eventos_na_fresta(linhas)
    lat = erro_lateral_na_boca(tag)
    if not cz:
        return dict(tag=tag, cruzou=False, eventos=len(ev), amcl=lat)
    t, y, yaw = cz[0]
    d = y - CY
    return dict(tag=tag, cruzou=True, y=y, desvio=d, yaw=yaw,
                meia=meia_largura(yaw), folga=folga(d, yaw),
                eventos=len(ev), amcl=lat)


def autoteste():
    """Sensível por construção: além de conferir a conta, prova que ela REAGE
    às duas parcelas (desvio e yaw) — senão passaria numa versão que ignora uma
    delas (é o defeito que a §2G.10 cometeu: olhou só uma metade)."""
    assert abs(meia_largura(0.0) - 0.25) < 1e-9
    # 10° engorda ~3,7 cm por lado
    assert abs(meia_largura(10.0) - (0.25 * math.cos(math.radians(10))
                                     + 0.25 * math.sin(math.radians(10)))) < 1e-12
    assert abs(meia_largura(10.0) - 0.2896) < 5e-4, meia_largura(10.0)
    # reta e centrado: sobra o orçamento inteiro
    assert abs(folga(0.0, 0.0) - 0.20) < 1e-9
    # a conta REAGE ao desvio...
    assert folga(0.12, 0.0) < folga(0.0, 0.0) - 0.11
    # ...e REAGE ao yaw, sozinho
    assert folga(0.0, 10.0) < folga(0.0, 0.0) - 0.03
    # a pose medida da noguard3 tem que dar folga POSITIVA mas pequena (raspou
    # com o oráculo medindo 0,000 — a conta geométrica não inclui o vies do
    # oráculo, então aqui só se exige "apertado")
    f = folga(0.121, -10.7)
    assert 0.02 < f < 0.05, f
    # travessias(): cruzamento sintético do plano, indo em +x
    lin = [(0.0, 7.0, 2.30, -5.0, 1.0, 'x', ''),
           (0.1, 8.0, 2.20, -5.0, 1.0, 'x', '')]
    cz = travessias(lin)
    assert len(cz) == 1 and abs(cz[0][1] - 2.25) < 1e-9, cz
    # e NÃO inventa cruzamento em quem fica de um lado só (o contorno)
    assert travessias([(0.0, 7.0, 2.3, 0, 1, 'x', ''),
                       (0.1, 7.2, 2.3, 0, 1, 'x', '')]) == []
    print('[travessia_fresta] autoteste OK')
    return 0


def main(argv):
    if not argv or argv[0] in ('-h', '--help'):
        print(__doc__)
        return 1
    if argv[0] == '--autoteste':
        return autoteste()
    print(f'{"volta":16s} {"y_cruz":>7s} {"desvio":>7s} {"yaw":>7s} '
          f'{"meia_l":>7s} {"folga":>7s} {"evt":>4s} | AMCL lateral na boca '
          f'(n / med / p90 / max, cm)')
    linhas_ok = []
    for tag in argv:
        try:
            m = mede(tag)
        except (FileNotFoundError, IndexError):
            print(f'{tag:16s} -- sem dados')
            continue
        a = m['amcl']
        sa = (f'{a[0]:5d} {a[1]*100:6.1f} {a[2]*100:6.1f} {a[3]*100:6.1f}'
              if a else '   -- nao passou pela boca')
        if not m['cruzou']:
            print(f'{tag:16s} {"-- CONTORNOU":>39s} {m["eventos"]:4d} | {sa}')
            continue
        linhas_ok.append(m)
        print(f'{tag:16s} {m["y"]:7.3f} {m["desvio"]*100:+7.1f} {m["yaw"]:7.1f} '
              f'{m["meia"]:7.3f} {m["folga"]*100:7.1f} {m["eventos"]:4d} | {sa}')
    if linhas_ok:
        yaws = [m['yaw'] for m in linhas_ok]
        desv = [abs(m['desvio']) for m in linhas_ok]
        fol = [m['folga'] for m in linhas_ok]
        print(f'\n{len(linhas_ok)} travessias: |desvio| {min(desv)*100:.1f}-'
              f'{max(desv)*100:.1f} cm | yaw {min(yaws):.1f} a {max(yaws):.1f}° '
              f'(mediana {st.median(yaws):.1f}) | folga {min(fol)*100:.1f}-'
              f'{max(fol)*100:.1f} cm')
        print(f'orçamento: ±20,0 cm reto | align_yaw do door_crossing = 3,0° '
              f'| align_lat = 8,0 cm')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
