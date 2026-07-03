#!/usr/bin/env python3
"""pause_budget — orçamento do TEMPO PARADO com goal ativo (o vilão da fluidez).

Por que existe (2026-07-03, dono): "ele para bastante ainda... tem momentos que
ele erra à toa e perde tempo esperando à toa". A régua: NÃO baixar limiar de
certeza nenhum — só cortar espera onde a decisão JÁ foi tomada. Este script lê
o freeze_capture.csv (cadeia de velocidade + estados) e atribui cada segundo
parado-com-goal a UMA causa, por precedência (a mais a jusante que explica):

  guard_hold    motion_guard em blocked/slowing segurando comando que existia
  collision     auto_vel_raw comandava, auto_vel ~0 (collision_monitor cortou)
  wz_engolido   cmd_vel manda giro (|wz|>=1.0) e o odom não gira (zona-morta/
                física/rodas) — o robô DECIDIU girar e não acontece
  vx_zona_morta cmd_vel manda 0<vx<0.20 e o robô não anda (comando fraco)
  vx_sem_efeito cmd_vel manda vx>=0.20 e o robô não anda (encalhe físico)
  unstuck       unstuck_vel comandando (manobra em curso) com robô parado
  mux_gap       follow_vel comandava e auto_vel_pre ~0 (mux não repassa)
  follower_off  follow_vel ~0 (o driver decidiu não comandar: replan/alvo/
                chegando) — inclui follow_state na quebra fina
  outro         parado sem nenhuma assinatura acima

Uso:  bin/pause_budget.py controle_web/logs/freeze_capture.csv
      (aceita o CSV antigo de 6 colunas; as categorias novas viram 'outro')

Read-only, sem ROS. Eu (assistente) rodo e leio — o dono só roda a rota.
"""
import csv
import math
import sys
from collections import defaultdict

# limiares (casados com o robô: zona-morta giro 1.7, min_speed 0.22)
STOP_VX = 0.05      # |vx| odom abaixo disso = não translada
STOP_WZ = 0.15      # |wz| odom abaixo disso = não gira
CMD_VX = 0.05       # comando linear "existe"
CMD_WZ = 0.50       # comando de giro "existe"
WZ_STRONG = 1.0     # comando de giro pra valer (deveria mexer o robô)
STALE = 0.6         # s sem msg num tópico -> valor considerado zerado


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


class Track:
    """último (vx,wz,t) de um tópico; zera se ficar velho (nó calou)."""

    def __init__(self):
        self.vx = self.wz = 0.0
        self.t = -math.inf

    def set(self, t, vx, wz):
        self.t, self.vx, self.wz = t, vx, wz

    def at(self, t):
        if t - self.t > STALE:
            return 0.0, 0.0
        return self.vx, self.wz


def classify(tr, states, t):
    """causa do instante parado t (precedência: mais a jusante primeiro)."""
    fv_x, fv_w = tr['follow_vel'].at(t)
    pre_x, pre_w = tr['auto_vel_pre'].at(t)
    raw_x, raw_w = tr['auto_vel_raw'].at(t)
    av_x, av_w = tr['auto_vel'].at(t)
    cv_x, cv_w = tr['cmd_vel'].at(t)
    us_x, us_w = tr['unstuck_vel'].at(t)

    if abs(cv_w) >= WZ_STRONG:
        return 'wz_engolido'
    if CMD_VX < abs(cv_x) < 0.20:
        return 'vx_zona_morta'
    if abs(cv_x) >= 0.20:
        return 'vx_sem_efeito'     # comando cheio e o robô não anda: encalhe
                                   # físico/rodas (não é decisão de ninguém)
    if abs(us_x) > 0.03 or abs(us_w) > 0.3:
        return 'unstuck'
    if states.get('guard_state') in ('blocked', 'slowing') and \
            (abs(pre_x) > CMD_VX or abs(pre_w) > CMD_WZ) and \
            abs(raw_x) <= CMD_VX and abs(raw_w) <= CMD_WZ:
        return 'guard_hold'
    if (abs(raw_x) > CMD_VX or abs(raw_w) > CMD_WZ) and \
            abs(av_x) <= CMD_VX and abs(av_w) <= CMD_WZ:
        return 'collision'
    if (abs(fv_x) > CMD_VX or abs(fv_w) > CMD_WZ) and \
            abs(pre_x) <= CMD_VX and abs(pre_w) <= CMD_WZ:
        return 'mux_gap'
    if abs(fv_x) <= CMD_VX and abs(fv_w) <= CMD_WZ:
        return 'follower_off[%s]' % states.get('follow_state', '?')
    return 'outro'


def main(path):
    tr = defaultdict(Track)
    states = {}
    goal = True          # CSV antigo não grava goal_active -> conta tudo
    goal_seen = False
    stopped_since = None
    budget = defaultdict(float)
    episodes = []          # (t_ini, dur, {causa: s})
    cur_ep = None
    last_t = None
    t0 = None
    total_goal = 0.0

    with open(path, newline='') as f:
        for row in csv.reader(f):
            if not row or row[0] == 't_wall':
                continue
            t = _f(row[0])
            if math.isnan(t):
                continue
            if t0 is None:
                t0 = t
            topic = row[1]
            extra = row[6] if len(row) > 6 else ''
            if topic in ('follow_state', 'guard_state'):
                states[topic] = extra
                continue
            if topic == 'goal_active':
                goal = (extra == '1')
                goal_seen = True
                continue
            vx, wz = _f(row[2]), _f(row[3])
            if topic == 'odom':
                stopped = abs(vx) < STOP_VX and abs(wz) < STOP_WZ
                if last_t is not None and goal:
                    dt = min(t - last_t, STALE)
                    total_goal += dt
                    if stopped and stopped_since is not None:
                        cause = classify(tr, states, t)
                        budget[cause] += dt
                        if cur_ep is None:
                            cur_ep = [stopped_since, defaultdict(float)]
                        cur_ep[1][cause] += dt
                if stopped:
                    if stopped_since is None:
                        stopped_since = t
                else:
                    if cur_ep is not None:
                        dur = sum(cur_ep[1].values())
                        if dur >= 1.0:
                            episodes.append((cur_ep[0], dur, dict(cur_ep[1])))
                        cur_ep = None
                    stopped_since = None
                last_t = t
            elif not math.isnan(vx):
                tr[topic].set(t, vx, wz)

    if cur_ep is not None:
        dur = sum(cur_ep[1].values())
        if dur >= 1.0:
            episodes.append((cur_ep[0], dur, dict(cur_ep[1])))

    tot_stop = sum(budget.values())
    print('janela total do CSV : %.0fs' % ((last_t or 0) - (t0 or 0)))
    print('tempo com goal ativo: %.0fs' % total_goal)
    print('PARADO com goal     : %.0fs (%.0f%% do tempo de missão)'
          % (tot_stop, 100 * tot_stop / total_goal if total_goal else 0))
    print('\n== ORÇAMENTO (quem segura o robô) ==')
    for cause, s in sorted(budget.items(), key=lambda kv: -kv[1]):
        print('  %-28s %6.1fs  (%4.1f%%)'
              % (cause, s, 100 * s / tot_stop if tot_stop else 0))
    print('\n== EPISÓDIOS >= 3s (os vilões) ==')
    big = [e for e in sorted(episodes, key=lambda e: -e[1]) if e[1] >= 3.0]
    for t_ini, dur, causes in big[:15]:
        main_c = max(causes, key=causes.get)
        mix = ' '.join('%s=%.1f' % (c, s)
                       for c, s in sorted(causes.items(), key=lambda kv: -kv[1]))
        print('  t+%5.0fs  %5.1fs  %-20s (%s)'
              % (t_ini - t0, dur, main_c, mix))
    if not big:
        print('  (nenhum)')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
