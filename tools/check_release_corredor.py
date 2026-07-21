#!/usr/bin/env python3
"""Checa a run do RELEASE POR CORREDOR (07-21) no motion_guard.csv.

Uso: python3 tools/check_release_corredor.py [caminho_do_csv]
Default: controle_web/logs/motion_guard.csv

O que procura (simples, só pra ver se aconteceu de novo):
  - FALSO-POSITIVO: maior sequência 'blocked' com n_moving=0 (ninguém em
    movimento). Antes do fix a run 07-20 tinha ~24s. Depois deve cair pra
    poucos segundos (só o clear_time). >8s = suspeito, olhar.
  - PROBE: episódios de micro-passo (state 'probing') — quantos e quanto tempo.
  - PASSTHROUGH: guard caiu pra pass-through (scan/TF sumiu) — quantas linhas.
  - Sanidade: total de linhas, janela de tempo, contagem por estado.
"""
import csv
import sys
from collections import Counter

path = sys.argv[1] if len(sys.argv) > 1 else 'controle_web/logs/motion_guard.csv'

rows = []
with open(path, newline='') as f:
    r = csv.DictReader(f)
    for row in r:
        try:
            rows.append((float(row['t']), row['state'], row['n_moving']))
        except (ValueError, KeyError):
            continue

if not rows:
    print('CSV vazio ou sem dados:', path)
    sys.exit(1)

t0, tN = rows[0][0], rows[-1][0]
states = Counter(s for _, s, _ in rows)

# maiores sequências de 'blocked' com n_moving == 0 (o falso-positivo)
fp = []                       # (dur, t_ini_rel, n_amostras)
cur_start = None
cur_n = 0
prev_t = None
for t, s, nm in rows:
    is_fp = (s == 'blocked' and nm in ('0', ''))
    if is_fp:
        if cur_start is None:
            cur_start = t
        cur_n += 1
        prev_t = t
    else:
        if cur_start is not None:
            fp.append((prev_t - cur_start, cur_start - t0, cur_n))
            cur_start, cur_n = None, 0
if cur_start is not None:
    fp.append((prev_t - cur_start, cur_start - t0, cur_n))
fp.sort(reverse=True)

# episódios de probe (state 'probing' contíguo)
probes = []
cur_start = None
prev_t = None
for t, s, _ in rows:
    if s == 'probing':
        if cur_start is None:
            cur_start = t
        prev_t = t
    else:
        if cur_start is not None:
            probes.append((cur_start - t0, prev_t - cur_start))
            cur_start = None
if cur_start is not None:
    probes.append((cur_start - t0, prev_t - cur_start))

print('=== run %s ===' % path)
print('linhas: %d   janela: %.0fs' % (len(rows), tN - t0))
print('estados:', dict(states))
print()
print('--- FALSO-POSITIVO (blocked com n_moving=0) ---')
if not fp:
    print('  nenhuma sequência blocked-sem-movimento. ✅')
else:
    print('  maior: %.1fs  (começa em rel %.0fs, %d amostras)'
          % (fp[0][0], fp[0][1], fp[0][2]))
    for dur, ini, n in fp[:5]:
        flag = '  <-- SUSPEITO (>8s)' if dur > 8.0 else ''
        print('    %5.1fs @ rel %.0fs%s' % (dur, ini, flag))
    if fp[0][0] <= 3.0:
        print('  veredito: ✅ curto (~clear_time), fix parece ter pego')
    elif fp[0][0] <= 8.0:
        print('  veredito: 🟡 médio — olhar o trecho no CSV')
    else:
        print('  veredito: 🔴 ainda trava longo sem ninguém — investigar')
print()
print('--- PROBE (micro-passo) ---')
if not probes:
    print('  0 episódios de probe.')
else:
    print('  %d episódio(s):' % len(probes))
    for ini, dur in probes:
        print('    rel %.0fs por %.1fs' % (ini, dur))
print()
print('--- PASSTHROUGH (guard caiu: scan/TF) ---')
n_pt = states.get('passthrough', 0)
print('  %d linhas%s' % (n_pt, '  <-- olhar se muitas' if n_pt > 20 else ''))
