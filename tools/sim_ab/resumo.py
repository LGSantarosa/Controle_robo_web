#!/usr/bin/env python3
"""Resumo comparativo de todas as voltas do A/B + o que o nav2 reclamou."""
import json, os, re, sys, glob
SP = os.environ.get("SIM_AB_DIR", "/home/rbe-luis/Workspace/Controle_robo_web/log/sim_ab")

def colisoes(tag):
    """quantos eventos de contato real (ground truth), não proxy de laser"""
    import csv as _csv
    f = f"{SP}/{tag}/colisao.csv"
    if not os.path.exists(f):
        return dict(bateu='?', raspou='?', folga_min=None)
    linhas = list(_csv.DictReader(open(f)))
    if not linhas:
        return dict(bateu='?', raspou='?', folga_min=None)
    ev = [l for l in linhas if l['evento'] == 'COLISAO']
    # agrupa amostras contíguas num evento só
    n = 0; ant = -9
    for l in ev:
        t = float(l['t'])
        if t - ant > 1.0: n += 1
        ant = t
    rasp = sum(1 for l in linhas if l['evento'] == 'raspao')
    return dict(bateu=n, raspou=rasp,
                folga_min=min(float(l['folga_min']) for l in linhas))


def diag(tag):
    """conta o que o nav2 reclamou na volta (não está no result.json)"""
    log = f"{SP}/{tag}/nav2.log"
    if not os.path.exists(log): return {}
    t = open(log, errors='ignore').read()
    return dict(
        plan_fail=len(re.findall(r'failed to plan', t)),
        start_obst=len(re.findall(r'start or goal pose are an obstacle', t)),
        backup=len(re.findall(r'Running backup', t)),
        spin=len(re.findall(r'Running spin', t)),
        wait=len(re.findall(r'Running wait', t)),
        no_progress=len(re.findall(r'Failed to make progress', t)),
    )

def agg(tag):
    p = f"{SP}/{tag}/result.json"
    if not os.path.exists(p): return None
    x = json.load(open(p)); g = x['goals']
    d = dict(ok=sum(1 for i in g if i['status'] == 'OK'), n=len(g),
             total=x['total_s'],
             parado=sum(i['parado'] for i in g),
             unstuck=sum(i['unstuck'] for i in g),
             minsc=min([i['min_scan'] for i in g if i['min_scan']] or [0]),
             falhou=[f"{i['goal']}:{i['status']}" for i in g if i['status'] != 'OK'])
    for k in ('LIMIT', 'STOP', 'APPROACH'):
        d[k] = sum(i['cm'].get(k, 0) for i in g)
    d.update(diag(tag))
    d.update(colisoes(tag))
    return d

VOLTAS = [('1 baseline (robot_nav)', 'baseline'), ('2 caixa fixa', 'destemido'),
          ('3 approach 0.3', 'destemido2'), ('4 limit frontal', 'destemido3'),
          ('5 limit + infl 0.45', 'destemido4'), ('6', 'destemido5'),
          ('7', 'destemido6'), ('8', 'destemido7')]
rows = [(n, agg(t)) for n, t in VOLTAS]
rows = [(n, d) for n, d in rows if d]
LIN = [('ok', 'goals OK'), ('total', 'tempo total (s)'), ('parado', 'parado (s)'),
       ('unstuck', 'unstuck (s)'), ('LIMIT', 'LIMIT (s)'), ('STOP', 'STOP (s)'),
       ('APPROACH', 'APPROACH (s)'), ('minsc', 'menor scan (m)'),
       ('plan_fail', 'falhas de plano'), ('backup', 'recovery BackUp'),
       ('spin', 'recovery Spin'), ('wait', 'recovery Wait'),
       ('no_progress', '"no progress"'), ('bateu', 'COLISÕES (real)'),
       ('raspou', 'amostras raspando'), ('folga_min', 'menor folga (m)')]
w = 22
print(f"{'':<20}" + "".join(f"{n:>{w}}" for n, _ in rows))
for k, lbl in LIN:
    vals = []
    for _, d in rows:
        v = d.get(k)
        vals.append(f"{v:>{w}.1f}" if isinstance(v, float) else f"{v:>{w}}")
    print(f"{lbl:<20}" + "".join(vals))
print()
for n, d in rows:
    if d['falhou']:
        print(f"  {n}: NAO chegou em {d['falhou']}")
