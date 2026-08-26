#!/usr/bin/env python3
"""A ULTIMA corrida da tag 'boa' foi limpa? exit 0 = sim. Ver ate_ficar_bom.sh."""
import csv, glob, json, math, sys

wps = json.load(open('maps/routes/trekking/rota2.json'))['waypoints']
tag = sys.argv[1] if len(sys.argv) > 1 else 'boa'
csvs = [f for f in sorted(glob.glob('log/cone_assoc/%s_*_t1.csv' % tag))
        if not f.endswith('_gt.csv')]
jsl = sorted(glob.glob('log/cone_assoc/%s_*.jsonl' % tag))
if not csvs or not jsl:
    print('  sem dado da corrida'); sys.exit(1)

d = json.loads(open(jsl[-1]).read().strip().split('\n')[-1])
R = list(csv.DictReader(open(csvs[-1])))

# giros de verdade (>=2 graus): o tick da troca de canto e' logado como turn
eps, cur = [], None
for r in R:
    if r['state'] == 'turn':
        if cur is None:
            cur = {'y0': float(r['yaw_deg'])}
        cur['y1'] = float(r['yaw_deg'])
    elif cur:
        eps.append(cur); cur = None
if cur:
    eps.append(cur)
giros = [round((e['y1'] - e['y0'] + 180) % 360 - 180, 1) for e in eps
         if abs((e['y1'] - e['y0'] + 180) % 360 - 180) >= 2.0]

visitas = {}
for r in R:
    if r['event'] == 'arrive':
        w = wps[int(r['idx'])]
        visitas[int(r['idx'])] = math.hypot(float(r['x']) - w['x'],
                                            float(r['y']) - w['y'])

erro = d.get('erro_final_cm')
ok_fim = d.get('fim') == 'concluiu'
ok_erro = erro is not None and erro < 60
ok_giros = len(giros) == 4
ok_wps = len(visitas) == len(wps) and all(v < 0.5 for v in visitas.values())

print('  giros=%d %s | pontos=%s | erro=%s cm | %s' % (
    len(giros), giros,
    {k: round(v, 2) for k, v in sorted(visitas.items())}, erro,
    'LIMPA' if (ok_fim and ok_erro and ok_giros and ok_wps) else 'nao'))
for nome, ok in [('concluiu', ok_fim), ('sem batida', ok_erro),
                 ('4 giros', ok_giros), ('passou nos 2 pontos', ok_wps)]:
    if not ok:
        print('    falhou: %s' % nome)
sys.exit(0 if (ok_fim and ok_erro and ok_giros and ok_wps) else 1)
