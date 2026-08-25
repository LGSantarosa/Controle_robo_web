#!/usr/bin/env python3
"""Resumo de um lote do banco minimo: ELE PARA SEMPRE NO MESMO PONTO?

Separa as duas perguntas que costumam ser confundidas:
  REPETIBILIDADE = quanto os trials espalham entre si (desvio ao centroide).
  EXATIDAO       = quanto o centroide esta longe do ponto ideal (vies).
Um robo pode ser repetivel e errado (vies) — e ai a culpa e de calibracao, nao
de ruido. O inverso (certo na media, espalhado) e ruido/derrapagem.
"""
import argparse, csv, math, sys

ap = argparse.ArgumentParser()
ap.add_argument('csv')
ap.add_argument('--standoff', type=float, default=1.2)
ap.add_argument('--spawn', default='2.0,2.5')
ap.add_argument('--cone-odom', default='4.0,0.0')
a = ap.parse_args()

sx, sy = [float(v) for v in a.spawn.split(',')]
cxo, cyo = [float(v) for v in a.cone_odom.split(',')]
cone_w = (sx + cxo, sy + cyo)                       # cone no mundo
ideal = (cone_w[0] - a.standoff, cone_w[1])         # onde ele DEVERIA parar

rows = list(csv.DictReader(open(a.csv)))
ok = [r for r in rows if r['fim'] == 'concluiu' and r['x']]
ruins = [r for r in rows if r not in ok]

print(f"trials: {len(rows)}   concluiram: {len(ok)}   problemas: {len(ruins)}")
for r in ruins:
    print(f"  trial {r['trial']}: {r['fim']}")
if not ok:
    sys.exit(0)

xs = [float(r['x']) for r in ok]
ys = [float(r['y']) for r in ok]
yaws = [float(r['yaw_deg']) for r in ok]
durs = [float(r['dur_s']) for r in ok if r['dur_s']]
n = len(xs)
mx, my = sum(xs)/n, sum(ys)/n
def dp(v, m):
    return math.sqrt(sum((x-m)**2 for x in v)/n) if n > 1 else 0.0
rad = [math.hypot(x-mx, y-my) for x, y in zip(xs, ys)]
span = max((math.hypot(x1-x2, y1-y2)
            for i, (x1, y1) in enumerate(zip(xs, ys))
            for x2, y2 in list(zip(xs, ys))[i+1:]), default=0.0)
myaw = sum(yaws)/n

print()
print(f"parada media (mundo): ({mx:.3f}, {my:.3f})   yaw {myaw:+.1f}°")
print(f"ponto ideal:          ({ideal[0]:.3f}, {ideal[1]:.3f})   "
      f"cone em ({cone_w[0]:.2f}, {cone_w[1]:.2f})")
print()
print("REPETIBILIDADE (espalhamento entre trials)")
print(f"  desvio x {dp(xs,mx)*100:6.1f} cm    desvio y {dp(ys,my)*100:6.1f} cm")
print(f"  raio medio ao centroide {sum(rad)/n*100:.1f} cm   pior {max(rad)*100:.1f} cm")
print(f"  maior distancia entre dois trials: {span*100:.1f} cm")
print(f"  desvio de yaw: {dp(yaws,myaw):.1f}°")
print()
vx, vy = mx-ideal[0], my-ideal[1]
print("EXATIDAO (vies do centroide vs ideal)")
print(f"  dx {vx*100:+.1f} cm   dy {vy*100:+.1f} cm   |vies| {math.hypot(vx,vy)*100:.1f} cm")
d_cone = [math.hypot(x-cone_w[0], y-cone_w[1]) for x, y in zip(xs, ys)]
print(f"  distancia ao CONE: media {sum(d_cone)/n*100:.0f} cm  "
      f"min {min(d_cone)*100:.0f} cm  (raio do robo ~ 26 cm)")
if durs:
    print(f"  duracao: media {sum(durs)/len(durs):.1f} s  "
          f"min {min(durs):.1f}  max {max(durs):.1f}")
