#!/usr/bin/env python3
"""Métricas de zigue-zague do follow_debug.csv (mesmas réguas das análises 07-09).

Divide o CSV em 'pernas' (segmentos contíguos com goal ativo, gap >5s separa) e
por perna reporta:
  - duração, distância, vx médio
  - % do tempo dirigindo com carrot ESTICADO (la > 1.0)
  - episódios de turning: total, giros PEQUENOS (<10°), mediana da amplitude
  - vai-e-volta: pares de turnings consecutivos com sinais opostos (<8s entre eles)
  - inversões de sinal de wz por minuto (wz != 0)
"""
import csv, math, sys
from statistics import median

def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                rows.append(dict(t=float(r['t']), state=r['state'],
                                 x=float(r['x']), y=float(r['y']),
                                 yaw=float(r['yaw_deg']), herr=float(r['herr_deg']),
                                 vx=float(r['vx']), wz=float(r['wz']),
                                 la=float(r['la'])))
            except (ValueError, KeyError):
                continue
    return rows

def segments(rows, gap=5.0, min_len=30.0):
    segs, cur = [], []
    for r in rows:
        if r['state'] in ('idle', 'done', ''):
            continue
        if cur and r['t'] - cur[-1]['t'] > gap:
            segs.append(cur); cur = []
        cur.append(r)
    if cur:
        segs.append(cur)
    return [s for s in segs if s[-1]['t'] - s[0]['t'] >= min_len]

def ang_diff(a, b):
    d = a - b
    while d > 180: d -= 360
    while d < -180: d += 360
    return d

def analyze(seg):
    t0, t1 = seg[0]['t'], seg[-1]['t']
    dur = t1 - t0
    dist = sum(math.hypot(b['x']-a['x'], b['y']-a['y'])
               for a, b in zip(seg, seg[1:]))
    drive = [r for r in seg if r['state'] == 'driving']
    far = sum(1 for r in drive if r['la'] > 1.0)
    pct_far = 100.0 * far / len(drive) if drive else 0.0

    # episódios de turning
    eps, cur = [], None
    for r in seg:
        if r['state'] == 'turning':
            if cur is None:
                cur = dict(t0=r['t'], yaw0=r['yaw'], wz=[])
            cur['t1'] = r['t']; cur['yaw1'] = r['yaw']; cur['wz'].append(r['wz'])
        elif cur is not None:
            eps.append(cur); cur = None
    if cur is not None:
        eps.append(cur)
    amps = [abs(ang_diff(e['yaw1'], e['yaw0'])) for e in eps]
    small = sum(1 for a in amps if a < 10.0)
    # vai-e-volta: episódios consecutivos com sentido oposto e <8s de intervalo
    vaivolta = 0
    for a, b in zip(eps, eps[1:]):
        sa = median(a['wz']) if a['wz'] else 0.0
        sb = median(b['wz']) if b['wz'] else 0.0
        if sa * sb < 0 and b['t0'] - a['t1'] < 8.0:
            vaivolta += 1
    turn_time = sum(e['t1'] - e['t0'] for e in eps)

    # inversões de wz (só amostras com wz relevante)
    flips, last = 0, 0
    for r in seg:
        s = 1 if r['wz'] > 0.05 else (-1 if r['wz'] < -0.05 else 0)
        if s and last and s != last:
            flips += 1
        if s:
            last = s
    return dict(dur=dur, dist=dist, vmed=dist/dur if dur else 0,
                pct_far=pct_far, n_turn=len(eps), small=small,
                amp_med=median(amps) if amps else 0.0, vaivolta=vaivolta,
                turn_pct=100.0*turn_time/dur if dur else 0,
                flips_min=60.0*flips/dur if dur else 0)

def main():
    for path in sys.argv[1:]:
        rows = load(path)
        print(f"\n=== {path} ({len(rows)} amostras)")
        for i, seg in enumerate(segments(rows), 1):
            a = analyze(seg)
            print(f"  perna {i}: {a['dur']:.0f}s  {a['dist']:.1f}m  "
                  f"v={a['vmed']:.3f}m/s  carrot_longe={a['pct_far']:.0f}%  "
                  f"turnings={a['n_turn']} (<10°: {a['small']}, mediana "
                  f"{a['amp_med']:.0f}°)  vai-e-volta={a['vaivolta']}  "
                  f"girando={a['turn_pct']:.0f}%  flips_wz/min={a['flips_min']:.1f}")

if __name__ == '__main__':
    main()
