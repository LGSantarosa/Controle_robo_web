#!/usr/bin/env python3
"""Consolida as voltas de um degrau de velocidade num CSV + resumo .md no REPO.

O scratchpad é temporário; o que vale fica em log/nav2_trekking_velocidade/.
Uso: consolida.py <rotulo> <tag1> [tag2 ...]
"""
import csv, json, os, re, sys

SP = os.environ.get("SIM_AB_DIR", "/home/rbe-luis/Workspace/Controle_robo_web/log/sim_ab")
DEST = "/home/rbe-luis/Workspace/Controle_robo_web/log/nav2_trekking_velocidade"


def colisoes(tag):
    f = f"{SP}/{tag}/colisao.csv"
    if not os.path.exists(f):
        return None, None, None
    L = list(csv.DictReader(open(f)))
    if not L:
        return None, None, None
    ev = [l for l in L if l['evento'] == 'COLISAO']
    n, ant = 0, -9.0
    for l in ev:                      # amostras contíguas = um evento só
        t = float(l['t'])
        if t - ant > 1.0:
            n += 1
        ant = t
    rasp, antr = 0, -9.0
    for l in [x for x in L if x['evento'] == 'raspao']:
        t = float(l['t'])
        if t - antr > 1.0:
            rasp += 1
        antr = t
    return n, rasp, min(float(l['folga_min']) for l in L)


def uma(tag):
    p = f"{SP}/{tag}/result.json"
    if not os.path.exists(p):
        return None
    x = json.load(open(p)); g = x['goals']
    log = f"{SP}/{tag}/nav2.log"
    t = open(log, errors='ignore').read() if os.path.exists(log) else ""
    bateu, raspou, folga = colisoes(tag)
    dist = sum(i['dist'] for i in g)
    return dict(
        volta=tag,
        goals_ok=sum(1 for i in g if i['status'] == 'OK'),
        goals=len(g),
        colisoes=bateu, raspoes=raspou,
        folga_min_m=round(folga, 3) if folga is not None else None,
        tempo_s=round(x['total_s'], 1),
        dist_m=round(dist, 1),
        v_media_pct=round(dist / x['total_s'], 3),          # média incl. paradas/giros
        v_media_andando=round(sum(i['v_med'] for i in g) / len(g), 3),
        v_max=round(max(i['v_max'] for i in g), 3),
        parado_s=round(sum(i['parado'] for i in g), 1),
        unstuck_s=round(sum(i['unstuck'] for i in g), 1),
        freio_s=round(sum(i['cm'].get('LIMIT', 0) + i['cm'].get('STOP', 0)
                          + i['cm'].get('APPROACH', 0) for i in g), 1),
        min_scan_m=round(min(i['min_scan'] for i in g if i['min_scan']), 3),
        falhas_plano=len(re.findall(r'failed to plan', t)),
        recoveries=len(re.findall(r'Running (backup|spin|wait)', t)),
        naochegou=";".join(f"{i['goal']}:{i['status']}" for i in g if i['status'] != 'OK'),
    )


def main():
    rotulo, tags = sys.argv[1], sys.argv[2:]
    os.makedirs(DEST, exist_ok=True)
    linhas = [l for l in (uma(t) for t in tags) if l]
    if not linhas:
        print("nenhuma volta com resultado"); return
    campos = list(linhas[0].keys())
    saida = f"{DEST}/{rotulo}.csv"
    with open(saida, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=campos); w.writeheader()
        for l in linhas:
            w.writerow(l)
    # guarda os CSVs brutos de colisão junto (é a prova do "não bateu")
    for t in tags:
        src = f"{SP}/{t}/colisao.csv"
        if os.path.exists(src):
            os.replace(src, f"{DEST}/{rotulo}_{t}_colisao.csv") if False else None
            import shutil; shutil.copy(src, f"{DEST}/{rotulo}_{t}_colisao.csv")
    n = len(linhas)
    ag = lambda k: sum(l[k] for l in linhas if l[k] is not None) / n
    print(f"=== {rotulo} — {n} voltas ===")
    print(f"  goals:      {sum(l['goals_ok'] for l in linhas)}/{sum(l['goals'] for l in linhas)}"
          f"   ({sum(1 for l in linhas if l['goals_ok']==l['goals'])}/{n} voltas completas)")
    col = [l['colisoes'] for l in linhas if l['colisoes'] is not None]
    print(f"  COLISÕES:   {sum(col) if col else '?'}   (raspões: {sum(l['raspoes'] for l in linhas if l['raspoes'] is not None)})")
    print(f"  folga mín:  {min(l['folga_min_m'] for l in linhas if l['folga_min_m'] is not None):.3f} m")
    print(f"  tempo:      {ag('tempo_s'):.1f} s   (min {min(l['tempo_s'] for l in linhas):.0f} / max {max(l['tempo_s'] for l in linhas):.0f})")
    print(f"  dist:       {ag('dist_m'):.1f} m")
    print(f"  v média:    {ag('v_media_pct'):.3f} m/s (porta a porta)  |  {ag('v_media_andando'):.3f} m/s (amostras)")
    print(f"  v máx:      {max(l['v_max'] for l in linhas):.3f} m/s")
    print(f"  parado:     {ag('parado_s'):.1f} s    unstuck: {ag('unstuck_s'):.1f} s    freio: {ag('freio_s'):.1f} s")
    print(f"  plano/rec:  {ag('falhas_plano'):.1f} falhas de plano, {ag('recoveries'):.1f} recoveries")
    print(f"\n  -> {saida}")


main()
