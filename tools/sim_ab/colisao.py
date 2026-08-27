#!/usr/bin/env python3
"""Detector de COLISÃO de verdade — ground truth do Gazebo, não proxy de laser.

Antes eu media `min_scan` (menor leitura do lidar) e chamava de segurança. Isso
é proxy: o laser está no CENTRO do robô, então um retorno de 0.25 m pode ser
"passando raspando" ou "já encostou", e nada distingue os dois.

Aqui a conta é geométrica e exata: o robô é um retângulo 0.5x0.5 girado pelo yaw
(OBB); cada parede/obstáculo do mundo é uma caixa (AABB). SAT (separating axis
theorem) sobre 4 eixos dá a SEPARAÇÃO real em metros:
    > 0  = folga livre        (o valor é a distância)
    <= 0 = ENCOSTOU/penetrou  (o valor é a profundidade)

Uso: colisao.py <mundo.sdf> <saida.csv>
"""
import math, re, subprocess, sys, time

R_HALF = 0.25          # meia-largura e meio-comprimento do robô (footprint ±0.25)
RASPAO = 0.02          # folga <= 2 cm sem penetrar = passou raspando


def caixas_do_mundo(sdf_path):
    """extrai (nome, cx, cy, sx, sy) de cada <model> estático com <box><size>"""
    txt = open(sdf_path).read()
    caixas = []
    for m in re.finditer(r'<model name="([^"]+)"[^>]*>(.*?)</model>', txt, re.S):
        nome, corpo = m.group(1), m.group(2)
        if nome in ('ground_plane', 'sim_robot'):
            continue
        pose = re.search(r'<pose>([^<]+)</pose>', corpo)
        size = re.search(r'<box><size>([^<]+)</size></box>', corpo)
        if not pose or not size:
            continue
        p = [float(v) for v in pose.group(1).split()]
        s = [float(v) for v in size.group(1).split()]
        caixas.append((nome, p[0], p[1], s[0], s[1]))
    return caixas


def separacao(rx, ry, ryaw, cx, cy, sx, sy):
    """SAT entre o OBB do robô e a AABB da caixa. >0 = folga, <=0 = penetração."""
    c, s = math.cos(ryaw), math.sin(ryaw)
    eixos = [(1.0, 0.0), (0.0, 1.0), (c, s), (-s, c)]
    cantos_r = [(rx + c * dx - s * dy, ry + s * dx + c * dy)
                for dx, dy in ((R_HALF, R_HALF), (R_HALF, -R_HALF),
                               (-R_HALF, -R_HALF), (-R_HALF, R_HALF))]
    hx, hy = sx / 2, sy / 2
    cantos_c = [(cx + hx, cy + hy), (cx + hx, cy - hy),
                (cx - hx, cy - hy), (cx - hx, cy + hy)]
    pior = -1e9          # queremos o MAIOR gap entre os eixos (SAT)
    for ax, ay in eixos:
        pr = [p[0] * ax + p[1] * ay for p in cantos_r]
        pc = [p[0] * ax + p[1] * ay for p in cantos_c]
        gap = max(min(pr) - max(pc), min(pc) - max(pr))
        pior = max(pior, gap)
    return pior


def main():
    import json
    mundo, saida = sys.argv[1], sys.argv[2]
    nome_mundo = mundo.split("/")[-1][:-4]
    caixas = caixas_do_mundo(mundo)
    print(f"[colisao] {len(caixas)} obstáculos/paredes no mundo", flush=True)
    f = open(saida, 'w')
    f.write("t,x,y,yaw_deg,folga_min,obj,evento\n")
    proc = subprocess.Popen(
        ['gz', 'topic', '-e', '-t', f'/world/{nome_mundo}/pose/info', '--json-output'],
        stdout=subprocess.PIPE, text=True, bufsize=1)
    t0 = time.time(); n_col = 0; n_rasp = 0; pior_global = 9e9; ultimo = -1.0
    eventos = []
    try:
        for linha in proc.stdout:
            linha = linha.strip()
            if not linha.startswith('{'):
                continue
            try:
                d = json.loads(linha)
            except Exception:
                continue
            robo = None
            for pose in d.get('pose', []):
                if pose.get('name') == 'sim_robot':
                    robo = pose
                    break
            if robo is None:
                continue
            # JSON do gz OMITE campos zero -> default 0.0 em cada get
            pos = robo.get('position', {})
            ori = robo.get('orientation', {})
            px = float(pos.get('x', 0.0)); py = float(pos.get('y', 0.0))
            qz = float(ori.get('z', 0.0)); qw = float(ori.get('w', 1.0))
            yaw = 2 * math.atan2(qz, qw)
            agora = time.time() - t0
            if agora - ultimo < 0.05:      # ~20 Hz basta (a 0.6 m/s = 3 cm/amostra)
                continue
            ultimo = agora
            pior = 9e9; alvo = ''
            for (cn, cx, cy, sx, sy) in caixas:
                g = separacao(px, py, yaw, cx, cy, sx, sy)
                if g < pior:
                    pior, alvo = g, cn
            ev = ''
            if pior <= 0:
                ev = 'COLISAO'; n_col += 1
            elif pior <= RASPAO:
                ev = 'raspao'; n_rasp += 1
            pior_global = min(pior_global, pior)
            f.write(f"{agora:.2f},{px:.3f},{py:.3f},{math.degrees(yaw):.1f},"
                    f"{pior:.4f},{alvo},{ev}\n")
            if ev == 'COLISAO':
                eventos.append((agora, alvo, pior, px, py))
                if len(eventos) < 40:
                    print(f"[colisao] {agora:7.1f}s  BATEU em {alvo} "
                          f"(penetrou {-pior*100:.1f} cm) em ({px:.2f},{py:.2f})",
                          flush=True)
            f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate(); f.close()
        print(f"[colisao] FIM: {n_col} amostras em colisão, {n_rasp} raspões, "
              f"menor folga {pior_global*100:.1f} cm", flush=True)


main()
