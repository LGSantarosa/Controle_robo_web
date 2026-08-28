#!/usr/bin/env python3
"""Detector de COLISÃO de verdade — ground truth do Gazebo, não proxy de laser.

Antes eu media `min_scan` (menor leitura do lidar) e chamava de segurança. Isso
é proxy: o laser está no CENTRO do robô, então um retorno de 0.25 m pode ser
"passando raspando" ou "já encostou", e nada distingue os dois.

Aqui a conta é geométrica e exata: o robô é um retângulo 0.5x0.5 girado pelo yaw
(OBB). Contra CAIXA usa SAT (separating axis theorem) sobre 4 eixos; contra
CILINDRO usa a distância assinada ponto-OBB menos o raio. Nos dois casos:
    > 0  = folga livre        (o valor é a distância)
    <= 0 = ENCOSTOU/penetrou  (o valor é a profundidade)

2026-08-28, duas correções para a arena do galpão:
  1. CILINDROS entram na conta. Antes só caixas eram lidas, então uma batida em
     CONE (que é cilindro) passava despercebida — e na arena o cone é o objetivo,
     encostar nele é justamente o que não pode. Sem isso o critério A4 (zero
     contato) era immensurável.
  2. Só geometria dentro de <collision> conta. Antes qualquer <box> do arquivo
     virava obstáculo, inclusive de <visual>. A plataforma amarela da arena é
     visual pura (marca de chão, o laser não vê) e viraria obstáculo fantasma,
     acusando "colisão" toda vez que o robô pisasse no alvo.

Uso: colisao.py <mundo.sdf> <saida.csv>
     colisao.py --autoteste          (geometria conferida contra valores na mão)
"""
import math, re, subprocess, sys, time

R_HALF = 0.25          # meia-largura e meio-comprimento do robô (footprint ±0.25)
RASPAO = 0.02          # folga <= 2 cm sem penetrar = passou raspando


def obstaculos_do_mundo(sdf_path):
    """Lê os <model> estáticos e devolve TODAS as geometrias COLIDÍVEIS.

    ('box', nome, cx, cy, sx, sy)  ou  ('cyl', nome, cx, cy, raio)

    Regras, cada uma paga com um bug:
    - só geometria dentro de <collision>: <visual> não colide, e a plataforma
      amarela da arena é visual pura (viraria obstáculo fantasma no alvo);
    - TODAS as <collision> de cada modelo, não a primeira. O modelo `pessoa` do
      sala_grande tem DUAS (perna_esq em y+0.15, perna_dir em y-0.15): pegar só a
      primeira perdia uma perna E colocava a outra no centro do modelo, que é o
      espaço VAZIO entre as pernas;
    - a <pose> local de cada colisão soma à pose do modelo, girada pelo yaw do
      modelo. Sem isso as duas pernas caíam no mesmo ponto.

    ⚠️ Caixa é tratada como AABB. Se um modelo com <box> tiver yaw != 0 a conta
    fica errada, então isso é AVISADO em vez de silenciosamente aceito.
    """
    txt = open(sdf_path).read()
    obst = []
    for m in re.finditer(r'<model name="([^"]+)"[^>]*>(.*?)</model>', txt, re.S):
        nome, corpo = m.group(1), m.group(2)
        if nome in ('ground_plane', 'sim_robot'):
            continue
        pose = re.search(r'<pose>([^<]+)</pose>', corpo)
        if not pose:
            continue
        p = [float(v) for v in pose.group(1).split()]
        mx, my = p[0], p[1]
        myaw = p[5] if len(p) >= 6 else 0.0
        cos, sen = math.cos(myaw), math.sin(myaw)
        for cm in re.finditer(r'<collision\b[^>]*>(.*?)</collision>', corpo, re.S):
            col = cm.group(1)
            lp = re.search(r'<pose>([^<]+)</pose>', col)
            lx, ly = (0.0, 0.0)
            if lp:
                v = [float(t) for t in lp.group(1).split()]
                lx, ly = v[0], v[1]
            cx = mx + cos * lx - sen * ly
            cy = my + sen * lx + cos * ly
            size = re.search(r'<box>\s*<size>([^<]+)</size>', col)
            raio = re.search(r'<cylinder>\s*<radius>([^<]+)</radius>', col)
            if size:
                sz = [float(v) for v in size.group(1).split()]
                if abs(myaw) > 1e-9:
                    print(f'[colisao] AVISO: modelo "{nome}" tem <box> com yaw '
                          f'{math.degrees(myaw):.1f}° — tratado como AABB, folga '
                          f'subestimada nesse obstáculo', flush=True)
                obst.append(('box', nome, cx, cy, sz[0], sz[1]))
            elif raio:
                obst.append(('cyl', nome, cx, cy, float(raio.group(1))))
    return obst


def separacao(rx, ry, ryaw, cx, cy, sx, sy):
    """SAT entre o OBB do robô e a AABB da caixa. >0 = folga, <=0 = penetração.

    ⚠️ O SINAL é exato (contato é contato). O VALOR positivo é o maior gap entre
    os eixos testados, que é uma COTA INFERIOR da distância euclidiana: com
    separação diagonal a distância real é hypot(gx, gy) >= max(gx, gy). Erra pro
    lado seguro (reporta MENOS folga do que há), mas não chamar de "folga exata".
    Contra CILINDRO (separacao_circulo) o valor é exato.
    """
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


def separacao_circulo(rx, ry, ryaw, cx, cy, raio):
    """Distância assinada entre o OBB do robô e um círculo. >0 folga, <=0 penetra.

    Leva o centro do círculo pro referencial do robô, mede a distância assinada
    até o retângulo e desconta o raio. Exato inclusive com o centro dentro do
    retângulo (aí a distância é negativa e vale a face mais próxima).
    """
    c, s = math.cos(ryaw), math.sin(ryaw)
    dx, dy = cx - rx, cy - ry
    lx = c * dx + s * dy          # centro do círculo no referencial do robô
    ly = -s * dx + c * dy
    qx = abs(lx) - R_HALF
    qy = abs(ly) - R_HALF
    if qx > 0.0 or qy > 0.0:      # centro fora do retângulo
        d = math.hypot(max(qx, 0.0), max(qy, 0.0))
    else:                         # centro dentro: distância negativa até a face
        d = max(qx, qy)
    return d - raio


def sep(obj, rx, ry, ryaw):
    """Despacha pela forma do obstáculo."""
    if obj[0] == 'box':
        _, _, cx, cy, sx, sy = obj
        return separacao(rx, ry, ryaw, cx, cy, sx, sy)
    _, _, cx, cy, raio = obj
    return separacao_circulo(rx, ry, ryaw, cx, cy, raio)


def autoteste():
    """Confere a geometria contra valores calculados na mão. Sai 1 se errar."""
    casos = [
        # (descricao, obj, rx, ry, yaw_deg, esperado)
        ("cone longe, robo reto",
         ('cyl', 'c', 1.0, 0.0, 0.17), 0, 0, 0, (1.0 - 0.25) - 0.17),
        ("cone encostando (tangente)",
         ('cyl', 'c', 0.42, 0.0, 0.17), 0, 0, 0, 0.0),
        ("cone PENETRANDO 2 cm",
         ('cyl', 'c', 0.40, 0.0, 0.17), 0, 0, 0, -0.02),
        ("cone na diagonal",
         ('cyl', 'c', 0.5, 0.5, 0.17), 0, 0, 0, math.hypot(0.25, 0.25) - 0.17),
        # robo a 45 graus aponta o CANTO (0.354) pro cone, entao a folga cai
        ("cone a 1 m, robo girado 45 (canto na frente)",
         ('cyl', 'c', 1.0, 0.0, 0.17), 0, 0, 45, 1.0 - math.hypot(0.25, 0.25) - 0.17),
        ("centro do cone DENTRO do robo",
         ('cyl', 'c', 0.0, 0.0, 0.17), 0, 0, 0, -0.25 - 0.17),
        # caixa: nao pode ter regredido
        ("caixa 0.4x2.0 centrada em x=1",
         ('box', 'b', 1.0, 0.0, 0.4, 2.0), 0, 0, 0, 0.8 - 0.25),
    ]
    ok = True
    for desc, obj, rx, ry, yawd, esperado in casos:
        got = sep(obj, rx, ry, math.radians(yawd))
        bom = abs(got - esperado) < 1e-9
        ok &= bom
        print(f"  [{'ok ' if bom else 'ERRO'}] {desc:48s} "
              f"esperado {esperado:+.4f}  obtido {got:+.4f}")
    print("[autoteste] " + ("TUDO CERTO" if ok else "FALHOU"))
    return 0 if ok else 1


def main():
    import json
    if sys.argv[1] == '--autoteste':
        sys.exit(autoteste())
    mundo, saida = sys.argv[1], sys.argv[2]
    nome_mundo = mundo.split("/")[-1][:-4]
    caixas = obstaculos_do_mundo(mundo)
    n_cil = sum(1 for o in caixas if o[0] == 'cyl')
    print(f"[colisao] {len(caixas)} obstáculos colidíveis no mundo "
          f"({len(caixas) - n_cil} caixas, {n_cil} cilindros)", flush=True)
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
            for obj in caixas:
                g = sep(obj, px, py, yaw)
                if g < pior:
                    pior, alvo = g, obj[1]
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
