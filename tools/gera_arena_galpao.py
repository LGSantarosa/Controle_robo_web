#!/usr/bin/env python3
"""Fonte ÚNICA da geometria da arena do galpão (prova de 2026-09-05).

Gera, da MESMA tabela:
  --sdf      worlds/arena_galpao.sdf   (o mundo do Gazebo)
  --mapa     maps/arena_galpao.{pgm,yaml}  (o mapa do nav2, exato — não é SLAM)
  --conferir valida as distâncias que o mundo precisa respeitar

Por que gerado e não escrito à mão: as mesmas coordenadas aparecem no mundo, no
mapa e nos probes de validação. Escritas em três lugares, divergem em uma semana.

O MAPA NÃO LEVA OS CONES, e isso é decisão consciente:
o cone é o OBJETIVO, e a missão manda chegar a 20 cm dele. Marcado na camada
estática, o goal nasce dentro de obstáculo+inflação e o nav2 recusa
("Either of the start or goal pose are an obstacle!"). O cone continua no MUNDO
— o laser o vê, o obstacle_layer o marca ao vivo, o cone_detector o acha e o
colisao.py cobra contato nele. Só não vira parede permanente do mapa.
Use --com-cones para gerar a variante com eles, se algum dia interessar.
"""
import argparse
import json
import math
import os

GALPAO_X, GALPAO_Y = 14.0, 9.0
T_MURO, H_MURO = 0.10, 1.5
ESP_BLOCO, H_BLOCO = 0.60, 0.80      # espessura 0.60: ver nota no SDF
R_CONE, H_CONE = 0.17, 0.70
PLAT = 1.2
STANDOFF = 1.4        # m do centro do cone ate o goal do nav2 (ver escreve_rota)
RES = 0.05                            # m/célula do mapa

PONTOS = {                            # nome: (x, y, tem_cone)
    'largada': (1.0, 1.0, False),
    'cone_1':  (4.5, 1.5, True),
    'cone_2':  (11.5, 1.8, True),
    'cone_3':  (12.2, 7.5, True),
    'cone_4':  (5.0, 7.8, True),
    'chegada': (1.5, 2.5, False),
}

# (nome, eixo_do_bloco, coord, [(ini,fim) de cada bloco], fresta_esperada, contorno)
OBST = [
    ('A_fresta90', 'x', 7.5, [(0.30, 1.80), (2.70, 4.20)], 0.90, 'por y > 4.20'),
    ('B_fresta70', 'y', 4.6, [(9.55, 11.05), (11.75, 14.00)], 0.70, 'por x < 9.55'),
    ('C_fresta60', 'x', 8.2, [(5.40, 7.20), (7.80, 9.00)], 0.60, 'por y < 5.40'),
    ('D_fresta80', 'y', 4.6, [(0.00, 2.20), (3.00, 4.60)], 0.80, 'por x > 4.60'),
]

# Probes de conectividade: um por perna, com o ponto antes e depois do obstáculo.
# Rotulos SEM espaco: a saida de --probes/--folgas e' pra ser colada direto na
# shell, e espaco no rotulo vira argumento solto.
PROBES = [
    (1.0, 1.0, 4.5, 1.5, 'largada->cone1_livre'),
    (4.5, 1.5, 11.5, 1.8, 'cone1->cone2_fresta0.90'),
    (11.5, 1.8, 12.2, 7.5, 'cone2->cone3_fresta0.70'),
    (12.2, 7.5, 5.0, 7.8, 'cone3->cone4_fresta0.60'),
    (5.0, 7.8, 1.5, 2.5, 'cone4->chegada_fresta0.80'),
]


def blocos():
    """Devolve (nome, cx, cy, sx, sy) de cada bloco de fresta."""
    out = []
    for nome, eixo, coord, faixas, _f, _c in OBST:
        for i, (ini, fim) in enumerate(faixas, 1):
            comp, meio = fim - ini, (ini + fim) / 2
            if eixo == 'x':
                out.append((f'{nome}_{i}', coord, meio, ESP_BLOCO, comp))
            else:
                out.append((f'{nome}_{i}', meio, coord, comp, ESP_BLOCO))
    return out


def tampao(nome):
    """Caixa que FECHA o vão de uma fresta — do batente ao batente, com a mesma
    espessura do bloco. Usada SÓ na pintura do mapa (--fecha-fresta); NUNCA entra
    em blocos()/corpo_sdf(), senão fecharia o mundo junto.

    Devolve (cx, cy, sx, sy), no mesmo formato de blocos()."""
    for n, eixo, coord, faixas, _f, _c in OBST:
        if n != nome:
            continue
        ini, fim = faixas[0][1], faixas[1][0]        # o vão
        comp, meio = fim - ini, (ini + fim) / 2
        if eixo == 'x':
            return (coord, meio, ESP_BLOCO, comp)
        return (meio, coord, comp, ESP_BLOCO)
    raise ValueError(
        f'fresta desconhecida: {nome!r} — conhecidas: '
        + ', '.join(o[0] for o in OBST))


# --- FRESTAS MARCADAS COMO PORTA (door_crossing) -----------------------------
# 2026-09-02, DIARIO_ARENA §2G.10: o dono quer o robô PASSANDO pela fresta A
# 100% das vezes. O vão de 0,90 m dá orçamento lateral de ±0,20 m e o erro do
# AMCL nesta arena chega a 0,49 m — nenhuma solução map-relative fecha isso. Quem
# atravessa é o `door_crossing`, que precisa dos DOIS BATENTES marcados.
#
# Os batentes saem daqui, da mesma tabela OBST que gera o mundo: são as bordas
# exatas dos blocos, não cliques a olho. Isso mata a classe de erro "eixo torto"
# NO SIM (no robô real o mapa vem de SLAM — ver §8.3 do spec).
#
# ⚠️ Marcar SÓ a fresta A nesta rodada (spec §5.1). Antes de acrescentar B ou D
# aqui, refaça a conta de margem_point_turn() para ela: se a margem for negativa,
# o robô bate no batente durante o PRÓPRIO giro de alinhamento. A fresta C (0,60)
# não deve ser marcada — com robot_radius 0.32 o Nav2 já a trata como parede.
# 2026-09-03: C e D SAIRAM por ordem do dono — "retira os doors 3 e 4, eles nao
# serao utilizados, pode descartar, focamos so no 1 e 2". O door_crossing deixa
# de armar nelas; a fresta segue existindo no mundo/mapa (o robo passa por ela
# com o nav2 puro, se a rota mandar).
MARCADAS_COMO_PORTA = ('A_fresta90', 'B_fresta70')

# Waypoint pré-fresta (2026-09-02, DIARIO_ARENA §2H.7) — OPT-IN, default OFF.
# O `door_crossing` só assume porta cujo goal do Nav2 TERMINOU dentro da zona
# dela (`_pick_door`, pendência C): sem um goal ali, o nó sobe e fica `idle`
# para sempre. Este waypoint é esse goal.
#
# A DISTÂNCIA não é livre. O §4.4-(a) do spec calcula a margem do point-turn no
# ponto EXATO, mas o robô para dentro do `xy_goal_tolerance = 0.15`
# (`nav2_params_arena.yaml:151`). Refeita para o pior canto do envelope:
#     0,6 m -> -1,8 cm  (o círculo varrido ENCOSTA no bloco)
#     0,8 m -> +10,7 cm
#     1,0 m -> +27,3 cm   <- escolhido
# 1,0 m também é o "ponto pré-porta" para o qual `zone_radius = 1.1` foi
# dimensionado (door_crossing.py:169-173) — não é número a dedo.
# 2026-09-02, ERRO 94 + marcação na quina: 1,0 m NÃO servia por DUAS razões.
# (a) A condição de arme é `dist(robô, centro do vão) <= zone_radius 1,1` no
#     instante do SUCCEEDED, e a 1,0 m sobravam só 0,09 m contra um
#     `xy_goal_tolerance` de 0,15 -> o robô podia concluir o goal FORA da zona e
#     a máquina nunca armar (foi o que aconteceu na volta `lim1`).
# (b) Com o plano da porta na QUINA (0,30 m mais perto), 1,0 m ficaria ainda
#     mais longe do centro do vão.
# A 0,80 m: pior canto do envelope = hypot(0,95 ; 0,30) = 0,996 < 1,1 -> ARMA
# SEMPRE, e a margem do point-turn sobe (o ponto sai da sombra do bloco).
PRE_FRESTA_DIST = 0.8
XY_GOAL_TOL = 0.15          # nav2_params_arena.yaml:151 (goal_checker)


def ponto_pre_fresta(nome='A_fresta90', dist=None):
    """(x, y, yaw) do waypoint pré-fresta, do lado por onde a rota chega.

    Sai da MESMA tabela OBST que gera o mundo e as portas. O yaw aponta pro vão:
    `approach_bearing = 70°` é medido do yaw do ROBÔ, então waypoint com yaw
    errado reprova o gate mesmo estando dentro da zona (achado do review).
    """
    # resolvido AQUI, não no default do parâmetro: default de argumento congela
    # o valor no `def` e a constante do módulo deixaria de valer (pegado por
    # test_distancia_curta_demais_ABORTA_a_geracao)
    dist = PRE_FRESTA_DIST if dist is None else dist
    (ax, ay), (bx, by) = batentes(nome)
    cx, cy = (ax + bx) / 2.0, (ay + by) / 2.0
    vx, vy = bx - ax, by - ay
    n = math.hypot(vx, vy)
    nx, ny = -vy / n, vx / n              # normal do vão
    # a rota vem de cone_1 (x < 7,5): escolhe o lado de onde ela chega
    px, py, _ = PONTOS['cone_1']
    lado = -1.0 if ((px - cx) * nx + (py - cy) * ny) < 0 else 1.0
    x, y = cx + lado * dist * nx, cy + lado * dist * ny
    return x, y, math.atan2(-lado * ny, -lado * nx)   # encarando o vão


def margem_pre_fresta(nome='A_fresta90', dist=None, tol=XY_GOAL_TOL):
    """Pior margem do point-turn considerando que o robô para em QUALQUER canto
    do envelope permitido pelo `xy_goal_tolerance` — não no ponto ideal."""
    dist = PRE_FRESTA_DIST if dist is None else dist
    x, y, _ = ponto_pre_fresta(nome, dist)
    cantos = []
    for n_, eixo, coord, faixas, _f, _c in OBST:
        if n_ != nome:
            continue
        for b in (faixas[0][1], faixas[1][0]):          # os 2 batentes
            for s_ in (-1, 1):                          # as 2 faces do bloco
                if eixo == 'x':
                    cantos.append((coord + s_ * ESP_BLOCO / 2, b))
                else:
                    cantos.append((b, coord + s_ * ESP_BLOCO / 2))
    pior = math.inf
    for dx in (-tol, 0.0, tol):
        for dy in (-tol, 0.0, tol):
            d = min(math.hypot(x + dx - cxx, y + dy - cyy) for cxx, cyy in cantos)
            pior = min(pior, d - RAIO_CIRCUNSCRITO)
    return pior

# Volume varrido por um 0,50 × 0,50 girando no lugar (raio circunscrito).
RAIO_CIRCUNSCRITO = 0.25 * math.sqrt(2)
# door_crossing.DoorCrossConfig.stage_dist — onde o alinhamento acontece.
STAGE_DIST = 0.60
ZONE_RADIUS = 1.1       # door_crossing.DoorCrossConfig.zone_radius
ROBO_MEIA_C = 0.25      # meio COMPRIMENTO do robo (0,50 x 0,50)


# Qual perna da rota atravessa cada fresta — define de que LADO o robô chega.
PERNA_DA_FRESTA = {
    'A_fresta90': ('cone_1', 'cone_2'),
    'B_fresta70': ('cone_2', 'cone_3'),
    'C_fresta60': ('cone_3', 'cone_4'),
    'D_fresta80': ('cone_4', 'chegada'),
}


def lado_de_chegada(nome):
    """-1 se o robô chega pelo lado de coordenada MENOR que o eixo do bloco,
    +1 se pelo lado maior. Sai da rota, não de chute."""
    for n, eixo, coord, _faixas, _f, _c in OBST:
        if n != nome:
            continue
        de, _para = PERNA_DA_FRESTA[nome]
        px, py, _ = PONTOS[de]
        v = px if eixo == 'x' else py
        return -1.0 if v < coord else 1.0
    raise ValueError(f'fresta desconhecida: {nome!r}')


def batentes(nome, face='entrada'):
    """Os 2 batentes do vão de uma fresta, em (x, y). Mesma fonte do mundo.

    `face` (2026-09-02, pedido do dono): onde fica o PLANO da porta.

      'meio'    — no eixo do bloco (o que existia até hoje).
      'entrada' — na QUINA por onde o robô chega, isto é, o eixo do bloco
                  recuado de meia espessura (ESP_BLOCO/2 = 0,30 m).

    Por que 'entrada' é o certo: o bloco tem 0,60 m de espessura, então a fresta
    é um TÚNEL curto — a boca fica 0,30 m antes do eixo. Marcando no meio, o
    `door_crossing` alinha e projeta o `will_clear` para um plano que está
    **dentro da parede**, 30 cm depois de onde o robô pode encostar. Ele se
    ajeita para o lugar errado. Marcando na quina, ele se alinha para ENTRAR.
    """
    for n, eixo, coord, faixas, _f, _c in OBST:
        if n != nome:
            continue
        ini, fim = faixas[0][1], faixas[1][0]        # o vão
        c = coord
        if face == 'entrada':
            c = round(coord + lado_de_chegada(nome) * ESP_BLOCO / 2.0, 6)
        elif face != 'meio':
            raise ValueError(f"face desconhecida: {face!r} (use 'entrada'/'meio')")
        if eixo == 'x':
            return (c, ini), (c, fim)
        return (ini, c), (fim, c)
    raise ValueError(f'fresta desconhecida: {nome!r}')


def margem_point_turn(nome, stage_dist=STAGE_DIST, face='entrada'):
    """Spec §4.4-(a): sobra espaço pro giro de alinhamento no ponto de preparação?

    O ponto fica a `stage_dist` do PLANO da porta, no eixo do vão. O obstáculo
    mais perto é a QUINA do bloco naquele plano: `stage_dist` ao longo da normal
    e `vão/2` ao longo da parede. Margem = essa distância − RAIO_CIRCUNSCRITO.
    Negativa = a fresta NÃO pode ser marcada (o robô bate girando).

    2026-09-02: com `face='entrada'` o plano é a boca do túnel, então a distância
    ao canto é `stage_dist` cheio. Com `face='meio'` (o antigo) o ponto ficava
    0,30 m "dentro da sombra" do bloco e a margem caía — é por isso que os
    números desta função melhoraram ao mudar a marcação.
    """
    (ax, ay), (bx, by) = batentes(nome, face)
    vao = math.hypot(bx - ax, by - ay)
    braco = stage_dist if face == 'entrada' else stage_dist - ESP_BLOCO / 2
    return math.hypot(braco, vao / 2) - RAIO_CIRCUNSCRITO


def margem_arme(nome, dist=None, tol=XY_GOAL_TOL, lat=0.0):
    """Folga até o `zone_radius` no PIOR canto do envelope de chegada do goal.

    O `door_crossing` só arma se `dist(robô, centro do vão) <= zone_radius` no
    instante em que o goal conclui. Positivo = arma sempre. Ver erro 94.
    """
    dist = PRE_FRESTA_DIST if dist is None else dist
    return ZONE_RADIUS - math.hypot(dist + tol, abs(lat) + tol)


def margem_saida(nome, exit_margin):
    """Quanto o robô AINDA está dentro do túnel quando a máquina solta.

    Positivo = já saiu inteiro. O bloco tem ESP_BLOCO de espessura e o plano da
    porta agora é a BOCA, então o robô só está fora quando o centro dele passa
    de `ESP_BLOCO + meio comprimento`.
    """
    return exit_margin - (ESP_BLOCO + ROBO_MEIA_C)


def portas(fecha=()):
    """Lista no schema do DoorStore (maps/<mapa>.doors.json), consumida pelo
    /doors. Uma fresta TAMPADA no mapa não vira porta: o planejador nem passa
    por lá, e uma zona armada em cima de parede é armadilha."""
    out = []
    for nome in MARCADAS_COMO_PORTA:
        if nome in fecha:
            continue
        m = margem_point_turn(nome)
        if m <= 0:
            raise SystemExit(
                f'{nome}: margem de point-turn {m:+.3f} m no ponto de preparação '
                f'— o robô bateria no batente durante o próprio giro de '
                f'alinhamento. NÃO pode ser marcada como porta (spec §4.4-a).')
        a, b = batentes(nome)
        out.append({'id': len(out) + 1, 'a': [a[0], a[1]], 'b': [b[0], b[1]]})
    return out


def resolve_fresta(rotulo):
    """Aceita 'A', 'a' ou o nome inteiro ('A_fresta90'). Erra alto se não achar."""
    r = str(rotulo).strip()
    for n, *_ in OBST:
        if r.upper() == n.split('_')[0] or r == n:
            return n
    raise ValueError(
        f'fresta desconhecida: {rotulo!r} — use A/B/C/D ou '
        + ', '.join(o[0] for o in OBST))


def muros():
    T = T_MURO
    return [
        ('muro_sul',   GALPAO_X / 2, -T / 2, GALPAO_X + 2 * T, T),
        ('muro_norte', GALPAO_X / 2, GALPAO_Y + T / 2, GALPAO_X + 2 * T, T),
        ('muro_oeste', -T / 2, GALPAO_Y / 2, T, GALPAO_Y + 2 * T),
        ('muro_leste', GALPAO_X + T / 2, GALPAO_Y / 2, T, GALPAO_Y + 2 * T),
    ]


# ------------------------------------------------------------------ conferir
def conferir():
    ok = True

    def diz(bom, msg):
        nonlocal ok
        ok &= bom
        print(f'  [{"ok " if bom else "ERRO"}] {msg}')

    print('LARGURA DAS FRESTAS (o que o mundo realmente produz)')
    for nome, _e, _c, faixas, esperada, contorno in OBST:
        real = faixas[1][0] - faixas[0][1]
        diz(abs(real - esperada) < 1e-9,
            f'{nome}: fresta {real:.3f} m (esperada {esperada:.2f}) — contorno {contorno}')

    # Cada obstáculo é uma ILHA: UMA ponta vedada e UMA aberta. A aberta É o
    # contorno — sem ela o obstáculo viraria parede e a missão dependeria da
    # fresta, que é justamente o que não pode (critério A5).
    VAO_MIN = 0.64                    # 2 x robot_radius do perfil arena
    print(f'\nILHA: exatamente UMA ponta vedada (< {VAO_MIN} m) e UMA aberta (o contorno)')
    for nome, eixo, _c, faixas, _f, contorno in OBST:
        lim = GALPAO_Y if eixo == 'x' else GALPAO_X
        pontas = {'inicial': faixas[0][0] - 0.0, 'final': lim - faixas[1][1]}
        vedadas = [k for k, v in pontas.items() if v < VAO_MIN]
        abertas = [k for k, v in pontas.items() if v >= VAO_MIN]
        detalhe = ', '.join(f'{k} {v:.2f} m' for k, v in pontas.items())
        diz(len(vedadas) == 1 and len(abertas) == 1,
            f'{nome}: vedada={vedadas or "nenhuma"} aberta={abertas or "nenhuma"} '
            f'({detalhe}) — contorno documentado: {contorno}')

    print('\nBLOCO x CONE >= 1.70 m (senão funde cluster / esconde o alvo)')
    pior, pior_nome = 9e9, ''
    for bn, bx, by, bsx, bsy in blocos():
        for cn, (cx, cy, tem) in PONTOS.items():
            if not tem:
                continue
            dx = max(abs(cx - bx) - bsx / 2, 0.0)
            dy = max(abs(cy - by) - bsy / 2, 0.0)
            d = math.hypot(dx, dy) - R_CONE
            if d < pior:
                pior, pior_nome = d, f'{cn} x {bn}'
    diz(pior >= 1.70, f'menor distância cone-bloco: {pior:.2f} m ({pior_nome})')

    print('\nCONE x CONE >= 3.0 m')
    cones = [(n, x, y) for n, (x, y, t) in PONTOS.items() if t]
    pior, pior_nome = 9e9, ''
    for i in range(len(cones)):
        for j in range(i + 1, len(cones)):
            d = math.hypot(cones[i][1] - cones[j][1], cones[i][2] - cones[j][2])
            if d < pior:
                pior, pior_nome = d, f'{cones[i][0]} x {cones[j][0]}'
    diz(pior >= 3.0, f'menor distância cone-cone: {pior:.2f} m ({pior_nome})')

    print('\nPONTOS dentro do galpão e longe de bloco (o robô precisa caber lá)')
    for n, (x, y, _t) in PONTOS.items():
        dentro = 0.5 < x < GALPAO_X - 0.5 and 0.5 < y < GALPAO_Y - 0.5
        livre = True
        for bn, bx, by, bsx, bsy in blocos():
            dx = max(abs(x - bx) - bsx / 2, 0.0)
            dy = max(abs(y - by) - bsy / 2, 0.0)
            if math.hypot(dx, dy) < 0.60:
                livre = False
        diz(dentro and livre, f'{n} em ({x:.2f},{y:.2f})')

    # O MAPA tem que medir a MESMA largura da geometria contínua. Isto pegou um
    # bug real (2026-08-28): pinta_caixa usava floor/ceil + 1, o bloco crescia uma
    # célula por lado e a fresta de 0.70 media 0.60 -> o validador declarava
    # "fechada" uma passagem que existe. Bug silencioso: o mapa parece certo.
    print('\nRASTER DO MAPA bate com a geometria contínua')
    try:
        import tempfile
        import numpy as np
        from scipy import ndimage
        d = tempfile.mkdtemp()
        gera_mapa(os.path.join(d, 'a.pgm'), os.path.join(d, 'a.yaml'), False)
        raw = open(os.path.join(d, 'a.pgm'), 'rb').read()
        i, campos = 0, []
        while len(campos) < 4:
            while raw[i:i + 1].isspace():
                i += 1
            if raw[i:i + 1] == b'#':
                while raw[i:i + 1] != b'\n':
                    i += 1
                continue
            j = i
            while not raw[j:j + 1].isspace():
                j += 1
            campos.append(raw[i:j]); i = j
        W, H = int(campos[1]), int(campos[2])
        i += 1
        a = np.frombuffer(raw[i:i + W * H], dtype=np.uint8).reshape(H, W)
        dist = ndimage.distance_transform_edt(a >= 100) * RES
        x0, y0 = -T_MURO - 0.5, -T_MURO - 0.5
        for nome, eixo, coord, faixas, esperada, _c in OBST:
            meio = (faixas[0][1] + faixas[1][0]) / 2
            px, py = (coord, meio) if eixo == 'x' else (meio, coord)
            col = int(round((px - x0) / RES))
            lin = int(round((H - 1) - (py - y0) / RES))
            largura = 2 * dist[lin, col]
            diz(abs(largura - esperada) < 1e-9,
                f'{nome}: mapa mede {largura:.3f} m, geometria diz {esperada:.2f} m')

        # A ROTA tem que ser navegável: um goal do nav2 em cima de obstáculo ou
        # inflação é recusado ("start or goal pose are an obstacle"), e aí a volta
        # nem começa. Confere no MESMO mapa que o nav2 vai carregar.
        RAIO = 0.32
        print(f'\nROTA navegável (goal precisa de folga >= {RAIO} m)')
        nav = ndimage.label(dist >= RAIO)[0]
        anterior = None
        for w in [{'x': PONTOS['largada'][0], 'y': PONTOS['largada'][1],
                   'alvo': 'largada'}] + rota_waypoints():
            col = int(round((w['x'] - x0) / RES))
            lin = int(round((H - 1) - (w['y'] - y0) / RES))
            folga, trecho = dist[lin, col], nav[lin, col]
            ok_folga = folga >= RAIO
            ligado = anterior is None or (trecho != 0 and trecho == anterior)
            diz(ok_folga and ligado,
                f"{w['alvo']:9s} ({w['x']:6.2f},{w['y']:5.2f}) folga {folga:.2f} m"
                + ('' if ligado else '  ⚠️ SEPARADO do anterior'))
            anterior = trecho
    except ImportError:
        print('  (pulado: numpy/scipy ausentes)')

    print('\n[conferir] ' + ('TUDO CERTO' if ok else 'FALHOU'))
    return 0 if ok else 1


# ---------------------------------------------------------------------- sdf
def _caixa(nome, cx, cy, sx, sy, sz, cor):
    return (f'    <model name="{nome}">\n'
            f'      <static>true</static><pose>{cx:.4f} {cy:.4f} {sz/2:.4f} 0 0 0</pose>\n'
            f'      <link name="link">\n'
            f'        <collision name="col"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>\n'
            f'        <visual name="vis"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>\n'
            f'          <material><ambient>{cor}</ambient><diffuse>{cor}</diffuse></material>\n'
            f'        </visual>\n      </link>\n    </model>\n')


def _cilindro(nome, cx, cy, r, h, cor):
    return (f'    <model name="{nome}">\n'
            f'      <static>true</static><pose>{cx:.4f} {cy:.4f} {h/2:.4f} 0 0 0</pose>\n'
            f'      <link name="link">\n'
            f'        <collision name="col"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>\n'
            f'        <visual name="vis"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>\n'
            f'          <material><ambient>{cor}</ambient><diffuse>{cor}</diffuse></material>\n'
            f'        </visual>\n      </link>\n    </model>\n')


def _plataforma(nome, cx, cy):
    return (f'    <model name="{nome}">\n'
            f'      <static>true</static><pose>{cx:.4f} {cy:.4f} 0.005 0 0 0</pose>\n'
            f'      <link name="link">\n'
            f'        <visual name="vis"><geometry><box><size>{PLAT} {PLAT} 0.01</size></box></geometry>\n'
            f'          <material><ambient>0.95 0.85 0.1 1</ambient><diffuse>0.95 0.85 0.1 1</diffuse></material>\n'
            f'        </visual>\n      </link>\n    </model>\n')


def corpo_sdf():
    p = []
    for n, cx, cy, sx, sy in muros():
        p.append(_caixa(n, cx, cy, sx, sy, H_MURO, '0.6 0.6 0.62 1'))
    for n, cx, cy, sx, sy in blocos():
        p.append(_caixa(n, cx, cy, sx, sy, H_BLOCO, '0.25 0.35 0.55 1'))
    for n, (x, y, tem) in PONTOS.items():
        p.append(_plataforma(f'plat_{n}', x, y))
        if tem:
            p.append(_cilindro(n, x, y, R_CONE, H_CONE, '0.95 0.35 0.05 1'))
    return ''.join(p)


# --------------------------------------------------------------------- mapa
def gera_mapa(destino_pgm, destino_yaml, com_cones=False, fecha=()):
    import numpy as np
    for d in (destino_pgm, destino_yaml):
        os.makedirs(os.path.dirname(os.path.abspath(d)), exist_ok=True)
    margem = 0.5
    x0, y0 = -T_MURO - margem, -T_MURO - margem
    W = int(round((GALPAO_X + 2 * T_MURO + 2 * margem) / RES))
    H = int(round((GALPAO_Y + 2 * T_MURO + 2 * margem) / RES))
    a = np.full((H, W), 254, dtype=np.uint8)          # livre

    def pinta_caixa(cx, cy, sx, sy):
        # EXATO, meio-aberto. Com floor/ceil + 1 (como estava) o bloco crescia ate
        # uma celula por lado e a fresta ENCOLHIA ate 0.10 m: a de 0.70 media 0.60
        # e aparecia como "fechada" no validador. O mapa tem que medir a mesma
        # largura que a geometria continua, senao o validador mente.
        c0 = int(round((cx - sx / 2 - x0) / RES))
        c1 = int(round((cx + sx / 2 - x0) / RES))
        l1 = int(round((H - 1) - (cy - sy / 2 - y0) / RES))
        l0 = int(round((H - 1) - (cy + sy / 2 - y0) / RES))
        a[max(l0, 0):min(l1, H), max(c0, 0):min(c1, W)] = 0

    for _n, cx, cy, sx, sy in muros() + blocos():
        pinta_caixa(cx, cy, sx, sy)
    # TAMPÃO: fecha a fresta SÓ aqui, no mapa do planejador. O mundo (SDF) segue
    # com o vão aberto — é a quebra deliberada da invariante "mapa = mundo", para
    # obrigar o Theta* a ir pelo contorno sem alterar o experimento físico.
    # Coberta por test_arena_tampao.py (o SDF é afirmado idêntico lá).
    for nome in fecha:
        pinta_caixa(*tampao(nome))
    if com_cones:
        for n, (x, y, tem) in PONTOS.items():
            if tem:
                pinta_caixa(x, y, 2 * R_CONE, 2 * R_CONE)

    with open(destino_pgm, 'wb') as f:
        f.write(b'P5\n')
        f.write(b'# arena_galpao - gerado por tools/gera_arena_galpao.py\n')
        f.write(b'%d %d\n255\n' % (W, H))
        f.write(a.tobytes())
    with open(destino_yaml, 'w') as f:
        f.write(f'image: {os.path.basename(destino_pgm)}\n'
                f'resolution: {RES}\n'
                f'origin: [{x0:.3f}, {y0:.3f}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return W, H, (x0, y0)


CABECALHO_SDF = r"""<?xml version="1.0"?>
<!--
  ARENA DO GALPÃO — prova de 2026-09-05.  Gerado a partir da tabela em
  tools/gera_arena_galpao.py (fonte única da geometria; não editar à mão).

  A MISSÃO: largada -> 4 cones -> chegada. Cada cone fica numa PLATAFORMA
  AMARELA grande; chegar a 20 cm do cone = está na plataforma = ponto marcado.
  Entre os pontos há blocos com frestas. PASSAR PELA FRESTA É OPCIONAL: é atalho,
  e SEMPRE existe contorno. Sem limite de tempo. Critério = chegar sem bater.

  ┌────────────────────────────────────────────────────────────────┐
  │ y                                                              │
  │ 9  ░░░░░░░░░[C_60]░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │
  │ 8              ▓C4          ░                    ▓C3           │
  │ 7                           ░(fresta 0.60 em y 7.20-7.80)      │
  │ 6                           ░                                  │
  │ 5  ──[D_80]──   ──────      ░        ─────[B_70]──────         │
  │ 4      (fresta 0.80         (fresta 0.70 em x 11.05-11.75)     │
  │ 3       em x 2.20-3.00)  ░                                     │
  │ 2  ▣chegada              ░(fresta 0.90 em y 1.80-2.70)  ▓C2    │
  │ 1  ▣largada    ▓C1       ░                                     │
  │ 0  ────────────────────────────────────────────────────────    │
  │    0    2    4    6    8   10   12   14                    x   │
  └────────────────────────────────────────────────────────────────┘

  GEOMETRIA, e por quê cada número:
  - Galpão 14 x 9 m com PAREDE nos quatro lados. O dono confirmou galpão fechado;
    parede é o que dá estrutura pro AMCL. Sem ela o mapa seria quase vazio.
  - CONE = cilindro r 0.17 (34 cm de largura aparente), altura 0.70. Passa na
    janela 0.04-0.45 m do cone_detector. É OBJETIVO, não obstáculo.
  - PLATAFORMA = placa amarela 1.2 x 1.2, **só <visual>, sem <collision>**. É
    marca de chão: o laser não vê, e o colisao.py (que desde 2026-08-28 só lê
    <collision>) não a conta como obstáculo. Sem isso, pisar no alvo acusaria
    "colisão".
  - BLOCO = 0.80 m de altura (acima de qualquer altura de LiDAR em discussão) e
    **0.60 m de espessura**. A espessura NÃO é estética: visto de topo, um bloco
    fino apresentaria uma face de ~0.40 m, que cai dentro da janela 0.04-0.45 do
    cone_detector e viraria CONE FALSO. Com 0.60 ele fica acima do teto da janela.
    De quebra, 0.60 de espessura faz a fresta ser um túnel curto, que é o caso
    realista pro door_crossing.
  - Cada par de blocos é uma ILHA: a ponta encostada no muro deixa < 0.30 m
    (vedada) e a outra ponta fica aberta = o contorno.

  DISTÂNCIAS CONFERIDAS (tools/gera_arena_galpao.py (modo conferir)):
  todo bloco fica a >= 1.7 m do cone mais próximo (não funde cluster no detector
  nem esconde o alvo) e os cones ficam a >= 3 m entre si.

  ⚠️ COM robot_radius 0.32 O NAV2 FECHA A FRESTA DE 0.60 — isso é ESPERADO, não
  bug: ela é atalho opcional e o modo conservador contorna. Confirme com
  tools/mapa_passagens.py com probe local (o percentual do maior componente não basta).
-->
<sdf version="1.9">
  <world name="arena_galpao">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <!-- IMU: sem este system o <sensor type="imu"> do sim_robot fica MUDO e o
         sim volta a estimar yaw por RODA (o modo degradado do robô real).
         Ver 2026-08-26, project_sim_imu_yaw. -->
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"></plugin>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.75 0.75 0.72 1</ambient>
            <diffuse>0.75 0.75 0.72 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

"""


def escreve_sdf(destino):
    os.makedirs(os.path.dirname(os.path.abspath(destino)), exist_ok=True)
    open(destino, "w").write(CABECALHO_SDF + corpo_sdf() + "  </world>\n</sdf>\n")


def escreve_rota(destino, pre_fresta=False):
    """Rota da missao: standoff de cada cone + chegada, na ordem.

    O goal do nav2 NAO vai no cone: cone (r 0.17) + robot_radius (0.32) da
    fronteira letal a 0.49 m, e a inflacao de 0.60 estende o custo ate 0.77 m
    do centro. Um goal ali nasce dentro de obstaculo/inflacao e o nav2 recusa.
    O goal fica no segmento vindo do ponto anterior, com o yaw APONTANDO pro
    cone — e' dali que a aproximacao final (A2) assume.

    STANDOFF 1.0 -> 1.4 em 2026-09-01 (DIARIO_ARENA §2G.8/§2G.9). O que manda
    aqui NAO e' so' "o goal ser planejavel": e' o POINT-TURN que o seguidor faz
    ao concluir o goal, girando NO LUGAR pra encarar o proximo. Nesse giro o
    canto do robo varre hypot(0.25,0.25) = 0.354 m, e o cone ocupa 0.17:

        margem do point-turn = STANDOFF - 0.354 - 0.17

    Com 1.0 isso da 0.477 m — e o erro de pose do AMCL medido nesta arena chega
    a 0.45 m (item 2c). Margem e erro do MESMO tamanho: na volta `nominal1` o
    robo parou 0.44 m alem do standoff, girou a 0.523 m do centro do cone e
    raspou 18 vezes (folga 0.0000). Com 1.4 a margem vai a 0.877 m — o dobro do
    pior erro medido. NAO e' correcao do defeito (item 1, o giro segue cego ao
    anel): e' mitigacao, e sai da tabela, sem codigo novo.
    """
    import json
    os.makedirs(os.path.dirname(os.path.abspath(destino)), exist_ok=True)
    json.dump({"name": "arena_galpao",
               "waypoints": rota_waypoints(pre_fresta=pre_fresta)},
              open(destino, "w"), indent=2)


def rota_waypoints(pre_fresta=False):
    """pre_fresta=True insere o goal pré-fresta A antes do cone_2 (§2H.7).
    Default OFF: a rota da prova sai IDÊNTICA a antes desta mudança."""
    ordem = ["largada", "cone_1", "cone_2", "cone_3", "cone_4", "chegada"]
    wps = []
    for i, nome in enumerate(ordem[1:], 1):
        if pre_fresta and nome == "cone_2":
            # a perna cone_1 -> cone_2 é a que atravessa a fresta A
            m = margem_pre_fresta()
            if m <= 0:
                raise SystemExit(
                    f'waypoint pré-fresta: margem de point-turn {m:+.3f} m no '
                    'pior canto do envelope de chegada — o robô bateria no '
                    'batente durante o próprio giro. Aumente PRE_FRESTA_DIST.')
            fx, fy, fyaw = ponto_pre_fresta()
            # `or 0.0` mata o -0.0 do atan2 (JSON com "-0.0" confunde quem lê)
            wps.append({"x": round(fx, 3), "y": round(fy, 3),
                        "yaw": round(fyaw, 4) or 0.0, "alvo": "pre_fresta_A"})
        ax, ay, _ = PONTOS[ordem[i - 1]]
        bx, by, tem_cone = PONTOS[nome]
        rumo = math.atan2(by - ay, bx - ax)
        if tem_cone:                      # para a STANDOFF do cone, encarando ele
            d = math.hypot(bx - ax, by - ay)
            gx = bx - STANDOFF * (bx - ax) / d
            gy = by - STANDOFF * (by - ay) / d
        else:                             # chegada nao tem cone: vai no ponto
            gx, gy = bx, by
        wps.append({"x": round(gx, 3), "y": round(gy, 3),
                    "yaw": round(rumo, 4), "alvo": nome})
    return wps


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--conferir', action='store_true')
    ap.add_argument('--sdf', metavar='DEST', help='escreve o world COMPLETO (cabeçalho + models)')
    ap.add_argument('--rota', metavar='DEST', help='escreve a rota da missão (.json do probe.py)')
    ap.add_argument('--pre-fresta', action='store_true',
                    help='SÓ COM --rota: insere o goal pré-fresta A (6,50;2,25) '
                         'antes do cone_2. É o que ARMA o door_crossing '
                         '(pendência C). Default: NÃO — a rota da prova não muda')
    ap.add_argument('--corpo-sdf', action='store_true',
                    help='imprime só os <model> (o cabeçalho do world é fixo no .sdf)')
    ap.add_argument('--mapa', metavar='DIR', help='gera arena_galpao.pgm/.yaml em DIR')
    ap.add_argument('--com-cones', action='store_true',
                    help='inclui os cones no mapa estático (default: NÃO — ver docstring)')
    ap.add_argument('--fecha-fresta', metavar='A[,B..]', default='',
                    help='SÓ COM --mapa: fecha a(s) fresta(s) no .pgm mantendo o '
                         'SDF intacto (obriga o planejador a ir pelo contorno). '
                         'Escreve arena_galpao_semX.{pgm,yaml} — nome diferente '
                         'de propósito, pro mapa tampado não ser confundido com '
                         'o oficial')
    ap.add_argument('--probes', action='store_true',
                    help='imprime os argumentos --probe pro mapa_passagens.py')
    a = ap.parse_args()
    # ANTES de qualquer ramo: o --sdf/--rota/--corpo-sdf retornam cedo, e a
    # guarda posta depois deles não guardava nada (pegado ao rodar à mão).
    fecha = [resolve_fresta(r) for r in a.fecha_fresta.split(',') if r.strip()]
    if fecha and not a.mapa:
        raise SystemExit('--fecha-fresta só vale com --mapa: ele NÃO altera o '
                         'mundo (--sdf), de propósito.')
    if a.conferir:
        raise SystemExit(conferir())
    if a.sdf:
        escreve_sdf(a.sdf); print(f'world completo escrito em {a.sdf}'); return
    if a.rota:
        escreve_rota(a.rota, pre_fresta=a.pre_fresta)
        print(f'rota escrita em {a.rota}'
              + ('  [COM waypoint pré-fresta A]' if a.pre_fresta else ''))
        if a.pre_fresta:
            print(f'  margem do point-turn no pior canto do envelope '
                  f'(xy_goal_tolerance {XY_GOAL_TOL}): '
                  f'{margem_pre_fresta():+.3f} m')
        for w in rota_waypoints(pre_fresta=a.pre_fresta):
            print(f"  {w['alvo']:9s} ({w['x']:6.2f},{w['y']:5.2f}) yaw {math.degrees(w['yaw']):6.1f}°")
        return
    if a.corpo_sdf:
        print(corpo_sdf(), end='')
        return
    if a.probes:
        for x1, y1, x2, y2, rot in PROBES:
            print(f'--probe {x1},{y1}:{x2},{y2}:{rot}')
        for nome, eixo, coord, faixas, esperada, _c in OBST:
            meio = (faixas[0][1] + faixas[1][0]) / 2
            px, py = (coord, meio) if eixo == 'x' else (meio, coord)
            print(f'--folga {px},{py}:{nome}_esperado{esperada:.2f}')
        return
    if a.mapa:
        sufixo = ('_sem' + ''.join(n.split('_')[0] for n in fecha)) if fecha else ''
        base = 'arena_galpao' + sufixo
        W, H, o = gera_mapa(os.path.join(a.mapa, base + '.pgm'),
                            os.path.join(a.mapa, base + '.yaml'), a.com_cones,
                            fecha=fecha)
        print(f'mapa {base} {W}x{H} células @ {RES} m, origin {o}, '
              f'cones {"INCLUÍDOS" if a.com_cones else "fora (o cone é objetivo)"}')
        if fecha:
            print('  TAMPÃO no .pgm: ' + ', '.join(fecha)
                  + '  (o SDF/mundo NÃO muda — o vão continua fisicamente aberto)')
        # Portas do door_crossing, no mesmo passo e da mesma tabela (§5.1 do
        # spec). Sai junto com o mapa de propósito: mapa e portas que discordam
        # é como o robô arma uma travessia em cima de parede.
        pts = portas(fecha=fecha)
        destino = os.path.join(a.mapa, base + '.doors.json')
        with open(destino, 'w', encoding='utf-8') as f:
            json.dump({'doors': pts}, f, indent=1)
            f.write('\n')
        if pts:
            for d in pts:
                nome = MARCADAS_COMO_PORTA[d['id'] - 1]
                print(f'  porta {d["id"]} ({nome}): {d["a"]} -> {d["b"]}  '
                      f'margem de point-turn {margem_point_turn(nome):+.3f} m')
        else:
            print(f'  {destino}: SEM portas (as marcadas estão tampadas no mapa)')
        return
    ap.print_help()


if __name__ == '__main__':
    main()
