import math

import pytest

from robot_nav.door_crossing import (
    DoorGeom,
    door_geometry,
    door_progress_lateral,
    crossing_yaw,
    plan_crosses_door,
    pre_door_waypoint,
    Cmd,
)


def test_pre_door_waypoint_no_eixo_recuado_de_frente():
    g = door_geometry((1.0, 2.0), (2.0, 2.0))   # centro (1.5,2.0), normal (0,1)
    # side=+1: aproxima de baixo (y<2). W fica 1.0m ABAIXO do centro, no eixo,
    # de frente pra porta (yaw=+pi/2).
    x, y, yaw = pre_door_waypoint(g, side=+1, standoff=1.0)
    assert (x, y) == pytest.approx((1.5, 1.0))
    assert yaw == pytest.approx(math.pi / 2)
    # side=-1: aproxima de cima; W 1.0m ACIMA, de frente (yaw=-pi/2)
    x2, y2, yaw2 = pre_door_waypoint(g, side=-1, standoff=1.0)
    assert (x2, y2) == pytest.approx((1.5, 3.0))
    assert yaw2 == pytest.approx(-math.pi / 2)


def test_cmd_nav_default_none():
    assert Cmd('idle', 0.0, 0.0, None).nav is None


def test_door_geometry_axis_horizontal_wall():
    # parede ao longo de x (porta "olhando" pra cima/baixo)
    g = door_geometry((1.0, 2.0), (2.0, 2.0))
    assert (g.cx, g.cy) == pytest.approx((1.5, 2.0))
    assert g.half_width == pytest.approx(0.5)
    assert (g.tx, g.ty) == pytest.approx((1.0, 0.0))
    assert (g.nx, g.ny) == pytest.approx((0.0, 1.0))


def test_progress_lateral_and_side():
    g = door_geometry((1.0, 2.0), (2.0, 2.0))
    # robô 1 m "abaixo" da porta, 0.2 m à direita do centro
    s, d = door_progress_lateral(g, 1.7, 1.0, side=+1)
    assert s == pytest.approx(-1.0)   # ainda não cruzou (progresso negativo)
    assert d == pytest.approx(0.2)    # offset lateral ao longo da parede
    # mesmo ponto com side=-1: progresso inverte, lateral mantém o sinal de t
    s2, _ = door_progress_lateral(g, 1.7, 1.0, side=-1)
    assert s2 == pytest.approx(1.0)


def test_crossing_yaw_faces_normal():
    g = door_geometry((1.0, 2.0), (2.0, 2.0))
    assert crossing_yaw(g, side=+1) == pytest.approx(math.pi / 2)   # +n = +y
    assert crossing_yaw(g, side=-1) == pytest.approx(-math.pi / 2)


def test_door_geometry_diagonal():
    g = door_geometry((0.0, 0.0), (1.0, 1.0))
    assert g.half_width == pytest.approx(math.sqrt(2) / 2)
    # n perpendicular a t, ambos unitários
    assert g.tx * g.nx + g.ty * g.ny == pytest.approx(0.0)
    assert math.hypot(g.nx, g.ny) == pytest.approx(1.0)


from robot_nav.door_crossing import gap_ahead


def _scan_one_point(x_robot, y_robot):
    # constrói um scan de 8 feixes com UM ponto em (x,y) no frame do robô
    a = math.atan2(y_robot, x_robot)
    r = math.hypot(x_robot, y_robot)
    angle_min, inc = -math.pi, math.pi / 4
    ranges = [float('inf')] * 8
    idx = int(round((a - angle_min) / inc)) % 8
    ranges[idx] = r
    return ranges, angle_min, inc


def test_gap_ahead_sees_obstacle_in_corridor():
    ranges, amin, ainc = _scan_one_point(0.5, 0.0)   # bem na frente
    g = gap_ahead(ranges, amin, ainc, pose=(0.0, 0.0, 0.0),
                  jambs=[], jamb_r=0.30)
    assert g == pytest.approx(0.5, abs=0.15)  # discretização de 8 feixes


def test_gap_ahead_ignores_lateral_and_behind():
    for px, py in [(0.0, 1.0), (-0.5, 0.0), (0.5, 0.6)]:
        ranges, amin, ainc = _scan_one_point(px, py)
        g = gap_ahead(ranges, amin, ainc, pose=(0.0, 0.0, 0.0),
                      jambs=[], jamb_r=0.30)
        assert math.isinf(g)


def test_gap_ahead_excludes_marked_jamb():
    # ponto na frente, mas que em coordenadas do MAPA cai no disco do batente
    ranges, amin, ainc = _scan_one_point(0.5, 0.0)
    pose = (3.0, 4.0, 0.0)                      # robô no mapa
    jamb = (3.5, 4.0)                            # batente exatamente ali
    g = gap_ahead(ranges, amin, ainc, pose=pose,
                  jambs=[jamb], jamb_r=0.30)
    assert math.isinf(g)                         # batente não conta como vão


from robot_nav.door_crossing import DoorCrossing, DoorCrossConfig

DOOR = {'id': 1, 'a': [1.0, 2.0], 'b': [2.0, 2.0]}   # parede em x, vão 1.0 m
# Config FIXA do teste (independente da afinação de produção, que muda em campo:
# stage_dist/zone_radius/align_timeout foram retunados 2026-06-15). Estes testes
# verificam a MÁQUINA DE ESTADOS, não os números de campo.
CFG = DoorCrossConfig(zone_radius=1.2, stage_dist=0.6, align_timeout=15.0,
                      total_timeout=40.0)


def mk():
    return DoorCrossing(CFG)


def step(dc, t, pose, goal=True, nav=True, gap=math.inf, fresh=True):
    return dc.update(t, pose, [DOOR], goal, nav, gap, fresh)


def step_plan(dc, t, pose, plan, goal=True, nav=True, gap=math.inf, fresh=True):
    return dc.update(t, pose, [DOOR], goal, nav, gap, fresh, plan=plan)


# --- fluxo novo (2026-06-18): posicionar via nav2, cruzar via door --------
GPLAN = [(1.5, 1.0), (1.5, 3.0)]          # rota que cruza a porta -> arma
GDEST = (1.5, 5.0, math.pi / 2)           # destino do usuário, além da porta


def step_wp(dc, t, pose, wp_status='idle', goal_g=GDEST, plan=GPLAN,
            goal=True, nav=True, gap=math.inf, fresh=True):
    return dc.update(t, pose, [DOOR], goal, nav, gap, fresh,
                     goal_g=goal_g, wp_status=wp_status, plan=plan)


def test_arma_manda_waypoint_e_vai_pro_positioning():
    dc = mk()
    c = step_wp(dc, 0.0, (1.5, 1.0, math.pi / 2))
    assert c.state == 'positioning'
    assert c.vx == 0.0 and c.wz == 0.0       # mãos quietas: nav2 dirige
    assert c.nav[0] == 'goto'
    wx, wy, wyaw = c.nav[1]
    assert (wx, wy) == pytest.approx((1.5, 2.0 - 1.0))   # W = eixo, 1m antes
    assert wyaw == pytest.approx(math.pi / 2)


def test_nao_arma_sem_goal_g():
    dc = mk()
    c = step_wp(dc, 0.0, (1.5, 1.0, math.pi / 2), goal_g=None)
    assert c.state == 'idle' and c.nav is None


def test_idle_sem_goal_ou_fora_da_zona():
    dc = mk()
    # na zona mas sem goal
    assert step(dc, 0.0, (1.5, 1.2, math.pi/2), goal=False).state == 'idle'
    # com goal mas longe (>zone_radius do centro)
    assert step(dc, 0.1, (1.5, -1.0, math.pi/2)).state == 'idle'
    # sem pose (TF caiu) nunca arma
    assert step(dc, 0.2, None).state == 'idle'


def test_arma_e_vai_pro_staging():
    dc = mk()
    # FORA do eixo (|d|>fit) -> staging dirige pro eixo. (usa /plan p/ armar sem
    # depender do bearing nessa pose lateral)
    plan = [(1.85, 1.0), (1.85, 3.0)]
    c = step_plan(dc, 0.0, (1.85, 1.0, math.pi/2), plan)
    assert c.state == 'staging'
    assert c.door_id == 1
    assert dc.side == +1


def test_staging_converge_e_rotaciona():
    dc = mk()
    plan = [(1.85, 1.0), (1.85, 3.0)]
    pose = (1.85, 1.2, 0.0)   # fora do eixo, yaw errado
    c = step_plan(dc, 0.0, pose, plan)
    assert c.state == 'staging'
    # entra no eixo (|d|<=fit): vira ROTATING (não persegue o ponto exato)
    stage_y = 2.0 - CFG.stage_dist
    c = step_plan(dc, 1.0, (1.5, stage_y, 0.0), plan)
    assert c.state == 'rotating'
    assert c.vx == pytest.approx(0.0)
    assert c.wz != 0.0   # girando pra encarar pi/2


def _ate_crossing(dc):
    stage_y = 2.0 - CFG.stage_dist
    step(dc, 0.0, (1.5, stage_y - 0.3, math.pi/2 - 0.3))  # arma torto -> rotating
    step(dc, 0.1, (1.5, stage_y, math.pi/2))              # reto, mas taxa alta
    c = step(dc, 0.15, (1.5, stage_y, math.pi/2))         # assentou (taxa~0) -> crossing
    assert c.state == 'crossing'
    return 0.15


def test_crossing_anda_reto_e_solta_depois_da_porta():
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi/2))
    assert c.state == 'crossing' and c.vx == pytest.approx(CFG.cross_speed)
    # passou do centro + exit_margin -> solta
    c = step(dc, t + 1.0, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi/2))
    assert c.state == 'idle'


# --- standoff do giro (2026-06-18): NÃO girar colado na porta (varre o canto no
# batente -> "virou perto demais e deu uma porradona"). Só gira longe o bastante;
# colado e torto -> ré reta pra ganhar distância; colado MAS já reto -> atravessa.

def test_staging_re_pra_ganhar_standoff_se_colado_e_torto():
    dc = mk()
    plan = [(1.5, 1.0), (1.5, 3.0)]   # cruza a porta (arma sem depender do bearing)
    # COLADO: s=-0.3 (y=1.7) > -turn_standoff(0.5), no eixo (x=1.5), 17° torto
    c = step_plan(dc, 0.0, (1.5, 1.7, math.pi / 2 - 0.3), plan)
    assert c.state == 'reversing'      # ré pra manobrar LONGE, não gira colado
    assert c.vx < 0


def test_staging_gira_quando_longe_o_bastante():
    dc = mk()
    plan = [(1.5, 1.0), (1.5, 3.0)]
    # LONGE: s=-0.8 (y=1.2) <= -turn_standoff -> gira no lugar (não ré)
    c = step_plan(dc, 0.0, (1.5, 1.2, math.pi / 2 - 0.3), plan)
    assert c.state == 'rotating'


def test_colado_mas_ja_alinhado_atravessa_sem_re():
    dc = mk()
    plan = [(1.5, 1.0), (1.5, 3.0)]
    # COLADO (s=-0.2) MAS já reto (yaw=pi/2): não dá ré (sem giro = sem varrer).
    c = step_plan(dc, 0.0, (1.5, 1.8, math.pi / 2), plan)
    assert c.state == 'rotating'       # 1o tick: taxa=inf, parka (não ré, não gira)
    c = step_plan(dc, 0.1, (1.5, 1.8, math.pi / 2), plan)
    assert c.state == 'crossing'       # assentou -> atravessa direto, colado


def test_crossing_solta_quando_passa_dos_batentes_nao_antes():
    # 2026-06-18: o door SOLTA assim que a traseira limpa o batente (s>exit_margin
    # 0.30), não meio metro depois. Era na faixa pós-porta que ele costurava e
    # enlouquecia; passou dos batentes -> papel cumprido -> nav2 assume.
    assert CFG.exit_margin == pytest.approx(0.30)
    # s = y - 2.0 (centro da porta em y=2.0, normal +y, side=+1)
    dc = mk()
    t = _ate_crossing(dc)
    # s=0.20: traseira ainda no vão -> AINDA atravessando (não solta cedo demais)
    c = step(dc, t, (1.5, 2.20, math.pi / 2))
    assert c.state == 'crossing'
    # s=0.35: passou dos batentes -> SOLTA pro nav2
    c = step(dc, t + 0.5, (1.5, 2.35, math.pi / 2))
    assert c.state == 'idle'


def test_crossing_solta_mesmo_com_parede_a_frente_depois_dos_batentes():
    # 2026-06-18: REGRESSÃO de campo. Robô já passou dos batentes (s>exit_margin)
    # mas tem parede a <stop_dist à frente (corredor de saída). A saída TEM que
    # vir antes do gap-stop: senão o caminho B congelava o robô já atravessado e
    # ele só largava pelo timeout de 8s ("atravessou e ficou parado e travado").
    dc = mk()
    t = _ate_crossing(dc)
    # s=0.35 (passou) E gap=0.30 (parede colada à frente) -> SOLTA, não segura
    c = step(dc, t + 0.5, (1.5, 2.35, math.pi / 2), gap=0.30)
    assert c.state == 'idle'


def test_crossing_aborta_se_goal_morre():
    # goal cancelado durante a travessia -> larga pro nav2.
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi/2), goal=False).state == 'idle'


# --- caminho B (2026-06-17): door é a autoridade de segurança no crossing.
# Como o door_vel fura o collision monitor, uma PESSOA no caminho só é pega pela
# checagem do próprio door -> agora ele PARA (vx=0) e segura, em vez de furar cego
# (antes ia de cara na pessoa) ou de só abortar.

def test_crossing_para_e_segura_se_pessoa_no_caminho():
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi / 2), gap=0.3)
    assert c.state == 'crossing'                 # NÃO aborta — segura a posição
    assert c.vx == pytest.approx(0.0)            # PARA (não fura pra cima da pessoa)


def test_crossing_para_mais_cedo_em_stop_dist():
    # para a stop_dist (0.6), não só no gap_min (0.45) -> não chega scary-close.
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi / 2), gap=0.5)  # 0.45 < 0.5 < 0.6
    assert c.state == 'crossing' and c.vx == pytest.approx(0.0)


def test_crossing_resume_quando_libera():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi / 2), gap=0.3).vx == pytest.approx(0.0)
    c = step(dc, t + 0.1, (1.5, 1.9, math.pi / 2), gap=math.inf)   # caminho liberou
    assert c.state == 'crossing' and c.vx == pytest.approx(CFG.cross_speed)


def test_crossing_stop_hold_timeout_aborta():
    # pessoa parada bloqueando por muito tempo -> larga pro nav2 (que freia/replana).
    dc = mk()
    t = _ate_crossing(dc)
    step(dc, t, (1.5, 1.9, math.pi / 2), gap=0.3)                  # entra no hold
    c = step(dc, t + CFG.stop_hold_timeout + 0.1, (1.5, 1.9, math.pi / 2), gap=0.3)
    assert c.state == 'idle'


def test_align_timeout_aborta_e_respeita_cooldown():
    dc = mk()
    plan = [(1.85, 1.0), (1.85, 3.0)]
    p = (1.85, 1.0, math.pi/2)                                 # fora do eixo: staging
    step_plan(dc, 0.0, p, plan)                                # arma
    c = step_plan(dc, CFG.align_timeout + 0.1, p, plan)
    assert c.state == 'idle'
    # cooldown: tick seguinte ainda não rearma
    assert step_plan(dc, CFG.align_timeout + 0.2, p, plan).state == 'idle'
    # passado o cooldown, rearma
    t = CFG.align_timeout + CFG.retrigger_cooldown + 0.3
    assert step_plan(dc, t, p, plan).state == 'staging'


def test_scan_velho_aborta_crossing():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi/2), fresh=False).state == 'idle'


def test_default_rot_speed_is_4():
    # 2026-06-16: 3.0 -> 4.0. Point-turn mais forte pra vencer o atrito do
    # skid-steer parado, sem ser agressivo a ponto de passar do |yaw|<5° (6.0
    # passava). NUNCA arco. Param ROS, sobe pra 6.0 ao vivo se patinar.
    assert DoorCrossConfig().rot_speed == 4.0


def test_default_rot_left_boost():
    # paridade com o spin do unstuck (spin_left_boost=1.4).
    assert DoorCrossConfig().rot_left_boost == 1.4


def test_cfg_mutation_is_live():
    # o callback de param do nó (2026-06-17) muta self.cfg em runtime; isto
    # garante o mecanismo: a máquina de estados guarda a MESMA referência e relê
    # cfg todo tick, então o valor novo pega sem reconstruir o objeto.
    # cfg PRÓPRIO (não o CFG compartilhado) — esta mutação pode poluir os outros.
    cfg = DoorCrossConfig(zone_radius=1.2, stage_dist=0.6, align_timeout=15.0,
                          total_timeout=40.0)
    dc = DoorCrossing(cfg)
    pose = (1.5, 1.0, math.pi / 2 - 0.15)         # centrado (no eixo), 8.6° torto
    dc.update(0.0, pose, [DOOR], True, True, math.inf, True)      # arma -> rotating
    c = dc.update(0.05, pose, [DOOR], True, True, math.inf, True)  # 8.6°>5° -> rotating
    assert c.state == 'rotating'
    cfg.align_yaw = math.radians(15.0)            # afrouxa o yaw AO VIVO
    # MESMA pose, parado (taxa~0): agora 8.6° < 15° -> "passo reto daqui?" passa ->
    # crossing. Prova que a mutação pegou (mesma ref de cfg, relida todo tick).
    c = dc.update(0.10, pose, [DOOR], True, True, math.inf, True)
    assert c.state == 'crossing'


# --- arming pelo /plan (2026-06-17): o gate de bearing fechava só DEPOIS da
# curva do Nav2 -> door assumia tarde/torto. Agora arma se a ROTA cruza a porta,
# independente de pra onde o nariz aponta. -------------------------------------

def test_plan_crosses_door_geometria():
    a, b = (1.0, 2.0), (2.0, 2.0)                 # vão em y=2, x∈[1,2]
    assert plan_crosses_door([(1.5, 1.0), (1.5, 3.0)], a, b) is True
    assert plan_crosses_door([(1.5, 1.0), (1.5, 1.8)], a, b) is False  # não chega
    assert plan_crosses_door([(3.0, 1.0), (3.0, 3.0)], a, b) is False  # cruza a parede FORA do vão
    assert plan_crosses_door([], a, b) is False
    assert plan_crosses_door([(1.5, 1.0)], a, b) is False


def test_sem_plano_de_costas_nao_arma():
    # baseline: de costas pra porta e sem /plan -> o bearing barra (>70°).
    dc = mk()
    assert step(dc, 0.0, (1.5, 1.0, -math.pi / 2)).state == 'idle'


def test_arma_pelo_plano_mesmo_de_costas():
    # MESMA pose de costas, mas com /plan cruzando a porta -> arma (desacoplado
    # do heading; assume antes da curva do Nav2). No eixo (|d|=0) + de costas
    # (yaw bem torto) -> vai pro rotating alinhar no lugar.
    dc = mk()
    plan = [(1.5, 1.0), (1.5, 3.0)]
    c = step_plan(dc, 0.0, (1.5, 1.0, -math.pi / 2), plan)
    assert c.state == 'rotating'


def test_plano_que_nao_cruza_nao_arma_nem_encarando():
    # anti-falso-positivo: encarando a porta, mas a ROTA não a cruza -> NÃO
    # rouba o volante (porta que ele só passa do lado).
    dc = mk()
    plan = [(1.5, 1.0), (1.5, 1.8)]
    c = step_plan(dc, 0.0, (1.5, 1.0, math.pi / 2), plan)
    assert c.state == 'idle'


# --- aborto-de-segurança na aproximação (2026-06-17): door_vel fura o collision
# monitor, então staging/rotating precisam largar pro nav2 se um obstáculo (não
# o batente) aparece na frente. -------------------------------------------------

def test_staging_aborta_se_obstaculo_na_aproximacao():
    dc = mk()
    plan = [(1.85, 1.0), (1.85, 3.0)]
    c = step_plan(dc, 0.0, (1.85, 1.0, math.pi / 2), plan)  # fora do eixo -> staging
    assert c.state == 'staging'
    # algo (não-batente) entra na frente durante a aproximação (gap < gap_min):
    c = step_plan(dc, 0.1, (1.85, 1.05, math.pi / 2), plan, gap=0.3)
    assert c.state == 'idle'


def test_staging_segue_sem_obstaculo():
    # sem obstáculo (gap=inf) a aproximação segue normal.
    dc = mk()
    plan = [(1.85, 1.0), (1.85, 3.0)]
    step_plan(dc, 0.0, (1.85, 1.0, math.pi / 2), plan)
    c = step_plan(dc, 0.1, (1.85, 1.05, math.pi / 2), plan)  # gap default = inf
    assert c.state == 'staging'


def test_rotating_aborta_se_obstaculo_na_frente():
    dc = mk()
    stage_y = 2.0 - CFG.stage_dist
    yaw = math.pi / 2 - 0.3                               # torto: fica no rotating
    step(dc, 0.0, (1.5, stage_y - 0.3, yaw))             # arma (staging)
    c = step(dc, 0.1, (1.5, stage_y, yaw))               # chega -> rotating
    assert c.state == 'rotating'
    c = step(dc, 0.2, (1.5, stage_y, 0.0), gap=0.3)      # obstáculo na frente -> abort
    assert c.state == 'idle'


# --- giro limpo no rotating (2026-06-17): troca o bang-bang por um giro de
# sentido único que PARA ao cruzar o alvo (mata o limit cycle que arrastava o
# lateral -> ping-pong staging<->rotating). ------------------------------------

def _into_rotating(dc, yaw, t0=0.0):
    """Arma a porta (facing) e teleporta pro ponto de staging com `yaw` -> cai
    no rotating no MESMO tick (staging->rotating é fall-through). Devolve o Cmd
    do rotating já com o yaw_err desejado."""
    stage_y = 2.0 - CFG.stage_dist
    plan = [(1.85, 1.0), (1.85, 3.0)]
    step_plan(dc, t0, (1.85, stage_y, math.pi / 2), plan)  # arma FORA do eixo (staging)
    return step(dc, t0 + 0.1, (1.5, stage_y, yaw))    # entra no eixo -> rotating fresh


def test_rotating_gira_um_lado_so_nao_inverte():
    dc = mk()
    stage_y = 2.0 - CFG.stage_dist
    cA = _into_rotating(dc, 0.0)                # yaw_err=-pi/2 -> gira p/ esq (+)
    assert cA.state == 'rotating' and cA.wz > 0
    # ainda do mesmo lado do alvo (yaw_err ainda <0): NÃO inverte o sentido
    cB = step(dc, 0.2, (1.5, stage_y, 0.3))
    assert cB.state == 'rotating' and cB.wz > 0


def test_rotating_para_ao_cruzar_o_alvo():
    dc = mk()
    stage_y = 2.0 - CFG.stage_dist
    cA = _into_rotating(dc, 0.0)               # gira p/ esquerda (wz>0)
    assert cA.wz > 0
    # passou do alvo (yaw > pi/2, fora do band de 5°): PARA, não reverte girando
    cB = step(dc, 0.2, (1.5, stage_y, math.pi / 2 + 0.2))
    assert cB.state == 'rotating' and cB.wz == pytest.approx(0.0)


def test_rotating_boost_a_esquerda_nao_a_direita():
    # giro à esquerda (want=+1) leva o boost; à direita (want=-1), não.
    dc = mk()
    cL = _into_rotating(dc, 0.0)               # yaw_err=-pi/2 -> esquerda
    assert cL.wz == pytest.approx(CFG.rot_speed * CFG.rot_left_boost)
    dc2 = mk()
    cR = _into_rotating(dc2, math.pi / 2 + 0.5)  # yaw_err=+0.5 -> direita
    assert cR.wz == pytest.approx(-CFG.rot_speed)


def test_giro_freia_perto_do_alvo():
    # 2026-06-17 (3ª rodada): o giro a velocidade cheia (4.0 = 11.5°/tick) passava
    # direto da banda de ±5° e oscilava esq/dir -> bateu no batente. Agora: LONGE do
    # alvo = velocidade cheia (quebra o atrito); PERTO (< rot_brake_angle) = freio
    # (~2 rad/s, ~5°/tick) pra ENCAIXAR sem overshoot.
    dc = mk()
    cFar = _into_rotating(dc, 0.0)                    # yaw_err=-pi/2 (longe) -> cheia
    assert abs(cFar.wz) == pytest.approx(CFG.rot_speed * CFG.rot_left_boost)
    dc2 = mk()
    cNear = _into_rotating(dc2, math.pi / 2 - 0.15)   # yaw_err=-0.15 (8.6°) -> freio
    assert abs(cNear.wz) == pytest.approx(CFG.rot_brake_speed)  # freio, SEM boost


from robot_nav.door_crossing import nav_engaging


def test_nav_engaging_true_when_rotating_or_forward():
    # girando pra alinhar (linear ~0) ou indo pra frente -> engajado (arma)
    assert nav_engaging(0.0, 0.02) is True
    assert nav_engaging(0.30, 0.02) is True
    # ruído de ré minúsculo dentro da banda ainda conta como engajado
    assert nav_engaging(-0.01, 0.02) is True


def test_nav_engaging_false_only_on_real_reverse():
    # ré sustentada (abaixo de -nav_move_lin) -> NÃO arma
    assert nav_engaging(-0.05, 0.02) is False


from robot_nav.door_crossing import nearest_door_in_zone


def test_nearest_door_in_zone_proximity_only():
    doors = [DOOR]                      # centro em (1.5, 2.0)
    # dentro da zona, mas de COSTAS pra porta (cone não importa aqui)
    d = nearest_door_in_zone((1.5, 1.0, -math.pi / 2), doors, zone_radius=1.2)
    assert d is not None and d['id'] == 1
    # fora da zona -> None
    assert nearest_door_in_zone((1.5, -1.0, 0.0), doors, zone_radius=1.2) is None
    # sem pose -> None
    assert nearest_door_in_zone(None, doors, zone_radius=1.2) is None


def test_nearest_door_in_zone_empty_list_is_none():
    assert nearest_door_in_zone((0.0, 0.0, 0.0), [], zone_radius=1.2) is None


def test_nearest_door_in_zone_picks_closest():
    doors = [DOOR, {'id': 2, 'a': [1.0, 5.0], 'b': [2.0, 5.0]}]  # centro (1.5,5)
    d = nearest_door_in_zone((1.5, 4.5, 0.0), doors, zone_radius=1.2)
    assert d is not None
    assert d['id'] == 2


# ---- ré de escape (2026-06-16) -----------------------------------------------

ECFG = DoorCrossConfig(zone_radius=1.2, stage_dist=0.6, align_timeout=15.0,
                       total_timeout=40.0)
P_STAGE = (1.85, 1.0, math.pi / 2)   # na zona, FORA do eixo (|d|>fit) -> staging
P_PLAN = [(1.85, 1.0), (1.85, 3.0)]  # /plan cruza a porta (arma sem depender do bearing)


def estep(dc, t, pose, front_gap=math.inf, rear_gap=math.inf,
          goal=True, nav=True, gap=math.inf, fresh=True):
    return dc.update(t, pose, [DOOR], goal, nav, gap, fresh, front_gap, rear_gap,
                     plan=P_PLAN)


def test_escape_reverse_on_front_block():
    dc = DoorCrossing(ECFG)
    assert estep(dc, 0.0, P_STAGE).state == 'staging'         # arma
    c = estep(dc, 0.1, P_STAGE, front_gap=0.10)               # parede perto -> ré
    assert c.state == 'reversing'
    assert c.vx < 0.0 and c.wz == pytest.approx(0.0)          # ré RETA, nunca arco


def test_escape_reverse_on_substuck_timeout():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)                                   # arma (align_t0=0)
    c = estep(dc, ECFG.escape_substuck_time + 0.1, P_STAGE)   # não progrediu -> ré
    assert c.state == 'reversing'


def test_escape_aborts_when_rear_blocked():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    # parede na frente E sem vão atrás -> não força, larga pro nav2/unstuck
    c = estep(dc, 0.1, P_STAGE, front_gap=0.10, rear_gap=0.05)
    assert c.state == 'idle'


def test_escape_target_capped_by_rear_gap():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    estep(dc, 0.1, P_STAGE, front_gap=0.10, rear_gap=0.25)
    # alvo = min(escape_reverse_dist, rear_gap - escape_rear_margin) = min(0.30,0.15)
    assert dc._esc_target == pytest.approx(0.15)


def test_reverse_returns_to_staging_after_distance():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    estep(dc, 0.1, P_STAGE, front_gap=0.10)                   # -> reversing (alvo 0.30)
    # recuou 0.4 m (afastou da porta, y caiu) -> volta pro staging
    c = estep(dc, 0.5, (1.5, 0.6, math.pi / 2))
    assert c.state == 'staging'
    assert dc._align_t0 == pytest.approx(0.5)   # relógio do substuck reiniciado


def test_reverse_returns_to_staging_if_rear_closes():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    estep(dc, 0.1, P_STAGE, front_gap=0.10)                   # -> reversing
    # algo entrou atrás no meio da ré -> para e volta pro staging
    c = estep(dc, 0.2, (1.5, 0.95, math.pi / 2), rear_gap=0.05)
    assert c.state == 'staging'
    assert c.vx == pytest.approx(0.0)


def test_escape_max_count_then_abort():
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)
    t = 0.1
    for _ in range(ECFG.escape_max_count):
        assert estep(dc, t, P_STAGE, front_gap=0.10).state == 'reversing'
        # completa a ré (recua bastante) -> staging
        assert estep(dc, t + 0.05, (1.5, 0.5, math.pi / 2)).state == 'staging'
        t += 0.2
    # estourou o nº de escapes -> próximo bloqueio aborta (larga pro unstuck)
    assert estep(dc, t, P_STAGE, front_gap=0.10).state == 'idle'


def test_moving_approach_does_not_trigger_substuck():
    # aproximação LEGÍTIMA: o robô se desloca a cada tick por > substuck_time
    # total -> a âncora de progresso reseta o relógio, NÃO dispara a ré.
    dc = DoorCrossing(ECFG)
    # FORA do eixo (x=1.75, |d|=0.25>fit) o tempo todo -> fica no staging, que é
    # onde o substuck por tempo poderia disparar.
    assert estep(dc, 0.0, (1.75, 1.0, math.pi / 2)).state == 'staging'  # arma
    # caminha de 1.0 -> 1.35 em y, ao longo de 7 s (bem além do substuck de 5 s)
    t, y = 0.5, 1.0
    last = None
    while t <= 7.0:
        y = min(1.35, y + 0.03)
        last = estep(dc, t, (1.75, y, math.pi / 2))
        t += 0.5
    assert last.state != 'reversing'   # nunca deu ré de escape durante o avanço
    assert last.state == 'staging'     # seguiu aproximando, off-axis


def test_escape_from_rotating_on_front_block():
    dc = DoorCrossing(ECFG)
    stage_y = 2.0 - ECFG.stage_dist
    yaw = math.pi / 2 - 0.3                              # torto: fica no rotating
    estep(dc, 0.0, (1.5, stage_y - 0.3, yaw))           # arma (staging)
    c = estep(dc, 0.1, (1.5, stage_y, yaw))             # chegou -> rotating
    assert c.state == 'rotating'
    c = estep(dc, 0.2, (1.5, stage_y, yaw), front_gap=0.10)  # parede perto
    assert c.state == 'reversing'
    assert c.wz == pytest.approx(0.0)                   # ré RETA, nunca arco


def test_no_substuck_escape_while_rotating():
    # 2026-06-16: girar parado pra alinhar NÃO é "estar travado". O substuck por
    # TEMPO não deve disparar a ré no rotating (senão a ré reta, com a traseira
    # apontada pra porta, parecia que o robô "entrava de ré na sala"). align_timeout
    # segue como rede de segurança; obstáculo real à frente ainda dispara.
    dc = DoorCrossing(ECFG)
    stage_y = 2.0 - ECFG.stage_dist
    yaw = math.pi / 2 - 0.3                              # 17° fora do eixo -> NÃO alinha
    estep(dc, 0.0, (1.5, stage_y - 0.3, yaw))           # arma (staging)
    c = estep(dc, 0.1, (1.5, stage_y, yaw))             # chegou -> rotating
    assert c.state == 'rotating'
    # girando parado por > substuck_time, frente livre -> NÃO pode dar ré
    c = estep(dc, ECFG.escape_substuck_time + 1.0, (1.5, stage_y, yaw))
    assert c.state == 'rotating'


# --- A: "passo reto daqui?" todo tick, mas SÓ quando PAROU de girar (2026-06-17)
# A travessia ativa quando: reto (|yaw|<=align_yaw) E cabe (|lat|<=fit_lat) E o robô
# PAROU de girar (taxa de yaw ~0). Sem a trava de taxa, no meio de um giro rápido um
# tick caía na banda -> ativava crossing -> inércia levava o robô torto -> batia.

def test_nao_atravessa_girando_rapido_so_quando_assenta():
    dc = mk()
    stage_y = 2.0 - CFG.stage_dist
    step(dc, 0.0, (1.5, stage_y, math.pi / 2 - 0.4))   # arma torto, no eixo -> rotating
    # alinhado, MAS vindo de um yaw bem diferente (girando rápido) -> NÃO cruza
    c = step(dc, 0.05, (1.5, stage_y, math.pi / 2))
    assert c.state == 'rotating'
    # assentou (mesmo yaw 2 ticks -> taxa ~0) -> agora cruza
    c = step(dc, 0.10, (1.5, stage_y, math.pi / 2))
    assert c.state == 'crossing'


def test_veio_reto_e_parado_atravessa():
    # reto e centrado, sem girar: cruza assim que tem leitura de taxa (~0).
    dc = mk()
    step(dc, 0.0, (1.5, 1.0, math.pi / 2))             # arma; sem histórico de yaw
    c = step(dc, 0.05, (1.5, 1.05, math.pi / 2))        # reto e parado -> crossing
    assert c.state == 'crossing'
    assert c.vx == pytest.approx(CFG.cross_speed)


def test_no_eixo_vai_direto_pro_rotating_sem_perseguir_ponto():
    # já centrado (no eixo) mas torto: vai alinhar NO LUGAR, não persegue o ponto
    # exato de staging (era o "se enrolando no indo-pro-eixo").
    dc = mk()
    c = step(dc, 0.0, (1.5, 1.0, math.pi / 2 - 0.3))    # |d|=0, yaw torto
    assert c.state == 'rotating'
    assert c.vx == pytest.approx(0.0)                   # point-turn, não dirige pro ponto


def test_fora_do_eixo_vai_pro_staging():
    # fora do eixo (|d|>fit): aí sim staging dirige PRO eixo.
    dc = mk()
    plan = [(1.85, 1.0), (1.85, 3.0)]                   # cruza em x=1.85
    c = step_plan(dc, 0.0, (1.85, 1.0, math.pi / 2), plan)  # |d|=0.35 > fit
    assert c.state == 'staging'
    assert c.vx == pytest.approx(CFG.stage_speed)       # dirige pro eixo


def test_passo_reto_respeita_gap_na_frente():
    # reto e centrado, MAS obstáculo no vão (gap < gap_min) -> aborta, não fura.
    dc = mk()
    c = step(dc, 0.0, (1.5, 1.0, math.pi / 2), gap=0.3)
    assert c.state == 'idle'


# --- B: folga geométrica (fit_lat) + fim do ping-pong (2026-06-17) ------------

from robot_nav.door_crossing import fit_lat


def test_fit_lat_porta_larga_relaxa_apertada_exige():
    cfg = DoorCrossConfig()                       # robot_half_width=0.25, margin=0.13
    larga = door_geometry((0.0, 0.0), (0.93, 0.0))      # meia 0.465
    apertada = door_geometry((0.0, 0.0), (0.70, 0.0))   # meia 0.35
    # porta real 0.93: 0.465-0.25-0.13 = 0.085 -> só cruza com |lat|<8.5cm (centrado)
    assert fit_lat(larga, cfg.robot_half_width, cfg.fit_margin) == pytest.approx(0.085)
    # apertada demais (0.70): não cabe com folga -> 0 (só dead-center)
    assert fit_lat(apertada, cfg.robot_half_width, cfg.fit_margin) == pytest.approx(0.0)


def test_rotating_drift_pequeno_nao_volta_pro_staging():
    # antes (align_lat=0.08) um drift de 10 cm jogava de volta pro staging =
    # ping-pong "caçando o meio". Agora o gate é fit_lat (0.12 nesta porta de 1.0m
    # com margem 0.13) -> drift de 10cm dentro da folga segue no rotating, sem bounce.
    dc = mk()
    stage_y = 2.0 - CFG.stage_dist
    _into_rotating(dc, 0.0)
    c = step(dc, 0.3, (1.5 + 0.10, stage_y, 0.3))   # d=0.10 (<0.20), ainda torto
    assert c.state == 'rotating'


# --- C: trava pós-travessia (cooldown 2 s) (2026-06-17) -----------------------

def test_cooldown_pos_travessia_nao_rearma_de_volta():
    dc = mk()
    t = _ate_crossing(dc)
    t_exit = t + 1.0
    # passou do centro + exit_margin -> solta
    assert step(dc, t_exit, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi / 2)).state == 'idle'
    # DENTRO do cooldown: mesmo de volta na aproximação, NÃO rearma (era o bug do
    # /plan defasado -> invertia o side -> voltava pra porta).
    assert step(dc, t_exit + 1.0, (1.5, 1.0, math.pi / 2)).state == 'idle'
    # passado o success_cooldown: rearma normal (reto+centrado, parado: 1 tick
    # sem histórico de taxa, no 2º assenta -> crossing).
    t2 = t_exit + CFG.success_cooldown + 0.2
    step(dc, t2, (1.5, 1.0, math.pi / 2))                  # rearma -> rotating
    c = step(dc, t2 + 0.05, (1.5, 1.0, math.pi / 2))       # assentou -> crossing
    assert c.state == 'crossing'
