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
# Config FIXA do teste (independente da afinação de produção). Estes testes
# verificam a MÁQUINA DE ESTADOS, não os números de campo.
CFG = DoorCrossConfig(zone_radius=1.2, total_timeout=40.0)


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


def _ate_positioning(dc, t=0.0):
    c = step_wp(dc, t, (1.5, 1.0, math.pi / 2))
    assert c.state == 'positioning'
    return GDEST


def test_positioning_succeeded_vai_pro_rotating():
    dc = mk(); _ate_positioning(dc)
    c = step_wp(dc, 0.1, (1.5, 1.0, math.pi / 2), wp_status='succeeded')
    assert c.state == 'rotating'


def test_positioning_aborted_remanda_w_ate_o_limite():
    dc = mk(); _ate_positioning(dc)
    c = step_wp(dc, 0.1, (1.5, 1.0, math.pi / 2), wp_status='aborted')
    assert c.state == 'positioning' and c.nav[0] == 'goto'     # 1a falha -> retry
    c = step_wp(dc, 0.2, (1.5, 1.0, math.pi / 2), wp_status='aborted')
    assert c.state == 'positioning' and c.nav[0] == 'goto'     # 2a falha -> retry
    c = step_wp(dc, 0.3, (1.5, 1.0, math.pi / 2), wp_status='aborted')
    assert c.state == 'idle'                                   # estourou -> desiste


def test_positioning_timeout_conta_como_falha():
    dc = mk(); _ate_positioning(dc)
    c = step_wp(dc, CFG.wp_timeout + 0.1, (1.5, 1.0, math.pi / 2),
                wp_status='active')                            # nunca chegou
    assert c.state == 'positioning' and c.nav[0] == 'goto'     # re-mandou W


def test_positioning_novo_goal_cancela():
    dc = mk(); _ate_positioning(dc)
    c = step_wp(dc, 0.1, (1.5, 1.0, math.pi / 2), wp_status='active',
                goal_g=(9.0, 9.0, 0.0))                        # destino mudou
    assert c.state == 'idle' and c.nav == ('cancel',)


def test_rotating_alinha_e_vai_pro_crossing_mesmo_fora_do_eixo():
    # nav2 entrega em W mas ~20cm fora do eixo (tolerância dele); o rotating só
    # alinha o YAW (point-turn não corrige lateral) e DEVE ir pro crossing mesmo
    # com |d|>fit — o crossing corrige o lateral andando (Task 5). Se exigisse fit
    # aqui, ficaria preso girando (point-turn não reduz d).
    dc = mk(); _ate_positioning(dc)
    step_wp(dc, 0.1, (1.5, 1.0, math.pi / 2), wp_status='succeeded')  # -> rotating
    off = (1.5 + 0.20, 1.0, math.pi / 2 - 0.5)      # 20cm fora, 28° torto
    step_wp(dc, 0.2, off)
    aligned_off = (1.5 + 0.20, 1.0, math.pi / 2)    # alinhado, ainda 20cm fora
    step_wp(dc, 0.25, aligned_off)                  # taxa alta
    c = step_wp(dc, 0.30, aligned_off)              # assentou -> crossing
    assert c.state == 'crossing'


def _into_crossing_from_w(dc):
    _ate_positioning(dc)
    step_wp(dc, 0.1, (1.5, 1.0, math.pi / 2), wp_status='succeeded')   # -> rotating
    step_wp(dc, 0.2, (1.5, 1.0, math.pi / 2))                         # taxa alta
    c = step_wp(dc, 0.25, (1.5, 1.0, math.pi / 2))                    # -> crossing
    assert c.state == 'crossing'
    return GDEST


def test_crossing_desde_w_corrige_lateral_andando():
    # em W, 15cm fora do eixo (ainda longe, s=-0.8): anda corrigindo o lateral
    dc = mk(); _into_crossing_from_w(dc)
    c = step_wp(dc, 0.4, (1.5 + 0.15, 1.2, math.pi / 2))
    assert c.state == 'crossing' and c.vx == pytest.approx(CFG.cross_speed)
    assert c.wz < 0          # corrige o +15cm de volta pro eixo


def test_crossing_aborta_se_descentrado_perto_dos_batentes():
    # perto dos batentes (s=-0.1 > -jamb_safety) e ainda 18cm fora (>fit) -> ABORTA
    dc = mk(); _into_crossing_from_w(dc)
    c = step_wp(dc, 0.4, (1.5 + 0.18, 1.9, math.pi / 2))
    assert c.state == 'positioning'          # volta a re-posicionar
    assert c.nav[0] == 'goto'


def test_crossing_solta_remandando_g():
    dc = mk(); G = _into_crossing_from_w(dc)
    c = step_wp(dc, 0.5, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi / 2))
    assert c.state == 'idle'
    assert c.nav == ('goto', G)              # continua pro destino do usuário


def test_idle_sem_goal_ou_fora_da_zona():
    dc = mk()
    # na zona mas sem goal
    assert step(dc, 0.0, (1.5, 1.2, math.pi/2), goal=False).state == 'idle'
    # com goal mas longe (>zone_radius do centro)
    assert step(dc, 0.1, (1.5, -1.0, math.pi/2)).state == 'idle'
    # sem pose (TF caiu) nunca arma
    assert step(dc, 0.2, None).state == 'idle'

# --- helpers do fluxo novo p/ os testes de crossing/rotating -------------------

def _ate_crossing(dc):
    """Leva o dc até o estado crossing pelo fluxo novo (positioning->rotating->
    crossing, centrado em W). Devolve o tempo do último tick."""
    _into_crossing_from_w(dc)
    return 0.25


def _into_rotating(dc, yaw, t0=0.0):
    """Arma (positioning), nav2 entrega em W (succeeded -> rotating) e devolve o
    Cmd do rotating já com o `yaw` desejado (robô no eixo, longe da porta)."""
    step_wp(dc, t0, (1.5, 1.0, math.pi / 2))                              # positioning
    step_wp(dc, t0 + 0.05, (1.5, 1.0, math.pi / 2), wp_status='succeeded')  # -> rotating
    return step(dc, t0 + 0.1, (1.5, 1.0, yaw))                            # rotating c/ yaw


# --- crossing: anda reto, corrige, solta ao passar dos batentes ---------------

def test_crossing_anda_reto_e_solta_depois_da_porta():
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi / 2))
    assert c.state == 'crossing' and c.vx == pytest.approx(CFG.cross_speed)
    c = step(dc, t + 1.0, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi / 2))
    assert c.state == 'idle'


def test_crossing_solta_quando_passa_dos_batentes_nao_antes():
    # solta assim que a traseira limpa o batente (s>exit_margin 0.30), não antes.
    assert CFG.exit_margin == pytest.approx(0.30)
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 2.20, math.pi / 2)).state == 'crossing'   # s=0.20 ainda
    assert step(dc, t + 0.5, (1.5, 2.35, math.pi / 2)).state == 'idle'  # s=0.35 solta


def test_crossing_solta_mesmo_com_parede_a_frente_depois_dos_batentes():
    # passou dos batentes E parede a <stop_dist -> SOLTA (não congela no caminho B).
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t + 0.5, (1.5, 2.35, math.pi / 2), gap=0.30)
    assert c.state == 'idle'


def test_crossing_aborta_se_goal_morre():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi / 2), goal=False).state == 'idle'


# --- caminho B: door para pra PESSOA no vão (door_vel fura o collision) --------

def test_crossing_para_e_segura_se_pessoa_no_caminho():
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi / 2), gap=0.3)
    assert c.state == 'crossing' and c.vx == pytest.approx(0.0)


def test_crossing_para_mais_cedo_em_stop_dist():
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi / 2), gap=0.5)   # 0.5 < stop_dist 0.6
    assert c.state == 'crossing' and c.vx == pytest.approx(0.0)


def test_crossing_resume_quando_libera():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi / 2), gap=0.3).vx == pytest.approx(0.0)
    c = step(dc, t + 0.1, (1.5, 1.9, math.pi / 2), gap=math.inf)
    assert c.state == 'crossing' and c.vx == pytest.approx(CFG.cross_speed)


def test_crossing_stop_hold_timeout_aborta():
    dc = mk()
    t = _ate_crossing(dc)
    step(dc, t, (1.5, 1.9, math.pi / 2), gap=0.3)
    c = step(dc, t + CFG.stop_hold_timeout + 0.1, (1.5, 1.9, math.pi / 2), gap=0.3)
    assert c.state == 'idle'


def test_scan_velho_aborta_crossing():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi / 2), fresh=False).state == 'idle'


def test_cooldown_pos_travessia_nao_rearma_de_volta():
    dc = mk()
    t = _ate_crossing(dc)
    t_exit = t + 1.0
    assert step(dc, t_exit, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi / 2)).state == 'idle'
    # DENTRO do cooldown: de volta na aproximação NÃO rearma (plano defasado).
    assert step_wp(dc, t_exit + 1.0, (1.5, 1.0, math.pi / 2)).state == 'idle'
    # passado o cooldown: rearma normal (-> positioning).
    t2 = t_exit + CFG.success_cooldown + 0.2
    assert step_wp(dc, t2, (1.5, 1.0, math.pi / 2)).state == 'positioning'


# --- arming pelo /plan (independe do heading) ---------------------------------

def test_plan_crosses_door_geometria():
    a, b = (1.0, 2.0), (2.0, 2.0)
    assert plan_crosses_door([(1.5, 1.0), (1.5, 3.0)], a, b) is True
    assert plan_crosses_door([(1.5, 1.0), (1.5, 1.8)], a, b) is False
    assert plan_crosses_door([(3.0, 1.0), (3.0, 3.0)], a, b) is False
    assert plan_crosses_door([], a, b) is False
    assert plan_crosses_door([(1.5, 1.0)], a, b) is False


def test_sem_plano_de_costas_nao_arma():
    dc = mk()
    c = step_wp(dc, 0.0, (1.5, 1.0, -math.pi / 2), plan=None)   # bearing barra (>70°)
    assert c.state == 'idle'


def test_arma_pelo_plano_mesmo_de_costas():
    # /plan cruza a porta -> arma mesmo de costas (novo fluxo: -> positioning).
    dc = mk()
    c = step_wp(dc, 0.0, (1.5, 1.0, -math.pi / 2))
    assert c.state == 'positioning'


def test_plano_que_nao_cruza_nao_arma_nem_encarando():
    dc = mk()
    c = step_wp(dc, 0.0, (1.5, 1.0, math.pi / 2), plan=[(1.5, 1.0), (1.5, 1.8)])
    assert c.state == 'idle'


# --- giro limpo no rotating (sentido único, freio perto do alvo) --------------

def test_rotating_gira_um_lado_so_nao_inverte():
    dc = mk()
    cA = _into_rotating(dc, 0.0)                # yaw_err=-pi/2 -> esq (+)
    assert cA.state == 'rotating' and cA.wz > 0
    cB = step(dc, 0.2, (1.5, 1.0, 0.3))         # ainda do mesmo lado -> não inverte
    assert cB.state == 'rotating' and cB.wz > 0


def test_rotating_para_ao_cruzar_o_alvo():
    dc = mk()
    cA = _into_rotating(dc, 0.0)
    assert cA.wz > 0
    cB = step(dc, 0.2, (1.5, 1.0, math.pi / 2 + 0.2))   # passou do alvo -> PARA
    assert cB.state == 'rotating' and cB.wz == pytest.approx(0.0)


def test_rotating_boost_a_esquerda_nao_a_direita():
    dc = mk()
    cL = _into_rotating(dc, 0.0)               # esquerda
    assert cL.wz == pytest.approx(CFG.rot_speed * CFG.rot_left_boost)
    dc2 = mk()
    cR = _into_rotating(dc2, math.pi / 2 + 0.5)  # direita
    assert cR.wz == pytest.approx(-CFG.rot_speed)


def test_giro_freia_perto_do_alvo():
    dc = mk()
    cFar = _into_rotating(dc, 0.0)                    # longe -> cheia
    assert abs(cFar.wz) == pytest.approx(CFG.rot_speed * CFG.rot_left_boost)
    dc2 = mk()
    cNear = _into_rotating(dc2, math.pi / 2 - 0.15)   # 8.6° -> freio
    assert abs(cNear.wz) == pytest.approx(CFG.rot_brake_speed)


def test_cfg_mutation_is_live():
    # o callback de param do nó muta self.cfg em runtime; a máquina relê todo tick.
    cfg = DoorCrossConfig(zone_radius=1.2, total_timeout=40.0)
    dc = DoorCrossing(cfg)
    step_wp(dc, 0.0, (1.5, 1.0, math.pi / 2))                              # positioning
    step_wp(dc, 0.05, (1.5, 1.0, math.pi / 2), wp_status='succeeded')      # -> rotating
    pose = (1.5, 1.0, math.pi / 2 - 0.15)         # 8.6° torto
    assert step(dc, 0.1, pose).state == 'rotating'    # 8.6° > 5° default
    cfg.align_yaw = math.radians(15.0)            # afrouxa AO VIVO
    assert step(dc, 0.15, pose).state == 'crossing'   # 8.6° < 15° -> cruza


# --- funções puras ------------------------------------------------------------

from robot_nav.door_crossing import nav_engaging, nearest_door_in_zone, fit_lat


def test_nav_engaging_true_when_rotating_or_forward():
    assert nav_engaging(0.0, 0.02) is True
    assert nav_engaging(0.30, 0.02) is True
    assert nav_engaging(-0.01, 0.02) is True


def test_nav_engaging_false_only_on_real_reverse():
    assert nav_engaging(-0.05, 0.02) is False


def test_nearest_door_in_zone_proximity_only():
    doors = [DOOR]
    d = nearest_door_in_zone((1.5, 1.0, -math.pi / 2), doors, zone_radius=1.2)
    assert d is not None and d['id'] == 1
    assert nearest_door_in_zone((1.5, -1.0, 0.0), doors, zone_radius=1.2) is None
    assert nearest_door_in_zone(None, doors, zone_radius=1.2) is None


def test_nearest_door_in_zone_empty_list_is_none():
    assert nearest_door_in_zone((0.0, 0.0, 0.0), [], zone_radius=1.2) is None


def test_nearest_door_in_zone_picks_closest():
    doors = [DOOR, {'id': 2, 'a': [1.0, 5.0], 'b': [2.0, 5.0]}]
    d = nearest_door_in_zone((1.5, 4.5, 0.0), doors, zone_radius=1.2)
    assert d is not None and d['id'] == 2


def test_fit_lat_porta_larga_relaxa_apertada_exige():
    cfg = DoorCrossConfig()                       # robot_half_width=0.25, margin=0.13
    larga = door_geometry((0.0, 0.0), (0.93, 0.0))      # meia 0.465
    apertada = door_geometry((0.0, 0.0), (0.70, 0.0))   # meia 0.35
    assert fit_lat(larga, cfg.robot_half_width, cfg.fit_margin) == pytest.approx(0.085)
    assert fit_lat(apertada, cfg.robot_half_width, cfg.fit_margin) == pytest.approx(0.0)


def test_default_rot_speed_is_4():
    assert DoorCrossConfig().rot_speed == 4.0


def test_default_rot_left_boost():
    assert DoorCrossConfig().rot_left_boost == 1.4
