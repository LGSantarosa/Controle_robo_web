import math

import pytest

from robot_nav.door_crossing import (
    DoorGeom,
    door_geometry,
    door_progress_lateral,
    crossing_yaw,
    plan_crosses_door,
    pre_door_waypoint,
)


def test_pre_door_waypoint_no_eixo_recuado_de_frente():
    # usado pela WEB pra pôr o ponto-pré-porta na rota (no eixo, recuado, de frente).
    g = door_geometry((1.0, 2.0), (2.0, 2.0))   # centro (1.5,2.0), normal (0,1)
    x, y, yaw = pre_door_waypoint(g, side=+1, standoff=1.0)
    assert (x, y) == pytest.approx((1.5, 1.0))
    assert yaw == pytest.approx(math.pi / 2)
    x2, y2, yaw2 = pre_door_waypoint(g, side=-1, standoff=1.0)
    assert (x2, y2) == pytest.approx((1.5, 3.0))
    assert yaw2 == pytest.approx(-math.pi / 2)


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


# --- fluxo novo (2026-06-18): a WEB põe o ponto-pré-porta NA ROTA; o door só
# toma o volante na porta (idle -> rotating -> crossing), sem mandar goal. -----
GPLAN = [(1.5, 1.0), (1.5, 3.0)]          # rota que cruza a porta -> arma


def test_idle_sem_goal_ou_fora_da_zona():
    dc = mk()
    assert step(dc, 0.0, (1.5, 1.2, math.pi / 2), goal=False).state == 'idle'
    assert step(dc, 0.1, (1.5, -1.0, math.pi / 2)).state == 'idle'   # fora da zona
    assert step(dc, 0.2, None).state == 'idle'                        # sem pose


def test_arma_toma_o_volante_direto_pro_rotating():
    # robô na frente da porta + /plan cruza -> door assume e ALINHA (rotating).
    dc = mk()
    c = step_plan(dc, 0.0, (1.5, 1.0, math.pi / 2 - 0.5), GPLAN)   # 28° torto
    assert c.state == 'rotating'
    assert c.door_id == 1 and dc.side == +1


def test_arma_pelo_plano_mesmo_de_costas():
    # /plan cruza a porta -> arma mesmo de costas (desacoplado do heading).
    dc = mk()
    c = step_plan(dc, 0.0, (1.5, 1.0, -math.pi / 2), GPLAN)
    assert c.state == 'rotating'


def test_sem_plano_de_costas_nao_arma():
    dc = mk()
    assert step(dc, 0.0, (1.5, 1.0, -math.pi / 2)).state == 'idle'   # bearing barra


def test_plano_que_nao_cruza_nao_arma_nem_encarando():
    dc = mk()
    c = step_plan(dc, 0.0, (1.5, 1.0, math.pi / 2), [(1.5, 1.0), (1.5, 1.8)])
    assert c.state == 'idle'


def _into_rotating(dc, yaw, t0=0.0):
    """Arma (idle->rotating) com o robô na frente da porta, e devolve o Cmd do
    rotating já com o `yaw` desejado."""
    step_plan(dc, t0, (1.5, 1.0, math.pi / 2), GPLAN)   # arma -> rotating
    return step(dc, t0 + 0.1, (1.5, 1.0, yaw))          # rotating c/ yaw


def _ate_crossing(dc):
    """Leva o dc até crossing (arma -> rotating alinhado -> crossing)."""
    step_plan(dc, 0.0, (1.5, 1.0, math.pi / 2), GPLAN)   # arma -> rotating
    step(dc, 0.1, (1.5, 1.0, math.pi / 2))               # reto, taxa alta
    c = step(dc, 0.15, (1.5, 1.0, math.pi / 2))          # assentou -> crossing
    assert c.state == 'crossing'
    return 0.15


# --- rotating: alinha no lugar (ponto-turn limpo) e vai pro crossing ----------

def test_rotating_alinha_e_vai_pro_crossing():
    dc = mk()
    _ate_crossing(dc)   # já exercita rotating -> crossing


def test_rotating_alinha_mesmo_fora_do_eixo():
    # entra fora do eixo (20cm): rotating alinha o YAW e vai pro crossing mesmo
    # com |d|>fit (o crossing corrige o lateral andando). Não fica preso girando.
    dc = mk()
    step_plan(dc, 0.0, (1.5 + 0.20, 1.0, math.pi / 2 - 0.5), GPLAN)  # arma -> rotating
    step(dc, 0.1, (1.5 + 0.20, 1.0, math.pi / 2))                    # taxa alta
    c = step(dc, 0.15, (1.5 + 0.20, 1.0, math.pi / 2))               # -> crossing
    assert c.state == 'crossing'


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


# --- crossing: anda reto, corrige, solta ao passar dos batentes (SEM goal) ----

def test_crossing_anda_reto_e_solta_depois_da_porta():
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi / 2))
    assert c.state == 'crossing' and c.vx == pytest.approx(CFG.cross_speed)
    c = step(dc, t + 1.0, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi / 2))
    assert c.state == 'idle'                       # solta, nav2 segue pro destino


def test_crossing_solta_quando_passa_dos_batentes_nao_antes():
    assert CFG.exit_margin == pytest.approx(0.30)
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 2.20, math.pi / 2)).state == 'crossing'    # s=0.20
    assert step(dc, t + 0.5, (1.5, 2.35, math.pi / 2)).state == 'idle'  # s=0.35


def test_crossing_solta_mesmo_com_parede_a_frente_depois_dos_batentes():
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t + 0.5, (1.5, 2.35, math.pi / 2), gap=0.30)
    assert c.state == 'idle'


def test_crossing_aborta_se_descentrado_perto_dos_batentes():
    # perto dos batentes (s=-0.1 > -jamb_safety) e ainda 18cm fora -> ABORTA (idle)
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5 + 0.18, 1.9, math.pi / 2))
    assert c.state == 'idle'


def test_crossing_aborta_se_goal_morre():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi / 2), goal=False).state == 'idle'


def test_scan_velho_aborta_crossing():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi / 2), fresh=False).state == 'idle'


# --- caminho B: para pra PESSOA no vão ----------------------------------------

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


def test_cooldown_pos_travessia_nao_rearma_de_volta():
    dc = mk()
    t = _ate_crossing(dc)
    t_exit = t + 1.0
    assert step(dc, t_exit, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi / 2)).state == 'idle'
    assert step_plan(dc, t_exit + 1.0, (1.5, 1.0, math.pi / 2), GPLAN).state == 'idle'
    t2 = t_exit + CFG.success_cooldown + 0.2
    assert step_plan(dc, t2, (1.5, 1.0, math.pi / 2), GPLAN).state == 'rotating'


def test_cfg_mutation_is_live():
    # o callback de param do nó muta self.cfg em runtime; a máquina relê todo tick.
    cfg = DoorCrossConfig(zone_radius=1.2, total_timeout=40.0)
    dc = DoorCrossing(cfg)
    step_plan(dc, 0.0, (1.5, 1.0, math.pi / 2), GPLAN)   # arma -> rotating
    pose = (1.5, 1.0, math.pi / 2 - 0.15)         # 8.6° torto
    assert step(dc, 0.1, pose).state == 'rotating'    # 8.6° > 5° default
    cfg.align_yaw = math.radians(15.0)            # afrouxa AO VIVO
    assert step(dc, 0.15, pose).state == 'crossing'   # 8.6° < 15° -> cruza


# --- arming pelo /plan + funções puras ----------------------------------------

def test_plan_crosses_door_geometria():
    a, b = (1.0, 2.0), (2.0, 2.0)
    assert plan_crosses_door([(1.5, 1.0), (1.5, 3.0)], a, b) is True
    assert plan_crosses_door([(1.5, 1.0), (1.5, 1.8)], a, b) is False
    assert plan_crosses_door([(3.0, 1.0), (3.0, 3.0)], a, b) is False
    assert plan_crosses_door([], a, b) is False
    assert plan_crosses_door([(1.5, 1.0)], a, b) is False


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
