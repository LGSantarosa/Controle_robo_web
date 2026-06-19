import math

import pytest

from robot_nav.door_crossing import (
    DoorGeom,
    door_geometry,
    door_progress_lateral,
    crossing_yaw,
)


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


def test_idle_sem_goal_ou_fora_da_zona():
    dc = mk()
    # na zona mas sem goal
    assert step(dc, 0.0, (1.5, 1.2, math.pi/2), goal=False).state == 'idle'
    # com goal mas longe (>zone_radius do centro)
    assert step(dc, 0.1, (1.5, -1.0, math.pi/2)).state == 'idle'
    # sem pose (TF caiu) nunca arma
    assert step(dc, 0.2, None).state == 'idle'


def test_arma_e_vai_pro_rotating():
    # 2026-06-19: a web (nav2 via ponto-pré-porta) entrega o robô centrado na
    # frente da porta -> o door arma DIRETO no rotating (só alinha o ângulo),
    # sem staging (que perseguia o centro do vão e estragava a posição boa).
    dc = mk()
    c = step(dc, 0.0, (1.5, 1.0, math.pi/2))
    assert c.state == 'rotating'
    assert c.door_id == 1
    assert dc.side == +1


def test_staging_converge_e_rotaciona():
    # staging não é mais o caminho do arme; só é alcançado como recuperação
    # pós-escape. Testado direto aqui (força o estado).
    dc = mk()
    step(dc, 0.0, (1.7, 1.2, 0.0))     # arma -> rotating
    dc.state = 'staging'               # recuperação: staging
    dc._align_t0 = 0.0
    dc._align_anchor = (1.7, 1.2)
    c = step(dc, 0.5, (1.7, 1.2, 0.0))
    assert c.state == 'staging'        # ainda indo pro ponto no eixo
    # teleporta pro ponto de staging (simula chegada): vira ROTATING
    stage_y = 2.0 - CFG.stage_dist
    c = step(dc, 1.0, (1.5, stage_y, 0.0))
    assert c.state == 'rotating'
    assert c.vx == pytest.approx(0.0)
    assert c.wz != 0.0   # girando pra encarar pi/2


def test_rotating_estavel_vira_crossing():
    dc = mk()
    stage_y = 2.0 - CFG.stage_dist
    step(dc, 0.0, (1.5, stage_y - 0.3, math.pi/2))    # arma (staging)
    step(dc, 0.1, (1.5, stage_y, math.pi/2))          # chegou -> rotating
    # já alinhado: precisa de align_stable ticks estáveis pra promover
    t = 0.2
    for _ in range(CFG.align_stable):
        c = step(dc, t, (1.5, stage_y, math.pi/2))
        t += 0.05
    assert c.state == 'crossing'


def _ate_crossing(dc):
    stage_y = 2.0 - CFG.stage_dist
    step(dc, 0.0, (1.5, stage_y - 0.3, math.pi/2))
    step(dc, 0.1, (1.5, stage_y, math.pi/2))
    t = 0.2
    for _ in range(CFG.align_stable):
        c = step(dc, t, (1.5, stage_y, math.pi/2))
        t += 0.05
    assert c.state == 'crossing'
    return t


def test_crossing_anda_reto_e_solta_depois_da_porta():
    dc = mk()
    t = _ate_crossing(dc)
    c = step(dc, t, (1.5, 1.9, math.pi/2))
    assert c.state == 'crossing' and c.vx == pytest.approx(CFG.cross_speed)
    # passou do centro + exit_margin -> solta
    c = step(dc, t + 1.0, (1.5, 2.0 + CFG.exit_margin + 0.05, math.pi/2))
    assert c.state == 'idle'


def test_crossing_aborta_se_vao_fecha_ou_goal_morre():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi/2), gap=0.3).state == 'idle'
    dc2 = mk()
    t2 = _ate_crossing(dc2)
    assert step(dc2, t2, (1.5, 1.9, math.pi/2), goal=False).state == 'idle'


def test_align_timeout_aborta_e_respeita_cooldown():
    dc = mk()
    step(dc, 0.0, (1.5, 1.0, math.pi/2))                       # arma
    c = step(dc, CFG.align_timeout + 0.1, (1.5, 1.0, math.pi/2))
    assert c.state == 'idle'
    # cooldown: tick seguinte ainda não rearma
    assert step(dc, CFG.align_timeout + 0.2, (1.5, 1.0, math.pi/2)).state == 'idle'
    # passado o cooldown, rearma
    t = CFG.align_timeout + CFG.retrigger_cooldown + 0.3
    assert step(dc, t, (1.5, 1.0, math.pi/2)).state == 'rotating'


def test_scan_velho_aborta_crossing():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi/2), fresh=False).state == 'idle'


def test_default_rot_speed_is_4():
    # 2026-06-16: 3.0 -> 4.0. Point-turn mais forte pra vencer o atrito do
    # skid-steer parado, sem ser agressivo a ponto de passar do |yaw|<5° (6.0
    # passava). NUNCA arco. Param ROS, sobe pra 6.0 ao vivo se patinar.
    assert DoorCrossConfig().rot_speed == 4.0


def test_default_rot_left_boost_is_1_4():
    assert DoorCrossConfig().rot_left_boost == 1.4


def test_rotating_gira_um_lado_so_sem_pingpong():
    # 2026-06-19: regressão do revert 46ec8ab reintroduziu o bang-bang -> robô
    # "doidinho esq-dir" sem nunca alinhar. Giro limpo: escolhe UM sentido e
    # mantém enquanto não cruzar o alvo (não inverte tick a tick).
    dc = mk()
    # yaw=pi pede girar pra DIREITA (want=-1) pra chegar em pi/2 (yaw_des +1)
    c = step(dc, 0.0, (1.5, 1.0, math.pi))     # arma -> rotating, gira
    assert c.state == 'rotating' and c.wz < 0.0
    d0 = dc._rot_dir
    assert d0 == -1
    # ticks seguintes ainda longe do alvo: MESMO sentido, sem inverter
    for t in (0.05, 0.10, 0.15):
        c = step(dc, t, (1.5, 1.0, math.pi - 0.05))
        assert c.wz < 0.0 and dc._rot_dir == -1


def test_rotating_para_ao_cruzar_em_vez_de_reverter():
    # ao CRUZAR o alvo (o lado necessário inverte), PARA (wz=0) e re-avalia no
    # próximo tick — não fica revertendo girando (é o que matava o limit cycle).
    dc = mk()
    c = step(dc, 0.0, (1.5, 1.0, math.pi/2 + 0.3))   # yaw_err>0 -> want=-1
    assert c.wz < 0.0 and dc._rot_dir == -1
    # próximo tick simula que passou do alvo (yaw_err<0 -> want=+1, inverteu)
    c = step(dc, 0.05, (1.5, 1.0, math.pi/2 - 0.3))
    assert c.state == 'rotating'
    assert c.wz == pytest.approx(0.0)
    assert dc._rot_dir == 0


def test_rotating_boost_de_forca_na_esquerda():
    # girar pra ESQUERDA (want=+1, wz>0) leva o rot_left_boost.
    dc = mk()
    c = step(dc, 0.0, (1.5, 1.0, math.pi/2 - 0.5))   # yaw_err<0 -> want=+1
    assert c.wz == pytest.approx(CFG.rot_speed * CFG.rot_left_boost)


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
P_STAGE = (1.5, 1.0, math.pi / 2)   # na zona, encarando a porta (centro 1.5,2.0)


def estep(dc, t, pose, front_gap=math.inf, rear_gap=math.inf,
          goal=True, nav=True, gap=math.inf, fresh=True):
    return dc.update(t, pose, [DOOR], goal, nav, gap, fresh, front_gap, rear_gap)


def test_escape_reverse_on_front_block():
    dc = DoorCrossing(ECFG)
    assert estep(dc, 0.0, P_STAGE).state == 'rotating'        # arma -> rotating
    c = estep(dc, 0.1, P_STAGE, front_gap=0.10)               # parede perto -> ré
    assert c.state == 'reversing'
    assert c.vx < 0.0 and c.wz == pytest.approx(0.0)          # ré RETA, nunca arco


def test_escape_reverse_on_substuck_timeout():
    # substuck (parado sem progredir) só vale no staging (recuperação); no
    # rotating é giro no lugar e NÃO conta como travado.
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, P_STAGE)            # arma -> rotating
    dc.state = 'staging'               # recuperação: staging
    dc._align_t0 = 0.0
    dc._align_anchor = (P_STAGE[0], P_STAGE[1])
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
    # aproximação LEGÍTIMA no staging (recuperação): o robô se desloca a cada
    # tick -> a âncora de progresso reseta o relógio, NÃO dispara a ré.
    dc = DoorCrossing(ECFG)
    estep(dc, 0.0, (1.5, 1.0, math.pi / 2))   # arma -> rotating
    dc.state = 'staging'                        # recuperação: staging
    dc._align_t0 = 0.0
    dc._align_anchor = (1.5, 1.0)
    # caminha de 1.0 -> 1.35 em y, ao longo de 7 s (bem além do substuck de 5 s)
    t, y = 0.5, 1.0
    last = None
    while t <= 7.0:
        y = min(1.35, y + 0.03)
        last = estep(dc, t, (1.5, y, math.pi / 2))
        t += 0.5
    assert last.state != 'reversing'   # nunca deu ré de escape durante o avanço


def test_escape_from_rotating_on_front_block():
    dc = DoorCrossing(ECFG)
    stage_y = 2.0 - ECFG.stage_dist
    estep(dc, 0.0, (1.5, stage_y - 0.3, math.pi / 2))   # arma (staging)
    c = estep(dc, 0.1, (1.5, stage_y, math.pi / 2))     # chegou -> rotating
    assert c.state == 'rotating'
    c = estep(dc, 0.2, (1.5, stage_y, math.pi / 2), front_gap=0.10)  # parede perto
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
