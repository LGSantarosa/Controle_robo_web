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


def test_arma_e_vai_pro_staging():
    dc = mk()
    # a 1.0 m do centro, olhando pra porta, nav empurrando
    c = step(dc, 0.0, (1.5, 1.0, math.pi/2))
    assert c.state == 'staging'
    assert c.door_id == 1
    # side foi escolhido pra aproximação: alvo de staging fica ENTRE o robô
    # e a porta (y = 2.0 - stage_dist)
    assert dc.side == +1


def test_staging_converge_e_rotaciona():
    dc = mk()
    t = 0.0
    pose = (1.7, 1.2, 0.0)   # fora do eixo, yaw errado
    c = step(dc, t, pose)
    assert c.state == 'staging'
    # teleporta pro ponto de staging (simula chegada): vira ROTATING
    stage_y = 2.0 - CFG.stage_dist
    c = step(dc, t + 1.0, (1.5, stage_y, 0.0))
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
    assert step(dc, t, (1.5, 1.0, math.pi/2)).state == 'staging'


def test_scan_velho_aborta_crossing():
    dc = mk()
    t = _ate_crossing(dc)
    assert step(dc, t, (1.5, 1.9, math.pi/2), fresh=False).state == 'idle'


def test_default_rot_speed_is_4():
    # 2026-06-16: 3.0 -> 4.0. Point-turn mais forte pra vencer o atrito do
    # skid-steer parado, sem ser agressivo a ponto de passar do |yaw|<5° (6.0
    # passava). NUNCA arco. Param ROS, sobe pra 6.0 ao vivo se patinar.
    assert DoorCrossConfig().rot_speed == 4.0
