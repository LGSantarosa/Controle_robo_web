import math

import pytest

from robot_nav.path_follower import (
    wrap,
    closest_index,
    carrot_point,
    FollowConfig,
    DecisiveFollower,
)


def test_wrap():
    assert wrap(0.0) == pytest.approx(0.0)
    assert wrap(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_closest_index():
    path = [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert closest_index(path, 2.1, 0.05) == 2
    assert closest_index(path, -0.4, 0.0) == 0


def test_carrot_at_lookahead_distance():
    # caminho reto em +x, passos de 0.1 m; carrot a 1.0 m do índice 0
    path = [(i * 0.1, 0.0) for i in range(40)]
    ci, (cx, cy) = carrot_point(path, 0, lookahead=1.0)
    assert cx == pytest.approx(1.0, abs=0.1)
    assert cy == pytest.approx(0.0)


def test_carrot_follows_the_bend_not_the_goal():
    # L: reto +x até (1.0,0), depois sobe +y. Carrot de 0.5 m do começo
    # deve cair AINDA no trecho +x (não pular pro goal lá em cima).
    path = [(i * 0.1, 0.0) for i in range(11)]            # (0,0)..(1.0,0)
    path += [(1.0, j * 0.1) for j in range(1, 11)]        # sobe
    ci, (cx, cy) = carrot_point(path, 0, lookahead=0.5)
    assert (cx, cy) == pytest.approx((0.5, 0.0), abs=0.1)
    # carrot longo (1.5 m) já entra no trecho de subida
    ci2, (cx2, cy2) = carrot_point(path, 0, lookahead=1.5)
    assert cy2 > 0.1


def test_carrot_clamps_to_goal_when_path_short():
    path = [(0, 0), (0.2, 0)]
    ci, p = carrot_point(path, 0, lookahead=1.0)
    assert p == (0.2, 0)


def _fol():
    return DecisiveFollower(FollowConfig())


def test_idle_when_no_goal_or_no_path():
    f = _fol()
    assert f.update((0, 0, 0), [(1, 0), (2, 0)], goal_active=False,
                    goal_yaw=0.0).state == 'idle'
    assert f.update((0, 0, 0), None, goal_active=True, goal_yaw=0.0).state == 'idle'


def test_drives_straight_when_aligned():
    f = _fol()
    path = [(x * 0.1, 0.0) for x in range(40)]   # reto +x, robô alinhado
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=0.0)
    assert cmd.state == 'driving'
    assert cmd.vx > 0.0 and cmd.wz == pytest.approx(0.0)


def test_turns_in_place_when_misaligned_shortest_angle():
    f = _fol()
    path = [(0.0, y * 0.1) for y in range(40)]   # caminho +y, robô olha +x
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=math.pi / 2)
    assert cmd.state == 'turning'
    assert cmd.vx == pytest.approx(0.0)
    assert cmd.wz > 0.0       # menor ângulo p/ +90° é girar +


def test_hysteresis_keeps_driving_through_small_error():
    # erro ~8° (entre turn_exit 3° e turn_enter 12°): estando DRIVING, continua
    # dirigindo (não cai em pulinho). path levemente inclinado 8°.
    f = _fol()
    f.state = 'driving'
    ang = math.radians(8)
    path = [(math.cos(ang) * x * 0.1, math.sin(ang) * x * 0.1) for x in range(40)]
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=ang)
    assert cmd.state == 'driving'


def test_hysteresis_keeps_turning_until_well_aligned():
    # estando TURNING com erro ~8° (acima do exit 3°), continua girando.
    f = _fol()
    f.state = 'turning'
    ang = math.radians(8)
    path = [(math.cos(ang) * x * 0.1, math.sin(ang) * x * 0.1) for x in range(40)]
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=ang)
    assert cmd.state == 'turning'


def test_turn_magnitude_respects_min_and_max():
    f = _fol()
    cfg = f.cfg
    assert abs(f._turn_cmd(math.pi)) == pytest.approx(cfg.rot_max)
    assert abs(f._turn_cmd(math.radians(5))) == pytest.approx(cfg.rot_min)


def test_rot_min_default_beats_deadzone_crawl():
    # 2026-07-02: rot_min 2.0 comandado ≈ 10°/s real (zona-morta 1.7 +
    # resposta 0.6·(cmd−1.7)) = rastejo que parece parada. 2.4 ≈ 25°/s.
    assert FollowConfig().rot_min == pytest.approx(2.4)


def test_turn_target_frozen_while_plan_shifts():
    # entra girando pra +90° (path +y); no meio do giro o plano vira pra -y.
    # SEM freeze ele inverteria o giro (caça alvo móvel); COM freeze segue +.
    f = _fol()
    path_up = [(0.0, y * 0.1) for y in range(40)]
    cmd = f.update((0.0, 0.0, 0.0), path_up, goal_active=True, goal_yaw=math.pi / 2)
    assert cmd.state == 'turning' and cmd.wz > 0.0
    path_down = [(0.0, -y * 0.1) for y in range(40)]
    cmd2 = f.update((0.0, 0.0, math.radians(45)), path_down, goal_active=True,
                    goal_yaw=-math.pi / 2)
    assert cmd2.state == 'turning'
    assert cmd2.wz > 0.0          # continua no alvo congelado (+90°), não flipa


def test_turn_target_cleared_after_alignment():
    # alinhou com o alvo congelado -> driving e o próximo giro re-mira o plano novo.
    f = _fol()
    path_up = [(0.0, y * 0.1) for y in range(40)]
    f.update((0.0, 0.0, 0.0), path_up, goal_active=True, goal_yaw=math.pi / 2)
    cmd = f.update((0.0, 0.0, math.pi / 2), path_up, goal_active=True,
                   goal_yaw=math.pi / 2)
    assert cmd.state == 'driving'
    assert f._turn_target is None


def _rot_path(origin, ang, n=40):
    ox, oy = origin
    return [(ox + math.cos(ang) * i * 0.1, oy + math.sin(ang) * i * 0.1)
            for i in range(n)]


def _turn_then_align(f, origin=(0.0, 0.0)):
    """gira pra +90° e completa (driving); retorna a pose alinhada."""
    path_up = _rot_path(origin, math.pi / 2)
    cmd = f.update((origin[0], origin[1], 0.0), path_up, goal_active=True,
                   goal_yaw=math.pi / 2)
    assert cmd.state == 'turning'
    cmd = f.update((origin[0], origin[1], math.pi / 2), path_up,
                   goal_active=True, goal_yaw=math.pi / 2)
    assert cmd.state == 'driving'
    return (origin[0], origin[1], math.pi / 2)


def test_commit_after_turn_ignores_replan_noise():
    # CAMPO 07-03 (54 inversões de giro presas num ponto): completou um giro,
    # o replan de 1Hz mexe o carrot ~20° -> SEM compromisso ele re-girava na
    # hora (turn_enter 12°). COM compromisso (não andou commit_dist ainda),
    # só re-gira acima de turn_enter_committed (35°) -> segue dirigindo.
    f = _fol()
    pose = _turn_then_align(f)
    swung = _rot_path((0.0, 0.0), math.pi / 2 + math.radians(20))
    cmd = f.update(pose, swung, goal_active=True,
                   goal_yaw=math.pi / 2 + math.radians(20))
    assert cmd.state == 'driving'          # ignora o ruído, commita no rumo


def test_commit_breaks_on_real_course_change():
    # mudança GRANDE (>35°) logo após o giro = curva de verdade -> age na hora
    f = _fol()
    pose = _turn_then_align(f)
    swung = _rot_path((0.0, 0.0), math.pi / 2 + math.radians(60))
    cmd = f.update(pose, swung, goal_active=True,
                   goal_yaw=math.pi / 2 + math.radians(60))
    assert cmd.state == 'turning'


def test_commit_expires_after_driving_commit_dist():
    # andou commit_dist (0.35m) desde o giro -> volta ao turn_enter normal (12°)
    f = _fol()
    _turn_then_align(f)
    moved = (0.0, 0.5, math.pi / 2)        # dirigiu 0.5m em +y
    swung = _rot_path((0.0, 0.5), math.pi / 2 + math.radians(20))
    cmd = f.update(moved, swung, goal_active=True,
                   goal_yaw=math.pi / 2 + math.radians(20))
    assert cmd.state == 'turning'


def test_target_behind_keeps_last_turn_side():
    # alvo ~180° atrás: os dois lados custam igual e o SINAL do erro é ruído
    # (+178 num replan, -178 no outro) -> alternava giros de 180° pros dois
    # lados. Com lado grudento: acima de sticky_behind (150°), mantém o lado
    # do último giro.
    f = _fol()
    pose = _turn_then_align(f)             # último giro foi pra ESQUERDA (+)
    ang = math.pi / 2 - math.radians(178)  # herr = -178° (sinal pede direita)
    behind = _rot_path((0.0, 0.0), ang)
    cmd = f.update(pose, behind, goal_active=True, goal_yaw=ang)
    assert cmd.state == 'turning'
    assert cmd.wz > 0.0                    # mantém a ESQUERDA do último giro


def test_turn_target_reset_when_goal_lost():
    f = _fol()
    path_up = [(0.0, y * 0.1) for y in range(40)]
    f.update((0.0, 0.0, 0.0), path_up, goal_active=True, goal_yaw=math.pi / 2)
    cmd = f.update((0.0, 0.0, 0.0), path_up, goal_active=False, goal_yaw=None)
    assert cmd.state == 'idle'
    assert f._turn_target is None


def test_goal_turn_then_arrived():
    f = _fol()
    path = [(0.0, 0.0), (0.05, 0.0)]   # goal coladinho
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=math.pi / 2)
    assert cmd.state == 'goal_turn' and cmd.wz > 0.0
    cmd = f.update((0.0, 0.0, math.pi / 2), path, goal_active=True,
                   goal_yaw=math.pi / 2)
    assert cmd.state == 'arrived' and (cmd.vx, cmd.wz) == (0.0, 0.0)
