import math

import pytest

from robot_nav.path_follower import (
    wrap,
    closest_index,
    next_corner_index,
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


def test_next_corner_straight_path_returns_goal():
    # caminho totalmente reto -> sem canto -> alvo é o último ponto
    path = [(i * 0.1, 0.0) for i in range(20)]
    j = next_corner_index(path, 0, corner_tol=math.radians(20), window=3)
    assert j == len(path) - 1


def test_next_corner_L_shape_finds_the_bend():
    # reto no +x até (1.0,0), depois dobra 90° pra +y
    path = [(i * 0.1, 0.0) for i in range(11)]              # (0,0)..(1.0,0)
    path += [(1.0, j * 0.1) for j in range(1, 11)]          # sobe em +y
    j = next_corner_index(path, 0, corner_tol=math.radians(20), window=3)
    cx, cy = path[j]
    # o canto é ~ (1.0, 0.0)
    assert cx == pytest.approx(1.0, abs=0.2)
    assert cy == pytest.approx(0.0, abs=0.2)


def _fol():
    return DecisiveFollower(FollowConfig())


def test_idle_when_no_goal_or_no_path():
    f = _fol()
    assert f.update((0, 0, 0), [(1, 0), (2, 0)], goal_active=False,
                    goal_yaw=0.0).state == 'idle'
    assert f.update((0, 0, 0), None, goal_active=True, goal_yaw=0.0).state == 'idle'


def test_drives_straight_when_aligned():
    f = _fol()
    path = [(x * 0.1, 0.0) for x in range(40)]   # reto em +x, goal longe (3.9m)
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=0.0)
    assert cmd.state == 'driving'
    assert cmd.vx > 0.0
    assert cmd.wz == pytest.approx(0.0)


def test_turns_in_place_when_misaligned_shortest_angle():
    f = _fol()
    # caminho vai pra +y, robô olhando +x (erro +90°) -> gira ESQUERDA (wz>0), sem andar
    path = [(0.0, y * 0.1) for y in range(40)]
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=math.pi / 2)
    assert cmd.state == 'turning'
    assert cmd.vx == pytest.approx(0.0)
    assert cmd.wz > 0.0       # menor ângulo p/ +90° é girar +


def test_turn_picks_shortest_side_not_wrong_way():
    f = _fol()
    # alvo a -10° do heading -> deve girar NEGATIVO (direita), nunca +350°
    # robô olhando +x (yaw 0), caminho levemente pra -y
    path = [(x * 0.1, -x * 0.02) for x in range(40)]
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=0.0)
    # erro pequeno (~ -11°): logo abaixo/acima do turn_tol; o que importa é o SINAL
    if cmd.state == 'turning':
        assert cmd.wz < 0.0


def test_turn_magnitude_respects_min_and_max():
    f = _fol()
    cfg = f.cfg
    # erro enorme (180°) -> satura no rot_max
    assert abs(f._turn_cmd(math.pi)) == pytest.approx(cfg.rot_max)
    # erro pequeno -> piso rot_min (vence a zona-morta)
    small = f._turn_cmd(math.radians(5))
    assert abs(small) == pytest.approx(cfg.rot_min)


def test_goal_turn_then_arrived():
    f = _fol()
    path = [(0.0, 0.0), (0.05, 0.0)]   # goal coladinho
    # robô em cima do goal, mas yaw errado -> goal_turn
    cmd = f.update((0.0, 0.0, 0.0), path, goal_active=True, goal_yaw=math.pi / 2)
    assert cmd.state == 'goal_turn'
    assert cmd.wz > 0.0
    # agora já encarando o yaw do goal -> arrived
    cmd = f.update((0.0, 0.0, math.pi / 2), path, goal_active=True,
                   goal_yaw=math.pi / 2)
    assert cmd.state == 'arrived'
    assert (cmd.vx, cmd.wz) == (0.0, 0.0)
