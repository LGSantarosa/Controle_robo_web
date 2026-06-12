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
