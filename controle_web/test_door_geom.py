import math

import pytest

from door_geom import door_on_segment, pre_door_waypoint


# porta: batentes em y=2, vão x∈[1,2] (parede ao longo de x, atravessa em y)
DOOR = {'id': 1, 'a': [1.0, 2.0], 'b': [2.0, 2.0]}


def test_door_on_segment_detecta_porta_no_caminho():
    # robô em (1.5, 0.5), destino (1.5, 4.0) -> a reta cruza a porta
    d = door_on_segment((1.5, 0.5), (1.5, 4.0), [DOOR])
    assert d is not None and d['id'] == 1


def test_door_on_segment_ignora_porta_fora_do_caminho():
    # destino do MESMO lado (não cruza a porta)
    assert door_on_segment((1.5, 0.5), (1.5, 1.5), [DOOR]) is None
    # cruza a parede FORA do vão (x=3)
    assert door_on_segment((3.0, 0.5), (3.0, 4.0), [DOOR]) is None
    # sem portas
    assert door_on_segment((1.5, 0.5), (1.5, 4.0), []) is None


def test_pre_door_waypoint_no_eixo_recuado_do_lado_do_robo():
    # robô ABAIXO da porta (y<2) -> ponto-pré-porta 1m ABAIXO do centro, de frente
    wx, wy, wyaw = pre_door_waypoint([1.0, 2.0], [2.0, 2.0], (1.5, 0.5), standoff=1.0)
    assert (wx, wy) == pytest.approx((1.5, 1.0))
    assert wyaw == pytest.approx(math.pi / 2)
    # robô ACIMA da porta (y>2) -> ponto 1m ACIMA, de frente pro outro lado
    wx2, wy2, wyaw2 = pre_door_waypoint([1.0, 2.0], [2.0, 2.0], (1.5, 3.5), standoff=1.0)
    assert (wx2, wy2) == pytest.approx((1.5, 3.0))
    assert wyaw2 == pytest.approx(-math.pi / 2)


def test_pre_door_waypoint_porta_na_vertical():
    # porta vertical (batentes em x=2, vão y∈[1,2]); robô à esquerda (x<2)
    wx, wy, wyaw = pre_door_waypoint([2.0, 1.0], [2.0, 2.0], (0.5, 1.5), standoff=1.0)
    assert (wx, wy) == pytest.approx((1.0, 1.5))   # 1m à esquerda do centro (2,1.5)
    assert wyaw == pytest.approx(0.0)               # de frente (+x)
