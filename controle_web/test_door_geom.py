import math

import pytest

from door_geom import (
    door_on_segment,
    expand_route_with_pre_door,
    pre_door_waypoint,
)


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


def test_expand_insere_pre_porta_quando_trecho_cruza():
    # robô (1.5,0.5) -> destino (1.5,4.0) cruza a porta -> insere pré-porta antes
    wps = [{'x': 1.5, 'y': 4.0, 'yaw': 0.0}]
    out = expand_route_with_pre_door((1.5, 0.5), wps, [DOOR], standoff=1.0)
    assert len(out) == 2
    assert (out[0]['x'], out[0]['y']) == pytest.approx((1.5, 1.0))   # pré-porta
    assert out[0]['yaw'] == pytest.approx(math.pi / 2)              # de frente
    assert (out[1]['x'], out[1]['y']) == pytest.approx((1.5, 4.0))  # destino original


def test_expand_nao_mexe_quando_nao_cruza():
    wps = [{'x': 1.5, 'y': 1.5, 'yaw': 0.0}]   # mesmo lado da porta
    out = expand_route_with_pre_door((1.5, 0.5), wps, [DOOR], standoff=1.0)
    assert out == wps


def test_expand_multi_waypoint_insere_so_no_trecho_que_cruza():
    # robô (1.5,0.5) -> wp1 (1.5,1.2) [não cruza] -> wp2 (1.5,4.0) [cruza]
    wps = [{'x': 1.5, 'y': 1.2, 'yaw': 0.0}, {'x': 1.5, 'y': 4.0, 'yaw': 0.0}]
    out = expand_route_with_pre_door((1.5, 0.5), wps, [DOOR], standoff=1.0)
    assert len(out) == 3
    assert (out[0]['x'], out[0]['y']) == pytest.approx((1.5, 1.2))   # wp1
    assert (out[1]['x'], out[1]['y']) == pytest.approx((1.5, 1.0))   # pré-porta (lado do wp1)
    assert (out[2]['x'], out[2]['y']) == pytest.approx((1.5, 4.0))   # wp2


def test_expand_sem_pose_do_robo_nao_mexe():
    wps = [{'x': 1.5, 'y': 4.0, 'yaw': 0.0}]
    assert expand_route_with_pre_door(None, wps, [DOOR]) == wps
