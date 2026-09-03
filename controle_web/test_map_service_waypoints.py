"""Testes do _wp_runner (rota de waypoints) — foco no shutdown.

BO 07-07: derrubar o app com waypoints ativos gerava traceback no
wait_for_service (nó rclpy já destruído durante o retry do runner).
"""
import threading
import types
from unittest.mock import Mock

from map_service import MapBridge


def _fake_bridge(srv):
    fb = types.SimpleNamespace()
    fb._wp_list = [{'x': 1.0, 'y': 2.0, 'yaw': 0.0}]
    fb._wp_current_idx = 0
    fb._wp_loop = False
    fb._wp_active = True
    fb._wp_lock = threading.Lock()
    fb._wp_stop = threading.Event()
    fb._wp_goal_done = threading.Event()
    fb._wp_goal_handle = None
    fb._wp_goal_status = 0
    fb._sock = Mock()
    fb._clear_costmap_srv = srv
    fb._wp_send_goal_action = Mock()
    fb._pulse_goal_light = Mock()
    return fb


def test_runner_sai_limpo_com_no_destruido():
    """wait_for_service explode (nó destruído no shutdown) → runner encerra
    sem propagar exceção e não tenta mandar goal num nó morto."""
    srv = Mock()
    srv.wait_for_service.side_effect = Exception(
        'cannot use Destroyable because destruction was requested')
    fb = _fake_bridge(srv)
    MapBridge._wp_runner(fb)  # não deve levantar
    assert fb._wp_send_goal_action.call_count == 0
    assert fb._wp_active is False


def test_runner_manda_goal_quando_servico_ok():
    """Caminho feliz do _send: serviço responde → goal é enviado."""
    srv = Mock()
    srv.wait_for_service.return_value = True
    fb = _fake_bridge(srv)
    fb._wp_stop.set()  # para o loop logo após o 1º _send
    MapBridge._wp_runner(fb)
    srv.call_async.assert_called_once()
    assert fb._wp_send_goal_action.call_count == 0  # _wp_stop já setado: não manda


def test_expand_route_via_plan_cai_na_reta_quando_o_plan_contorna_a_porta():
    """Arena 2026-09-02: a fresta C pode ser tratada como parede pelo Nav2.

    Nesse caso o /plan faz contorno e `door_on_path()` não vê a porta, mas o
    trecho da rota foi desenhado PARA atravessar ela. Sem o fallback da reta o
    pré-porta não entra e o door_crossing nunca ganha a chance de assumir.
    """
    fb = types.SimpleNamespace()
    fb._doors = types.SimpleNamespace(doors=[{
        'id': 3, 'a': [8.5, 7.2], 'b': [8.5, 7.8],
    }])
    fb._plan_path_xy = Mock(return_value=[(11.6, 6.9), (11.6, 8.8), (5.6, 8.8), (5.6, 7.8)])
    fb._clear_pre_door_point = Mock(side_effect=lambda door, wx, wy: (wx, wy))
    out = MapBridge._expand_route_via_plan(
        fb, (11.6, 6.9), [{'x': 5.6, 'y': 7.8, 'yaw': 3.1}]
    )
    assert len(out) == 2
    assert out[0]['x'] == 9.3
    assert out[0]['y'] == 7.5
    assert out[0]['_light'] is False
    assert out[1]['x'] == 5.6 and out[1]['y'] == 7.8


def test_runner_pisca_luz_no_waypoint_real_sucesso():
    srv = Mock()
    srv.wait_for_service.return_value = False
    fb = _fake_bridge(srv)
    fb._wp_goal_done.set()
    fb._wp_goal_status = 4  # GoalStatus.STATUS_SUCCEEDED
    MapBridge._wp_runner(fb)
    fb._pulse_goal_light.assert_called_once()


def test_runner_nao_pisca_luz_no_waypoint_tecnico():
    srv = Mock()
    srv.wait_for_service.return_value = False
    fb = _fake_bridge(srv)
    fb._wp_list = [{'x': 1.0, 'y': 2.0, 'yaw': 0.0, '_light': False}]
    fb._wp_goal_done.set()
    fb._wp_goal_status = 4  # GoalStatus.STATUS_SUCCEEDED
    MapBridge._wp_runner(fb)
    fb._pulse_goal_light.assert_not_called()
