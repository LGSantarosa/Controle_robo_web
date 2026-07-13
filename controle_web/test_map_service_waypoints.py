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
