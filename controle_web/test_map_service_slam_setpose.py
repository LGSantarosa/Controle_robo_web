"""Testes das partes puras do set-pose em SLAM (caminho interactive-marker).

build_interactive_feedback monta o InteractiveMarkerFeedback que o
slam_toolbox consome em /slam_toolbox/feedback; latest_marker_name escolhe
o nó mais recente do pose-graph (= pose atual do robô) entre os markers.
"""
import math

from map_service import build_interactive_feedback, latest_marker_name
from builtin_interfaces.msg import Time
from visualization_msgs.msg import InteractiveMarkerFeedback


def test_build_interactive_feedback_mouse_up_pose_e_ponto():
    msg = build_interactive_feedback(
        x=1.5, y=-2.0, yaw=math.pi / 2, marker_name='7', stamp=Time()
    )
    assert isinstance(msg, InteractiveMarkerFeedback)
    assert msg.header.frame_id == 'map'
    assert msg.marker_name == '7'
    # MOUSE_UP é o evento que dispara addMovedNodes no slam_toolbox.
    assert msg.event_type == InteractiveMarkerFeedback.MOUSE_UP
    # O handler lê x,y de mouse_point (precisa ser válido) e yaw de pose.
    assert msg.mouse_point_valid is True
    assert abs(msg.mouse_point.x - 1.5) < 1e-9
    assert abs(msg.mouse_point.y - (-2.0)) < 1e-9
    assert abs(msg.pose.position.x - 1.5) < 1e-9
    assert abs(msg.pose.position.y - (-2.0)) < 1e-9
    # yaw=pi/2 -> qz=qw=0.7071
    assert abs(msg.pose.orientation.z - 0.70710678) < 1e-3
    assert abs(msg.pose.orientation.w - 0.70710678) < 1e-3


def test_build_interactive_feedback_frame_customizavel():
    msg = build_interactive_feedback(
        x=0.0, y=0.0, yaw=0.0, marker_name='1', stamp=Time(), frame='odom'
    )
    assert msg.header.frame_id == 'odom'


def test_latest_marker_name_pega_maior_numerico():
    # Não pode ser comparação lexical: '10' > '2' numericamente.
    assert latest_marker_name(['1', '2', '10', '3']) == '10'


def test_latest_marker_name_ignora_nao_numericos():
    assert latest_marker_name(['1', 'foo', '2', '']) == '2'


def test_latest_marker_name_vazio_ou_so_lixo_retorna_none():
    assert latest_marker_name([]) is None
    assert latest_marker_name(['foo', 'bar']) is None
