"""Prazo do eixo de gamepad vindo do browser.

BO medido no sim 2026-08-24: "dou um toque no joystick e ele desliza tudo pra
direita". O `gamepad.js` roda em requestAnimationFrame, que o navegador CONGELA
quando a aba perde o foco (o dono estava olhando a janela do Gazebo). O
republicador do ROS2Controller manda o último eixo a 50 Hz enquanto for != 0 —
então o toque ficava aplicado pra sempre.

Nem o timeout do twist_mux nem o watchdog do sim_actuator_model pegam isso: os
dois só agem quando o tópico SECA, e aqui ele continua a 50 Hz.
"""
from controllers.robot_controller import ROS2Controller, gamepad_expirou


def test_eixo_recente_vale():
    assert gamepad_expirou(0.1, 0.6) is False


def test_eixo_velho_e_descartado():
    """Aba congelada: parou de reenviar -> solta o comando."""
    assert gamepad_expirou(0.7, 0.6) is True


def test_no_limiar():
    assert gamepad_expirou(0.6, 0.6) is False
    assert gamepad_expirou(0.61, 0.6) is True


def test_timeout_zero_desliga():
    assert gamepad_expirou(99.0, 0.0) is False


def test_prazo_maior_que_o_keepalive_do_browser():
    """O gamepad.js reenvia a cada 200 ms. Se o prazo do servidor fosse menor
    ou igual, segurar o stick parado seria lido como 'cliente sumiu'."""
    KEEPALIVE_MS = 200
    assert ROS2Controller.GAMEPAD_TIMEOUT > (KEEPALIVE_MS / 1000.0) * 2
