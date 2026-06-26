"""Testes das curvas puras do sim_actuator_model (giro + zona-morta linear)."""
from robot_nav.sim_actuator_model import model_linear, model_theta

# Defaults do nó (spin_calib 2026-06-19).
DZ, GAIN, SAT, RF, LF = 1.7, 0.6, 2.5, 1.05, 1.0


# ---- giro ----
def test_theta_deadzone_nao_gira():
    assert model_theta(1.0, DZ, GAIN, SAT, RF, LF) == 0.0
    assert model_theta(-1.69, DZ, GAIN, SAT, RF, LF) == 0.0


def test_theta_acima_da_deadzone_gira():
    # 2.5 -> 0.6*(2.5-1.7)=0.48, esquerda (w>0) sem fator
    assert abs(model_theta(2.5, DZ, GAIN, SAT, RF, LF) - 0.48) < 1e-6


def test_theta_satura():
    # comando enorme satura em sat (antes do fator de assimetria)
    assert abs(model_theta(100.0, DZ, GAIN, SAT, RF, LF) - SAT) < 1e-6


def test_theta_assimetria_direita_gira_mais():
    e = model_theta(3.0, DZ, GAIN, SAT, RF, LF)     # esquerda
    d = model_theta(-3.0, DZ, GAIN, SAT, RF, LF)    # direita
    assert abs(d) > abs(e)                           # direita entrega mais
    assert e > 0 and d < 0                           # sinais preservados


# ---- zona-morta linear (o BO) ----
def test_linear_deadzone_trava_comando_pequeno():
    # 0.11 m/s (o ramp do path_follower no min_speed antigo) -> robô NÃO anda
    assert model_linear(0.11, 0.15) == 0.0
    assert model_linear(-0.11, 0.15) == 0.0


def test_linear_acima_da_deadzone_passa_direto():
    # 0.25 m/s (cruise) -> passa igual; e o fix min_speed=0.22 também anda
    assert model_linear(0.25, 0.15) == 0.25
    assert model_linear(0.22, 0.15) == 0.22


def test_linear_deadzone_zero_passa_tudo():
    # deadzone 0 = comportamento antigo (passa direto), inclusive valores baixos
    assert model_linear(0.05, 0.0) == 0.05
