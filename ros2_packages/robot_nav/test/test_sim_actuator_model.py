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


# ---- watchdog: o DiffDrive do Gazebo TRAVA o ultimo comando ----
#
# BO medido no sim 2026-08-24, com o dono dirigindo de PS4: "quando eu giro pra
# direita o robo gira pra caralho do nada". Nao era salto — era comando preso.
# Soltando o L1 o teleop para de publicar, o twist_mux expira em 0,5 s e para
# tambem, e o DiffDrive do Gazebo mantem a ultima velocidade de roda PARA
# SEMPRE. Medido: /cmd_vel sem nenhuma mensagem e o robo girando a +58,8 graus/s.
#
#   sem ninguem publicando     : +40,4 graus/s
#   depois de ZERO explicito   :  +0,0 graus/s
#   1,5 s apos soltar o comando: +58,8 graus/s
#
# O robo real para quando o comando cessa (watchdog do firmware), entao isto
# tambem e fidelidade sim=real.
from robot_nav.sim_actuator_model import watchdog_deve_parar


def test_watchdog_para_quando_a_entrada_seca():
    assert watchdog_deve_parar(0.4, 0.3, ultimo_foi_zero=False) is True


def test_watchdog_calado_enquanto_chega_comando():
    assert watchdog_deve_parar(0.1, 0.3, ultimo_foi_zero=False) is False


def test_watchdog_nao_repete_zero():
    """Ja parado = nao fica martelando zero a 20 Hz no barramento."""
    assert watchdog_deve_parar(5.0, 0.3, ultimo_foi_zero=True) is False


def test_watchdog_desligavel():
    """timeout <= 0 volta ao comportamento antigo (A/B)."""
    assert watchdog_deve_parar(99.0, 0.0, ultimo_foi_zero=False) is False
