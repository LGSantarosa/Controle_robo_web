"""Regras puras da pose do SIM (rodas dão velocidade, IMU dá rumo).

Contexto (2026-08-26): até esta data o sim não tinha IMU e o yaw vinha das
RODAS — que patinam no pivô do skid-steer. Medido contra verdade-terreno, a
odom girava 7,8° no primeiro tick de cada point-turn enquanto o robô girava
0,5°. O robô real usa IMU e não tem esse erro.
"""
import math

import pytest

from robot_nav.sim_trekking_pose import integra_pose


def test_reto_sem_giro_anda_no_eixo():
    x, y, yaw = integra_pose(0.0, 0.0, 0.0, v=1.0, yaw_rate=0.0, dt=0.1)
    assert x == pytest.approx(0.1)
    assert y == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_giro_parado_nao_desloca():
    """Point-turn: a IMU manda o yaw, mas v=0 então o robô não sai do lugar."""
    x, y, yaw = integra_pose(2.0, 3.0, 0.0, v=0.0, yaw_rate=1.0, dt=0.1)
    assert (x, y) == (2.0, 3.0)
    assert yaw == pytest.approx(0.1)


def test_rumo_vem_do_yaw_rate_e_nao_da_velocidade():
    """O ponto do arquivo inteiro: quem gira a pose é a IMU."""
    _, _, yaw = integra_pose(0.0, 0.0, 0.0, v=5.0, yaw_rate=2.0, dt=0.5)
    assert yaw == pytest.approx(1.0)


def test_meio_passo_no_yaw_durante_o_deslocamento():
    """Virando 90° enquanto anda: o rumo do deslocamento é 45°, não 0° nem 90°.

    (dt fica em 0,5 s = o teto da guarda; 1,0 s seria rejeitado por buraco.)
    """
    x, y, yaw = integra_pose(0.0, 0.0, 0.0, v=1.0, yaw_rate=math.pi, dt=0.5)
    assert math.degrees(yaw) == pytest.approx(90.0)
    assert math.degrees(math.atan2(y, x)) == pytest.approx(45.0)


def test_yaw_embrulha_em_pi():
    _, _, yaw = integra_pose(0.0, 0.0, 3.0, v=0.0, yaw_rate=1.0, dt=0.5)
    assert -math.pi <= yaw <= math.pi
    assert yaw == pytest.approx(3.5 - 2 * math.pi)


@pytest.mark.parametrize('dt', [0.0, -0.1, 0.51, 10.0])
def test_dt_invalido_nao_mexe_no_estado(dt):
    """dt<=0 = carimbo repetido/relógio pra trás (BO do órfão de /clock);
    dt grande = buraco de mensagens. Integrar por cima inventa deslocamento."""
    assert integra_pose(1.0, 2.0, 0.5, v=9.0, yaw_rate=9.0, dt=dt) == (1.0, 2.0, 0.5)
