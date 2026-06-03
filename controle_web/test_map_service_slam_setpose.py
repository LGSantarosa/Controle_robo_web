"""Testes da parte pura do set-pose em SLAM (correção de DIREÇÃO / yaw).

No SLAM não dá pra resetar posição sem deformar o mapa, então o set-pose vira
yaw-only: a web manda o delta de rotação (desejado − atual) pro pose_estimator.
yaw_delta calcula esse delta na rotação mais curta ([-π, π]).
"""
import math

from map_service import yaw_delta


def test_yaw_delta_simples():
    assert abs(yaw_delta(1.0, 0.0) - 1.0) < 1e-9
    assert abs(yaw_delta(0.0, 0.0)) < 1e-9


def test_yaw_delta_negativo():
    assert abs(yaw_delta(0.0, 1.0) - (-1.0)) < 1e-9


def test_yaw_delta_pega_rotacao_mais_curta_no_wraparound():
    # desejado=+3.0, atual=-3.0: diff cru = 6.0, mas a rotação curta é
    # 6.0 - 2π ≈ -0.283 rad (gira pro outro lado).
    d = yaw_delta(3.0, -3.0)
    assert abs(d - (6.0 - 2 * math.pi)) < 1e-9
    assert -math.pi <= d <= math.pi


def test_yaw_delta_meia_volta_exata_fica_em_pi():
    d = yaw_delta(math.pi, 0.0)
    assert abs(abs(d) - math.pi) < 1e-9
