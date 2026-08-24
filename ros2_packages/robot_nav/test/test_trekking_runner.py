"""Autoridade de atuador do trekking (o que o nav2 aprendeu e o trekking não sabia).

O trekking_runner congelou em 2026-06-12. Uma semana depois o `spin_calib`
(06-19) mediu a zona-morta do giro (1.7 rad/s) e o `arc_calib` (06-25) provou
que o robô NÃO arqueia andando (2-3% de fidelidade até wz=1.2, 19% no máximo a
2.5). O PID antigo saturava TODO giro em omega_max=1.2 — dentro da zona-morta,
ou seja: no PLAY o robô nunca girava de verdade.

Aqui travamos as duas regras físicas: nunca emitir comando morto, e nunca
tentar reto+giro ao mesmo tempo.
"""
import math

import pytest

from robot_nav.trekking_runner import DriveConfig, drive_cmd


CFG = DriveConfig()


def _turn(h_err_deg, turning=False, dist=5.0, is_last=False, cfg=CFG):
    return drive_cmd(math.radians(h_err_deg), dist, is_last, turning, cfg)


# --- zona-morta do giro (spin_calib 06-19) ---

def test_giro_nunca_sai_abaixo_da_zona_morta():
    """Qualquer wz emitido gira a roda de verdade — ou é zero."""
    for err_deg in (0.5, 5, 20, 45, 90, 179):
        for turning in (False, True):
            vx, wz, _ = _turn(err_deg, turning=turning)
            assert wz == 0.0 or abs(wz) >= CFG.rot_deadzone, (
                f'wz morto {wz:.2f} em erro {err_deg}° (turning={turning})')


def test_erro_grande_gira_com_piso_rot_min():
    vx, wz, turning = _turn(20.1)
    assert turning is True
    assert wz >= CFG.rot_min


def test_giro_satura_em_rot_max():
    vx, wz, _ = _turn(179)
    assert wz == pytest.approx(CFG.rot_max)


def test_sinal_do_giro_segue_o_erro():
    _, wz_esq, _ = _turn(45)
    _, wz_dir, _ = _turn(-45)
    assert wz_esq > 0 and wz_dir < 0
    assert wz_esq == pytest.approx(-wz_dir)


# --- reto OU giro, nunca arco (arc_calib 06-25) ---

def test_point_turn_nao_anda():
    vx, wz, turning = _turn(90)
    assert turning is True
    assert vx == 0.0


def test_reto_nao_gira():
    """Erro pequeno: anda reto e NÃO manda wz fraco (comando morto)."""
    vx, wz, turning = _turn(3)
    assert turning is False
    assert wz == 0.0
    assert vx > 0.0


# --- histerese (não fica liga-desliga na fronteira) ---

def test_histerese_entra_e_so_sai_no_turn_exit():
    # entra girando
    _, _, turning = _turn(math.degrees(CFG.turn_enter) + 1)
    assert turning is True
    # ainda girando num erro que sozinho NÃO teria entrado
    _, _, turning = _turn(math.degrees(CFG.turn_enter) - 5, turning=True)
    assert turning is True
    # só solta abaixo do turn_exit
    _, _, turning = _turn(math.degrees(CFG.turn_exit) - 1, turning=True)
    assert turning is False


# --- zona-morta LINEAR (path_follower 06-26: "0.11 trava, 0.25 anda") ---

def test_avanco_nunca_rasteja_perto_do_ultimo_ponto():
    """O freio do último waypoint não pode cair abaixo da zona-morta linear."""
    for dist in (0.6, 0.4, 0.3, 0.26):
        vx, wz, _ = _turn(0, dist=dist, is_last=True)
        assert vx >= CFG.min_speed, f'rastejo vx={vx:.3f} a {dist} m do último ponto'


def test_reta_longa_vai_na_velocidade_maxima():
    vx, _, _ = _turn(0, dist=5.0, is_last=False)
    assert vx == pytest.approx(CFG.v_max)


def test_waypoint_intermediario_nao_freia():
    """Só o último ponto freia — nos intermediários passa voado."""
    vx, _, _ = _turn(0, dist=0.3, is_last=False)
    assert vx == pytest.approx(CFG.v_max)


# ---------------------------------------------------------------------------
# Captura do cone na gravação (BO do dono, sim 2026-08-24)
#
# Ele gravou 4 pontos e só 2 saíram com cone. Medido na rota salva: nos wp1 e
# wp2 o cone real estava a 1,44 m e 2,47 m — perto — mas com bearing +170° e
# +134°, ou seja ATRÁS. A regra antiga só olhava o semicírculo frontal (±90°),
# então ele perdia todo cone que ficasse ao lado ou já tivesse passado.
#
# Pedido: valer pra TODOS OS LADOS, validando só por PROXIMIDADE.
# ---------------------------------------------------------------------------
from robot_nav.trekking_runner import pick_cone


def test_cone_ao_lado_esquerdo_vale():
    # robô na origem olhando +x; cone a 1,5 m na esquerda (+90°)
    c = pick_cone([(0.0, 1.5, 0.22)], 0.0, 0.0, 0.0, 3.0)
    assert c is not None
    assert c[0] == pytest.approx(0.0) and c[1] == pytest.approx(1.5)
    assert math.degrees(c[2]) == pytest.approx(90.0)


def test_cone_ao_lado_direito_vale():
    c = pick_cone([(0.0, -1.5, 0.22)], 0.0, 0.0, 0.0, 3.0)
    assert c is not None
    assert math.degrees(c[2]) == pytest.approx(-90.0)


def test_cone_ATRAS_vale_agora():
    """Era o BO: bearing 180° era descartado mesmo estando a 1,4 m."""
    c = pick_cone([(-1.44, 0.0, 0.22)], 0.0, 0.0, 0.0, 3.0)
    assert c is not None
    assert abs(math.degrees(c[2])) == pytest.approx(180.0)


def test_longe_demais_nao_vale():
    assert pick_cone([(0.0, 3.5, 0.22)], 0.0, 0.0, 0.0, 3.0) is None


def test_colado_demais_e_ruido():
    assert pick_cone([(0.02, 0.0, 0.22)], 0.0, 0.0, 0.0, 3.0) is None


def test_escolhe_o_MAIS_PROXIMO_em_qualquer_direcao():
    cones = [(2.5, 0.0, 0.22),      # frente, longe
             (0.0, -0.9, 0.22)]     # direita, perto
    c = pick_cone(cones, 0.0, 0.0, 0.0, 3.0)
    assert c[1] == pytest.approx(-0.9)


def test_bearing_e_relativo_ao_yaw_gravado():
    """O bearing gravado é o que o PLAY usa pra conferir o cone — tem que ser
    relativo ao yaw do robô na hora da gravação, não absoluto."""
    c = pick_cone([(0.0, 1.5, 0.22)], 0.0, 0.0, math.radians(90.0), 3.0)
    assert math.degrees(c[2]) == pytest.approx(0.0)   # está bem à frente
