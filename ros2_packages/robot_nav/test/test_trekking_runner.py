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


# --- trilha densa (teach-and-repeat, 2026-08-25) -----------------------------
#
# O RECORD so guardava os waypoints do botao; entre eles o PLAY inventa uma
# reta. `trail_step` e o gate que transforma a pose em migalhas de trilha.

from robot_nav.trekking_runner import trail_step

DS, DYAW = 0.10, math.radians(10.0)


def _step(last, x, y, yaw=0.0, t=1.0):
    return trail_step(last, x, y, yaw, t, DS, DYAW)


def test_primeira_migalha_sempre_entra():
    p = _step(None, 1.0, 2.0, 0.5, t=0.0)
    assert p is not None
    assert (p['x'], p['y']) == (1.0, 2.0)
    assert p['v'] == 0.0 and p['wz'] == 0.0


def test_parado_nao_gera_migalha():
    """O motivo de amostrar por distancia: 30 Hz parado encheria o arquivo."""
    last = _step(None, 0.0, 0.0, t=0.0)
    for i in range(100):
        assert _step(last, 0.001, 0.0, t=i * 0.03) is None


def test_anda_o_passo_gera_migalha():
    last = _step(None, 0.0, 0.0, t=0.0)
    assert _step(last, DS - 0.001, 0.0, t=1.0) is None
    assert _step(last, DS + 0.001, 0.0, t=1.0) is not None


def test_point_turn_entra_pelo_gate_de_yaw():
    """Giro no lugar anda 0 m — sem o gate de yaw ele sumiria da trilha,
    justo onde o robo mais erra."""
    last = _step(None, 0.0, 0.0, yaw=0.0, t=0.0)
    assert _step(last, 0.0, 0.0, yaw=math.radians(9.0), t=1.0) is None
    p = _step(last, 0.0, 0.0, yaw=math.radians(11.0), t=1.0)
    assert p is not None and p['v'] == 0.0 and p['wz'] > 0


def test_yaw_cruzando_pi_nao_vira_giro_gigante():
    """+179° -> -179° sao 2° de giro, nao 358°."""
    last = _step(None, 0.0, 0.0, yaw=math.radians(179.0), t=0.0)
    # Sem wrap, 179 -> -179 daria 358° e passaria o gate de 10° gritando.
    assert _step(last, 0.0, 0.0, yaw=math.radians(-179.0), t=1.0) is None
    # Com o gate baixado pra 1°, passa — e o wz tem que ser 2°/s, nao 358°/s.
    p2 = trail_step(last, 0.0, 0.0, math.radians(-179.0), 1.0, DS, math.radians(1.0))
    assert abs(p2['wz']) == pytest.approx(math.radians(2.0), abs=1e-3)


def test_v_sai_da_pose_e_nao_do_comando():
    """Em skid o cmd_vel mente; a trilha registra o que ANDOU."""
    last = _step(None, 0.0, 0.0, t=0.0)
    p = _step(last, 0.5, 0.0, t=2.0)
    assert p['v'] == pytest.approx(0.25)      # 0.5 m em 2 s


def test_dt_zero_nao_divide_por_zero():
    last = _step(None, 0.0, 0.0, t=1.0)
    p = _step(last, 1.0, 0.0, t=1.0)
    assert p is not None and p['v'] == 0.0 and p['wz'] == 0.0


# --- PLAY seguindo a trilha (2026-08-25) -------------------------------------

from robot_nav.trekking_runner import trail_lookahead, trail_progress

RETA = [{'x': i * 0.1, 'y': 0.0, 'yaw': 0.0} for i in range(50)]   # 4,9 m em +x


def test_progresso_acompanha_o_robo():
    assert trail_progress(RETA, 0, 0.0, 0.0) == 0
    assert trail_progress(RETA, 0, 1.0, 0.0) == 10
    assert trail_progress(RETA, 10, 2.0, 0.0) == 20


def test_progresso_NUNCA_volta():
    """Retroceder e esquecer o que ja andou — o robo fica em looping no trecho."""
    p = trail_progress(RETA, 20, 2.0, 0.0)
    assert trail_progress(RETA, p, 0.5, 0.0) == p


def test_progresso_so_olha_a_janela_a_frente():
    """Rota que volta pelo mesmo corredor nao pode teleportar o progresso."""
    volta = RETA + [{'x': 4.9 - i * 0.1, 'y': 0.02, 'yaw': math.pi}
                    for i in range(50)]
    # robo no comeco geometricamente, mas ja no trecho de volta (indice 60)
    assert trail_progress(volta, 60, 3.0, 0.0, window=25) >= 60


def test_mira_vai_a_frente_do_progresso():
    tx, ty, is_last = trail_lookahead(RETA, 10, 0.6)
    assert tx == pytest.approx(1.6, abs=0.11)
    assert ty == 0.0 and is_last is False


def test_mira_satura_no_fim_e_avisa():
    """`is_last` e o que faz frear no final em vez de chegar voado."""
    tx, ty, is_last = trail_lookahead(RETA, 47, 0.6)
    assert is_last is True
    assert tx == pytest.approx(4.9)


def test_trilha_vazia_nao_quebra():
    assert trail_lookahead([], 0, 0.6) is None
    assert trail_progress([], 3, 1.0, 1.0) == 3


def test_mira_segue_a_CURVA_e_nao_corta():
    """O motivo da trilha existir: entre dois waypoints o PLAY antigo fazia
    reta. Aqui a mira tem que sair da linha reta junto com o caminho."""
    curva = [{'x': 0.1 * i, 'y': 0.0, 'yaw': 0.0} for i in range(10)] + \
            [{'x': 1.0, 'y': 0.1 * i, 'yaw': math.pi / 2} for i in range(1, 15)]
    tx, ty, _ = trail_lookahead(curva, 9, 0.6)
    assert ty > 0.4          # subiu na perna vertical
    assert tx == pytest.approx(1.0)
