import math

import pytest

from nav2_trekking.fused_odom import (
    FusedOdom,
    blend_yaw_rate,
    flow_alpha,
    flow_plausible,
    flow_tick_velocity,
    flow_yaw_gate,
    fuse_translation,
    heading_correction,
    slip_estimate,
    wheel_twist,
)


def test_flow_yaw_gate_full_when_slow():
    # parado ou curva mansa → flow passa inteiro
    assert flow_yaw_gate(0.0, 0.4, 1.2) == 1.0
    assert flow_yaw_gate(0.3, 0.4, 1.2) == 1.0
    assert flow_yaw_gate(-0.3, 0.4, 1.2) == 1.0   # simétrico no sinal


def test_flow_yaw_gate_zero_when_fast():
    # giro rápido (qualquer sinal) → flow ignorado
    assert flow_yaw_gate(2.0, 0.4, 1.2) == 0.0
    assert flow_yaw_gate(-1.5, 0.4, 1.2) == 0.0


def test_flow_yaw_gate_linear_ramp():
    # no meio da banda, rampa linear
    assert flow_yaw_gate(0.8, 0.4, 1.2) == pytest.approx(0.5)


def test_wheel_twist_straight():
    vx, w = wheel_twist(1.0, 1.0, 1.0, 1.0, wheel_base=0.5)
    assert vx == pytest.approx(1.0)
    assert w == pytest.approx(0.0)


def test_wheel_twist_spin_in_place():
    # lado esquerdo recua, direito avança → gira (omega > 0)
    vx, w = wheel_twist(-0.5, 0.5, -0.5, 0.5, wheel_base=0.5)
    assert vx == pytest.approx(0.0)
    assert w == pytest.approx((0.5 - (-0.5)) / 0.5)


def test_flow_alpha_zero_when_stale():
    assert flow_alpha(245.0, q_mid=80.0, q_slope=20.0,
                      flow_age=1.0, flow_timeout=0.5) == 0.0


def test_flow_alpha_high_when_quality_good():
    a = flow_alpha(200.0, q_mid=80.0, q_slope=20.0,
                   flow_age=0.05, flow_timeout=0.5)
    assert a > 0.99


def test_flow_alpha_half_at_qmid():
    a = flow_alpha(80.0, q_mid=80.0, q_slope=20.0,
                   flow_age=0.05, flow_timeout=0.5)
    assert a == pytest.approx(0.5)


def test_flow_plausible_rejects_emi_garbage():
    # EMI do motor faz o PMW3901 cuspir velocidades impossiveis com quality ALTA
    # (medido na bancada: flow=-10.6 m/s e +2.27 m/s com as rodas paradas). O gate
    # de qualidade nao pega; o de plausibilidade fisica sim.
    assert flow_plausible(0.30, 0.05, v_max=0.8) is True     # andar normal passa
    assert flow_plausible(-0.8, 0.0, v_max=0.8) is True      # no limite passa
    assert flow_plausible(-10.61, 0.0, v_max=0.8) is False   # lixo de EMI
    assert flow_plausible(0.0, 2.27, v_max=0.8) is False     # lixo lateral
    assert flow_plausible(0.0, 0.0, v_max=0.8) is True       # parado passa


def test_flow_tick_velocity_basic():
    # deslocamento acumulado / dt do tick = velocidade do tick
    vx, vy = flow_tick_velocity(0.04, -0.02, dt=0.02)
    assert vx == pytest.approx(2.0)
    assert vy == pytest.approx(-1.0)
    # dt nao-positivo nao explode
    assert flow_tick_velocity(0.04, 0.0, dt=0.0) == (0.0, 0.0)


def test_flow_tick_velocity_conserves_displacement_under_bursty_arrival():
    # REGRESSAO do bug que dobrava a pose: o flow chega em RAJADA (2 msgs numa
    # janela de tick, 0 na seguinte). Cada msg anda 0.02 m; 100 msgs = 2.00 m
    # reais. Acumular o deslocamento e dividir pelo dt do TICK conserva os 2.00 m;
    # o jeito antigo (flow_vx = d/dt_chegada SEGURADO e re-integrado a 50 Hz)
    # inflava ~2x (medido na bancada: odom_net 4.88 m num percurso de 2 m).
    tick_dt = 0.02
    msgs_per_tick = [2, 0] * 50            # 100 msgs, padrao em rajada
    true_total = 100 * 0.02               # 2.00 m

    # NOVO (correto): acumula deslocamento, vel = accum/dt_tick, integra, zera
    accum = 0.0
    integrated_new = 0.0
    for n in msgs_per_tick:
        accum += n * 0.02
        vx, _ = flow_tick_velocity(accum, 0.0, tick_dt)
        integrated_new += vx * tick_dt
        accum = 0.0
    assert integrated_new == pytest.approx(true_total)

    # ANTIGO (bug): vel instantanea do intervalo de chegada (tick/2 na rajada),
    # SEGURADA e re-integrada no tick vazio seguinte tambem -> dobra.
    held_v = 0.0
    integrated_old = 0.0
    for n in msgs_per_tick:
        if n > 0:
            held_v = 0.02 / (tick_dt / n)
        integrated_old += held_v * tick_dt
    assert integrated_old == pytest.approx(2.0 * true_total)


def test_time_jump_must_discard_accumulated_flow_displacement():
    # Contrato do pose_estimator._tick (B2 da AUDITORIA_2026-06-11): num salto
    # de tempo (dt <= 0 ou dt > 0.5) o tick NAO integra e DRENA o acumulador do
    # flow. Sem o drain, o deslocamento da janela perdida re-integrado no tick
    # normal seguinte (dt=0.02) vira velocidade ~25x a real:
    vx, vy = flow_tick_velocity(0.5, 0.0, 0.02)   # 0.5 m presos no acumulador
    assert vx == pytest.approx(25.0)              # 25 m/s fantasma
    # Hoje o gate de EMI mascara (descarta a amostra = janela de flow perdida),
    # mas e' o gate escondendo um bug aritmetico — por isso o drain no _tick.
    assert not flow_plausible(vx, vy, v_max=0.8)


def test_fuse_translation_alpha_zero_is_wheel_only():
    vx, vy = fuse_translation(vx_wheel=0.8, flow_vx=0.2, flow_vy=0.1, alpha=0.0)
    assert vx == pytest.approx(0.8)
    assert vy == pytest.approx(0.0)


def test_fuse_translation_alpha_one_is_flow_only():
    vx, vy = fuse_translation(vx_wheel=0.8, flow_vx=0.2, flow_vy=0.1, alpha=1.0)
    assert vx == pytest.approx(0.2)
    assert vy == pytest.approx(0.1)


def test_no_imu_uses_wheel_yaw():
    # Sem IMU, girando: yaw integra do diferencial de roda
    fo = FusedOdom(wheel_base=0.5)
    r = fo.step(dt=0.1, v_fl=-0.5, v_fr=0.5, v_rl=-0.5, v_rr=0.5,
                imu_fresh=False, imu_yaw_rate=0.0,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    assert r.yaw_source == 'wheel'
    assert r.yaw == pytest.approx(2.0 * 0.1)  # omega=2 rad/s * dt
    assert r.yaw_rate == pytest.approx(2.0)


def test_imu_fresh_integrates_gyro_rate_ignoring_wheels():
    # MPU6050: com IMU fresca, o yaw INTEGRA a taxa do giro (não o diferencial
    # de roda), mesmo com as rodas girando a outra velocidade.
    fo = FusedOdom(wheel_base=0.5)
    r = fo.step(dt=0.1, v_fl=-0.5, v_fr=0.5, v_rl=-0.5, v_rr=0.5,
                imu_fresh=True, imu_yaw_rate=0.3,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    assert r.yaw_source == 'imu'
    assert r.yaw == pytest.approx(0.3 * 0.1)  # integra taxa do giro, não a roda
    assert r.yaw_rate == pytest.approx(0.3)


def test_imu_dropout_continues_wheel_from_last_yaw():
    # Yaw acumulado pelo giro (10 rad/s * 0.1 = 1.0), IMU cai → integra do
    # diferencial de roda a partir do último yaw, sem voltar a 0.
    fo = FusedOdom(wheel_base=0.5)
    fo.step(dt=0.1, v_fl=0.0, v_fr=0.0, v_rl=0.0, v_rr=0.0,
            imu_fresh=True, imu_yaw_rate=10.0,
            flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    r = fo.step(dt=0.1, v_fl=-0.5, v_fr=0.5, v_rl=-0.5, v_rr=0.5,
                imu_fresh=False, imu_yaw_rate=0.0,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    assert r.yaw_source == 'wheel'
    assert r.yaw == pytest.approx(1.0 + 2.0 * 0.1)  # continua de 1.0


def test_stale_stream_freezes_pose_no_phantom_spin():
    # REGRESSAO do giro fantasma (2026-06-09): a MEGA travou (I2C lockup) e
    # PAROU de mandar frames. Sem novos /imu/data nem /hoverboard/wheel_velocities,
    # _imu_yaw_rate e v_fl..v_rr ficam CONGELADOS no ultimo valor — e o robo
    # estava GIRANDO (rodas assimetricas). O tick segue rodando a 50 Hz. Sem
    # guarda de freshness, o yaw integrava o diferencial de roda congelado pra
    # sempre -> heading girava no mapa com o robo fisicamente parado.
    # Com IMU stale E rodas stale: NAO integra nada (pose congela).
    fo = FusedOdom(wheel_base=0.5)
    fo.yaw = 1.0  # estava em algum heading
    # rodas congeladas num giro (esquerda recua, direita avanca)
    r = fo.step(dt=0.02, v_fl=-0.5, v_fr=0.5, v_rl=-0.5, v_rr=0.5,
                imu_fresh=False, imu_yaw_rate=0.0,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0,
                wheel_fresh=False)
    assert r.yaw_rate == 0.0          # nao gira
    assert r.yaw == pytest.approx(1.0)  # heading CONGELADO
    assert fo.x == pytest.approx(0.0)
    assert fo.y == pytest.approx(0.0)


def test_stale_wheels_fresh_imu_uses_imu_yaw_no_wheel_translation():
    # MEGA viva mas rodas sem feedback (placa hoverboard muda): IMU manda o yaw,
    # mas a translacao de roda congelada NAO entra (vx_wheel zerado).
    fo = FusedOdom(wheel_base=0.5)
    r = fo.step(dt=0.1, v_fl=2.0, v_fr=2.0, v_rl=2.0, v_rr=2.0,  # rodas congeladas "andando"
                imu_fresh=True, imu_yaw_rate=0.3,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0,
                wheel_fresh=False)
    assert r.yaw_source == 'imu'
    assert r.yaw == pytest.approx(0.3 * 0.1)   # yaw da IMU
    assert fo.x == pytest.approx(0.0)          # rodas congeladas NAO movem a pose
    assert fo.y == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# IMU #2 (BNO055): taxa de yaw redundante + heading absoluto do magnetometro
# ---------------------------------------------------------------------------


def test_blend_yaw_rate_media_das_duas_imus():
    rate, source, disagree = blend_yaw_rate(
        imu_fresh=True, imu_rate=0.40,
        imu2_fresh=True, imu2_rate=0.50, imu2_weight=0.5)
    assert rate == pytest.approx(0.45)
    assert source == 'imu+imu2'
    assert disagree is False


def test_blend_yaw_rate_peso_zero_e_a_imu1_pura():
    # use_imu2 ligado mas peso 0: a #2 nao move a taxa (so o heading dela conta)
    rate, source, _ = blend_yaw_rate(True, 0.40, True, 0.20, imu2_weight=0.0)
    assert rate == pytest.approx(0.40)
    assert source == 'imu+imu2'


def test_blend_yaw_rate_peso_alto_rejeita_o_bias_do_mpu():
    # POR QUE o default e' 0.8 e nao 0.5: o que faz o yaw derivar e' BIAS, e
    # media herda o bias do pior sensor na proporcao do peso. Cenario: robo
    # PARADO, MPU com 0.02 rad/s de bias (calibracao de boot ruim), BNO055
    # limpa (recalibra o proprio bias continuamente).
    bias = 0.02
    meia, _, _ = blend_yaw_rate(True, bias, True, 0.0, imu2_weight=0.5)
    pesada, _, _ = blend_yaw_rate(True, bias, True, 0.0, imu2_weight=0.8)
    assert meia == pytest.approx(0.010)     # metade do bias vaza pro yaw
    assert pesada == pytest.approx(0.004)   # 20% dele
    # em 60 s parado isso e' a diferenca entre ~34 graus e ~14 graus de deriva
    assert math.degrees(meia * 60) == pytest.approx(34.4, abs=0.5)
    assert math.degrees(pesada * 60) == pytest.approx(13.8, abs=0.5)


def test_blend_yaw_rate_cai_pra_imu_viva():
    assert blend_yaw_rate(True, 0.3, False, 9.9, 0.5)[:2] == (0.3, 'imu')
    assert blend_yaw_rate(False, 9.9, True, 0.3, 0.5)[:2] == (0.3, 'imu2')
    # nenhuma fresca -> None (o chamador cai pro diferencial de roda)
    assert blend_yaw_rate(False, 9.9, False, 9.9, 0.5)[:2] == (None, 'wheel')


def test_blend_yaw_rate_sinal_oposto_descarta_a_segunda():
    # MODO DE FALHA que este gate existe pra pegar: BNO055 montada girada ->
    # imu2_yaw_sign errado -> as duas leem o MESMO giro com sinais opostos. A
    # media ingenua daria ZERO (robo gira no chao e nao gira no mapa).
    rate, source, disagree = blend_yaw_rate(True, 0.80, True, -0.80, 0.5)
    assert rate == pytest.approx(0.80)   # NAO e' 0.0
    assert source == 'imu'               # a #2 saiu da conta
    assert disagree is True


def test_blend_yaw_rate_ruido_parado_nao_dispara_o_gate():
    # Parado as duas leem ~0 com ruido de sinal aleatorio; isso NAO e'
    # discordancia (|w| abaixo de disagree_min) — senao o log viveria gritando.
    rate, source, disagree = blend_yaw_rate(True, 0.01, True, -0.01, 0.5)
    assert disagree is False
    assert source == 'imu+imu2'
    assert rate == pytest.approx(0.0)


def test_heading_correction_puxa_pro_ref_e_satura():
    # erro pequeno: corr = ganho * erro * dt, sem bater no teto
    corr = heading_correction(yaw=0.0, yaw_ref=0.10, gain=0.2, dt=0.02, max_rate=0.15)
    assert corr == pytest.approx(0.2 * 0.10 * 0.02)
    # erro GIGANTE (mag maluco por EMI): satura no teto max_rate*dt — nunca um
    # salto de heading, so um giro lento e visivel
    corr = heading_correction(0.0, math.pi, gain=0.2, dt=0.02, max_rate=0.15)
    assert corr == pytest.approx(0.15 * 0.02)
    # sinal segue o lado mais curto do circulo (wrap): ref logo ATRAS de -pi
    corr = heading_correction(yaw=3.10, yaw_ref=-3.10, gain=0.2, dt=0.02, max_rate=10.0)
    assert corr > 0.0   # gira PRA FRENTE cruzando +-pi, nao 6.2 rad pra tras
    # ganho 0 = desligado
    assert heading_correction(0.0, 1.0, gain=0.0, dt=0.02, max_rate=0.15) == 0.0


def test_step_sem_imu2_e_identico_ao_comportamento_antigo():
    # CONTRATO da compatibilidade: os argumentos da BNO055 sao opcionais e, sem
    # eles, o passo tem que dar exatamente o mesmo resultado de antes dela.
    a = FusedOdom(wheel_base=0.5)
    b = FusedOdom(wheel_base=0.5)
    for _ in range(10):
        ra = a.step(dt=0.02, v_fl=0.4, v_fr=0.6, v_rl=0.4, v_rr=0.6,
                    imu_fresh=True, imu_yaw_rate=0.25,
                    flow_vx=0.0, flow_vy=0.0, alpha=0.0)
        rb = b.step(dt=0.02, v_fl=0.4, v_fr=0.6, v_rl=0.4, v_rr=0.6,
                    imu_fresh=True, imu_yaw_rate=0.25,
                    flow_vx=0.0, flow_vy=0.0, alpha=0.0,
                    imu2_fresh=False, abs_yaw=None)
    assert (ra.x, ra.y, ra.yaw) == (rb.x, rb.y, rb.yaw)
    assert ra.yaw_source == 'imu'
    assert ra.heading_corr == 0.0


def test_step_ancora_magnetica_remove_a_deriva_do_yaw():
    # Robo PARADO com a IMU acusando um bias de giro (deriva classica do yaw
    # integrado): sem ancora o yaw foge; com a ancora ele fica preso no heading
    # absoluto. 20 s a 50 Hz.
    drift_rate = 0.02          # rad/s de bias
    sem = FusedOdom(wheel_base=0.5)
    com = FusedOdom(wheel_base=0.5)
    for _ in range(1000):
        sem.step(dt=0.02, v_fl=0.0, v_fr=0.0, v_rl=0.0, v_rr=0.0,
                 imu_fresh=True, imu_yaw_rate=drift_rate,
                 flow_vx=0.0, flow_vy=0.0, alpha=0.0)
        com.step(dt=0.02, v_fl=0.0, v_fr=0.0, v_rl=0.0, v_rr=0.0,
                 imu_fresh=True, imu_yaw_rate=drift_rate,
                 flow_vx=0.0, flow_vy=0.0, alpha=0.0,
                 abs_yaw=0.0, heading_gain=0.2, heading_max_rate=0.15)
    assert sem.yaw == pytest.approx(drift_rate * 20.0, abs=1e-6)   # ~0.40 rad = 23 graus
    # Regime permanente da malha: erro ≈ taxa_de_deriva / ganho = 0.02/0.2 = 0.1 rad
    assert abs(com.yaw) < 0.12
    assert abs(com.yaw) < abs(sem.yaw) / 3.0


def test_step_ancora_nao_contamina_o_yaw_rate_publicado():
    # A correcao de deriva mexe na POSE, nao na velocidade angular medida: se
    # vazasse pro twist, o controlador veria um giro que nao existe.
    fo = FusedOdom(wheel_base=0.5)
    r = fo.step(dt=0.02, v_fl=0.0, v_fr=0.0, v_rl=0.0, v_rr=0.0,
                imu_fresh=True, imu_yaw_rate=0.0,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0,
                abs_yaw=1.0, heading_gain=0.2, heading_max_rate=0.15)
    assert r.yaw_rate == 0.0          # nenhum giro medido
    assert r.heading_corr > 0.0       # mas a pose foi corrigida
    assert r.yaw == pytest.approx(r.heading_corr)


def test_step_imu2_sozinha_segura_o_yaw_se_o_mpu_morrer():
    # Redundancia real: MPU mudo, BNO055 viva -> o yaw continua vindo de giro,
    # nao do diferencial de roda (que derrapa).
    fo = FusedOdom(wheel_base=0.5)
    r = fo.step(dt=0.1, v_fl=-0.5, v_fr=0.5, v_rl=-0.5, v_rr=0.5,
                imu_fresh=False, imu_yaw_rate=0.0,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0,
                imu2_fresh=True, imu2_yaw_rate=0.3, imu2_rate_weight=0.5)
    assert r.yaw_source == 'imu2'
    assert r.yaw == pytest.approx(0.03)


def test_degenerate_matches_wheel_only_odom():
    # Sem IMU, sem flow: deve bater com a integração ponto-médio do odom_publisher
    fo = FusedOdom(wheel_base=0.5)
    # avanço com leve giro
    v_fl = v_rl = 0.8
    v_fr = v_rr = 1.0
    dt = 0.1
    r = fo.step(dt=dt, v_fl=v_fl, v_fr=v_fr, v_rl=v_rl, v_rr=v_rr,
                imu_fresh=False, imu_yaw_rate=0.0,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    # Espelha odom_publisher: linear=(vr+vl)/2, angular=(vr-vl)/wb, ponto-médio
    v_left = (v_fl + v_rl) / 2.0
    v_right = (v_fr + v_rr) / 2.0
    linear = (v_left + v_right) / 2.0
    angular = (v_right - v_left) / 0.5
    theta_mid = 0.0 + 0.5 * angular * dt
    exp_x = linear * math.cos(theta_mid) * dt
    exp_y = linear * math.sin(theta_mid) * dt
    assert r.x == pytest.approx(exp_x)
    assert r.y == pytest.approx(exp_y)
    assert r.yaw == pytest.approx(angular * dt)


def test_slip_estimate_sem_referencia_devolve_nan():
    assert math.isnan(slip_estimate(2.0, 0.0, 0.0))
    assert math.isnan(slip_estimate(2.0, 0.0, 0.1))


def test_slip_estimate_com_referencia_mede_a_divergencia():
    assert slip_estimate(1.0, 0.4, 0.9) == pytest.approx(0.6)
    assert slip_estimate(0.4, 1.0, 0.9) == pytest.approx(-0.6)


def test_slip_estimate_nan_nunca_dispara_o_warn():
    slip = slip_estimate(5.0, 0.0, 0.0)
    assert not (abs(slip) > 0.15)
