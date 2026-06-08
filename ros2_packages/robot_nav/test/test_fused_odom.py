import math

import pytest

from robot_nav.fused_odom import (
    FusedOdom,
    flow_alpha,
    flow_tick_velocity,
    flow_yaw_gate,
    fuse_translation,
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
