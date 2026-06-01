import math

import pytest

from robot_nav.fused_odom import (
    FusedOdom,
    flow_alpha,
    fuse_translation,
    wheel_twist,
)


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
                imu_fresh=False, imu_yaw=0.0, imu_yaw_rate=0.0,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    assert r.yaw_source == 'wheel'
    assert r.yaw == pytest.approx(2.0 * 0.1)  # omega=2 rad/s * dt
    assert r.yaw_rate == pytest.approx(2.0)


def test_imu_fresh_uses_imu_yaw_ignoring_wheels():
    # Com IMU fresca, o yaw é o absoluto da IMU mesmo com rodas girando
    fo = FusedOdom(wheel_base=0.5)
    r = fo.step(dt=0.1, v_fl=-0.5, v_fr=0.5, v_rl=-0.5, v_rr=0.5,
                imu_fresh=True, imu_yaw=0.7, imu_yaw_rate=0.3,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    assert r.yaw_source == 'imu'
    assert r.yaw == pytest.approx(0.7)
    assert r.yaw_rate == pytest.approx(0.3)


def test_imu_dropout_snaps_to_wheel_from_last_yaw():
    # IMU presente (yaw=1.0), depois cai → integra do último yaw, sem voltar a 0
    fo = FusedOdom(wheel_base=0.5)
    fo.step(dt=0.1, v_fl=0.0, v_fr=0.0, v_rl=0.0, v_rr=0.0,
            imu_fresh=True, imu_yaw=1.0, imu_yaw_rate=0.0,
            flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    r = fo.step(dt=0.1, v_fl=-0.5, v_fr=0.5, v_rl=-0.5, v_rr=0.5,
                imu_fresh=False, imu_yaw=0.0, imu_yaw_rate=0.0,
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
                imu_fresh=False, imu_yaw=0.0, imu_yaw_rate=0.0,
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
