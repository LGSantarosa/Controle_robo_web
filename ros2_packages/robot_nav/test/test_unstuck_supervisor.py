import math

import pytest

from robot_nav.unstuck_supervisor import (
    UnstuckConfig,
    UnstuckSupervisor,
    rear_blocked,
)


# ---- helpers puros ---------------------------------------------------------

def _scan_with_obstacle_at(angle_rad, dist, n=360):
    # scan de -pi..pi, tudo livre (inf) menos um feixe no ângulo pedido
    angle_min = -math.pi
    angle_increment = 2 * math.pi / n
    ranges = [float("inf")] * n
    i = int(round((angle_rad - angle_min) / angle_increment)) % n
    ranges[i] = dist
    return ranges, angle_min, angle_increment


def test_rear_blocked_detects_obstacle_behind():
    ranges, amin, ainc = _scan_with_obstacle_at(math.pi, 0.20)
    assert rear_blocked(ranges, amin, ainc, sector_deg=30, clearance=0.35) is True


def test_rear_blocked_clear_when_obstacle_only_in_front():
    ranges, amin, ainc = _scan_with_obstacle_at(0.0, 0.20)
    assert rear_blocked(ranges, amin, ainc, sector_deg=30, clearance=0.35) is False


def test_rear_blocked_clear_when_obstacle_far_behind():
    ranges, amin, ainc = _scan_with_obstacle_at(math.pi, 1.50)
    assert rear_blocked(ranges, amin, ainc, sector_deg=30, clearance=0.35) is False


def test_rear_blocked_ignores_obstacle_outside_sector():
    # 90° à esquerda não é "atrás"
    ranges, amin, ainc = _scan_with_obstacle_at(math.pi / 2, 0.20)
    assert rear_blocked(ranges, amin, ainc, sector_deg=30, clearance=0.35) is False


# ---- máquina de estados ----------------------------------------------------

def _cfg(**kw):
    base = dict(
        stuck_timeout=10.0,
        stuck_radius=0.05,
        reverse_distance=0.30,
        reverse_speed=0.25,
        reverse_time_cap=6.0,
        grace=2.0,
        nav_latch=15.0,
    )
    base.update(kw)
    return UnstuckConfig(**base)


def _tick(sup, t, pos=(0.0, 0.0), nav=True, rear=False):
    return sup.update(t, nav_wants_move=nav, position=pos, rear_blocked=rear)


def test_no_action_before_timeout():
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    cmd = _tick(sup, 9.9)
    assert cmd.active is False


def test_reverses_after_timeout_when_not_displacing():
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    cmd = _tick(sup, 10.1)
    assert cmd.active is True
    assert cmd.lin == pytest.approx(-0.25)
    assert cmd.ang == pytest.approx(0.0)


def test_never_spins():
    # GIRO REMOVIDO (decisão 2026-06-10): a manobra é SEMPRE ré, nunca angular.
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    for dt in (10.1, 10.2, 11.0, 12.0):
        cmd = _tick(sup, dt)
        assert cmd.ang == pytest.approx(0.0)


def test_micro_wiggle_still_fires():
    # BUG REAL: robô "tentando girar" sem sair do lugar (RotationShim/recoveries
    # do nav2) mexia uns mm e resetava o relógio. Deslocamento <stuck_radius
    # NÃO pode resetar — tem que disparar a ré mesmo assim.
    sup = UnstuckSupervisor(_cfg())
    fired = False
    t = 0.0
    while t <= 12.0:
        wiggle = (0.02 * math.sin(t * 7.0), 0.015 * math.cos(t * 5.0))
        cmd = _tick(sup, t, pos=wiggle)
        if cmd.active:
            fired = True
            break
        t += 0.1
    assert fired is True
    assert t <= 11.0


def test_robot_actually_moving_never_fires():
    # robô andando de verdade (>stuck_radius continuamente) nunca dispara
    sup = UnstuckSupervisor(_cfg())
    t = 0.0
    while t <= 20.0:
        cmd = _tick(sup, t, pos=(0.1 * t, 0.0))  # 0.1 m/s
        assert cmd.active is False
        t += 0.5


def test_no_fire_without_nav_goal():
    # sem o nav2 comandar (goal cancelado/atingido), parado NÃO é travado
    sup = UnstuckSupervisor(_cfg())
    t = 0.0
    while t <= 20.0:
        cmd = _tick(sup, t, nav=False)
        assert cmd.active is False
        t += 0.5


def test_nav_latch_tolerates_abort_gaps():
    # nav2 aborta e fica ~1-2s sem comandar; o latch segura e a ré ainda sai
    sup = UnstuckSupervisor(_cfg())
    fired = False
    t = 0.0
    while t <= 12.0:
        nav = (t % 9.0) < 8.0  # comanda 8s, cala 1s, repete
        cmd = _tick(sup, t, nav=nav)
        if cmd.active:
            fired = True
            break
        t += 0.5
    assert fired is True


def test_goal_gone_stops_firing():
    # nav silencia de vez (goal cancelado) -> depois do nav_latch, nada dispara
    sup = UnstuckSupervisor(_cfg(nav_latch=5.0))
    _tick(sup, 0.0, nav=True)
    t = 0.5
    while t <= 30.0:
        cmd = _tick(sup, t, nav=False)
        if t > 5.0:
            assert cmd.active is False, f"disparou em t={t} sem goal"
        t += 0.5


def test_rear_blocked_holds_then_fires_when_clear():
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0, rear=True)
    cmd = _tick(sup, 10.1, rear=True)
    assert cmd.active is False  # traseira bloqueada: segura (não gira, não ré)
    cmd = _tick(sup, 10.5, rear=False)  # liberou -> ré
    assert cmd.active is True
    assert cmd.lin == pytest.approx(-0.25)


def test_reverse_ends_with_explicit_stop():
    # FIM DA RÉ TEM QUE MANDAR ZERO: o cmd_vel_to_wheels segura o último
    # comando; sem um Twist 0 explícito o robô continuaria de ré até alguém
    # publicar de novo (nav2 pode estar mudo, abortado).
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    _tick(sup, 10.1)  # entra em RÉ na origem
    cmd = _tick(sup, 11.0, pos=(-0.30, 0.0))  # completou a distância
    assert cmd.active is True   # ainda publica...
    assert cmd.lin == pytest.approx(0.0)  # ...mas é o STOP
    assert cmd.ang == pytest.approx(0.0)
    cmd = _tick(sup, 11.2, pos=(-0.30, 0.0))  # grace: solta o canal
    assert cmd.active is False


def test_reverse_stops_after_time_cap():
    sup = UnstuckSupervisor(_cfg(reverse_time_cap=6.0))
    _tick(sup, 0.0)
    _tick(sup, 10.1)  # entra em RÉ
    cmd = _tick(sup, 16.3, pos=(0.0, 0.0))  # passou do cap sem recuar
    assert cmd.active is True and cmd.lin == pytest.approx(0.0)  # STOP explícito
    cmd = _tick(sup, 16.5, pos=(0.0, 0.0))
    assert cmd.active is False


def test_grace_before_rearming():
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    _tick(sup, 10.1)  # RÉ
    _tick(sup, 11.0, pos=(-0.30, 0.0))  # completou -> STOP -> grace
    cmd = _tick(sup, 11.5, pos=(-0.30, 0.0))
    assert cmd.active is False  # ainda no grace


# ---- gate por STATUS do goal (autoritativo quando disponível) ---------------

def test_goal_inactive_blocks_fire_even_with_nav_msgs():
    # goal cancelado/atingido (status diz INATIVO) -> NUNCA dá ré póstuma,
    # mesmo que o flag de nav_vel_raw tenha ficado True pra trás
    sup = UnstuckSupervisor(_cfg())
    t = 0.0
    while t <= 25.0:
        cmd = sup.update(t, nav_wants_move=True, position=(0.0, 0.0),
                         rear_blocked=False, goal_active=False)
        assert cmd.active is False
        t += 0.5


def test_goal_active_fires_even_with_controller_silent():
    # BT em recovery (controller mudo, nav_vel_raw sem msg) mas goal ATIVO:
    # robô sem se deslocar 10s -> ré mesmo assim
    sup = UnstuckSupervisor(_cfg())
    fired = False
    t = 0.0
    while t <= 12.0:
        cmd = sup.update(t, nav_wants_move=False, position=(0.0, 0.0),
                         rear_blocked=False, goal_active=True)
        if cmd.active:
            fired = True
            break
        t += 0.5
    assert fired is True


def test_goal_status_none_falls_back_to_latch():
    # sem status (tópico não visto): comportamento antigo via nav_latch
    sup = UnstuckSupervisor(_cfg())
    _tick(sup, 0.0)
    cmd = _tick(sup, 10.1)
    assert cmd.active is True


def test_refires_repeatedly_if_still_stuck():
    # "nunca para sem razão": continua travado -> ré de novo, e de novo
    sup = UnstuckSupervisor(_cfg(reverse_time_cap=2.0, grace=1.0))
    fires = 0
    t = 0.0
    while t <= 60.0 and fires < 3:
        was_idle = not sup_active(sup)
        cmd = _tick(sup, t)  # posição parada pra sempre (ré não surte efeito)
        if cmd.active and was_idle:
            fires += 1
        t += 0.5
    assert fires >= 3


def sup_active(sup):
    return sup.state != "monitoring"
