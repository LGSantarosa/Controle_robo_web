import math

import pytest

from robot_nav.unstuck_supervisor import (
    UnstuckConfig,
    UnstuckSupervisor,
    is_frozen,
    rear_blocked,
)


# ---- helpers puros ---------------------------------------------------------

def test_is_frozen_true_when_below_thresholds():
    assert is_frozen(0.01, 0.02, zero_lin=0.02, zero_ang=0.05) is True


def test_is_frozen_false_when_linear_moving():
    assert is_frozen(0.10, 0.0, zero_lin=0.02, zero_ang=0.05) is False


def test_is_frozen_false_when_rotating():
    # robô girando de verdade não está congelado
    assert is_frozen(0.0, 0.20, zero_lin=0.02, zero_ang=0.05) is False


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
        reverse_distance=0.30,
        reverse_speed=0.15,
        reverse_time_cap=3.0,
        spin_speed=0.5,
        spin_angle=1.0,
        escalate_after=3,
        same_spot_radius=0.5,
        escalate_window=60.0,
        grace=2.0,
    )
    base.update(kw)
    return UnstuckConfig(**base)


def _stuck(sup, t, pos=(0.0, 0.0), rear=False):
    return sup.update(
        t,
        stop_active=True,
        frozen=True,
        nav_wants_move=True,
        position=pos,
        rear_blocked=rear,
    )


def test_no_action_before_timeout():
    sup = UnstuckSupervisor(_cfg())
    _stuck(sup, 0.0)
    cmd = _stuck(sup, 9.9)
    assert cmd.active is False


def test_reverses_after_timeout_when_rear_clear():
    sup = UnstuckSupervisor(_cfg())
    _stuck(sup, 0.0)
    cmd = _stuck(sup, 10.1)
    assert cmd.active is True
    assert cmd.lin == pytest.approx(-0.15)
    assert cmd.ang == pytest.approx(0.0)


def test_spins_after_timeout_when_rear_blocked():
    sup = UnstuckSupervisor(_cfg())
    _stuck(sup, 0.0, rear=True)
    cmd = _stuck(sup, 10.1, rear=True)
    assert cmd.active is True
    assert cmd.lin == pytest.approx(0.0)
    assert cmd.ang == pytest.approx(0.5)


def test_timer_resets_when_condition_breaks():
    # se em algum momento ele NÃO está travado, o contador zera
    sup = UnstuckSupervisor(_cfg())
    _stuck(sup, 0.0)
    # robô andou (não congelado) no meio do caminho
    sup.update(5.0, stop_active=False, frozen=False, nav_wants_move=True,
               position=(0.0, 0.0), rear_blocked=False)
    cmd = _stuck(sup, 10.1)
    assert cmd.active is False  # só passaram 0.1s desde que re-travou


def test_reverse_stops_after_distance():
    sup = UnstuckSupervisor(_cfg())
    _stuck(sup, 0.0)
    _stuck(sup, 10.1)  # entra em RÉ na origem
    # recuou 0.30m -> deve soltar
    cmd = sup.update(11.0, stop_active=True, frozen=False, nav_wants_move=True,
                     position=(-0.30, 0.0), rear_blocked=False)
    assert cmd.active is False


def test_reverse_stops_after_time_cap():
    sup = UnstuckSupervisor(_cfg(reverse_time_cap=3.0))
    _stuck(sup, 0.0)
    _stuck(sup, 10.1)  # entra em RÉ
    # passou do cap sem recuar (odom não progrediu)
    cmd = sup.update(13.2, stop_active=True, frozen=True, nav_wants_move=True,
                     position=(0.0, 0.0), rear_blocked=False)
    assert cmd.active is False


def test_grace_before_rearming():
    sup = UnstuckSupervisor(_cfg())
    _stuck(sup, 0.0)
    _stuck(sup, 10.1)  # RÉ
    sup.update(11.0, stop_active=True, frozen=False, nav_wants_move=True,
               position=(-0.30, 0.0), rear_blocked=False)  # solta -> grace
    # logo após soltar, mesmo travado de novo, ainda está no grace
    cmd = _stuck(sup, 11.5, pos=(-0.30, 0.0))
    assert cmd.active is False


def _run_stuck_cycle(sup, t0, pos, cap=1.0, grace=0.1):
    """Roda um ciclo travamento->manobra->fim->volta a MONITORANDO.

    Retorna (cmd_da_manobra, t_pronto_pro_proximo_ciclo).
    """
    _stuck(sup, t0, pos=pos)                  # arma o contador
    cmd = _stuck(sup, t0 + 10.1, pos=pos)     # dispara a manobra
    # encerra a manobra pelo cap de tempo (robô não progrediu)
    t_end = t0 + 10.1 + cap + 0.1
    sup.update(t_end, stop_active=True, frozen=True, nav_wants_move=True,
               position=pos, rear_blocked=False)
    # passa do grace -> volta pra MONITORANDO (condição quebra)
    t_clear = t_end + grace + 0.1
    sup.update(t_clear, stop_active=False, frozen=False, nav_wants_move=False,
               position=pos, rear_blocked=False)
    return cmd, t_clear


def test_escalates_to_spin_after_repeated_stuck_same_spot():
    sup = UnstuckSupervisor(_cfg(reverse_time_cap=1.0, grace=0.1))
    t = 0.0
    cmds = []
    for _ in range(3):
        cmd, t = _run_stuck_cycle(sup, t, pos=(0.0, 0.0))
        cmds.append(cmd)
    # 1ª e 2ª: ré (rear livre); a 3ª no mesmo ponto escala pro giro
    assert cmds[0].lin == pytest.approx(-0.15) and cmds[0].ang == 0.0
    assert cmds[2].lin == pytest.approx(0.0) and cmds[2].ang == pytest.approx(0.5)


def test_does_not_escalate_when_robot_escaped():
    # travamentos em pontos distantes não contam como "mesmo ponto"
    sup = UnstuckSupervisor(_cfg(reverse_time_cap=1.0, grace=0.1))
    t = 0.0
    for k in range(3):
        pos = (2.0 * k, 0.0)  # cada travamento 2m adiante
        cmd, t = _run_stuck_cycle(sup, t, pos=pos)
        # rear livre -> sempre ré, nunca giro, porque nunca é "o mesmo ponto"
        assert cmd.lin == pytest.approx(-0.15)
        assert cmd.ang == pytest.approx(0.0)
