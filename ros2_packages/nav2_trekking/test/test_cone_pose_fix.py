import math

import pytest

from nav2_trekking.cone_pose_fix import (
    ConeFixConfirmer,
    apply_pose_fix,
    cone_bearing,
    cone_fix_delta,
    cone_id_do_match,
)


def test_cone_fix_delta():
    dx, dy = cone_fix_delta((2.0, 3.0), (1.7, 3.4))
    assert dx == pytest.approx(0.3)
    assert dy == pytest.approx(-0.4)


def test_apply_pose_fix_accepts_small():
    nx, ny, ok = apply_pose_fix(10.0, 5.0, 0.4, -0.2, gain=0.5, max_mag=0.6)
    assert ok is True
    assert nx == pytest.approx(10.2)
    assert ny == pytest.approx(4.9)


def test_apply_pose_fix_rejects_large():
    nx, ny, ok = apply_pose_fix(10.0, 5.0, 0.7, 0.0, gain=0.5, max_mag=0.6)
    assert ok is False
    assert (nx, ny) == (10.0, 5.0)


def test_cone_bearing_relative_to_recorded_yaw():
    assert cone_bearing(0.0, 0.0, 0.0, 1.0, 1.0) == pytest.approx(math.pi / 4)
    assert cone_bearing(0.0, 0.0, math.pi / 2, 1.0, 1.0) == pytest.approx(-math.pi / 4)


def test_confirmer_stable_sequence_confirms():
    c = ConeFixConfirmer(confirm_frames=4, stable_eps=0.10)
    pos = (1.0, 2.0)
    results = [c.update(pos, n_candidates=1) for _ in range(4)]
    assert results == [False, False, False, True]


def test_confirmer_moving_never_confirms():
    c = ConeFixConfirmer(confirm_frames=4, stable_eps=0.10)
    confirmed = False
    for i in range(10):
        confirmed = confirmed or c.update((1.0 + 0.2 * i, 2.0), n_candidates=1)
    assert confirmed is False


def test_confirmer_ambiguous_skips():
    c = ConeFixConfirmer(confirm_frames=2, stable_eps=0.10)
    pos = (1.0, 2.0)
    assert c.update(pos, n_candidates=2) is False
    assert c.update(pos, n_candidates=2) is False
    assert c.update(pos, n_candidates=2) is False


def test_confirmer_no_match_resets():
    c = ConeFixConfirmer(confirm_frames=2, stable_eps=0.10)
    pos = (1.0, 2.0)
    assert c.update(pos, n_candidates=1) is False
    assert c.update(None, n_candidates=1) is False
    assert c.update(pos, n_candidates=1) is False


def test_confirmer_count_exposes_progress():
    c = ConeFixConfirmer(confirm_frames=4, stable_eps=0.10)
    assert c.count == 0
    c.update((1.0, 2.0), n_candidates=1)
    assert c.count == 1
    c.update((1.0, 2.0), n_candidates=1)
    assert c.count == 2


# ---------------------------------------------------------------- identidade
# INSTRUMENTAÇÃO 2026-08-26: `cone_id_do_match` é a coluna do CSV que separa
# "a pose derivou" de "a associação pulou de cone". Os dois viram Δ grande.

def test_cone_id_escolhe_o_cone_gravado_mais_proximo():
    gravados = [(1.0, 0.0), (5.0, 0.0), (9.0, 0.0)]
    assert cone_id_do_match((5.2, 0.1), gravados) == 1


def test_cone_id_denuncia_o_pulo_pro_cone_anterior():
    """O BO real: a pose corrigida passeia e a detecção casa com o cone ANTERIOR."""
    gravados = [(1.0, 0.0), (5.0, 0.0)]
    assert cone_id_do_match((4.9, 0.0), gravados) == 1   # antes: cone 2
    assert cone_id_do_match((1.3, 0.0), gravados) == 0   # depois: voltou pro 1


def test_cone_id_sem_match_ou_sem_cones_devolve_menos_um():
    assert cone_id_do_match(None, [(1.0, 0.0)]) == -1
    assert cone_id_do_match((1.0, 0.0), []) == -1
