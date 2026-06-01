import math

import pytest

from robot_nav.cone_pose_fix import (
    ConeFixConfirmer,
    apply_pose_fix,
    cone_bearing,
    cone_fix_delta,
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
