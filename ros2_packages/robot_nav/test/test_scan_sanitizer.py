import math

import numpy as np
import pytest

from robot_nav.scan_sanitizer import sanitize_ranges


def test_phantom_below_threshold_becomes_inf():
    # Assinatura real da captura 2026-06-12: retornos a ~6 cm do sensor,
    # dentro do chassi.
    out, n = sanitize_ranges([0.057, 1.2, 0.13, 3.4], min_valid=0.15)
    assert n == 2
    assert math.isinf(out[0]) and math.isinf(out[2])
    assert out[1] == pytest.approx(1.2)
    assert out[3] == pytest.approx(3.4)


def test_valid_ranges_untouched():
    out, n = sanitize_ranges([0.15, 0.5, 24.9], min_valid=0.15)
    assert n == 0
    assert list(out) == pytest.approx([0.15, 0.5, 24.9])


def test_zero_inf_nan_pass_through():
    # 0.0 = "sem retorno" do driver; inf/NaN já são tratados pelos
    # consumidores — o filtro não inventa nada em cima deles.
    out, n = sanitize_ranges([0.0, math.inf, math.nan], min_valid=0.15)
    assert n == 0
    assert out[0] == 0.0
    assert math.isinf(out[1])
    assert math.isnan(out[2])


def test_obstacle_touching_bumper_not_eaten():
    # Obstáculo real encostado no para-choque (~25 cm do centro) NUNCA pode
    # ser filtrado — o limiar fica abaixo do footprint de propósito.
    out, n = sanitize_ranges([0.25, 0.26], min_valid=0.15)
    assert n == 0
    assert list(out) == pytest.approx([0.25, 0.26])


def test_accepts_ndarray_and_returns_float32():
    arr = np.array([0.05, 2.0], dtype=np.float32)
    out, n = sanitize_ranges(arr, min_valid=0.15)
    assert n == 1
    assert out.dtype == np.float32


def test_empty_scan():
    out, n = sanitize_ranges([], min_valid=0.15)
    assert n == 0
    assert out.size == 0
