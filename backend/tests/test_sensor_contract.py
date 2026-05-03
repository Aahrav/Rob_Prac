"""Tests for backend.sensor_contract."""

import pytest

from backend.sensor_contract import normalise_sample


def test_normalise_coerces_types():
    d = normalise_sample({"t": "100", "r": "1.5", "p": 2, "y": -3})
    assert d["t"] == 100
    assert d["r"] == 1.5
    assert d["p"] == 2.0
    assert d["y"] == -3.0
    assert d["gx"] == d["gy"] == d["gz"] == 0.0


def test_normalise_missing_key():
    with pytest.raises(ValueError):
        normalise_sample({"t": 1, "r": 0})
