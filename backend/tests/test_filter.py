"""Tests for backend.filter."""

from backend.filter import ComplementaryFilter


def test_filter_initialises_from_first_sample():
    f = ComplementaryFilter(alpha=0.5)
    r, p, y = f.step({"t": 0, "r": 10.0, "p": 20.0, "y": 30.0, "gx": 0, "gy": 0, "gz": 0})
    assert (r, p, y) == (10.0, 20.0, 30.0)


def test_filter_moves_with_gyro():
    f = ComplementaryFilter(alpha=0.95)
    f.step({"t": 0, "r": 0.0, "p": 0.0, "y": 0.0, "gx": 0, "gy": 0, "gz": 0})
    r2, _, _ = f.step({"t": 1000, "r": 0.0, "p": 0.0, "y": 0.0, "gx": 10.0, "gy": 0, "gz": 0})
    assert r2 > 0


def test_reset():
    f = ComplementaryFilter()
    f.step({"t": 0, "r": 5, "p": 5, "y": 5, "gx": 0, "gy": 0, "gz": 0})
    f.reset()
    r, p, y = f.step({"t": 10, "r": 1, "p": 2, "y": 3, "gx": 0, "gy": 0, "gz": 0})
    assert (r, p, y) == (1.0, 2.0, 3.0)
