"""Tests for backend.replay_io."""

from pathlib import Path

import pytest

from backend.replay_io import compute_sleep_seconds, iter_replay_rows


def test_compute_sleep_caps_gap():
    assert compute_sleep_seconds(0, 10_000) == 0.5


def test_iter_fixture(tmp_path: Path):
    src = Path(__file__).resolve().parents[2] / "recordings" / "fixtures" / "synthetic_smooth.csv"
    if not src.exists():
        pytest.skip("fixture CSV not present")
    rows = list(iter_replay_rows(src))
    assert len(rows) >= 2
    assert "t" in rows[0] and "r" in rows[0]


def test_iter_bad_header(tmp_path: Path):
    bad = tmp_path / "bad.csv"
    bad.write_text("a,b,c\n1,2,3\n", encoding="utf-8")
    with pytest.raises(ValueError, match="missing"):
        list(iter_replay_rows(bad))
