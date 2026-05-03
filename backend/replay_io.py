"""
CSV replay timing helpers (MVP §P1-T5).
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterator

from backend.sensor_contract import REQUIRED_KEYS, normalise_sample

_MAX_GAP_S = 0.5


def compute_sleep_seconds(t_prev_ms: int, t_curr_ms: int) -> float:
    """Wall-clock sleep between samples; caps huge gaps at ``_MAX_GAP_S``."""
    delta_s = (t_curr_ms - t_prev_ms) / 1000.0
    return max(0.0, min(delta_s, _MAX_GAP_S))


def iter_replay_rows(path: Path) -> Iterator[dict]:
    """
    Yield normalised sensor dicts from a UTF-8 CSV file.

    Raises:
        ValueError: empty file or missing required columns.
    """
    path = Path(path)
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV file is empty or has no header.")
        missing = [k for k in REQUIRED_KEYS if k not in reader.fieldnames]
        if missing:
            raise ValueError(
                f"CSV header missing required keys: {missing}. "
                f"Found: {list(reader.fieldnames)}"
            )
        for row in reader:
            try:
                yield normalise_sample(row)
            except ValueError:
                continue
