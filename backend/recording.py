"""
CSV recording helpers — shared header / row shape with replay_io.
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import IO, Any

from backend.sensor_contract import CSV_FIELDNAMES


def open_record_csv(path: str | Path) -> tuple[IO[str], csv.DictWriter]:
    """
    Create/truncate a CSV file and write the header row.

    Returns:
        (file_handle, DictWriter) — caller must close the file handle.
    """
    record_path = Path(path)
    record_path.parent.mkdir(parents=True, exist_ok=True)
    fh = record_path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        fh,
        fieldnames=CSV_FIELDNAMES,
        extrasaction="ignore",
    )
    writer.writeheader()
    fh.flush()
    return fh, writer


def row_from_sample(sample: dict[str, Any]) -> dict[str, Any]:
    """Build one CSV row dict from a normalised sensor sample."""
    return {
        "t": sample.get("t", 0),
        "r": sample.get("r", 0.0),
        "p": sample.get("p", 0.0),
        "y": sample.get("y", 0.0),
        "gx": sample.get("gx", 0.0),
        "gy": sample.get("gy", 0.0),
        "gz": sample.get("gz", 0.0),
    }
