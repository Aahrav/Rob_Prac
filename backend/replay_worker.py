#!/usr/bin/env python3
"""
ReplayWorker — CSV replay producer (Part 2, P2-T2).

Reads a recorded CSV file row-by-row and emits sensor dicts at the timing
encoded in the ``t`` (milliseconds) column — reproducing original capture
timing. Uses Part 1's ``replay_io`` helpers when available.

Unified producer signal interface (P2-T1):
    sample_received(dict)   — one normalised sensor dict per row
    producer_error(str)     — missing file, bad header, or unrecoverable IO error
    producer_status(str)    — state messages ("Replay running", "Replay complete", …)

Design decisions (documented per spec §P2-T4):
  - Timing: ``compute_sleep_seconds`` from Part 1 caps extreme gaps at 500 ms.
  - Stop: ``_stop_flag`` checked before each row; thread joined by caller.
  - Missing file: emits ``producer_error`` and returns; app stays open.
  - Loop mode: disabled by default; set ``worker.loop = True`` before start().

Part 1 import strategy:
  - Tries ``from backend.replay_io import iter_replay_rows, compute_sleep_seconds``.
  - Falls back to _local stub implementations if Part 1 module isn't available yet.
  - TODO: remove fallback stubs once Part 1 lands (search "P1_FALLBACK").

CSV contract (§3.3): header must contain at minimum ``t,r,p,y``.
"""

import csv
import time
from pathlib import Path
from typing import Iterator

from PyQt6.QtCore import QThread, pyqtSignal

# ── Part 1 import with fallback ─────────────────────────────────────────────
try:
    from backend.replay_io import iter_replay_rows, compute_sleep_seconds  # type: ignore
    _P1_REPLAY_IO_AVAILABLE = True
except ImportError:
    _P1_REPLAY_IO_AVAILABLE = False


# ── P1_FALLBACK: local stubs (remove when Part 1 replay_io lands) ───────────

_REQUIRED_KEYS = ("t", "r", "p", "y")
_MAX_SLEEP_S = 0.5  # cap extreme timing gaps (mirrors Part 1 spec)


def _fallback_iter_replay_rows(path: Path) -> Iterator[dict]:
    """Minimal CSV reader matching the contract. Used only when Part 1 is absent."""
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV file is empty or has no header.")
        missing = [k for k in _REQUIRED_KEYS if k not in reader.fieldnames]
        if missing:
            raise ValueError(
                f"CSV header missing required keys: {missing}. "
                f"Found: {list(reader.fieldnames)}"
            )
        for row in reader:
            try:
                sample = {
                    "t":  int(float(row["t"])),
                    "r":  float(row["r"]),
                    "p":  float(row["p"]),
                    "y":  float(row["y"]),
                    "gx": float(row.get("gx", 0.0) or 0.0),
                    "gy": float(row.get("gy", 0.0) or 0.0),
                    "gz": float(row.get("gz", 0.0) or 0.0),
                }
                yield sample
            except (ValueError, KeyError):
                # Skip malformed rows silently (mirrors Part 1 parser behaviour)
                continue


def _fallback_compute_sleep_seconds(t_prev_ms: int, t_curr_ms: int) -> float:
    """Cap timing gap at _MAX_SLEEP_S to avoid hanging on sparse data."""
    delta_s = (t_curr_ms - t_prev_ms) / 1000.0
    return max(0.0, min(delta_s, _MAX_SLEEP_S))


# ── Resolve which implementation to use ─────────────────────────────────────

if _P1_REPLAY_IO_AVAILABLE:
    _iter_rows = iter_replay_rows
    _sleep_s   = compute_sleep_seconds
else:
    _iter_rows = _fallback_iter_replay_rows
    _sleep_s   = _fallback_compute_sleep_seconds


# ── ReplayWorker ─────────────────────────────────────────────────────────────

class ReplayWorker(QThread):
    """CSV replay producer.

    Parameters
    ----------
    file_path:
        Path to the CSV file to replay. Missing file emits ``producer_error``.
    loop:
        If ``True``, replay loops indefinitely until ``stop()`` is called.
        Default: ``False`` (play once).

    Usage::

        worker = ReplayWorker("recordings/session.csv")
        worker.sample_received.connect(window._on_sample_received)
        worker.producer_error.connect(window._on_producer_error)
        worker.producer_status.connect(window._on_producer_status)
        worker.start()
        ...
        worker.stop()
        worker.wait(3000)
    """

    # ── Unified producer signal interface (P2-T1) ────────────────────────────
    sample_received = pyqtSignal(dict)
    producer_error  = pyqtSignal(str)
    producer_status = pyqtSignal(str)

    def __init__(self, file_path: str, loop: bool = False, parent=None):
        super().__init__(parent)
        self._file_path = Path(file_path)
        self.loop = loop
        self._stop_flag = False

    # ── Public control API ────────────────────────────────────────────────────

    def stop(self) -> None:
        """Signal the run loop to exit on next iteration."""
        self._stop_flag = True

    # ── QThread entry point ───────────────────────────────────────────────────

    def run(self) -> None:
        """Main loop — runs in a separate thread. Do not call directly."""
        self._stop_flag = False

        if not self._file_path.exists():
            self.producer_error.emit(
                f"Replay file not found: {self._file_path}"
            )
            return

        self.producer_status.emit(f"Replay running — {self._file_path.name}")

        try:
            self._replay_once_or_loop()
        except (OSError, ValueError) as exc:
            self.producer_error.emit(f"Replay error: {exc}")
            return

        if not self._stop_flag:
            self.producer_status.emit("Replay complete")

    def _replay_once_or_loop(self) -> None:
        """Inner replay loop — plays file once or loops until stop()."""
        while True:
            t_prev_ms: int | None = None

            for sample in _iter_rows(self._file_path):
                if self._stop_flag:
                    return

                t_curr_ms = sample.get("t", 0)

                # Sleep to reproduce original timing
                if t_prev_ms is not None:
                    sleep_s = _sleep_s(t_prev_ms, t_curr_ms)
                    if sleep_s > 0:
                        time.sleep(sleep_s)

                t_prev_ms = t_curr_ms

                if self._stop_flag:
                    return

                self.sample_received.emit(dict(sample))

            # File exhausted
            if not self.loop or self._stop_flag:
                break

            # Loop mode: brief pause then restart
            self.producer_status.emit("Replay looping…")
            time.sleep(0.1)
