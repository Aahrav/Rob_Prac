#!/usr/bin/env python3
"""
ReplayWorker — CSV replay producer (Part 2, P2-T2).

Uses ``backend.replay_io`` for iteration and timing (Part 1).
"""

from __future__ import annotations

import time
from pathlib import Path

from PyQt6.QtCore import QThread, pyqtSignal

from backend.replay_io import compute_sleep_seconds, iter_replay_rows


class ReplayWorker(QThread):
    """CSV replay producer."""

    sample_received = pyqtSignal(dict)
    producer_error = pyqtSignal(str)
    producer_status = pyqtSignal(str)

    def __init__(self, file_path: str, loop: bool = False, parent=None):
        super().__init__(parent)
        self._file_path = Path(file_path)
        self.loop = loop
        self._stop_flag = False

    def stop(self) -> None:
        self._stop_flag = True

    def run(self) -> None:
        self._stop_flag = False

        if not self._file_path.exists():
            self.producer_error.emit(f"Replay file not found: {self._file_path}")
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
        while True:
            t_prev_ms: int | None = None

            for sample in iter_replay_rows(self._file_path):
                if self._stop_flag:
                    return

                t_curr_ms = int(sample["t"])

                if t_prev_ms is not None:
                    sleep_s = compute_sleep_seconds(t_prev_ms, t_curr_ms)
                    if sleep_s > 0:
                        time.sleep(sleep_s)

                t_prev_ms = t_curr_ms

                if self._stop_flag:
                    return

                self.sample_received.emit(dict(sample))

            if not self.loop or self._stop_flag:
                break

            self.producer_status.emit("Replay looping…")
            time.sleep(0.1)
