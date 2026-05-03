#!/usr/bin/env python3
"""
SimWorker — Simulation producer (Part 2, P2-T3).

A dedicated QThread that synthesises sinusoidal sensor data and emits it
through the unified producer signal interface:

    sample_received(dict)   — one valid sensor dict per tick
    producer_error(str)     — not used by SimWorker; present for interface parity
    producer_status(str)    — human-readable state changes

Emitted dict shape (satisfies sensor_contract REQUIRED_KEYS + optional gyros):
    {"t": int, "r": float, "p": float, "y": float,
     "gx": 0.0, "gy": 0.0, "gz": 0.0}

Where `t` is wall-clock milliseconds (monotonic).

Design decisions:
  - Tick interval: 50 ms (≈20 Hz), matches original inline Simulator.
  - Gyros emitted as 0.0 — acceptable for MVP filter testing per spec §3.3.
  - Stop: _stop_flag checked at top of each loop; thread.wait() in caller.
"""

import time
import math

from PyQt6.QtCore import QThread, pyqtSignal


class SimWorker(QThread):
    """Simulation producer — emits synthetic sinusoidal sensor dicts.

    Usage::

        worker = SimWorker()
        worker.sample_received.connect(window._on_sample_received)
        worker.producer_status.connect(window._on_producer_status)
        worker.start()
        ...
        worker.stop()
        worker.wait(3000)
    """

    # ── Unified producer signal interface (P2-T1) ───────────────────────────
    sample_received = pyqtSignal(dict)
    producer_error  = pyqtSignal(str)   # kept for interface parity; not used here
    producer_status = pyqtSignal(str)

    # Tick interval in seconds (20 Hz)
    TICK_S = 0.05

    def __init__(self, parent=None):
        super().__init__(parent)
        self._stop_flag = False

    # ── Public control API ───────────────────────────────────────────────────

    def stop(self) -> None:
        """Signal the run loop to exit on next iteration."""
        self._stop_flag = True

    # ── QThread entry point ──────────────────────────────────────────────────

    def run(self) -> None:
        """Main loop — runs in a separate thread. Do not call directly."""
        self._stop_flag = False
        self.producer_status.emit("Simulation running")

        t = 0.0
        start_ms = int(time.monotonic() * 1000)

        while not self._stop_flag:
            now_ms = int(time.monotonic() * 1000)

            roll  = 20.0 * math.sin(t * 0.5)
            pitch = 15.0 * math.sin(t * 0.7 + 1.0)
            yaw   = 30.0 * math.sin(t * 0.3 + 2.0)

            sample = {
                "t":  now_ms - start_ms,  # relative ms since worker started
                "r":  round(roll,  4),
                "p":  round(pitch, 4),
                "y":  round(yaw,   4),
                "gx": 0.0,
                "gy": 0.0,
                "gz": 0.0,
            }

            self.sample_received.emit(sample)

            t += self.TICK_S
            time.sleep(self.TICK_S)

        self.producer_status.emit("Simulation stopped")
