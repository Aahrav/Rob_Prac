#!/usr/bin/env python3
"""
SerialWorker — Hardware USB-serial producer stub (Part 2, P2-T6).

This class is a **placeholder** for the hardware phase.
It exposes the same unified producer signal interface as SimWorker and
ReplayWorker so that MainWindow can connect to it identically when hardware
is available, without any UI changes.

Future implementation (hardware phase):
  1. Accept `port: str` and `baud: int` in __init__.
  2. In run(): open pyserial port, read UTF-8 lines in a loop.
  3. Pass each line through ``backend.parser.parse_sensor_line``.
  4. Emit ``sample_received`` for valid dicts; ``producer_error`` for errors.
  5. Emit ``producer_status`` for port open/close events.

No changes to Part 1 filter or Part 3 calibration are required as long as
the contract dict shape is honoured (see backend/sensor_contract.py §3.1).
"""

from PyQt6.QtCore import QThread, pyqtSignal


class SerialWorker(QThread):
    """Hardware USB-serial producer — NOT YET IMPLEMENTED.

    Stub class that satisfies the unified producer signal interface so that
    MainWindow wiring and Part 3 UI can reference this class without error.
    Calling ``start()`` will raise ``NotImplementedError`` at runtime.
    """

    # ── Unified producer signal interface (P2-T1) ───────────────────────────
    sample_received = pyqtSignal(dict)
    producer_error  = pyqtSignal(str)
    producer_status = pyqtSignal(str)

    def __init__(self, port: str = "", baud: int = 115200, parent=None):
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._stop_flag = False

    def stop(self) -> None:
        """Signal the run loop to exit (no-op in stub)."""
        self._stop_flag = True

    def run(self) -> None:
        """Entry point — raises NotImplementedError (hardware phase only)."""
        raise NotImplementedError(
            "SerialWorker is not yet implemented. "
            "Hardware phase: read UTF-8 lines from pyserial → "
            "parse_sensor_line → emit sample_received."
        )
