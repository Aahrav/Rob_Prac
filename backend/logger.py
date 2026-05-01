#!/usr/bin/env python3
"""
Centralised logging configuration for RoboSim.

Usage (in any module):
    from backend.logger import get_logger
    log = get_logger(__name__)
    log.debug("something happened")

Log file: logs/robosim.log  (rotating, max 2 MB × 5 backups)
Console:  WARNING and above (keeps the terminal clean during normal use)
"""

import logging
import logging.handlers
import os
import sys
from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────────────────────
_ROOT = Path(__file__).resolve().parent.parent   # Rob_Prac/
_LOG_DIR = _ROOT / "logs"
_LOG_FILE = _LOG_DIR / "robosim.log"

# ── Formats ───────────────────────────────────────────────────────────────────
_FILE_FMT = (
    "%(asctime)s  %(levelname)-8s  %(name)-35s  %(funcName)-30s  %(message)s"
)
_CONSOLE_FMT = "%(levelname)-8s  %(name)s — %(message)s"
_DATE_FMT = "%Y-%m-%d %H:%M:%S"

# ── One-time setup flag ───────────────────────────────────────────────────────
_configured = False


def _setup() -> None:
    global _configured
    if _configured:
        return
    _configured = True

    _LOG_DIR.mkdir(parents=True, exist_ok=True)

    root = logging.getLogger("robosim")
    root.setLevel(logging.DEBUG)          # capture everything at root level
    root.propagate = False

    # ── Rotating file handler (DEBUG+) ────────────────────────────────────
    fh = logging.handlers.RotatingFileHandler(
        _LOG_FILE,
        maxBytes=2 * 1024 * 1024,   # 2 MB
        backupCount=5,
        encoding="utf-8",
    )
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(_FILE_FMT, datefmt=_DATE_FMT))
    root.addHandler(fh)

    # ── Console handler (WARNING+) ────────────────────────────────────────
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.WARNING)
    ch.setFormatter(logging.Formatter(_CONSOLE_FMT))
    root.addHandler(ch)

    root.info("=" * 80)
    root.info("RoboSim logger initialised  —  log file: %s", _LOG_FILE)
    root.info("=" * 80)


def get_logger(name: str) -> logging.Logger:
    """
    Return a child logger under the 'robosim' hierarchy.

    Args:
        name: typically __name__ of the calling module.
              The 'Rob_Prac.' prefix is stripped for brevity.
    """
    _setup()
    # Strip the package prefix so names stay short in the log
    short = name.replace("Rob_Prac.", "").replace("rob_prac.", "")
    return logging.getLogger(f"robosim.{short}")


def set_console_level(level: int) -> None:
    """Raise or lower the console handler's level at runtime."""
    root = logging.getLogger("robosim")
    for h in root.handlers:
        if isinstance(h, logging.StreamHandler) and not isinstance(
            h, logging.handlers.RotatingFileHandler
        ):
            h.setLevel(level)


def set_file_level(level: int) -> None:
    """Raise or lower the file handler's level at runtime."""
    root = logging.getLogger("robosim")
    for h in root.handlers:
        if isinstance(h, logging.handlers.RotatingFileHandler):
            h.setLevel(level)
