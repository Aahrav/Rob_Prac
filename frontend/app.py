#!/usr/bin/env python3
"""
AppName - Real-Time Robotic Arm Simulation
Main application entry point.

CLI flags (Part 2, P2-T4):

    --replay PATH
        CSV file to replay. The replay starts automatically after the main
        window is shown (no Connect button required). Use recordings made with
        --record or fixtures under recordings/fixtures/.

    --record PATH
        File path where raw sensor dicts will be recorded in CSV format
        (contract header: t,r,p,y,gx,gy,gz). Recording captures data BEFORE
        calibration and filtering to preserve full replay fidelity.

    --no-filter
        Bypass the ComplementaryFilter entirely. Raw r,p,y values from the
        producer are passed directly to the display pipeline. Useful for
        debugging noisy sensor data or testing the filter in isolation.

Examples:

    python run.py
    python run.py --replay recordings/fixtures/synthetic_smooth.csv
    python run.py --record my_session.csv
    python run.py --replay foo.csv --record bar.csv
    python run.py --replay foo.csv --no-filter
"""

import sys
import argparse
import logging
from PyQt6.QtWidgets import QApplication

from backend.logger import get_logger, set_console_level

# Initialise logger before any other import that might use it
log = get_logger(__name__)


def _build_arg_parser() -> argparse.ArgumentParser:
    """Return the argument parser for the application CLI."""
    parser = argparse.ArgumentParser(
        prog="robosim",
        description="RoboSim — Real-Time Robotic Arm Simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python run.py\n"
            "  python run.py --replay recordings/fixtures/synthetic_smooth.csv\n"
            "  python run.py --record my_session.csv\n"
            "  python run.py --replay foo.csv --record bar.csv\n"
            "  python run.py --no-filter\n"
        ),
    )
    parser.add_argument(
        "--replay",
        metavar="PATH",
        default=None,
        help=(
            "CSV file to replay. Auto-starts after the window is shown. "
            "Header must contain: t,r,p,y (gx,gy,gz optional)."
        ),
    )
    parser.add_argument(
        "--record",
        metavar="PATH",
        default=None,
        help=(
            "Record raw sensor dicts to a CSV file. Data is captured before "
            "calibration and filtering for full replay fidelity."
        ),
    )
    parser.add_argument(
        "--no-filter",
        action="store_true",
        dest="no_filter",
        default=False,
        help=(
            "Bypass the ComplementaryFilter. Raw r,p,y values are used "
            "directly without gyro-fusion. Useful for debugging."
        ),
    )
    return parser


def main():
    """Create and run the Qt application."""
    # ── Parse CLI args (strip Qt's own args first) ───────────────────────────
    # We parse known args so that PyQt6 / platform args are not rejected.
    parser = _build_arg_parser()
    args, _qt_args = parser.parse_known_args()

    log.info(
        "RoboSim starting up | replay=%s | record=%s | no_filter=%s",
        args.replay,
        args.record,
        args.no_filter,
    )

    # Uncomment to see DEBUG messages in the terminal:
    # set_console_level(logging.DEBUG)

    from .main_window import MainWindow

    app = QApplication(sys.argv)
    window = MainWindow(
        replay_path=args.replay,
        record_path=args.record,
        use_filter=not args.no_filter,
    )
    window.show()
    log.info("MainWindow shown — entering event loop")
    exit_code = app.exec()
    log.info("Application exited with code %d", exit_code)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
