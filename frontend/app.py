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
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Print INFO-level logs to the console (default: WARNING+).",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print DEBUG-level logs to the console.",
    )
    return parser


def _install_qt_message_logger() -> None:
    """Forward Qt critical/fatal messages into ``robosim.qt``."""
    try:
        from PyQt6.QtCore import QtMsgType, qInstallMessageHandler
    except ImportError:
        return

    log_qt = get_logger("qt")

    def _handler(mode, context, message: str) -> None:
        fn = getattr(context, "file", None) or "?"
        ln = getattr(context, "line", None) or 0
        text = f"{message} ({fn}:{ln})"
        if mode == QtMsgType.QtFatalMsg:
            log_qt.critical(text)
        elif mode == QtMsgType.QtCriticalMsg:
            log_qt.error(text)
        elif mode == QtMsgType.QtWarningMsg:
            log_qt.warning(text)
        else:
            log_qt.debug(text)

    qInstallMessageHandler(_handler)


def main():
    """Create and run the Qt application."""
    parser = _build_arg_parser()
    args, qt_argv = parser.parse_known_args()

    log = get_logger(__name__)
    if args.debug:
        set_console_level(logging.DEBUG)
    elif args.verbose:
        set_console_level(logging.INFO)

    log.info(
        "RoboSim starting up | replay=%s | record=%s | no_filter=%s",
        args.replay,
        args.record,
        args.no_filter,
    )
    if qt_argv:
        log.debug("Argv not consumed by argparse (not passed to Qt): %s", qt_argv)

    from .main_window import MainWindow

    # Pass only the program path to Qt. IDE/debuggers often inject ``-X``, ``-m``, etc.
    # into ``sys.argv``; Qt treats unknown flags as errors and can quit instantly.
    qapp_argv = [sys.argv[0]]
    log.debug("QApplication argv: %s", qapp_argv)
    app = QApplication(qapp_argv)
    _install_qt_message_logger()

    exit_code = 1
    try:
        window = MainWindow(
            replay_path=args.replay,
            record_path=args.record,
            use_filter=not args.no_filter,
        )
        window.show()
        log.info("MainWindow shown — entering event loop")
        exit_code = app.exec()
        log.info("Application exited with code %d", exit_code)
    except BaseException:
        log.exception("Fatal error during startup or event loop")
        raise
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
