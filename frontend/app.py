#!/usr/bin/env python3
"""
AppName - Real-Time Robotic Arm Simulation
Main application entry point.
"""

import sys
from PyQt6.QtWidgets import QApplication
from backend.logger import get_logger, set_console_level
import logging

# Initialise logger before any other import that might use it
log = get_logger(__name__)


def main():
    """Create and run the Qt application."""
    log.info("RoboSim starting up")
    # Uncomment the line below to also see DEBUG messages in the terminal:
    # set_console_level(logging.DEBUG)

    from .main_window import MainWindow
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    log.info("MainWindow shown — entering event loop")
    exit_code = app.exec()
    log.info("Application exited with code %d", exit_code)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
