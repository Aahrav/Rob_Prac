#!/usr/bin/env python3
"""
AppName - Real-Time Robotic Arm Simulation
Main application entry point.
"""

import sys
from PyQt6.QtWidgets import QApplication
from .main_window import MainWindow


def main():
    """Create and run the Qt application."""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
