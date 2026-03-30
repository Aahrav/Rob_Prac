#!/usr/bin/env python3
"""
ConnectionPanel - UI for selecting mode and connecting.
"""

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QLabel, QComboBox, QPushButton
from PyQt6.QtCore import pyqtSignal, Qt
from backend.config import DEFAULT_BAUD_RATE
from backend.serial_test import list_available_ports


class ConnectionPanel(QGroupBox):
    """Panel for connection/control mode configuration."""

    connect_requested = pyqtSignal(str, int)  # port, baud
    disconnect_requested = pyqtSignal()
    status_changed = pyqtSignal(str)  # status message
    mode_changed = pyqtSignal(str)  # 'simulated' or 'interactive'

    def __init__(self, parent=None):
        super().__init__("Connection", parent)
        self.is_connected = False

        layout = QVBoxLayout(self)

        # Mode selector
        layout.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Simulation", "Interactive"])
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        layout.addWidget(self.mode_combo)

        # Port selector (only for Simulation)
        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        layout.addWidget(self.port_combo)

        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self.connect_btn)

        # Status label
        self.status_label = QLabel("Disconnected")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold; padding: 5px;")
        layout.addWidget(self.status_label)

        self.setStyleSheet("""
            QGroupBox {
                border: 1px solid #444;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

        # Initially hide port selector (show only in Simulation mode)
        self.port_combo.setVisible(True)  # visible for now

    def refresh_ports(self):
        """Populate the port combo with available serial ports plus simulation option."""
        self.port_combo.clear()
        ports = list_available_ports()
        if ports:
            self.port_combo.addItems(ports)
        # Always add simulation option
        self.port_combo.addItem("SIMULATED (no hardware)")

    def _on_mode_changed(self, mode):
        if mode == "Interactive":
            self.port_combo.setVisible(False)
        else:
            self.port_combo.setVisible(True)
        self.mode_changed.emit(mode.lower())

    def _on_connect_clicked(self):
        if not self.is_connected:
            mode = self.mode_combo.currentText()
            if mode == "Interactive":
                self.connect_requested.emit("INTERACTIVE", 0)
            else:
                port = self.port_combo.currentText()
                if port and port != "No ports found":
                    self.connect_requested.emit(port, DEFAULT_BAUD_RATE)
        else:
            self.disconnect_requested.emit()

    def set_connected(self, connected: bool):
        self.is_connected = connected
        if connected:
            self.connect_btn.setText("Disconnect")
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: #2ecc71; font-weight: bold; padding: 5px;")
            self.mode_combo.setEnabled(False)
        else:
            self.connect_btn.setText("Connect")
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold; padding: 5px;")
            self.mode_combo.setEnabled(True)

    def set_status(self, message: str):
        self.status_label.setText(message)
        if "error" in message.lower() or "failed" in message.lower():
            self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold; padding: 5px;")
        else:
            self.status_label.setStyleSheet("color: #f39c12; font-weight: bold; padding: 5px;")
