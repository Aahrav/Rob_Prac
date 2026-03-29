#!/usr/bin/env python3
"""
ConnectionPanel - UI for selecting port and connecting.
"""

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QLabel, QComboBox, QPushButton
from PyQt6.QtCore import pyqtSignal, Qt
from backend.config import DEFAULT_BAUD_RATE
from backend.serial_test import list_available_ports


class ConnectionPanel(QGroupBox):
    """Panel for serial connection configuration."""

    connect_requested = pyqtSignal(str, int)  # port, baud
    disconnect_requested = pyqtSignal()
    status_changed = pyqtSignal(str)  # status message

    def __init__(self, parent=None):
        super().__init__("Connection", parent)
        self.is_connected = False

        layout = QVBoxLayout(self)

        # Port selector
        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        layout.addWidget(self.port_combo)

        # Refresh button? Maybe not needed.

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

    def refresh_ports(self):
        """Populate the port combo with available serial ports plus simulation option."""
        self.port_combo.clear()
        ports = list_available_ports()
        if ports:
            self.port_combo.addItems(ports)
        # Always add simulation option
        self.port_combo.addItem("SIMULATED (no hardware)")
        self.port_combo.setEnabled(True)

    def _on_connect_clicked(self):
        if not self.is_connected:
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
            self.port_combo.setEnabled(False)
        else:
            self.connect_btn.setText("Connect")
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold; padding: 5px;")
            self.port_combo.setEnabled(True)

    def set_status(self, message: str):
        self.status_label.setText(message)
        # Optionally color based on content
        if "error" in message.lower() or "failed" in message.lower():
            self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold; padding: 5px;")
        else:
            self.status_label.setStyleSheet("color: #f39c12; font-weight: bold; padding: 5px;")
