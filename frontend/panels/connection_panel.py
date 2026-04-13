#!/usr/bin/env python3
"""
ConnectionPanel - Mode selector and connection controls.
Kinetic Obsidian theme: no QGroupBox border, pill-style toggles, gradient connect button.
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                              QPushButton, QFrame)
from PyQt6.QtCore import pyqtSignal, Qt
from backend.config import DEFAULT_BAUD_RATE
from backend.serial_test import list_available_ports
from frontend.panels.custom_combo_box import CustomComboBox


# ── Shared style tokens ─────────────────────────────────────────────────────
COMBO_STYLE = """
    QComboBox {
        background-color: #0e0e0e;
        color: #e5e2e1;
        padding: 6px 10px;
        border: none;
        border-radius: 4px;
        font-size: 11px;
    }
    QComboBox::drop-down {
        border: none;
        width: 24px;
    }
    QComboBox::down-arrow {
        image: none;
        border-left: 1px solid #353535;
        width: 20px;
        height: 20px;
    }
    QComboBox QAbstractItemView {
        background-color: #202020;
        color: #e5e2e1;
        selection-background-color: #3498db;
        border: 1px solid #353535;
        border-radius: 4px;
        padding: 4px;
    }
"""

PILL_ACTIVE = """
    QPushButton {
        background-color: #3498db;
        color: #ffffff;
        border: none;
        border-radius: 4px;
        padding: 6px 14px;
        font-size: 11px;
        font-weight: 600;
    }
"""

PILL_INACTIVE = """
    QPushButton {
        background-color: #202020;
        color: #89929b;
        border: none;
        border-radius: 4px;
        padding: 6px 14px;
        font-size: 11px;
        font-weight: 500;
    }
    QPushButton:hover {
        background-color: #2a2a2a;
        color: #bfc7d2;
    }
"""

CONNECT_STYLE = """
    QPushButton {
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
            stop:0 #3498db, stop:1 #2980b9);
        color: #ffffff;
        font-weight: 700;
        font-size: 12px;
        padding: 10px;
        border: none;
        border-radius: 6px;
        min-height: 36px;
    }
    QPushButton:hover {
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
            stop:0 #2980b9, stop:1 #1f6aa5);
    }
    QPushButton:pressed {
        background-color: #1f6aa5;
    }
    QPushButton:disabled {
        background-color: #353535;
        color: #89929b;
    }
"""

DISCONNECT_STYLE = """
    QPushButton {
        background-color: #3a3a3c;
        color: #e5e2e1;
        font-weight: 700;
        font-size: 12px;
        padding: 10px;
        border: none;
        border-radius: 6px;
        min-height: 36px;
    }
    QPushButton:hover {
        background-color: #454548;
    }
"""

SECTION_LABEL = "color: #89929b; font-size: 10px; font-weight: 600; letter-spacing: 0.06em; text-transform: uppercase;"


class ConnectionPanel(QWidget):
    """Panel for connection/control mode configuration."""

    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()
    status_changed = pyqtSignal(str)
    mode_changed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_connected = False
        self._current_mode = "Simulation"  # "Simulation" or "Interactive"

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # ── Interface mode pills ────────────────────────────────────────────
        lbl_mode = QLabel("INTERFACE MODE")
        lbl_mode.setStyleSheet(SECTION_LABEL)
        layout.addWidget(lbl_mode)

        pill_row = QHBoxLayout()
        pill_row.setSpacing(4)

        self.btn_simulation = QPushButton("Simulation")
        self.btn_simulation.setCheckable(False)
        self.btn_simulation.setStyleSheet(PILL_ACTIVE)
        self.btn_simulation.clicked.connect(lambda: self._switch_mode("Simulation"))
        pill_row.addWidget(self.btn_simulation)

        self.btn_interactive = QPushButton("Interactive")
        self.btn_interactive.setCheckable(False)
        self.btn_interactive.setStyleSheet(PILL_INACTIVE)
        self.btn_interactive.clicked.connect(lambda: self._switch_mode("Interactive"))
        pill_row.addWidget(self.btn_interactive)

        layout.addLayout(pill_row)

        # ── Port selector (Simulation only) ────────────────────────────────
        self.port_label = QLabel("PORT")
        self.port_label.setStyleSheet(SECTION_LABEL)
        layout.addWidget(self.port_label)

        self.port_combo = CustomComboBox()
        self.port_combo.setStyleSheet(COMBO_STYLE)
        self.refresh_ports()
        layout.addWidget(self.port_combo)

        # ── Connect button ──────────────────────────────────────────────────
        self.connect_btn = QPushButton("● Connect")
        self.connect_btn.setStyleSheet(CONNECT_STYLE)
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self.connect_btn)

        # ── Status chip ─────────────────────────────────────────────────────
        self.status_label = QLabel("⬤  Disconnected")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet(
            "color: #e74c3c; font-size: 11px; font-weight: 600; padding: 4px 0;"
        )
        layout.addWidget(self.status_label)

        # Internal: keep mode_combo attribute for compatibility with main_window
        # (main_window accesses self.connection_panel.mode_combo.currentText())
        self.mode_combo = _CompatModeProxy(self)

    # ── Public helpers ───────────────────────────────────────────────────────

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list_available_ports()
        if ports:
            self.port_combo.addItems(ports)
        self.port_combo.addItem("SIMULATED (no hardware)")

    def set_connected(self, connected: bool):
        self.is_connected = connected
        if connected:
            self.connect_btn.setText("■  Disconnect")
            self.connect_btn.setStyleSheet(DISCONNECT_STYLE)
            self.btn_simulation.setEnabled(False)
            self.btn_interactive.setEnabled(False)
        else:
            self.connect_btn.setText("● Connect")
            self.connect_btn.setStyleSheet(CONNECT_STYLE)
            self.btn_simulation.setEnabled(True)
            self.btn_interactive.setEnabled(True)

    def set_status(self, message: str):
        msg_lower = message.lower()
        if "error" in msg_lower or "failed" in msg_lower or "disconnect" in msg_lower:
            color = "#e74c3c"
            dot = "⬤"
        elif "connect" in msg_lower or "running" in msg_lower or "active" in msg_lower:
            color = "#2ecc71"
            dot = "⬤"
        else:
            color = "#f39c12"
            dot = "⬤"
        self.status_label.setText(f"{dot}  {message}")
        self.status_label.setStyleSheet(
            f"color: {color}; font-size: 11px; font-weight: 600; padding: 4px 0;"
        )

    # ── Private slots ────────────────────────────────────────────────────────

    def _switch_mode(self, mode: str):
        if self.is_connected:
            return
        self._current_mode = mode
        if mode == "Simulation":
            self.btn_simulation.setStyleSheet(PILL_ACTIVE)
            self.btn_interactive.setStyleSheet(PILL_INACTIVE)
            self.port_label.setVisible(True)
            self.port_combo.setVisible(True)
        else:
            self.btn_simulation.setStyleSheet(PILL_INACTIVE)
            self.btn_interactive.setStyleSheet(PILL_ACTIVE)
            self.port_label.setVisible(False)
            self.port_combo.setVisible(False)
        self.btn_simulation.clearFocus()
        self.btn_interactive.clearFocus()
        self.mode_changed.emit(mode.lower())

    def _on_connect_clicked(self):
        if not self.is_connected:
            if self._current_mode == "Interactive":
                self.connect_requested.emit("INTERACTIVE", 0)
            else:
                port = self.port_combo.currentText()
                if port and port != "No ports found":
                    self.connect_requested.emit(port, DEFAULT_BAUD_RATE)
        else:
            self.disconnect_requested.emit()


class _CompatModeProxy:
    """Compatibility shim so main_window.py can still call
    connection_panel.mode_combo.currentText() without changes."""

    def __init__(self, panel: ConnectionPanel):
        self._panel = panel

    def currentText(self):
        return self._panel._current_mode

    def clearFocus(self):
        pass
