#!/usr/bin/env python3
"""
ConnectionPanel — Mode selector and connection controls.
Kinetic Obsidian theme.

P3-T1: Three data-source modes (visuals only — Part 2 wires workers):
    • Simulate        — built-in SimWorker, no hardware required.
    • Replay file…    — user picks a CSV; path is emitted via connect_requested.
    • USB Serial      — disabled pill, coming soon when hardware ships.

Signals:
    connect_requested(port: str, baud: int)
        Simulate → ``SIMULATED (no hardware)``
        Replay   → ``REPLAY:<absolute CSV path>``, baud=0
    disconnect_requested()
    mode_changed(mode: str)  — "simulate" | "replay" | "serial"
    status_changed(str)
"""

import os
import serial.tools.list_ports
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFrame, QFileDialog, QSizePolicy, QComboBox
)
from PyQt6.QtCore import pyqtSignal, Qt, QTimer
from backend.config import DEFAULT_BAUD_RATE


# ── Style tokens ─────────────────────────────────────────────────────────────

PILL_ACTIVE = """
    QPushButton {
        background-color: #3498db;
        color: #ffffff;
        border: none;
        border-radius: 4px;
        padding: 6px 10px;
        font-size: 10px;
        font-weight: 600;
    }
"""

PILL_INACTIVE = """
    QPushButton {
        background-color: #202020;
        color: #89929b;
        border: none;
        border-radius: 4px;
        padding: 6px 10px;
        font-size: 10px;
        font-weight: 500;
    }
    QPushButton:hover { background-color: #2a2a2a; color: #bfc7d2; }
"""

CONNECT_STYLE = """
    QPushButton {
        background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
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
        background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
            stop:0 #2980b9, stop:1 #1f6aa5);
    }
    QPushButton:pressed { background-color: #1f6aa5; }
    QPushButton:disabled { background-color: #353535; color: #89929b; }
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
    QPushButton:hover { background-color: #454548; }
"""

BROWSE_STYLE = """
    QPushButton {
        background-color: #202020;
        color: #bfc7d2;
        border: 1px solid #353535;
        border-radius: 4px;
        padding: 4px 10px;
        font-size: 10px;
        font-weight: 500;
        min-height: 24px;
    }
    QPushButton:hover { background-color: #2a2a2a; color: #e5e2e1; border-color: #454548; }
"""

COMBO_STYLE = """
    QComboBox {
        background-color: #0e0e0e;
        color: #bfc7d2;
        border: 1px solid #353535;
        border-radius: 4px;
        padding: 2px 8px;
        font-size: 10px;
        font-family: 'Consolas', monospace;
        min-height: 24px;
    }
    QComboBox::drop-down { border: none; }
    QComboBox::down-arrow { image: none; border-left: 4px solid transparent; border-right: 4px solid transparent; border-top: 4px solid #89929b; width: 0; height: 0; margin-right: 8px; }
    QComboBox QAbstractItemView { background-color: #202020; color: #bfc7d2; selection-background-color: #3498db; outline: none; border: 1px solid #353535; }
"""

FILE_LABEL_STYLE = (
    "color: #89929b; font-size: 10px; font-family: 'Consolas', monospace;"
)

SECTION_LABEL = (
    "color: #89929b; font-size: 10px; font-weight: 600;"
    "letter-spacing: 0.06em; text-transform: uppercase;"
)


# ─────────────────────────────────────────────────────────────────────────────

class ConnectionPanel(QWidget):
    """Panel for data-source mode selection and connection control."""

    connect_requested    = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()
    status_changed       = pyqtSignal(str)
    mode_changed         = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_connected   = False
        self._current_mode  = "Simulate"   # "Simulate" | "Replay" | "Serial"
        self._replay_path   = ""           # absolute path selected by user

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # ── Source mode pills ─────────────────────────────────────────────
        lbl_mode = QLabel("DATA SOURCE")
        lbl_mode.setStyleSheet(SECTION_LABEL)
        layout.addWidget(lbl_mode)

        pill_row = QHBoxLayout()
        pill_row.setSpacing(4)

        self.btn_simulate = QPushButton("Simulate")
        self.btn_simulate.setStyleSheet(PILL_ACTIVE)
        self.btn_simulate.clicked.connect(lambda: self._switch_mode("Simulate"))
        pill_row.addWidget(self.btn_simulate)

        self.btn_replay = QPushButton("Replay file…")
        self.btn_replay.setStyleSheet(PILL_INACTIVE)
        self.btn_replay.clicked.connect(lambda: self._switch_mode("Replay"))
        pill_row.addWidget(self.btn_replay)

        self.btn_serial = QPushButton("USB Serial")
        self.btn_serial.setStyleSheet(PILL_INACTIVE)
        self.btn_serial.clicked.connect(lambda: self._switch_mode("Serial"))
        pill_row.addWidget(self.btn_serial)

        layout.addLayout(pill_row)

        # ── Replay file row (hidden unless Replay mode active) ────────────
        self._replay_row = QFrame()
        self._replay_row.setStyleSheet(
            "QFrame { background-color: #0e0e0e; border-radius: 4px; }"
        )
        replay_inner = QVBoxLayout(self._replay_row)
        replay_inner.setContentsMargins(8, 6, 8, 6)
        replay_inner.setSpacing(4)

        lbl_file = QLabel("REPLAY FILE")
        lbl_file.setStyleSheet(SECTION_LABEL)
        replay_inner.addWidget(lbl_file)

        file_row = QHBoxLayout()
        file_row.setSpacing(6)

        self._file_label = QLabel("No file selected")
        self._file_label.setStyleSheet(FILE_LABEL_STYLE)
        self._file_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred
        )
        self._file_label.setWordWrap(False)
        file_row.addWidget(self._file_label, 1)

        browse_btn = QPushButton("Browse…")
        browse_btn.setStyleSheet(BROWSE_STYLE)
        browse_btn.clicked.connect(self._browse_replay_file)
        file_row.addWidget(browse_btn)

        replay_inner.addLayout(file_row)
        self._replay_row.setVisible(False)
        layout.addWidget(self._replay_row)

        # ── Serial Port row (hidden unless Serial mode active) ────────────
        self._serial_row = QFrame()
        self._serial_row.setStyleSheet(
            "QFrame { background-color: #0e0e0e; border-radius: 4px; }"
        )
        serial_inner = QVBoxLayout(self._serial_row)
        serial_inner.setContentsMargins(8, 6, 8, 6)
        serial_inner.setSpacing(4)

        lbl_port = QLabel("USB PORT")
        lbl_port.setStyleSheet(SECTION_LABEL)
        serial_inner.addWidget(lbl_port)

        port_row = QHBoxLayout()
        port_row.setSpacing(6)

        self.port_combo = QComboBox()
        self.port_combo.setStyleSheet(COMBO_STYLE)
        self.port_combo.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        port_row.addWidget(self.port_combo, 1)

        refresh_btn = QPushButton("Refresh")
        refresh_btn.setStyleSheet(BROWSE_STYLE)
        refresh_btn.clicked.connect(self.refresh_ports)
        port_row.addWidget(refresh_btn)

        serial_inner.addLayout(port_row)
        self._serial_row.setVisible(False)
        layout.addWidget(self._serial_row)

        # ── Connect button ────────────────────────────────────────────────
        self.connect_btn = QPushButton("● Connect")
        self.connect_btn.setStyleSheet(CONNECT_STYLE)
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self.connect_btn)

        # ── Status chip ───────────────────────────────────────────────────
        self.status_label = QLabel("⬤  Disconnected")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet(
            "color: #e74c3c; font-size: 11px; font-weight: 600; padding: 4px 0;"
        )
        layout.addWidget(self.status_label)

        # Compat shim — main_window reads .mode_combo.currentText()
        self.mode_combo = _CompatModeProxy(self)

        # Initial port scan
        self.refresh_ports()

    # ── Public helpers ────────────────────────────────────────────────────────

    def set_connected(self, connected: bool):
        self.is_connected = connected
        if connected:
            self.connect_btn.setText("■  Disconnect")
            self.connect_btn.setStyleSheet(DISCONNECT_STYLE)
            self.btn_simulate.setEnabled(False)
            self.btn_replay.setEnabled(False)
            self.btn_serial.setEnabled(False)
        else:
            self.connect_btn.setText("● Connect")
            self.connect_btn.setStyleSheet(CONNECT_STYLE)
            self.btn_simulate.setEnabled(True)
            self.btn_replay.setEnabled(True)
            self.btn_serial.setEnabled(True)

    def set_status(self, message: str):
        msg_lower = message.lower()
        if "error" in msg_lower or "failed" in msg_lower or "disconnect" in msg_lower:
            color, dot = "#e74c3c", "⬤"
        elif "connect" in msg_lower or "running" in msg_lower or "active" in msg_lower:
            color, dot = "#2ecc71", "⬤"
        else:
            color, dot = "#f39c12", "⬤"
        self.status_label.setText(f"{dot}  {message}")
        self.status_label.setStyleSheet(
            f"color: {color}; font-size: 11px; font-weight: 600; padding: 4px 0;"
        )

    def refresh_ports(self):
        """Scan for available serial ports."""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.port_combo.addItem("No ports found")
            self.port_combo.setEnabled(False)
        else:
            self.port_combo.setEnabled(True)
            for p in ports:
                # Include friendly name if available
                desc = f"{p.device} ({p.description})" if p.description != p.device else p.device
                self.port_combo.addItem(desc, p.device)

    # ── Private slots ─────────────────────────────────────────────────────────

    def _switch_mode(self, mode: str):
        if self.is_connected:
            return
        self._current_mode = mode

        # Update pill styles
        self.btn_simulate.setStyleSheet(
            PILL_ACTIVE if mode == "Simulate" else PILL_INACTIVE
        )
        self.btn_replay.setStyleSheet(
            PILL_ACTIVE if mode == "Replay" else PILL_INACTIVE
        )
        self.btn_serial.setStyleSheet(
            PILL_ACTIVE if mode == "Serial" else PILL_INACTIVE
        )

        # Show/hide relevant options
        self._replay_row.setVisible(mode == "Replay")
        self._serial_row.setVisible(mode == "Serial")

        self.btn_simulate.clearFocus()
        self.btn_replay.clearFocus()
        self.btn_serial.clearFocus()
        self.mode_changed.emit(mode.lower())

    def _browse_replay_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select Replay CSV",
            os.path.expanduser("~"),
            "CSV files (*.csv);;All files (*)"
        )
        if path:
            self._replay_path = path
            truncated = os.path.basename(path)
            self._file_label.setText(truncated)
            self._file_label.setToolTip(path)
            self._file_label.setStyleSheet(FILE_LABEL_STYLE.replace("#89929b", "#bfc7d2"))

    def _on_connect_clicked(self):
        if self.is_connected:
            self.disconnect_requested.emit()
            return

        if self._current_mode == "Simulate":
            self.connect_requested.emit("SIMULATED (no hardware)", DEFAULT_BAUD_RATE)

        elif self._current_mode == "Replay":
            if self._replay_path:
                self.connect_requested.emit(f"REPLAY:{self._replay_path}", 0)
            else:
                self.set_status("No replay file selected — click Browse…")
        
        elif self._current_mode == "Serial":
            port = self.port_combo.currentData()
            if port and port != "No ports found":
                self.connect_requested.emit(port, DEFAULT_BAUD_RATE)
            else:
                self.set_status("No valid serial port selected")


# ── Compat shim ───────────────────────────────────────────────────────────────

class _CompatModeProxy:
    """Allows main_window.py to call connection_panel.mode_combo.currentText()
    without any changes to its wiring code."""

    def __init__(self, panel: ConnectionPanel):
        self._panel = panel

    def currentText(self) -> str:
        return self._panel._current_mode

    def clearFocus(self):
        pass


# ── Compat shim ───────────────────────────────────────────────────────────────

class _CompatModeProxy:
    """Allows main_window.py to call connection_panel.mode_combo.currentText()
    without any changes to its wiring code."""

    def __init__(self, panel: ConnectionPanel):
        self._panel = panel

    def currentText(self) -> str:
        return self._panel._current_mode

    def clearFocus(self):
        pass
