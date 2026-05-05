#!/usr/bin/env python3
"""
replay_bar.py — Generic media-player control bar for animation replay.

Design principles:
  - Knows ONLY about ReplayController and ReplayBuffer.
  - No robot model, canvas, or kinematic imports.
  - Drop into any QLayout with: layout.addWidget(ReplayControlBar(controller))
  - All button actions proxy directly to controller methods.
"""

from __future__ import annotations

import shutil
from typing import Optional

from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton,
    QSlider, QComboBox, QFrame, QSizePolicy, QToolButton, QMenu,
)
from PyQt6.QtCore import Qt, pyqtSignal, QSize
from PyQt6.QtGui import QAction

from backend.replay_buffer import ReplayBuffer
from backend.replay_controller import ReplayController, ReplayState, ReplayMode


# ──────────────────────────────────────────────────────────────────────────────
#  Style constants
# ──────────────────────────────────────────────────────────────────────────────

_BAR_BG = "#0c0c0c"
_BTN_BASE = """
    QPushButton {
        background: #1e1e1e;
        color: #c8d0da;
        border: 1px solid #2a2a2a;
        border-radius: 4px;
        padding: 2px 8px;
        font-size: 14px;
        min-width: 28px;
        min-height: 26px;
    }
    QPushButton:hover  { background: #2a2a2a; color: #fff; border-color: #3a3a3a; }
    QPushButton:pressed { background: #111; }
    QPushButton:disabled { color: #444; border-color: #1a1a1a; }
"""
_BTN_ACTIVE = _BTN_BASE + """
    QPushButton { background: #c0392b; color: #fff; border-color: #e74c3c; }
    QPushButton:hover { background: #e74c3c; }
"""
_BTN_PLAY_ACTIVE = _BTN_BASE + """
    QPushButton { background: #27ae60; color: #fff; border-color: #2ecc71; }
    QPushButton:hover { background: #2ecc71; }
"""
_COMBO_STYLE = """
    QComboBox {
        background: #1a1a1a; color: #c8d0da;
        border: 1px solid #2a2a2a; border-radius: 4px;
        padding: 2px 6px; font-size: 10px; min-height: 26px;
    }
    QComboBox::drop-down { border: none; width: 14px; }
    QComboBox QAbstractItemView {
        background: #1a1a1a; color: #c8d0da;
        border: 1px solid #353535; selection-background-color: #3498db;
    }
"""
_SCRUBBER_STYLE = """
    QSlider::groove:horizontal {
        height: 4px; background: #222; border-radius: 2px;
    }
    QSlider::handle:horizontal {
        width: 12px; height: 12px; margin: -4px 0;
        background: #3498db; border-radius: 6px;
    }
    QSlider::handle:horizontal:hover { background: #5dade2; }
    QSlider::sub-page:horizontal { background: #3498db; border-radius: 2px; }
"""
_LABEL_STYLE = "color: #89929b; font-size: 10px; font-family: 'Consolas', monospace;"
_SEP_STYLE   = "background: #2a2a2a; max-width: 1px; min-width: 1px;"


def _vline() -> QFrame:
    sep = QFrame()
    sep.setFrameShape(QFrame.Shape.VLine)
    sep.setStyleSheet(_SEP_STYLE)
    return sep


# ──────────────────────────────────────────────────────────────────────────────
#  ReplayControlBar
# ──────────────────────────────────────────────────────────────────────────────

class ReplayControlBar(QWidget):
    """
    Horizontal media-player-style control bar.

    Signals
    -------
    export_requested()     — user clicked 💾; parent should show ExportDialog
    record_toggled(bool)   — True = start recording, False = stop
    """

    export_requested = pyqtSignal()
    record_toggled   = pyqtSignal(bool)

    def __init__(self, controller: ReplayController, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._ctrl = controller
        self._scrubbing = False     # suppress frame_changed while dragging
        self._build_ui()
        self._connect_signals()
        self._update_all(ReplayState.IDLE.name)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.setFixedHeight(52)
        self.setStyleSheet(f"background-color: {_BAR_BG};")

        root = QHBoxLayout(self)
        root.setContentsMargins(10, 0, 10, 0)
        root.setSpacing(4)

        # ── Record button ──────────────────────────────────────────
        self.btn_record = QPushButton("⏺")
        self.btn_record.setToolTip("Start / Stop recording")
        self.btn_record.setStyleSheet(_BTN_BASE)
        self.btn_record.setCheckable(True)
        root.addWidget(self.btn_record)

        root.addWidget(_vline())

        # ── Transport controls ─────────────────────────────────────
        self.btn_stop       = QPushButton("⏹")
        self.btn_step_back  = QPushButton("⏮")
        self.btn_play_pause = QPushButton("▶")
        self.btn_step_fwd   = QPushButton("⏭")

        for btn, tip in [
            (self.btn_stop,       "Stop & reset to frame 0"),
            (self.btn_step_back,  "Step backward one frame"),
            (self.btn_play_pause, "Play / Pause  [Space]"),
            (self.btn_step_fwd,   "Step forward one frame"),
        ]:
            btn.setToolTip(tip)
            btn.setStyleSheet(_BTN_BASE)
            root.addWidget(btn)

        root.addWidget(_vline())

        # ── Mode combo ─────────────────────────────────────────────
        lbl_mode = QLabel("Mode")
        lbl_mode.setStyleSheet(_LABEL_STYLE)
        root.addWidget(lbl_mode)

        self.combo_mode = QComboBox()
        self.combo_mode.setStyleSheet(_COMBO_STYLE)
        self.combo_mode.setToolTip("Playback mode")
        self.combo_mode.setFixedWidth(90)
        for label in ["Single ▶", "Loop 🔁", "Ping-Pong ↔", "Reverse ◀"]:
            self.combo_mode.addItem(label)
        root.addWidget(self.combo_mode)

        # ── Speed combo ────────────────────────────────────────────
        lbl_speed = QLabel("Speed")
        lbl_speed.setStyleSheet(_LABEL_STYLE)
        root.addWidget(lbl_speed)

        self.combo_speed = QComboBox()
        self.combo_speed.setStyleSheet(_COMBO_STYLE)
        self.combo_speed.setToolTip("Playback speed")
        self.combo_speed.setFixedWidth(68)
        for label in ["0.25×", "0.5×", "1×", "2×", "4×"]:
            self.combo_speed.addItem(label)
        self.combo_speed.setCurrentIndex(2)  # default 1×
        root.addWidget(self.combo_speed)

        root.addWidget(_vline())

        # ── Timeline scrubber (stretchy) ───────────────────────────
        self.scrubber = QSlider(Qt.Orientation.Horizontal)
        self.scrubber.setMinimum(0)
        self.scrubber.setMaximum(0)
        self.scrubber.setValue(0)
        self.scrubber.setStyleSheet(_SCRUBBER_STYLE)
        self.scrubber.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.scrubber.setToolTip("Drag to seek")
        root.addWidget(self.scrubber, stretch=1)

        root.addWidget(_vline())

        # ── Frame counter + timestamp ──────────────────────────────
        counter_col = QVBoxLayout()
        counter_col.setSpacing(0)
        counter_col.setContentsMargins(0, 0, 0, 0)

        self.lbl_frames = QLabel("0 / 0")
        self.lbl_frames.setStyleSheet(_LABEL_STYLE)
        self.lbl_frames.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.lbl_time = QLabel("0.0s / 0.0s")
        self.lbl_time.setStyleSheet(_LABEL_STYLE)
        self.lbl_time.setAlignment(Qt.AlignmentFlag.AlignCenter)

        counter_col.addWidget(self.lbl_frames)
        counter_col.addWidget(self.lbl_time)

        root.addLayout(counter_col)

        root.addWidget(_vline())

        # ── Export button ──────────────────────────────────────────
        self.btn_export = QPushButton("💾")
        self.btn_export.setToolTip("Export (CSV / PNG / GIF / MP4)")
        self.btn_export.setStyleSheet(_BTN_BASE)
        root.addWidget(self.btn_export)

    # ── Signal wiring ─────────────────────────────────────────────────────────

    def _connect_signals(self) -> None:
        # Controller → UI
        self._ctrl.frame_changed.connect(self._on_frame_changed)
        self._ctrl.state_changed.connect(self._update_all)
        self._ctrl.buffer_changed.connect(self._on_buffer_changed)
        self._ctrl.recording_state_changed.connect(self._on_recording_changed)

        # UI → Controller
        self.btn_play_pause.clicked.connect(self._ctrl.toggle_play_pause)
        self.btn_stop.clicked.connect(self._ctrl.stop)
        self.btn_step_back.clicked.connect(lambda: self._ctrl.step(-1))
        self.btn_step_fwd.clicked.connect(lambda: self._ctrl.step(+1))

        # Scrubber: suppress controller updates while dragging
        self.scrubber.sliderPressed.connect(self._on_scrub_start)
        self.scrubber.sliderReleased.connect(self._on_scrub_end)
        self.scrubber.valueChanged.connect(self._on_scrub_value)

        # Mode combo
        _mode_map = [ReplayMode.SINGLE, ReplayMode.LOOP, ReplayMode.PINGPONG, ReplayMode.REVERSE]
        self.combo_mode.currentIndexChanged.connect(
            lambda i: self._ctrl.set_mode(_mode_map[i])
        )

        # Speed combo
        _speed_map = [0.25, 0.5, 1.0, 2.0, 4.0]
        self.combo_speed.currentIndexChanged.connect(
            lambda i: self._ctrl.set_speed(_speed_map[i])
        )

        # Record button
        self.btn_record.toggled.connect(self._on_record_toggled)

        # Export
        self.btn_export.clicked.connect(self.export_requested)

    # ── Scrubber helpers ──────────────────────────────────────────────────────

    def _on_scrub_start(self) -> None:
        self._scrubbing = True

    def _on_scrub_end(self) -> None:
        self._scrubbing = False
        self._ctrl.seek(self.scrubber.value())

    def _on_scrub_value(self, value: int) -> None:
        if self._scrubbing:
            # Live preview while dragging (seek without emitting frame yet)
            self._ctrl.seek(value)
            self._update_labels(value)

    # ── Record handling ───────────────────────────────────────────────────────

    def _on_record_toggled(self, checked: bool) -> None:
        if checked:
            self.btn_record.setStyleSheet(_BTN_ACTIVE)
            self.btn_record.setToolTip("Recording… click to stop")
        else:
            self.btn_record.setStyleSheet(_BTN_BASE)
            self.btn_record.setToolTip("Start recording")
        self.record_toggled.emit(checked)

    def _on_recording_changed(self, active: bool) -> None:
        """Sync button if recording state was changed externally."""
        self.btn_record.blockSignals(True)
        self.btn_record.setChecked(active)
        self.btn_record.blockSignals(False)
        if active:
            self.btn_record.setStyleSheet(_BTN_ACTIVE)
        else:
            self.btn_record.setStyleSheet(_BTN_BASE)

    # ── Controller → UI updates ───────────────────────────────────────────────

    def _on_buffer_changed(self, count: int) -> None:
        self.scrubber.setMaximum(max(0, count - 1))
        self._update_labels(0)

    def _on_frame_changed(self, idx: int) -> None:
        if not self._scrubbing:
            self.scrubber.blockSignals(True)
            self.scrubber.setValue(idx)
            self.scrubber.blockSignals(False)
        self._update_labels(idx)

    def _update_labels(self, idx: int) -> None:
        total = self._ctrl.frame_count
        self.lbl_frames.setText(f"{idx + 1} / {total}")
        cur_t = self._ctrl.current_time
        dur   = self._ctrl.duration
        self.lbl_time.setText(f"{cur_t:.1f}s / {dur:.1f}s")

    def _update_all(self, state_name: str) -> None:
        """Sync all button states to the controller's current state."""
        try:
            state = ReplayState[state_name]
        except KeyError:
            state = ReplayState.IDLE

        is_playing  = state == ReplayState.PLAYING
        has_buffer  = self._ctrl.frame_count > 0

        self.btn_play_pause.setText("⏸" if is_playing else "▶")
        self.btn_play_pause.setStyleSheet(_BTN_PLAY_ACTIVE if is_playing else _BTN_BASE)
        self.btn_play_pause.setEnabled(has_buffer)
        self.btn_stop.setEnabled(has_buffer and state != ReplayState.STOPPED)
        self.btn_step_back.setEnabled(has_buffer)
        self.btn_step_fwd.setEnabled(has_buffer)
        self.scrubber.setEnabled(has_buffer)
        self.btn_export.setEnabled(has_buffer)
