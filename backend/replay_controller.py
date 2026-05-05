#!/usr/bin/env python3
"""
replay_controller.py — Pure state-machine controller for animation replay.

Design principles:
  - Zero imports of robot models, canvas, or UI panels.
  - Drives playback via a QTimer; emits frame_changed(int) signal only.
  - Consumers (MainWindow, AnalysisDashboard) subscribe to signals — no coupling.
  - All playback modes (SINGLE, LOOP, PINGPONG, REVERSE) handled here.
"""

from __future__ import annotations

from enum import Enum, auto
from typing import Optional

from PyQt6.QtCore import QObject, QTimer, pyqtSignal, Qt

from backend.replay_buffer import ReplayBuffer


# ──────────────────────────────────────────────────────────────────────────────
#  Enums
# ──────────────────────────────────────────────────────────────────────────────

class ReplayState(Enum):
    IDLE    = auto()   # no buffer loaded or cleared
    PLAYING = auto()   # actively ticking
    PAUSED  = auto()   # frozen at current frame
    STOPPED = auto()   # reset to frame 0


class ReplayMode(Enum):
    SINGLE   = auto()   # 0 → N, stop
    LOOP     = auto()   # 0 → N → 0 → N (restart)
    PINGPONG = auto()   # 0 → N → 0 → N (reverse direction)
    REVERSE  = auto()   # N → 0, stop


# ──────────────────────────────────────────────────────────────────────────────
#  Speed presets (multiplier → base interval of 33ms @ 30fps)
# ──────────────────────────────────────────────────────────────────────────────

_BASE_INTERVAL_MS = 33   # ~30 fps base

SPEED_PRESETS = {
    0.25: int(_BASE_INTERVAL_MS / 0.25),
    0.5:  int(_BASE_INTERVAL_MS / 0.5),
    1.0:  int(_BASE_INTERVAL_MS / 1.0),
    2.0:  int(_BASE_INTERVAL_MS / 2.0),
    4.0:  int(_BASE_INTERVAL_MS / 4.0),
}


# ──────────────────────────────────────────────────────────────────────────────
#  ReplayController
# ──────────────────────────────────────────────────────────────────────────────

class ReplayController(QObject):
    """
    State-machine controller that drives frame-by-frame animation replay.

    Signals
    -------
    frame_changed(int)          emitted each tick; consumers render the frame
    state_changed(str)          emitted when PLAYING/PAUSED/STOPPED/IDLE changes
    finished()                  emitted when SINGLE or REVERSE reaches its end
    buffer_changed(int)         emitted when a new buffer is attached (carries len)
    recording_state_changed(bool)  True = recording started, False = stopped

    Usage
    -----
    ctrl = ReplayController()
    ctrl.set_buffer(my_buffer)
    ctrl.frame_changed.connect(lambda idx: canvas.draw_frame(buf.get_frame(idx)))
    ctrl.play()
    """

    frame_changed           = pyqtSignal(int)
    state_changed           = pyqtSignal(str)
    finished                = pyqtSignal()
    buffer_changed          = pyqtSignal(int)          # carries frame count
    recording_state_changed = pyqtSignal(bool)         # True = active

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)

        self._buffer: Optional[ReplayBuffer] = None
        self._state: ReplayState = ReplayState.IDLE
        self._mode: ReplayMode = ReplayMode.SINGLE
        self._speed: float = 1.0
        self._current_idx: int = 0
        self._direction: int = 1          # +1 forward, -1 backward (pingpong)
        self._recording: bool = False

        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self._tick)
        self._timer.setInterval(SPEED_PRESETS[1.0])

    # ── Buffer management ─────────────────────────────────────────────────────

    def set_buffer(self, buffer: ReplayBuffer) -> None:
        """Attach a new ReplayBuffer. Stops any active playback first."""
        self.stop()
        self._buffer = buffer
        self._current_idx = 0
        self._direction = 1
        n = len(buffer)
        self.buffer_changed.emit(n)
        # Transition to STOPPED (ready) rather than IDLE
        self._set_state(ReplayState.STOPPED)

    def get_buffer(self) -> Optional[ReplayBuffer]:
        return self._buffer

    # ── Playback controls ─────────────────────────────────────────────────────

    def play(self) -> None:
        """Start or resume playback from the current frame."""
        if not self._buffer or len(self._buffer) == 0:
            return
        if self._state == ReplayState.PLAYING:
            return

        # If at the end and mode is SINGLE/REVERSE, restart
        if self._state == ReplayState.STOPPED:
            if self._mode == ReplayMode.REVERSE:
                self._current_idx = len(self._buffer) - 1
            else:
                self._current_idx = 0
            self._direction = -1 if self._mode == ReplayMode.REVERSE else 1

        self._timer.start()
        self._set_state(ReplayState.PLAYING)
        # Emit immediately so the renderer shows the current frame at once
        self.frame_changed.emit(self._current_idx)

    def pause(self) -> None:
        """Freeze at the current frame."""
        if self._state != ReplayState.PLAYING:
            return
        self._timer.stop()
        self._set_state(ReplayState.PAUSED)

    def toggle_play_pause(self) -> None:
        """Convenience: play if paused/stopped, pause if playing."""
        if self._state == ReplayState.PLAYING:
            self.pause()
        else:
            self.play()

    def stop(self) -> None:
        """Stop and reset to frame 0."""
        self._timer.stop()
        self._current_idx = 0
        self._direction = 1
        self._set_state(ReplayState.STOPPED)
        if self._buffer:
            self.frame_changed.emit(0)

    def step(self, delta: int) -> None:
        """
        Advance or retreat by |delta| frames (manual step).
        Works regardless of play state.
        """
        if not self._buffer:
            return
        was_playing = self._state == ReplayState.PLAYING
        if was_playing:
            self._timer.stop()

        self._current_idx = max(0, min(self._current_idx + delta, len(self._buffer) - 1))
        self.frame_changed.emit(self._current_idx)

        if was_playing:
            self._timer.start()

    def seek(self, idx: int) -> None:
        """Jump directly to a specific frame index."""
        if not self._buffer:
            return
        self._current_idx = max(0, min(idx, len(self._buffer) - 1))
        self.frame_changed.emit(self._current_idx)

    def seek_time(self, t: float) -> None:
        """Jump to the frame closest to time t (seconds)."""
        if not self._buffer:
            return
        idx = self._buffer.nearest_frame_to_time(t)
        self.seek(idx)

    # ── Mode and speed ────────────────────────────────────────────────────────

    def set_mode(self, mode: ReplayMode) -> None:
        self._mode = mode
        # Reset direction for clean state
        if mode == ReplayMode.REVERSE:
            self._direction = -1
        else:
            self._direction = 1

    def set_speed(self, multiplier: float) -> None:
        """Set playback speed. Accepts any positive float; snaps to nearest preset."""
        self._speed = multiplier
        # Find the nearest preset interval
        nearest = min(SPEED_PRESETS.keys(), key=lambda k: abs(k - multiplier))
        interval = SPEED_PRESETS.get(nearest, _BASE_INTERVAL_MS)
        self._timer.setInterval(interval)

    # ── Recording state ───────────────────────────────────────────────────────

    def start_recording(self, buffer: ReplayBuffer) -> None:
        """
        Attach buffer and signal that recording has started.
        Actual frame capture is done externally (e.g., in MainWindow._log_telemetry).
        """
        self._buffer = buffer
        buffer.start_recording()
        self._recording = True
        self.recording_state_changed.emit(True)

    def stop_recording(self) -> None:
        """Signal that recording is done; buffer is now ready for playback."""
        if not self._recording:
            return
        self._recording = False
        self.recording_state_changed.emit(False)
        if self._buffer and len(self._buffer) > 0:
            self._current_idx = 0
            self.buffer_changed.emit(len(self._buffer))
            self._set_state(ReplayState.STOPPED)

    @property
    def is_recording(self) -> bool:
        return self._recording

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def state(self) -> ReplayState:
        return self._state

    @property
    def mode(self) -> ReplayMode:
        return self._mode

    @property
    def speed(self) -> float:
        return self._speed

    @property
    def current_index(self) -> int:
        return self._current_idx

    @property
    def frame_count(self) -> int:
        return len(self._buffer) if self._buffer else 0

    @property
    def current_time(self) -> float:
        if self._buffer and self._buffer:
            try:
                return self._buffer.get_frame(self._current_idx).t
            except (IndexError, AttributeError):
                pass
        return 0.0

    @property
    def duration(self) -> float:
        return self._buffer.duration if self._buffer else 0.0

    # ── Internal tick ─────────────────────────────────────────────────────────

    def _tick(self) -> None:
        """Called by QTimer at the current speed interval."""
        if not self._buffer or len(self._buffer) == 0:
            self._timer.stop()
            return

        n = len(self._buffer)

        # Advance the frame index
        self._current_idx += self._direction
        self.frame_changed.emit(self._current_idx)

        # Handle end-of-sequence based on mode
        if self._direction == 1 and self._current_idx >= n - 1:
            self._on_reached_end()
        elif self._direction == -1 and self._current_idx <= 0:
            self._on_reached_start()

    def _on_reached_end(self) -> None:
        self._current_idx = len(self._buffer) - 1
        if self._mode == ReplayMode.SINGLE:
            self._timer.stop()
            self._set_state(ReplayState.STOPPED)
            self.finished.emit()
        elif self._mode == ReplayMode.LOOP:
            self._current_idx = 0
        elif self._mode == ReplayMode.PINGPONG:
            self._direction = -1   # reverse
        elif self._mode == ReplayMode.REVERSE:
            # Already going forward in REVERSE mode? Shouldn't happen — stop.
            self._timer.stop()
            self._set_state(ReplayState.STOPPED)
            self.finished.emit()

    def _on_reached_start(self) -> None:
        self._current_idx = 0
        if self._mode == ReplayMode.REVERSE:
            self._timer.stop()
            self._set_state(ReplayState.STOPPED)
            self.finished.emit()
        elif self._mode == ReplayMode.PINGPONG:
            self._direction = 1    # reverse back to forward
        elif self._mode == ReplayMode.LOOP:
            self._current_idx = len(self._buffer) - 1

    def _set_state(self, state: ReplayState) -> None:
        if self._state != state:
            self._state = state
            self.state_changed.emit(state.name)
