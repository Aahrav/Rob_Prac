#!/usr/bin/env python3
"""
CV Hand Gesture Controller — Backend Worker
============================================
Runs MediaPipe Hands in a background QThread. Detects hand landmarks,
classifies gestures, applies a moving-average smoothing filter, and emits
a HandFrame signal at up to 30 FPS that the UI panel can consume directly.

No sockets/pipes are used — the worker lives in-process but on its own
QThread so it never blocks the main GUI thread.

Gesture → Joint mapping
-----------------------
Revolute joints  : wrist tilt angle  → joint angle in [q_min, q_max]
Prismatic joints : palm-centre Y pos → joint displacement in [q_min, q_max]

Special gestures (override tilt/position):
  open_palm   → PAUSED  (freeze current angle)
  fist        → RESET   (→ 0°)
  pinch       → LOCKED  (confirm + stay)
  two_hands   → ESTOP   (emergency stop)
"""

from __future__ import annotations

import time
import math
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from PyQt6.QtCore import QThread, pyqtSignal, QMutex, QMutexLocker

from backend.logger import get_logger

_log = get_logger(__name__)

# ── Optional deps — lazy import so the rest of the app still loads ──────────
try:
    import cv2                          # type: ignore
    _CV2_OK = True
except ImportError:
    _CV2_OK = False
    _log.warning("opencv-python not installed — CV Hand Control unavailable")

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
    _MP_OK = True
except ImportError:
    _MP_OK = False
    _log.warning("mediapipe not installed — CV Hand Control unavailable")


# ═══════════════════════════════════════════════════════════════════════════
#  Data structures
# ═══════════════════════════════════════════════════════════════════════════

GESTURE_ROTATING  = "ROTATING"
GESTURE_PAUSED    = "PAUSED"
GESTURE_RESET     = "RESET"
GESTURE_LOCKED    = "LOCKED"
GESTURE_ESTOP     = "ESTOP"
GESTURE_MOVING    = "MOVING"
GESTURE_IDLE      = "IDLE"


@dataclass
class HandFrame:
    """One processed frame worth of hand-control data."""
    # Joint movement delta (degrees for revolute, metres for prismatic)
    delta_value: float = 0.0
    # Gesture label string
    gesture: str = GESTURE_IDLE
    # Raw wrist-tilt angle in degrees (-90 … +90)
    tilt_deg: float = 0.0
    # Palm Y position normalised 0…1 (0 = top, 1 = bottom)
    palm_y_norm: float = 0.5
    # Number of extended fingers (0..5)
    finger_count: int = 0
    # Number of hands detected (0, 1, or 2)
    hand_count: int = 0
    # BGR frame with landmarks drawn — bytes (JPEG-encoded) for Qt display
    frame_jpeg: Optional[bytes] = None
    # Timestamp
    t: float = field(default_factory=time.monotonic)


# ═══════════════════════════════════════════════════════════════════════════
#  Smoothing helper
# ═══════════════════════════════════════════════════════════════════════════

class MovingAverageFilter:
    """Simple sliding-window average for scalar values."""

    def __init__(self, window: int = 5):
        self._buf: deque = deque(maxlen=window)

    def update(self, value: float) -> float:
        self._buf.append(value)
        return float(np.mean(self._buf))

    def reset(self):
        self._buf.clear()


# ═══════════════════════════════════════════════════════════════════════════
#  Drawing helper (replacing mp.solutions.drawing_utils)
# ═══════════════════════════════════════════════════════════════════════════

_HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),        # Thumb
    (0, 5), (5, 6), (6, 7), (7, 8),        # Index
    (5, 9), (9, 10), (10, 11), (11, 12),   # Middle
    (9, 13), (13, 14), (14, 15), (15, 16), # Ring
    (13, 17), (17, 18), (18, 19), (19, 20),# Pinky
    (0, 17)                                # Palm base
]

def _draw_landmarks(frame, landmarks):
    h, w = frame.shape[:2]
    # Draw connections
    for start_idx, end_idx in _HAND_CONNECTIONS:
        start_pt = (int(landmarks[start_idx].x * w), int(landmarks[start_idx].y * h))
        end_pt = (int(landmarks[end_idx].x * w), int(landmarks[end_idx].y * h))
        cv2.line(frame, start_pt, end_pt, (200, 200, 200), 2)
        
    # Draw keypoints
    for lm in landmarks:
        pt = (int(lm.x * w), int(lm.y * h))
        cv2.circle(frame, pt, 4, (0, 0, 255), -1)


# ═══════════════════════════════════════════════════════════════════════════
#  Gesture classifier
# ═══════════════════════════════════════════════════════════════════════════

def _finger_extended(lm, finger_tip_idx: int, finger_pip_idx: int) -> bool:
    """Return True if a finger is extended (tip Y < pip Y in image coords)."""
    return lm[finger_tip_idx].y < lm[finger_pip_idx].y


def classify_gesture(landmarks, hand_count: int = 1) -> Tuple[str, int]:
    """
    Classify hand gesture from a list of MediaPipe normalized landmarks.

    Returns (gesture_label, finger_count).
    """
    if hand_count >= 2:
        return GESTURE_ESTOP, -1

    lm = landmarks

    # Finger tip and PIP landmark indices (MediaPipe convention):
    # Thumb: 4 (tip), 3 (ip), 2 (mcp)
    # Index: 8 (tip), 6 (pip)
    # Middle: 12 (tip), 10 (pip)
    # Ring: 16 (tip), 14 (pip)
    # Pinky: 20 (tip), 18 (pip)

    thumb_tip   = lm[4]
    index_tip   = lm[8]
    index_pip   = lm[6]
    middle_tip  = lm[12]
    middle_pip  = lm[10]
    ring_tip    = lm[16]
    ring_pip    = lm[14]
    pinky_tip   = lm[20]
    pinky_pip   = lm[18]

    # Thumb: compare tip X vs MCP X (left/right hand aware via wrist position)
    wrist_x = lm[0].x
    thumb_mcp_x = lm[2].x
    thumb_extended = (thumb_tip.x < thumb_mcp_x) if wrist_x > 0.5 else (thumb_tip.x > thumb_mcp_x)

    fingers = [
        thumb_extended,
        _finger_extended(lm, 8, 6),
        _finger_extended(lm, 12, 10),
        _finger_extended(lm, 16, 14),
        _finger_extended(lm, 20, 18),
    ]
    count = sum(fingers)

    # Pinch: thumb and index tips very close
    dx = thumb_tip.x - index_tip.x
    dy = thumb_tip.y - index_tip.y
    pinch_dist = math.sqrt(dx * dx + dy * dy)
    if pinch_dist < 0.05:
        return GESTURE_LOCKED, count

    # Open palm: 5 fingers extended
    if count == 5:
        return GESTURE_PAUSED, count

    # Closed fist: 0 or 1 fingers (thumb only)
    if count <= 1 and not fingers[1]:  # index not extended
        return GESTURE_RESET, count

    return GESTURE_ROTATING, count


def compute_wrist_tilt(landmarks) -> float:
    """
    Compute the tilt angle of the hand in degrees.
    Uses the vector from wrist (lm[0]) to middle-finger MCP (lm[9]).
    Returns angle in [-90, +90]: negative = left tilt, positive = right tilt.
    """
    lm = landmarks
    wrist   = lm[0]
    mid_mcp = lm[9]
    dx = mid_mcp.x - wrist.x
    dy = mid_mcp.y - wrist.y  # positive = downward in image coords
    # Angle from vertical axis (pointing up in image = negative dy)
    angle_rad = math.atan2(dx, -dy)
    return math.degrees(angle_rad)


def compute_palm_center(landmarks) -> Tuple[float, float]:
    """Return normalised (cx, cy) of the palm centre (average of palm base points)."""
    lm = landmarks
    palm_indices = [0, 1, 5, 9, 13, 17]  # wrist + MCPs
    cx = float(np.mean([lm[i].x for i in palm_indices]))
    cy = float(np.mean([lm[i].y for i in palm_indices]))
    return cx, cy


# ═══════════════════════════════════════════════════════════════════════════
#  Main QThread worker
# ═══════════════════════════════════════════════════════════════════════════

class CVHandController(QThread):
    """
    Background QThread that:
      1. Opens the webcam via OpenCV
      2. Runs MediaPipe Hands on each frame
      3. Classifies gesture + computes joint value
      4. Emits hand_frame signal with a HandFrame payload
      5. Emits error_occurred signal on fatal errors

    Usage
    -----
    ctrl = CVHandController()
    ctrl.set_joint(joint, joint_index)     # select joint to control
    ctrl.set_sensitivity(1.0)              # 0.5 low, 1.0 medium, 2.0 high
    ctrl.set_dead_zone(0.02)               # normalised tilt dead-zone
    ctrl.hand_frame.connect(my_slot)
    ctrl.start()
    # … later:
    ctrl.stop(); ctrl.wait()
    """

    # Signals
    hand_frame     = pyqtSignal(object)   # HandFrame object
    error_occurred = pyqtSignal(str)      # error message string

    # Maximum joint speed (deg/s for revolute, m/s for prismatic)
    MAX_JOINT_SPEED_DEG = 120.0
    MAX_JOINT_SPEED_M   = 0.5

    def __init__(self, camera_index: int = 0, parent=None):
        super().__init__(parent)
        self._camera_index = camera_index
        self._running = False
        self._mutex = QMutex()

        # Joint to control
        self._joint = None           # DHJoint or None
        self._joint_index: int = 0
        self._joint_value: float = 0.0   # current value (deg or m)

        # Control parameters
        self._sensitivity: float = 1.0   # multiplier on tilt→angle mapping
        self._dead_zone: float = 0.03    # normalised dead-zone
        self._smoothing_window: int = 5

        # Smoothing filters
        self._tilt_filter   = MovingAverageFilter(self._smoothing_window)
        self._palmy_filter  = MovingAverageFilter(self._smoothing_window)

        # State
        self._paused  = False
        self._locked  = False
        self._estop   = False
        self._last_t  = time.monotonic()

        # Gesture hysteresis counter (prevents single-frame flickers)
        self._gesture_hold: str = GESTURE_IDLE
        self._gesture_hold_count: int = 0
        self._GESTURE_HOLD_FRAMES = 3

    # ── Public configuration API ────────────────────────────────────────────

    def set_joint(self, joint, joint_index: int):
        """Set the joint to control. joint is a DHJoint (or None to idle)."""
        with QMutexLocker(self._mutex):
            self._joint = joint
            self._joint_index = joint_index
            # Don't reset value — allows smooth resume

    def set_sensitivity(self, value: float):
        """sensitivity: 0.5 = low, 1.0 = medium, 2.0 = high."""
        with QMutexLocker(self._mutex):
            self._sensitivity = max(0.1, float(value))

    def set_dead_zone(self, value: float):
        """dead_zone: 0.0–0.1 normalised tilt fraction."""
        with QMutexLocker(self._mutex):
            self._dead_zone = max(0.0, float(value))

    def set_smoothing_window(self, window: int):
        with QMutexLocker(self._mutex):
            self._smoothing_window = max(1, window)
            self._tilt_filter  = MovingAverageFilter(self._smoothing_window)
            self._palmy_filter = MovingAverageFilter(self._smoothing_window)

    def stop(self):
        with QMutexLocker(self._mutex):
            self._running = False

    # ── QThread entry point ──────────────────────────────────────────────────

    def run(self):
        if not _CV2_OK or not _MP_OK:
            self.error_occurred.emit(
                "mediapipe and opencv-python are required.\n"
                "Install: pip install mediapipe opencv-python"
            )
            return

        with QMutexLocker(self._mutex):
            self._running = True

        cap = cv2.VideoCapture(self._camera_index)
        if not cap.isOpened():
            self.error_occurred.emit(
                f"Cannot open webcam (index {self._camera_index}). "
                "Check that a camera is connected and not in use."
            )
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        # Setup MediaPipe Tasks HandLandmarker
        base_options = mp_python.BaseOptions(model_asset_path='backend/models/hand_landmarker.task')
        options = mp_vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=2,
            min_hand_detection_confidence=0.75,
            min_hand_presence_confidence=0.75,
            min_tracking_confidence=0.75)
        detector = mp_vision.HandLandmarker.create_from_options(options)

        _log.info("CVHandController started (camera=%d)", self._camera_index)

        try:
            while True:
                with QMutexLocker(self._mutex):
                    if not self._running:
                        break

                ok, frame = cap.read()
                if not ok:
                    _log.warning("Webcam frame read failed — retrying")
                    time.sleep(0.05)
                    continue

                # Mirror for natural interaction
                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Convert to MP Image
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                results = detector.detect(mp_image)

                hand_count = 0
                gesture    = GESTURE_PAUSED  # Default to paused if no hands
                tilt_deg   = 0.0
                palm_y     = 0.5
                finger_cnt = 0

                if results.hand_landmarks:
                    hand_count = len(results.hand_landmarks)

                    # Use first (primary) hand for control
                    primary_lm = results.hand_landmarks[0]

                    # Draw all detected hands
                    for hand_lm in results.hand_landmarks:
                        _draw_landmarks(frame, hand_lm)

                    gesture, finger_cnt = classify_gesture(primary_lm, hand_count)
                    tilt_deg = compute_wrist_tilt(primary_lm)
                    _, palm_y = compute_palm_center(primary_lm)

                # Apply smoothing
                with QMutexLocker(self._mutex):
                    sensitivity = self._sensitivity
                    dead_zone   = self._dead_zone
                    joint       = self._joint

                smooth_tilt  = self._tilt_filter.update(tilt_deg)
                smooth_palmy = self._palmy_filter.update(palm_y)

                # Gesture hysteresis
                if gesture == self._gesture_hold:
                    self._gesture_hold_count += 1
                else:
                    self._gesture_hold_count = 0
                    self._gesture_hold = gesture
                stable_gesture = gesture if self._gesture_hold_count >= self._GESTURE_HOLD_FRAMES else GESTURE_IDLE

                # Compute joint delta
                delta_value = self._compute_joint_delta(
                    joint=joint,
                    gesture=stable_gesture,
                    tilt_deg=smooth_tilt,
                    palm_y=smooth_palmy,
                    sensitivity=sensitivity,
                    dead_zone=dead_zone,
                )

                # Draw HUD overlay on frame
                self._draw_hud(frame, stable_gesture, smooth_tilt, delta_value, joint)

                # JPEG encode for Qt display (320×240 thumbnail)
                thumb = cv2.resize(frame, (320, 240))
                _, jpeg_buf = cv2.imencode('.jpg', thumb, [cv2.IMWRITE_JPEG_QUALITY, 75])
                frame_jpeg = jpeg_buf.tobytes()

                hf = HandFrame(
                    delta_value=delta_value,
                    gesture=stable_gesture,
                    tilt_deg=smooth_tilt,
                    palm_y_norm=smooth_palmy,
                    finger_count=finger_cnt,
                    hand_count=hand_count,
                    frame_jpeg=frame_jpeg,
                )
                self.hand_frame.emit(hf)

                # Target ~30 FPS
                time.sleep(max(0.0, 0.033 - (time.monotonic() - hf.t)))

        except Exception as exc:
            _log.exception("CVHandController crashed")
            self.error_occurred.emit(f"Hand tracking error: {exc}")
        finally:
            if 'detector' in locals():
                detector.close()
            cap.release()
            _log.info("CVHandController stopped")

    # ── Joint delta computation ──────────────────────────────────────────────

    def _compute_joint_delta(
        self,
        joint,
        gesture: str,
        tilt_deg: float,
        palm_y: float,
        sensitivity: float,
        dead_zone: float,
    ) -> float:

        # Emergency stop, pause, lock -> no movement
        if gesture in (GESTURE_ESTOP, GESTURE_PAUSED, GESTURE_LOCKED, GESTURE_RESET):
            if gesture == GESTURE_ESTOP:
                self._estop = True
            elif gesture == GESTURE_RESET:
                self._tilt_filter.reset()
                self._palmy_filter.reset()
            return 0.0

        self._estop = False

        if joint is None:
            return 0.0

        now = time.monotonic()
        dt  = now - self._last_t
        self._last_t = now
        dt = min(dt, 0.1)   # clamp for large gaps

        delta = 0.0

        if joint.type == 'revolute':
            eff_tilt = tilt_deg
            if abs(eff_tilt) < dead_zone * 90.0:
                eff_tilt = 0.0

            # rate in deg/s
            rate = eff_tilt * sensitivity
            max_step = self.MAX_JOINT_SPEED_DEG * dt
            delta = np.clip(rate * dt, -max_step, max_step)

        elif joint.type == 'prismatic':
            eff_y = palm_y
            if abs(eff_y - 0.5) < dead_zone:
                eff_y = 0.5

            # rate in m/s. 0.5 is center.
            rate = (0.5 - eff_y) * 2.0 * sensitivity
            max_step = self.MAX_JOINT_SPEED_M * dt
            delta = np.clip(rate * dt, -max_step, max_step)

        return float(delta)

    # ── HUD drawing ──────────────────────────────────────────────────────────

    @staticmethod
    def _draw_hud(frame, gesture: str, tilt_deg: float, delta_value: float, joint):
        h, w = frame.shape[:2]

        # Gesture badge
        COLORS = {
            GESTURE_ROTATING : (0, 200,  60),
            GESTURE_PAUSED   : (0, 200, 255),
            GESTURE_RESET    : (0, 140, 255),
            GESTURE_LOCKED   : (255, 200,  0),
            GESTURE_ESTOP    : (0,   0, 255),
            GESTURE_IDLE     : (120, 120, 120),
            GESTURE_MOVING   : (0, 200, 60),
        }
        color = COLORS.get(gesture, (180, 180, 180))

        cv2.rectangle(frame, (8, 8), (220, 36), (20, 20, 20), -1)
        cv2.putText(frame, gesture, (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)

        # Tilt bar
        cx = w // 2
        bar_y = h - 20
        bar_half = 80
        cv2.line(frame, (cx - bar_half, bar_y), (cx + bar_half, bar_y),
                 (60, 60, 60), 4)
        tilt_px = int(np.clip(tilt_deg / 90.0, -1.0, 1.0) * bar_half)
        cv2.line(frame, (cx, bar_y), (cx + tilt_px, bar_y), color, 4)
        cv2.circle(frame, (cx + tilt_px, bar_y), 6, color, -1)

        # Joint delta readout
        jtype = joint.type if joint else "–"
        unit  = "°/s" if jtype == "revolute" else " m/s"
        # Convert delta per frame (~30fps) to per second for display
        rate = delta_value * 30.0 
        val_str = f"{rate:+.1f}{unit}"
        cv2.putText(frame, val_str, (w - 110, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (220, 220, 220), 2, cv2.LINE_AA)
