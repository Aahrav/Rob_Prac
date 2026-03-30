#!/usr/bin/env python3
"""
TrajectoryPanel - Interactive control for moving the arm to a target.
Allows setting start/end positions and animating the motion.
"""

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QLabel, QSlider, QPushButton, QDoubleSpinBox, QHBoxLayout
from PyQt6.QtCore import Qt, pyqtSignal


class TrajectoryPanel(QGroupBox):
    """Panel for interactive trajectory control."""

    target_updated = pyqtSignal(float, float, float)  # roll, pitch, yaw

    def __init__(self, parent=None):
        super().__init__("Trajectory Control", parent)

        self.layout = QVBoxLayout(self)

        # Header
        desc = QLabel("Set target orientation and animate movement.")
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #aaa; font-size: 11px; padding: 5px;")
        self.layout.addWidget(desc)

        # Target inputs with sliders
        self._create_angle_control("Roll", -180, 180, 0, 0)
        self._create_angle_control("Pitch", -90, 90, 0, 1)
        self._create_angle_control("Yaw", -180, 180, 0, 2)

        # Animate button
        self.btn_animate = QPushButton("Animate to Target")
        self.btn_animate.setStyleSheet("""
            QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 10px; }
            QPushButton:hover { background-color: #2ecc71; }
        """)
        self.btn_animate.clicked.connect(self._animate_clicked)
        self.layout.addWidget(self.btn_animate)

        # Stop button
        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setStyleSheet("""
            QPushButton { background-color: #c0392b; color: white; padding: 10px; }
            QPushButton:hover { background-color: #e74c3c; }
        """)
        self.btn_stop.clicked.connect(self._stop_clicked)
        self.btn_stop.setEnabled(False)
        self.layout.addWidget(self.btn_stop)

        # Status
        self.lbl_status = QLabel("Ready")
        self.lbl_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_status.setStyleSheet("color: #ddd; padding: 5px;")
        self.layout.addWidget(self.lbl_status)

        self.setStyleSheet("""
            QGroupBox {
                border: 1px solid #444;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

        # Animation state
        self.animating = False
        self.current_angles = [0.0, 0.0, 0.0]
        self.target_angles = [0.0, 0.0, 0.0]
        self.animation_timer = None

    def _create_angle_control(self, name, min_val, max_val, default, idx):
        """Create a slider + spinbox pair for an angle."""
        hbox = QHBoxLayout()
        lbl = QLabel(f"{name}:")
        lbl.setFixedWidth(50)
        hbox.addWidget(lbl)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(default)
        slider.valueChanged.connect(lambda v, i=idx: self._on_slider_changed(i, v))
        hbox.addWidget(slider)

        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setValue(default)
        spinbox.setFixedWidth(70)
        spinbox.valueChanged.connect(lambda v, i=idx: self._on_spinbox_changed(i, v))
        hbox.addWidget(spinbox)

        # Store references
        setattr(self, f"slider_{name.lower()}", slider)
        setattr(self, f"spin_{name.lower()}", spinbox)

        self.layout.addLayout(hbox)

    def _on_slider_changed(self, idx, value):
        """Slider moved - update spinbox and target."""
        names = ['roll', 'pitch', 'yaw']
        spin = getattr(self, f"spin_{names[idx]}")
        spin.setValue(value)
        self._update_target()

    def _on_spinbox_changed(self, idx, value):
        """Spinbox changed - update slider and target."""
        names = ['roll', 'pitch', 'yaw']
        slider = getattr(self, f"slider_{names[idx]}")
        slider.setValue(int(value))
        self._update_target()

    def _update_target(self):
        """Update target from UI values."""
        self.target_angles = [
            self.slider_roll.value(),
            self.slider_pitch.value(),
            self.slider_yaw.value()
        ]
        self.target_updated.emit(*self.target_angles)

    def _on_target_changed(self):
        self._update_target()

    def _animate_clicked(self):
        """Start animation to target."""
        if self.animating:
            return
        self.animating = True
        self.btn_animate.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.lbl_status.setText("Animating...")

        # Get target
        target = self.target_angles
        start = self.current_angles

        # Animation parameters
        self.anim_duration = 2000  # ms
        self.anim_start_time = None
        self.animation_timer = None

        from PyQt6.QtCore import QTimer
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animation_step)
        self.animation_timer.start(16)  # ~60fps

        # We'll track elapsed manually.
        self._anim_start_time = None  # set on first tick

    def _animation_step(self):
        """One frame of animation."""
        import time
        if self._anim_start_time is None:
            self._anim_start_time = time.time()

        elapsed = (time.time() - self._anim_start_time) * 1000  # ms
        t = min(elapsed / self.anim_duration, 1.0)

        # Ease in-out
        t = 3*t*t - 2*t*t*t  # smoothstep

        # Interpolate angles
        start = self.current_angles
        target = self.target_angles
        current = [start[i] + (target[i] - start[i]) * t for i in range(3)]

        # Emit
        self.target_updated.emit(*current)

        if t >= 1.0:
            self.animation_timer.stop()
            self.animating = False
            self.current_angles = target[:]
            self.btn_animate.setEnabled(True)
            self.btn_stop.setEnabled(False)
            self.lbl_status.setText("Complete")

    def _stop_clicked(self):
        """Stop animation early."""
        if self.animation_timer:
            self.animation_timer.stop()
        self.animating = False
        self.btn_animate.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText("Stopped")

    def set_current_angles(self, roll, pitch, yaw):
        """Update the displayed current angles (from external source)."""
        self.current_angles = [roll, pitch, yaw]
        # If not animating, reflect in UI sliders
        if not self.animating:
            self.slider_roll.setValue(int(roll))
            self.slider_pitch.setValue(int(pitch))
            self.slider_yaw.setValue(int(yaw))
            self._update_target()
