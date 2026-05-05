#!/usr/bin/env python3
"""
DataPanel - Displays live Roll, Pitch, Yaw values and packets/sec rate.
Kinetic Obsidian theme: compact monospace readout card.

P3-T3: Packets/sec counter uses a 1-second QTimer window.
       Call record_sample() each time a validated sample arrives;
       the timer fires every 1 000 ms, latches count → rate, then resets.
"""

from PyQt6.QtWidgets import QWidget, QGridLayout, QLabel, QFrame, QVBoxLayout
from PyQt6.QtCore import Qt, QTimer


# ── Style tokens ────────────────────────────────────────────────────────────
CARD_STYLE = """
    QWidget#data_card {
        background-color: #0e0e0e;
        border-radius: 4px;
    }
"""

VAL_LABEL_STYLE = (
    "color: #92ccff;"
    "font-family: 'Consolas', monospace;"
    "font-size: 13px;"
    "font-weight: 700;"
)
KEY_LABEL_STYLE = (
    "color: #89929b;"
    "font-size: 10px;"
    "font-weight: 600;"
    "letter-spacing: 0.05em;"
)
UNIT_LABEL_STYLE = "color: #3f4850; font-size: 10px;"

# Packet-rate colours
_RATE_COLOR_HEALTHY = "#2ecc71"   # green  — ≥ 5 pps (good live feed)
_RATE_COLOR_SLOW    = "#f39c12"   # amber  — 1–4 pps (slow / replay)
_RATE_COLOR_ZERO    = "#3f4850"   # muted  — 0 pps (idle)


class DataPanel(QWidget):
    """Live joint angle / data readout panel with packets-per-second counter."""

    def __init__(self, parent=None):
        super().__init__(parent)

        # ── Packet counter state (P3-T3) ──────────────────────────────────
        self._sample_count: int = 0       # incremented by record_sample()
        self._current_rate: float = 0.0  # latched every 1 s by _flush_rate

        self._rate_timer = QTimer(self)
        self._rate_timer.setInterval(1000)   # 1-second window
        self._rate_timer.timeout.connect(self._flush_rate)
        self._rate_timer.start()

        self._init_ui()

    # ── UI construction ──────────────────────────────────────────────────────

    def _init_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # Milled card container
        card = QFrame()
        card.setObjectName("data_card")
        card.setStyleSheet(
            "QFrame#data_card { background-color: #0e0e0e; border-radius: 4px; }"
        )
        grid = QGridLayout(card)
        grid.setContentsMargins(10, 8, 10, 8)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(4)

        def _row(r, key, unit_text="°"):
            lbl_key = QLabel(key)
            lbl_key.setStyleSheet(KEY_LABEL_STYLE)
            grid.addWidget(lbl_key, r, 0)

            lbl_val = QLabel("  0.0")
            lbl_val.setStyleSheet(VAL_LABEL_STYLE)
            lbl_val.setAlignment(
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
            )
            grid.addWidget(lbl_val, r, 1)

            lbl_unit = QLabel(unit_text)
            lbl_unit.setStyleSheet(UNIT_LABEL_STYLE)
            grid.addWidget(lbl_unit, r, 2)
            return lbl_val

        self.lbl_angles = []
        for i in range(6):
            label = _row(i, f"J{i+1}", "°")
            label.setVisible(i < 3)  # default to 3
            self.lbl_angles.append(label)
            
        # Legacy aliases for 3-DOF compatibility
        self.lbl_roll  = self.lbl_angles[0]
        self.lbl_pitch = self.lbl_angles[1]
        self.lbl_yaw   = self.lbl_angles[2]

        # ── Divider ────────────────────────────────────────────────────────
        self.divider = QFrame()
        self.divider.setFrameShape(QFrame.Shape.HLine)
        self.divider.setStyleSheet("color: #1e1e1e; background: #1e1e1e; border: none; max-height: 1px;")
        grid.addWidget(self.divider, 6, 0, 1, 3)

        # ── Packets/sec row (P3-T3) ────────────────────────────────────────
        self.lbl_rate = _row(7, "PKT/S", "pps")
        self.lbl_rate.setText("   0")
        self.lbl_rate.setStyleSheet(
            VAL_LABEL_STYLE.replace("#92ccff", _RATE_COLOR_ZERO)
        )

        outer.addWidget(card)

    # ── Public API ───────────────────────────────────────────────────────────

    def update_values(self, *angles, rate: float = None) -> None:
        """Update joint angle readouts. Handles 1 to 6 angles."""
        num_angles = len(angles)
        
        for i, val in enumerate(angles):
            if i >= 6: break
            lbl = self.lbl_angles[i]
            lbl.setVisible(True)
            lbl.setText(f"{val:+7.1f}")
            
            # Colour-code by magnitude
            mag = abs(val)
            color = "#ffba4b" if mag > 60 else "#92ccff" if mag > 30 else "#abcae8"
            lbl.setStyleSheet(VAL_LABEL_STYLE.replace("#92ccff", color))
            
        # Hide unused labels (if we moved from 6 joints to 3)
        for i in range(num_angles, 6):
            self.lbl_angles[i].setVisible(False)

        # If caller supplies an explicit rate, show it immediately
        if rate is not None:
            self._show_rate(float(rate))

    def record_sample(self) -> None:
        """Increment the packet counter — invoked once per displayed sample."""
        self._sample_count += 1

    # ── Private helpers ──────────────────────────────────────────────────────

    def _flush_rate(self) -> None:
        """Slot called every 1 000 ms by the QTimer.

        Latches the accumulated sample count as the current pps reading,
        then resets the counter for the next window.
        """
        rate = float(self._sample_count)
        self._sample_count = 0
        self._current_rate = rate
        self._show_rate(rate)

    def _show_rate(self, rate: float) -> None:
        """Update the PKT/S label text and colour."""
        if rate >= 5:
            color = _RATE_COLOR_HEALTHY
        elif rate >= 1:
            color = _RATE_COLOR_SLOW
        else:
            color = _RATE_COLOR_ZERO

        self.lbl_rate.setText(f"{int(rate):4d}")
        self.lbl_rate.setStyleSheet(
            VAL_LABEL_STYLE.replace("#92ccff", color)
        )
