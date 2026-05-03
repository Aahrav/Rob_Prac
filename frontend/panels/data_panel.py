#!/usr/bin/env python3
"""
DataPanel - Displays live Roll, Pitch, Yaw values and packets/sec rate.
Kinetic Precision theme: glassmorphic card with colored axis indicators.

P3-T3: Packets/sec counter uses a 1-second QTimer window.
       Call record_sample() each time a validated sample arrives;
       the timer fires every 1 000 ms, latches count → rate, then resets.
"""

from PyQt6.QtWidgets import QWidget, QGridLayout, QLabel, QFrame, QVBoxLayout, QHBoxLayout
from PyQt6.QtCore import Qt, QTimer


# ── Style tokens ────────────────────────────────────────────────────────────
CARD_STYLE = """
    QFrame#data_card {
        background-color: #0e0e0e;
        border-radius: 6px;
        border: 1px solid rgba(255, 255, 255, 0.04);
    }
"""

VAL_LABEL_STYLE = (
    "color: #92ccff;"
    "font-family: 'Space Grotesk', 'Consolas', monospace;"
    "font-size: 13px;"
    "font-weight: 700;"
    "letter-spacing: 0.03em;"
)
KEY_LABEL_STYLE = (
    "color: #89929b;"
    "font-family: 'Space Grotesk', 'Inter', sans-serif;"
    "font-size: 10px;"
    "font-weight: 600;"
    "letter-spacing: 0.08em;"
)
UNIT_LABEL_STYLE = "color: #3f4850; font-family: 'Space Grotesk', monospace; font-size: 10px;"

# Axis indicator colors (matching trajectory panel convention)
_AXIS_COLORS = {
    "ROLL":  "#92ccff",  # blue
    "PITCH": "#2ecc71",  # green
    "YAW":   "#ffba4b",  # amber
}

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

        # Glassmorphic card container
        card = QFrame()
        card.setObjectName("data_card")
        card.setStyleSheet(CARD_STYLE)
        grid = QGridLayout(card)
        grid.setContentsMargins(0, 8, 10, 8)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(4)

        def _row(r, key, unit_text="°"):
            # Colored left-border indicator
            indicator = QFrame()
            indicator.setFixedWidth(3)
            indicator.setStyleSheet(
                f"background-color: {_AXIS_COLORS.get(key, '#3f4850')}; border-radius: 1px; border: none;"
            )
            grid.addWidget(indicator, r, 0)

            lbl_key = QLabel(key)
            lbl_key.setStyleSheet(KEY_LABEL_STYLE)
            grid.addWidget(lbl_key, r, 1)

            lbl_val = QLabel("  0.0")
            lbl_val.setStyleSheet(VAL_LABEL_STYLE)
            lbl_val.setAlignment(
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
            )
            grid.addWidget(lbl_val, r, 2)

            lbl_unit = QLabel(unit_text)
            lbl_unit.setStyleSheet(UNIT_LABEL_STYLE)
            grid.addWidget(lbl_unit, r, 3)
            return lbl_val

        self.lbl_roll  = _row(0, "ROLL",  "°")
        self.lbl_pitch = _row(1, "PITCH", "°")
        self.lbl_yaw   = _row(2, "YAW",   "°")

        # ── Divider ────────────────────────────────────────────────────────
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        divider.setStyleSheet("color: rgba(255,255,255,0.04); background: rgba(255,255,255,0.04); border: none; max-height: 1px;")
        grid.addWidget(divider, 3, 0, 1, 4)

        # ── Packets/sec row (P3-T3) ────────────────────────────────────────
        # PKT/S indicator (no colored bar, just muted left indicator)
        pkt_indicator = QFrame()
        pkt_indicator.setFixedWidth(3)
        pkt_indicator.setStyleSheet("background-color: #3f4850; border-radius: 1px; border: none;")
        grid.addWidget(pkt_indicator, 4, 0)
        self._pkt_indicator = pkt_indicator

        lbl_pkt_key = QLabel("PKT/S")
        lbl_pkt_key.setStyleSheet(KEY_LABEL_STYLE)
        grid.addWidget(lbl_pkt_key, 4, 1)

        self.lbl_rate = QLabel("   0")
        self.lbl_rate.setStyleSheet(
            VAL_LABEL_STYLE.replace("#92ccff", _RATE_COLOR_ZERO)
        )
        self.lbl_rate.setAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        )
        grid.addWidget(self.lbl_rate, 4, 2)

        lbl_pps = QLabel("pps")
        lbl_pps.setStyleSheet(UNIT_LABEL_STYLE)
        grid.addWidget(lbl_pps, 4, 3)

        outer.addWidget(card)

    # ── Public API ───────────────────────────────────────────────────────────

    def update_values(self, roll: float, pitch: float, yaw: float,
                      rate: float = None) -> None:
        """Update roll/pitch/yaw readouts.

        Args:
            roll, pitch, yaw: angles in degrees.
            rate: optional explicit pps value; if provided, overrides the
                  internal timer-based counter display directly.  Useful for
                  Part 2's _on_sample_received pipeline if it tracks rate
                  centrally.  Leave None to use the built-in counter.
        """
        self.lbl_roll.setText(f"{roll:+7.1f}")
        self.lbl_pitch.setText(f"{pitch:+7.1f}")
        self.lbl_yaw.setText(f"{yaw:+7.1f}")

        # Colour-code angle labels by magnitude
        for val, lbl in [
            (abs(roll),  self.lbl_roll),
            (abs(pitch), self.lbl_pitch),
            (abs(yaw),   self.lbl_yaw),
        ]:
            color = "#ffba4b" if val > 60 else "#92ccff" if val > 30 else "#abcae8"
            lbl.setStyleSheet(VAL_LABEL_STYLE.replace("#92ccff", color))

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
        # Update PKT/S indicator color to match
        self._pkt_indicator.setStyleSheet(
            f"background-color: {color}; border-radius: 1px; border: none;"
        )
