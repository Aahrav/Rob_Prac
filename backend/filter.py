"""
Complementary filter — fuse gyro integration with measured Euler tilt.

Uses independent axes (roadmap MVP): each angle blends gyro propagation with
the instantaneous measured angle from the sample (``r``, ``p``, ``y`` in degrees).
"""

from __future__ import annotations

from typing import Any


class ComplementaryFilter:
    """Stateful complementary fusion per axis."""

    def __init__(self, alpha: float = 0.98) -> None:
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("alpha must be in [0, 1]")
        self._alpha = alpha
        self._last_t_ms: int | None = None
        self._angle = [0.0, 0.0, 0.0]  # roll, pitch, yaw estimates

    def reset(self) -> None:
        self._last_t_ms = None
        self._angle = [0.0, 0.0, 0.0]

    def step(self, sample: dict[str, Any]) -> tuple[float, float, float]:
        """
        One fusion step.

        Args:
            sample: Normalised dict with ``t``, ``r``, ``p``, ``y``, ``gx``, ``gy``, ``gz``.

        Returns:
            Roll, pitch, yaw degrees after fusion.
        """
        t = int(sample["t"])
        meas = [float(sample["r"]), float(sample["p"]), float(sample["y"])]
        gyro = [
            float(sample.get("gx", 0.0)),
            float(sample.get("gy", 0.0)),
            float(sample.get("gz", 0.0)),
        ]

        if self._last_t_ms is None:
            self._angle = meas.copy()
            self._last_t_ms = t
            return self._angle[0], self._angle[1], self._angle[2]

        dt = max((t - self._last_t_ms) / 1000.0, 1e-9)
        self._last_t_ms = t
        a = self._alpha
        for i in range(3):
            self._angle[i] = (
                a * (self._angle[i] + gyro[i] * dt) + (1.0 - a) * meas[i]
            )

        return self._angle[0], self._angle[1], self._angle[2]
