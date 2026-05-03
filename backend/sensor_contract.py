"""
Shared sensor data contract (MVP §3).

All producers emit dicts that normalise to the canonical keys below.
CSV replay/record uses the same column names as HEADER_ROW.
"""

from __future__ import annotations

from typing import Any

CONTRACT_VERSION = 1

REQUIRED_KEYS = ("t", "r", "p", "y")
OPTIONAL_GYRO_KEYS = ("gx", "gy", "gz")

CSV_FIELDNAMES = ["t", "r", "p", "y", "gx", "gy", "gz"]
CSV_HEADER_ROW = ",".join(CSV_FIELDNAMES)


def normalise_sample(d: dict[str, Any]) -> dict[str, Any]:
    """
    Validate presence of ``t,r,p,y`` and coerce numeric types.

    Raises:
        ValueError: missing keys or non-numeric values after coercion.

    Returns:
        Dict with ``t`` int; ``r,p,y,gx,gy,gz`` floats (gyros default 0).
    """
    missing = [k for k in REQUIRED_KEYS if k not in d]
    if missing:
        raise ValueError(f"Sample missing keys {missing}: {d!r}")

    try:
        t = int(float(d["t"]))
        r = float(d["r"])
        p = float(d["p"])
        y = float(d["y"])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Non-numeric required fields in {d!r}") from exc

    out = {"t": t, "r": r, "p": p, "y": y}
    for g in OPTIONAL_GYRO_KEYS:
        try:
            v = d.get(g, 0.0)
            out[g] = float(v) if v not in (None, "") else 0.0
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Bad gyro field {g} in {d!r}") from exc

    return out
