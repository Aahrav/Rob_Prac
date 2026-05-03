"""
Parse JSON lines from serial or instrumentation into sensor dicts.
"""

from __future__ import annotations

import json
from typing import Any

from backend.sensor_contract import normalise_sample


def parse_sensor_line(line: str) -> dict[str, Any] | None:
    """
    Parse one newline-terminated JSON object.

    Returns:
        Normalised sensor dict; ``{"_error": msg}`` for firmware error payloads;
        ``None`` for empty lines / malformed JSON (caller skips silently).

    Examples:
        >>> parse_sensor_line('{"t":1,"r":0,"p":0,"y":0}')
        {'t': 1, 'r': 0.0, 'p': 0.0, 'y': 0.0, 'gx': 0.0, 'gy': 0.0, 'gz': 0.0}
    """
    stripped = line.strip()
    if not stripped:
        return None

    try:
        obj = json.loads(stripped)
    except json.JSONDecodeError:
        return None

    if not isinstance(obj, dict):
        return None

    err = obj.get("error")
    if err is not None:
        return {"_error": str(err)}

    try:
        return normalise_sample(obj)
    except ValueError:
        return None
