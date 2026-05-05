#!/usr/bin/env python3
"""
replay_buffer.py — Generic, robot-agnostic frame store for animation replay.

Design principles:
  - No knowledge of robot models, analysis modes, or UI.
  - Stores generic Frame dataclasses (t, joints[], ee_pos[], torques[], metadata{}).
  - Fully serialisable to JSON (.rba) and CSV.
  - Can be constructed from the MainWindow telemetry dict or recorded live.
"""

from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Callable, Iterable, Iterator, List, Optional


# ──────────────────────────────────────────────────────────────────────────────
#  Frame — the atomic unit of recorded robot state
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class Frame:
    """
    A single captured snapshot of the robot at time t.

    Deliberately robot-agnostic:
      - joints: N floats (degrees for revolute, metres for prismatic)
      - ee_pos: [x, y, z] in metres
      - torques: N floats in Nm (or empty list if not computed)
      - metadata: any extra key/value pairs for extensibility
    """
    index: int
    t: float                           # seconds since recording started
    joints: List[float]                # variable length — works for any DOF
    ee_pos: List[float]                # always [x, y, z]
    torques: List[float] = field(default_factory=list)
    metadata: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict) -> "Frame":
        return Frame(
            index=int(d["index"]),
            t=float(d["t"]),
            joints=[float(v) for v in d["joints"]],
            ee_pos=[float(v) for v in d["ee_pos"]],
            torques=[float(v) for v in d.get("torques", [])],
            metadata=d.get("metadata", {}),
        )


# ──────────────────────────────────────────────────────────────────────────────
#  ReplayBuffer — ordered collection of Frames
# ──────────────────────────────────────────────────────────────────────────────

class ReplayBuffer:
    """
    An ordered, robot-agnostic store of animation frames.

    Usage
    -----
    # Record live:
    buf = ReplayBuffer()
    buf.record(Frame(index=0, t=0.0, joints=[0,0,0], ee_pos=[0.3,0,0.1]))

    # Build from existing telemetry dict:
    buf = ReplayBuffer.from_telemetry(main_window._telemetry)

    # Serialise:
    buf.save("session.rba")
    buf2 = ReplayBuffer.load("session.rba")
    """

    def __init__(self, name: str = ""):
        self._frames: List[Frame] = []
        self.name: str = name or f"recording_{int(time.time())}"
        self._recording_start: Optional[float] = None

    # ── Recording API ─────────────────────────────────────────────────────────

    def start_recording(self) -> None:
        """Reset buffer and mark start time for relative timestamps."""
        self._frames.clear()
        self._recording_start = time.monotonic()

    def record(self, frame: Frame) -> None:
        """Append a pre-built Frame."""
        self._frames.append(frame)

    def record_state(
        self,
        joints: List[float],
        ee_pos: List[float],
        torques: Optional[List[float]] = None,
        metadata: Optional[dict] = None,
    ) -> Frame:
        """
        Convenience recorder: auto-assigns index + timestamp.
        Returns the Frame that was appended.
        """
        t_now = time.monotonic()
        if self._recording_start is None:
            self._recording_start = t_now
        t_rel = t_now - self._recording_start
        idx = len(self._frames)
        frame = Frame(
            index=idx,
            t=t_rel,
            joints=list(joints),
            ee_pos=list(ee_pos),
            torques=list(torques) if torques else [],
            metadata=dict(metadata) if metadata else {},
        )
        self._frames.append(frame)
        return frame

    def clear(self) -> None:
        self._frames.clear()
        self._recording_start = None

    # ── Query API ─────────────────────────────────────────────────────────────

    def __len__(self) -> int:
        return len(self._frames)

    def __iter__(self) -> Iterator[Frame]:
        return iter(self._frames)

    def __bool__(self) -> bool:
        return len(self._frames) > 0

    def get_frame(self, idx: int) -> Frame:
        """Return frame at index, clamped to valid range."""
        if not self._frames:
            raise IndexError("ReplayBuffer is empty")
        idx = max(0, min(idx, len(self._frames) - 1))
        return self._frames[idx]

    @property
    def duration(self) -> float:
        """Total duration in seconds."""
        if len(self._frames) < 2:
            return 0.0
        return self._frames[-1].t - self._frames[0].t

    @property
    def fps(self) -> float:
        """Average frames per second."""
        if self.duration <= 0 or len(self._frames) < 2:
            return 0.0
        return (len(self._frames) - 1) / self.duration

    def slice(self, start: int, end: int) -> "ReplayBuffer":
        """Return a new ReplayBuffer with frames[start:end] (re-indexed)."""
        sliced = ReplayBuffer(name=f"{self.name}_slice")
        for new_idx, frame in enumerate(self._frames[start:end]):
            sliced._frames.append(Frame(
                index=new_idx,
                t=frame.t - self._frames[start].t,
                joints=list(frame.joints),
                ee_pos=list(frame.ee_pos),
                torques=list(frame.torques),
                metadata=dict(frame.metadata),
            ))
        return sliced

    def nearest_frame_to_time(self, t: float) -> int:
        """Return the index of the frame closest to time t."""
        if not self._frames:
            return 0
        best_idx = 0
        best_dt = abs(self._frames[0].t - t)
        for i, f in enumerate(self._frames):
            dt = abs(f.t - t)
            if dt < best_dt:
                best_dt = dt
                best_idx = i
        return best_idx

    # ── Serialisation API ─────────────────────────────────────────────────────

    def save(self, path: str | Path) -> None:
        """Save to a human-readable JSON .rba session file."""
        path = Path(path)
        data = {
            "version": 1,
            "name": self.name,
            "frame_count": len(self._frames),
            "duration": self.duration,
            "frames": [f.to_dict() for f in self._frames],
        }
        path.write_text(json.dumps(data, indent=2), encoding="utf-8")

    @staticmethod
    def load(path: str | Path) -> "ReplayBuffer":
        """Load a .rba JSON session file."""
        path = Path(path)
        data = json.loads(path.read_text(encoding="utf-8"))
        buf = ReplayBuffer(name=data.get("name", path.stem))
        for d in data.get("frames", []):
            buf._frames.append(Frame.from_dict(d))
        return buf

    def export_csv(self, path: str | Path) -> None:
        """Export all frames to a CSV file (one row per frame)."""
        path = Path(path)
        if not self._frames:
            return
        n_joints = len(self._frames[0].joints)
        n_torques = len(self._frames[0].torques)

        headers = (
            ["frame", "time_s"]
            + [f"j{i+1}" for i in range(n_joints)]
            + ["ee_x", "ee_y", "ee_z"]
            + [f"torque_{i+1}" for i in range(n_torques)]
        )
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            for frame in self._frames:
                row = (
                    [frame.index, round(frame.t, 6)]
                    + [round(v, 6) for v in frame.joints]
                    + [round(v, 6) for v in frame.ee_pos]
                    + [round(v, 6) for v in frame.torques]
                )
                writer.writerow(row)

    # ── Factory methods ───────────────────────────────────────────────────────

    @staticmethod
    def from_telemetry(tel: dict, name: str = "telemetry") -> "ReplayBuffer":
        """
        Build a ReplayBuffer from the MainWindow._telemetry dict format:
          { 't': [...], 'joints': [...], 'ee_pos': [...],
            'vel': [...], 'acc': [...], 'torques': [...] }
        """
        buf = ReplayBuffer(name=name)
        times = tel.get("t", [])
        joints_list = tel.get("joints", [])
        ee_list = tel.get("ee_pos", [])
        torques_list = tel.get("torques", [])
        vel_list = tel.get("vel", [])
        acc_list = tel.get("acc", [])

        n = min(len(times), len(joints_list), len(ee_list))
        t0 = times[0] if times else 0.0

        for i in range(n):
            meta: dict = {}
            if i < len(vel_list):
                meta["vel"] = vel_list[i]
            if i < len(acc_list):
                meta["acc"] = acc_list[i]
            frame = Frame(
                index=i,
                t=float(times[i]) - float(t0),
                joints=list(joints_list[i]),
                ee_pos=list(ee_list[i]),
                torques=list(torques_list[i]) if i < len(torques_list) else [],
                metadata=meta,
            )
            buf._frames.append(frame)

        return buf

    def __repr__(self) -> str:
        return (
            f"ReplayBuffer(name={self.name!r}, "
            f"frames={len(self)}, duration={self.duration:.2f}s)"
        )
