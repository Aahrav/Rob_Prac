# MVP Software Specification (No Hardware)

Parallel implementation guide for **three independent tracks** when **no ESP32/USB serial** is available. Work merges cleanly if everyone respects the **shared data contract** and **file ownership** rules below.

---

## 1. Purpose & scope

**In scope:** Parser, complementary filter, CSV record/replay, Qt workers emitting a unified signal shape, CLI flags, calibration UI, packets/sec, disconnected overlay, simulate/replay modes, software test checklist, README polish.

**Out of scope for this document:** Firmware changes, COM port bring-up, wiring, hardware demo kit. When hardware exists later, **SerialWorker** is added as another producer of the same `SensorReading` dict (see §10).

**Goal:** One pipeline for all sources:

```text
Producer (SimWorker | ReplayWorker | [future SerialWorker])
  → validate/normalise dict (optional duplicate guard)
  → apply calibration offsets (Part 3-owned state, applied in MainWindow or thin controller)
  → filter.update() → filtered roll, pitch, yaw
  → compute_arm_positions(...)
  → ArmCanvas + DataPanel
```

---

## 2. Shared vocabulary

| Term | Meaning |
|------|---------|
| **Producer** | Any `QObject` + `QThread` worker that emits timed sensor dicts |
| **Sensor dict** | One logical sample after JSON parse or CSV row decode |
| **Pipeline** | Parser (optional if producer already builds dicts), filter, kinematics, UI updates |

---

## 3. Frozen data contract (all parts MUST match)

### 3.1 Canonical keys (Python dict after parse)

| Key | Type | Required | Notes |
|-----|------|----------|--------|
| `t` | `int` | yes | Monotonic-ish timestamp **milliseconds** (Arduino `millis()` or CSV column) |
| `r` | `float` | yes | Roll (degrees) |
| `p` | `float` | yes | Pitch (degrees) |
| `y` | `float` | yes | Yaw (degrees) |
| `gx` | `float` | optional until filter needs gyros | Gyro deg/s |
| `gy` | `float` | optional | |
| `gz` | `float` | optional | |

### 3.2 Error object (single sample)

```json
{"error":"sensor_init_failed"}
```

Part 1 parser **must** surface this as a structured event (exception or `{"_error": "sensor_init_failed"}`) so Part 3 can show UI without crashing. **Do not** mix error rows into the filter as ordinary angles.

### 3.3 CSV format (replay/record)

**Header row (exact names preferred):**

```text
t,r,p,y,gx,gy,gz
```

- `t` in milliseconds.
- Omit `gx,gy,gz` columns only if filter is temporarily bypassed for that file; Part 1 filter **requires** gyros when enabled — generate synthetic gyros in fixtures if needed (`0.0` acceptable for coarse testing).

**Encoding:** UTF-8, one row per line, `\n` newline.

### 3.4 Versioning the contract

If any track changes keys or units, they **must** bump **`CONTRACT_VERSION`** in one shared module (see §7.1) and notify other tracks the same day.

---

## 4. File ownership (avoid merge conflicts)

| Path / area | Owner |
|-------------|--------|
| `backend/parser.py` (new) | Part 1 |
| `backend/filter.py` (new) | Part 1 |
| `backend/replay_io.py` or `backend/recording.py` (new) | Part 1 |
| `backend/sensor_contract.py` (new, constants + tiny helpers) | Part 1 **only** — others import, do not duplicate literals |
| `backend/tests/*.py` (new) | Part 1 |
| `recordings/fixtures/*.csv` | Part 1 |
| `backend/serial_worker.py` (new, optional stub) | Part 2 |
| `backend/replay_worker.py` or under `frontend/` per project convention | Part 2 |
| `run.py`, `frontend/app.py` CLI parsing | Part 2 |
| `frontend/main_window.py` wiring | Part 2 coordinates **slots**; Part 3 does **layout/strings/styles** in panels |
| `frontend/panels/connection_panel.py` | Part 3 (labels/modes); Part 2 connects signals only |
| `frontend/panels/data_panel.py` | Part 3 |
| `frontend/panels/arm_canvas.py` overlay | Part 3 |
| `docs/test_checklist_software.md` | Part 3 |
| `README.md` | Part 3 (content); Part 2 adds CLI examples |

**Rule:** Part 2 **does not** edit `filter.py` internals. Part 1 **does not** edit `main_window.py` except at integration PR if explicitly agreed.

---

## 5. Part 1 — Core pipeline (pure Python)

### P1-T1 — Shared contract module

**Description:** Add `backend/sensor_contract.py` defining:

- `CONTRACT_VERSION = 1`
- `REQUIRED_KEYS = ("t", "r", "p", "y")`
- `OPTIONAL_GYRO_KEYS = ("gx", "gy", "gz")`
- `def normalise_sample(d: dict) -> dict` — casts numeric strings from CSV to float/int where needed; raises `ValueError` if required keys missing.

**Context:** Part 2 producers must either emit dicts already satisfying this or pass strings through Part 1 parser. Part 3 never redefines key names.

**Deliverable:** Module importable with no Qt dependency.

**Conflicts prevented:** Single source of truth for column names and keys.

---

### P1-T2 — Line parser

**Description:** Add `backend/parser.py`:

- `def parse_sensor_line(line: str) -> dict | None`
  - Strip whitespace; skip empty lines.
  - `json.loads` for `{...}` lines.
  - If `"error"` key present, return `{"_error": str}` — **not** passed to filter as angles.
  - Else call `normalise_sample`; return dict or `None` on failure (malformed JSON).

**Context:** Future `SerialWorker` will feed raw UTF-8 lines here. `ReplayWorker` may decode CSV rows directly to dicts **without** this parser — but replay-generated dicts must still satisfy `normalise_sample`.

**Deliverable:** Pure functions + docstrings.

---

### P1-T3 — Complementary filter

**Description:** Add `backend/filter.py` with a **stateful** class, e.g. `ComplementaryFilter`:

- **Inputs per step:** `t_ms`, `r, p, y` (degrees), `gx, gy, gz` (deg/s), previous internal state.
- **Formula (per axis):** maintain angle estimate;  
  `angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle`  
  Use roadmap semantics; document `alpha` default (e.g. `0.98`) in module docstring.
- **Accel angles:** Derive roll/pitch from gravity components if Part 1 adds helper from `r,p,y` **or** accept that MVP uses **Euler passed as r,p,y** and gyro integration only — **pick one** and document:

  **Recommended MVP without IMU raw vectors:** treat incoming `r,p,y` as accel tilt reference each frame, fuse with gyro integration:

  `angle = alpha * (angle + gyro * dt) + (1 - alpha) * measured_angle`

  where `measured_angle` is the corresponding `r`, `p`, or `y` input.

- **API:**

```python
class ComplementaryFilter:
    def __init__(self, alpha: float = 0.98): ...
    def reset(self) -> None: ...
    def step(self, sample: dict) -> tuple[float, float, float]:
        """Returns (roll, pitch, yaw) degrees."""
```

**Context:** MainWindow (Part 2/3 integration) calls `step` once per received sample **after** calibration subtraction (subtract offsets from `r,p,y` **before** `step`, unless you document otherwise).

**Deliverable:** Implementation + clear docstring on axis convention.

**Conflicts prevented:** Part 3 must not embed filter math in UI; all fusion lives here.

---

### P1-T4 — CSV record helper

**Description:** Add `backend/recording.py` (name flexible):

- `def open_record_csv(path: Path) -> csv.DictWriter` context manager or file handle with header written once.
- `def append_sample(writer, sample: dict)` writes one row using contract keys.

**Context:** Part 2 CLI `--record` opens file at startup and appends each **producer** sample (dict after normalise). Use locking if producer thread writes — prefer **signal to MainWindow** that appends on GUI thread or use a **queue**.

**Recommendation:** Producer emits sample → MainWindow slot appends row (simplest, avoids thread file IO).

**Conflicts prevented:** Part 2 must not invent a second CSV schema; import header constant from `sensor_contract` or `recording.py`.

---

### P1-T5 — CSV replay reader

**Description:** Same module or `replay_io.py`:

- `def iter_replay_rows(path: Path) -> Iterator[dict]` yields normalised dicts in file order.
- `def compute_sleep_seconds(t_prev_ms, t_curr_ms) -> float` caps extreme gaps (e.g. max 500 ms) to avoid hanging.

**Context:** `ReplayWorker` (Part 2) calls `iter_replay_rows` and sleeps between emissions.

**Deliverable:** Works on fixtures under `recordings/fixtures/`.

---

### P1-T6 — Fixtures & unit tests

**Description:**

- Add `recordings/fixtures/synthetic_smooth.csv` (~5–10 s of data, includes `gx,gy,gz`, can be zeros).
- Add `backend/tests/test_parser.py`, `test_filter.py`, `test_replay_io.py` using `pytest`.

**Context:** CI optional for MVP; tests still protect Part 2 from breaking contracts.

**Conflicts prevented:** Part 2 must not commit CSVs that violate header rules without updating Part 1 tests.

---

## 6. Part 2 — Producers, threading, CLI

### P2-T1 — Unified producer signals (interface)

**Description:** Define a small protocol **documented** in `backend/sensor_contract.py` or `docs/MVP_SOFTWARE_PARALLEL_SPEC.md`:

- Every producer worker emits **identical** Qt signals, e.g.  
  `sample_received(dict)`, `producer_error(str)`, `producer_status(str)`  
  (exact names project-standardised).

**Context:** Part 3 connects MainWindow **once** to these signals regardless of Sim vs Replay.

**Conflicts prevented:** Part 3 does not branch on worker class type except for Connect button wiring.

---

### P2-T2 — ReplayWorker (`QThread`)

**Description:**

- Subclass `QThread` or use worker `QObject` moved to thread (either pattern OK).
- Loop: for each row from `iter_replay_rows`, emit `sample_received(dict)`, sleep using §P1-T5.
- Clean stop: flag checked each iteration; close file safely.
- Emit `producer_error` on missing file / bad header.

**Context:** Uses Part 1 replay reader only — **no copy-paste** of CSV logic.

**Conflicts prevented:** Part 1 owns CSV semantics; Part 2 owns threading only.

---

### P2-T3 — SimWorker (optional refactor)

**Description:** Replace inline `Simulator` in `main_window.py` with dedicated worker emitting **same signals** as ReplayWorker and emitting dicts:

```python
{"t": int(monotonic_ms), "r": ..., "p": ..., "y": ..., "gx": 0.0, "gy": 0.0, "gz": 0.0}
```

**Context:** Enables filter + record on simulated data without replay file.

**Conflicts:** Coordinate with Part 3 — they may still adjust UI labels; Part 2 removes duplicated timer logic from MainWindow.

---

### P2-T4 — argparse / entrypoint

**Description:** Extend `run.py` or `frontend/app.py`:

- `--replay path` → start app; auto-start replay after window shown OR require Connect — **document choice**.
- `--record path` → append all samples that pass validation through MainWindow pipeline.
- `--no-filter` optional flag if Part 1 filter needs bypass for debugging only.

**Context:** Part 3 updates README with exact commands.

**Conflicts prevented:** CLI parsing lives in **one file**; panels do not parse `sys.argv`.

---

### P2-T5 — Pipeline orchestration in MainWindow

**Description:** Single slot `_on_sample_received(sample: dict)`:

1. If `sample.get("_error")`, forward to status/UI (Part 3 overlay patterns).
2. Apply calibration offsets (stored floats on MainWindow — Part 3 UI writes them).
3. Call `filter.step(sample_adjusted)` → `(r,p,y)`.
4. Call existing `_on_data_received(r,p,y)` or equivalent + increment packet counter for Part 3.

**Context:** This is the **only** place filter is invoked for live UX.

**Conflicts prevented:** Part 1 filter not called from ArmCanvas or DataPanel directly.

**Recording:** If `--record`, append **either** raw dict before filter or filtered angles — **pick one** and document (recommended: store **raw** dict rows matching CSV contract for future replay fidelity).

---

### P2-T6 — SerialWorker stub (optional)

**Description:** Empty or minimal class raising `NotImplementedError` with docstring: “Hardware phase — implements same signals as ReplayWorker.”

**Context:** Prevents Part 3 redesign later.

---

## 7. Part 3 — UI, UX, documentation

### P3-T1 — Connection / mode clarity

**Description:** Update `connection_panel.py` (and strings) so modes are honest without hardware:

- e.g. **Simulate**, **Replay file…**, and disabled **USB Serial (coming soon)** or hidden until implemented.

**Context:** Part 2 wires “Replay” to file picker + starts `ReplayWorker`. Part 3 owns visuals only.

**Conflicts prevented:** Part 2 must not hardcode QLabel text; use constants or signals Part 3 defines.

---

### P3-T2 — Calibration

**Description:**

- Button **Calibrate**: capture current filtered **or raw** angles (document which); store `(off_r, off_p, off_y)` on MainWindow.
- Future samples: `r' = r - off_r` (same for p, y) **before** filter unless filter docs say otherwise — align with P2-T5.

**Context:** Works for Sim and Replay.

**Conflicts prevented:** Offsets owned by MainWindow; Part 1 stays stateless except filter internal state.

---

### P3-T3 — Packets per second

**Description:** Data panel shows rate: count samples in MainWindow over last 1 s wall clock (timer) or exponential moving average.

**Context:** Uses post-validation samples only.

---

### P3-T4 — Disconnected / idle overlay

**Description:** On `ArmCanvas`, when no producer connected or replay ended, draw semi-transparent `text2D` “No data — choose Simulate or Replay” (wording per theme).

**Context:** Hide when first sample arrives; show on disconnect/stop.

**Conflicts prevented:** Overlay logic in canvas; connection state from MainWindow slot calls `canvas.set_idle_message(...)`.

---

### P3-T5 — Status & errors

**Description:** QMessageBox or status bar for fatal replay errors; non-blocking status for skipped malformed lines (if surfaced from pipeline).

**Context:** Parser returning `None` may be silent; aggregate “parse_errors count” optional.

---

### P3-T6 — Software test checklist

**Description:** Add `docs/test_checklist_software.md`:

- Missing replay path, wrong CSV header, empty file, stop mid-replay, `--record` creates non-empty CSV, calibration resets behaviour, simulate + replay switch without restart (if supported).

---

### P3-T7 — README & Help

**Description:** README: install, `poetry run …`, examples `--replay recordings/fixtures/synthetic_smooth.csv`, screenshot. Menu: About + shortcuts (even if stub dialog).

---

## 8. Integration checkpoints

| Checkpoint | Verification |
|------------|----------------|
| **IC-1** | Import `sensor_contract`, parse one fixture line in REPL |
| **IC-2** | Replay fixture; arm moves; no Qt threading warnings |
| **IC-3** | `--record` produces CSV readable by `--replay` round-trip |
| **IC-4** | Filter on/off behaviour visible; calibration shifts neutral pose |
| **IC-5** | Checklist completed |

---

## 9. Ordering when merging PRs

1. Merge **Part 1** first (contract + parser + filter + IO + tests).
2. Merge **Part 2** (workers + MainWindow orchestration + CLI).
3. Merge **Part 3** (panels + overlay + docs).

If parallel: Part 3 can branch off Part 1 **interfaces** (mock slot signatures) until Part 2 lands.

---

## 10. Future hardware attach (non-blocking note)

Add `SerialWorker`:

- Read UTF-8 lines → `parse_sensor_line` → emit same `sample_received` dict as ReplayWorker.
- UI mode enables COM dropdown again.

No changes to Part 1 filter or Part 3 calibration logic required if contract holds.

---

## 11. Quick task index

| ID | Summary |
|----|---------|
| P1-T1 | `sensor_contract.py` |
| P1-T2 | `parser.py` |
| P1-T3 | `filter.py` |
| P1-T4 | `recording.py` append API |
| P1-T5 | replay iterator + sleep helper |
| P1-T6 | fixtures + pytest |
| P2-T1 | producer signal names documented |
| P2-T2 | `ReplayWorker` |
| P2-T3 | `SimWorker` refactor |
| P2-T4 | argparse |
| P2-T5 | MainWindow unified pipeline slot |
| P2-T6 | Serial stub (optional) |
| P3-T1 | mode labels / layout |
| P3-T2 | calibration |
| P3-T3 | packets/s |
| P3-T4 | idle overlay |
| P3-T5 | error/status UX |
| P3-T6 | software checklist md |
| P3-T7 | README + Help |

---

*Document version: 1.0 — aligns with roadmap MVP software items minus hardware/firmware.*
