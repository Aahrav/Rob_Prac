# RoboSim — Software Test Checklist

Manual test cases for the **MVP software-only phase** (no hardware required).
Run these before every integration PR merge.

> **Legend:**  ☐ Not tested &nbsp;|&nbsp; ✅ Pass &nbsp;|&nbsp; ❌ Fail &nbsp;|&nbsp; ⚠️ Partial

---

## TC-01 — Simulate mode: basic smoke test

| Step | Expected |
|------|----------|
| Launch app (`poetry run app`) | Window opens; idle overlay visible on 3D canvas |
| Click **Simulate** pill (default) | Pill turns blue; file row hidden |
| Click **● Connect** | Status chip turns green "Simulation running"; arm starts moving; overlay disappears |
| Watch **PKT/S** in data panel | Shows ~20 pps (timer fires at 20 Hz) |
| Click **■ Disconnect** | Status "Disconnected"; overlay reappears |

**Pass criteria:** No Python exceptions; overlay, PKT/S, arm all behave as described.

---

## TC-02 — Replay file: missing / no path selected

| Step | Expected |
|------|----------|
| Click **Replay file…** pill | Pill turns blue; file-picker row appears |
| Click **● Connect** WITHOUT selecting a file | Status shows "No replay file selected — click Browse…"; no crash |
| Click **Browse…**, press Cancel | File label stays "No file selected"; Connect stays non-fatal |

**Pass criteria:** App does not crash; error is non-blocking.

---

## TC-03 — Replay file: wrong CSV header / malformed content

| Step | Expected |
|------|----------|
| Create a CSV with header `a,b,c` (wrong) | — |
| Select it via Browse…, click Connect | Fatal error QMessageBox appears: "Replay / Producer Error" |
| Dismiss dialog | Connection stopped; overlay visible; status "Error — connection stopped" |

> **Note:** This test becomes fully exercisable once Part 2's `ReplayWorker` emits `producer_error`. Until then verify the `show_producer_error(msg, fatal=True)` method manually in REPL.

**Pass criteria:** QMessageBox shown; app recovers without restart.

---

## TC-04 — Replay file: empty file

| Step | Expected |
|------|----------|
| Create `empty.csv` (zero rows, no header) | — |
| Select and Connect | Fatal error dialog; or immediate producer_error emitted |

**Pass criteria:** App does not hang; error surfaced within 1 s.

---

## TC-05 — Replay file: stop mid-replay

| Step | Expected |
|------|----------|
| Load valid `synthetic_smooth.csv`, Connect | Arm moves; PKT/S shows live rate |
| Click **■ Disconnect** mid-replay | Replay stops cleanly; overlay reappears; PKT/S drops to 0 after 1 s |
| Reconnect immediately | Works without restart |

**Pass criteria:** No zombie threads; PKT/S resets correctly.

---

## TC-06 — `--record` creates a non-empty CSV

| Step | Expected |
|------|----------|
| Run `poetry run app --record /tmp/test_rec.csv` | App starts normally |
| Connect in Simulate mode, wait 5 s | — |
| Disconnect, close app | — |
| Inspect `/tmp/test_rec.csv` | File exists; has header row + ≥1 data rows; columns match `t,r,p,y` |

> **Note:** Requires Part 2 `--record` CLI flag and `recording.py`.

**Pass criteria:** CSV non-empty, header correct, parseable by `iter_replay_rows`.

---

## TC-07 — Calibration: resets neutral pose

| Step | Expected |
|------|----------|
| Connect in Simulate mode | Arm moving; R/P/Y values live |
| Wait until arm is at some non-zero pose (e.g. R=+18°) | — |
| Click **⊕ Calibrate** | Status bar: "Calibrated — offsets R=+18.0° …"; calib status label updates |
| Observe arm in next frames | Arm shifts: previously +18° roll now reads ~0° after offset |
| Click Calibrate again at new position | Offsets update again |
| Disconnect, reconnect | Offsets reset to 0.0 / 0.0 / 0.0 (verify lbl_calib_status) |

**Pass criteria:** Offset subtraction visible in data panel; resets on reconnect.

---

## TC-08 — Simulate → Replay switch without restart

| Step | Expected |
|------|----------|
| Connect in Simulate mode | Arm animating |
| Disconnect | Overlay visible |
| Click Replay pill, browse valid CSV | File label shows filename |
| Click Connect | Replay starts; arm follows CSV data |
| Disconnect again | Overlay reappears |
| Switch back to Simulate, Connect | Simulation resumes |

**Pass criteria:** Entire flow without app restart; no exceptions.

---

## TC-09 — Idle overlay visibility lifecycle

| Step | Expected |
|------|----------|
| App launch (before any Connect) | Overlay **visible**: "No data — Choose Simulate or Replay" |
| Connect (any mode) | Overlay **hidden** as soon as first `draw_arm()` fires |
| Disconnect | Overlay **visible** again |
| Call `arm_canvas.set_idle_message(True)` in REPL | Overlay appears |
| Call `arm_canvas.set_idle_message(False)` | Overlay disappears |

**Pass criteria:** Overlay never flickers; `set_idle_message` is idempotent.

---

## TC-10 — PKT/S counter accuracy

| Step | Expected |
|------|----------|
| Connect Simulate (default 20 Hz timer) | PKT/S ≈ 20 after first 1-s window |
| Observe over 5 s | Stays in 18–22 range (QTimer jitter) |
| Disconnect | PKT/S drops to 0 after ≤ 2 s (next timer tick) |
| Replay a CSV with ~10 rows/s | PKT/S ≈ 10 |

**Pass criteria:** Rate matches expected frequency; resets on disconnect.

---

## TC-11 — Non-fatal parse warnings in status bar

| Step | Expected |
|------|----------|
| Replay a CSV that has 1 malformed line mid-file | Status bar shows "⚠ …" in amber for 5 s |
| After 5 s | Status bar colour resets to grey (non-blocking) |
| Replay continues | Malformed line skipped; arm keeps moving |

> **Note:** Requires Part 2 `producer_error` signal (non-fatal variant).

**Pass criteria:** No modal dialog; replay not interrupted.

---

## TC-12 — Help menu

| Step | Expected |
|------|----------|
| Open **Help → About RoboSim** (or press F1) | About dialog with version, stack info |
| Open **Help → Keyboard Shortcuts** (or Ctrl+/) | Shortcuts dialog listing stub keys |
| Press Escape or OK | Dialog closes; app continues normally |

**Pass criteria:** Both dialogs open and close cleanly.

---

## Integration checkpoint cross-reference

| IC | Checklist TCs |
|----|---------------|
| IC-1 (contract importable) | (pytest — Part 1) |
| IC-2 (replay; arm moves) | TC-05, TC-08 |
| IC-3 (record → replay round-trip) | TC-06 |
| IC-4 (calibration shifts pose) | TC-07 |
| IC-5 (checklist completed) | **this file** |
