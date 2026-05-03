# RoboSim — Robotic Arm Simulation Platform

> **Status:** MVP Software Phase — hardware (ESP32 + MPU-6050) integration pending.

Real-time 3D simulation of a robotic arm driven by a sensor glove.
The full pipeline (sensor → filter → kinematics → visualisation) is built and
testable in software before any physical hardware is connected.

---

## Screenshots

Add `docs/screenshot.png` after capturing the running app (optional).

---

## Tech stack

| Layer | Library |
|-------|---------|
| UI | PyQt6 |
| 3D visualisation | Matplotlib (mpl_toolkits.mplot3d) |
| Kinematics | NumPy (custom FK/IK in `backend/kinematics.py`) |
| Serial (future) | pyserial |
| Dependency management | Poetry |

---

## Troubleshooting

- **Window flashes then closes:** often caused by extra flags on `sys.argv` confusing Qt when launching from an IDE. This build passes only the executable path into `QApplication`. Run `python run.py -v` from a terminal and inspect `logs/robosim.log` / `logs/crash.log`. Use `poetry install` from `Rob_Prac`.
- **Nothing in the console:** by default only WARNING+ is printed; use `-v` (INFO) or `--debug` (DEBUG).

## Installation

### Prerequisites

- Python **3.11+**
- [Poetry](https://python-poetry.org/docs/#installation)

### Clone & install

```bash
git clone https://github.com/<your-org>/robosim.git
cd robosim
poetry install
```

---

## Running the app

### Default (Simulate mode)

```bash
poetry run app
```

Opens the GUI. Select **Simulate** in the left panel and click **Connect** to
start the built-in motion simulator. The 3D arm animates immediately — no
hardware required.

### Replay a recorded CSV

```bash
poetry run app --replay recordings/fixtures/synthetic_smooth.csv
```

The app loads the file; select **Replay file…** in the connection panel,
browse to the CSV (or use the pre-loaded path from CLI), and click **Connect**.

### Record a session to CSV

```bash
poetry run app --record recordings/my_session.csv
```

Every validated sample is appended to the file while the app runs.
Stop recording by closing the app or disconnecting.

### Combine replay + record (round-trip test)

```bash
# Step 1 — record a simulated session
poetry run app --record /tmp/session.csv

# Step 2 — replay it back
poetry run app --replay /tmp/session.csv
```

## CSV data format

Files must be UTF-8, one row per line, with this exact header:

```
t,r,p,y,gx,gy,gz
```

| Column | Type | Unit | Required |
|--------|------|------|----------|
| `t` | int | milliseconds | ✅ |
| `r` | float | degrees (roll) | ✅ |
| `p` | float | degrees (pitch) | ✅ |
| `y` | float | degrees (yaw) | ✅ |
| `gx,gy,gz` | float | deg/s | optional |

---

## Project structure

```
robosim/
├── backend/
│   ├── kinematics.py          # FK + IK, ArmConfig, KinematicChain
│   ├── config.py              # Serial port defaults
│   ├── logger.py              # Rotating file logger
│   ├── sensor_contract.py     # Shared data-contract constants  [Part 1]
│   ├── parser.py              # Line parser (JSON / CSV rows)   [Part 1]
│   ├── filter.py              # Complementary filter             [Part 1]
│   ├── recording.py           # CSV record/replay helpers        [Part 1]
│   ├── replay_worker.py       # QThread replay producer          [Part 2]
│   └── serial_worker.py       # QThread serial stub              [Part 2]
├── frontend/
│   ├── main_window.py         # Main window, layout, calibration
│   └── panels/
│       ├── arm_canvas.py      # 3D Matplotlib canvas + idle overlay
│       ├── connection_panel.py# Mode pills: Simulate / Replay / USB Serial
│       ├── data_panel.py      # R/P/Y readout + PKT/S counter
│       ├── trajectory_panel.py
│       ├── robot_config_panel.py
│       └── kinematic_chain_panel.py
├── docs/
│   ├── MVP_SOFTWARE_PARALLEL_SPEC.md
│   └── test_checklist_software.md
├── recordings/
│   └── fixtures/
│       └── synthetic_smooth.csv   # ~5–10 s synthetic data  [Part 1]
├── pyproject.toml
└── run.py
```

---

## Tests

```bash
poetry install --with dev
poetry run pytest backend/tests/ -v
```

---

## Calibration

While connected (Simulate or Replay), click **⊕ Calibrate** in the
**Robot Parameters** sidebar section. The current roll/pitch/yaw is captured as
the zero-reference. All subsequent samples subtract the offsets so the arm
rests at the neutral pose. Offsets reset to zero when you disconnect.

---

## Help menu

In the running app: **Help → About RoboSim** (`F1`) or
**Help → Keyboard Shortcuts** (`Ctrl+/`).

---

## Roadmap

- [ ] Hardware phase: `SerialWorker` + COM port selection
- [ ] Dark-mode QSplitter handle drag indicator
- [ ] Export trajectory as CSV from Trajectory Control panel
