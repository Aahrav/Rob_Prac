# AppName — Real-Time Robotic Arm Simulation

**Team:** Krunal, Veer, Aarav, Sujal

## Setup

```bash
# Install dependencies with Poetry
poetry install

# Run the application
poetry run app
```

## Project Structure

- `backend/` — Serial communication, filtering, kinematics, data pipeline
- `frontend/` — PyQt6 GUI, 3D visualization (Matplotlib → VisPy)
- `firmware/` — ESP32 Arduino sketch for MPU-6050 glove
- `docs/` — Documentation

## Development Roadmap

See `AppName — Full Development Roadmap.pdf` for the complete 30-step plan from MVP to production.

## Step 1 — Current Status

- [ ] Hardware: ESP32 + MPU-6050 wired and printing raw values
- [ ] Backend: `config.py` and `serial_test.py` ready
- [ ] Frontend: `app.py` + `main_window.py` with two-panel layout
- [ ] Commit and tag checkpoint
