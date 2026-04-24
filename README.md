# RN08 Float
**Team RN08** — Autonomous vertical profiler for MATE ROV competition.

---

## MATE ROV Mission Requirements

The float must autonomously execute **two complete vertical profiles** after deployment.
Surfacing on its own is a **5-point penalty** — the float must wait to be recovered by the ROV.

### Each Profile

| Step | Target | Valid window | Requirement |
|------|--------|-------------|-------------|
| Descend | 2.5 m | 2.17 – 2.83 m (±33 cm) | Reach target before holding |
| Hold bottom | 2.5 m | 2.17 – 2.83 m | Log **7 packets** at **5-second intervals**; clock resets if depth drifts out |
| Ascend | 0.4 m | 0.07 – 0.73 m (±33 cm) | Must not break the surface |
| Hold surface | 0.4 m | 0.07 – 0.73 m | Log **7 packets** at **5-second intervals**; same clock-reset rule |

**Minimum packets required:** 7 per hold × 2 holds × 2 profiles = **28 packets total** (20 minimum per manual).

### Data Packet Format

One packet every 5 seconds, continuously throughout the dive:
```
RN08  HH:MM:SS  XXXX.X kPa  X.XX meters
```
Fields: team number · local time · pressure in kPa · depth in meters.

### Plot Requirements
- X axis: elapsed time (seconds)
- Y axis: depth (meters, **0 at top** — oceanographic convention)
- Must include **≥ 20 data points** across both profiles
- Must be **computer-generated** and saved as a JPG
- Should show two distinct "U" shapes

### Example Plot

![Example output](example_plot.jpg)

### After the Dive
Float holds position passively. The ROV physically recovers it and brings it to the surface.
Data is **downloaded on the pool deck** via the REST API — not transmitted during the dive.

---

## Pre-Deployment Checklist

```
□ 1. Run test_extend.py  — piston fully extended (maximum buoyancy)
□ 2. Open browser → http://<pi-ip>:5000
□ 3. Click "Calibrate bias"  — float must be at the surface in air
□ 4. Confirm bias reads ≈ –0.09 to –0.12 m (air reading is expected negative)
□ 5. Place float in water — should float with some freeboard
□ 6. Click "Start Mission"
□ 7. Watch packets_logged increment on status page
□ 8. After ROV recovery: open browser, click "Download CSV" and visit /plot
```

---

## Setup

```bash
git clone git@github.com:slvusd/float.git
cd float
bash install.sh
```

`install.sh` does everything in one step:
1. Installs all packages via `apt` (no internet required after first run)
2. Creates a Python venv
3. Installs and enables the `float` systemd service — server starts on boot

```bash
sudo systemctl status float        # check server
sudo systemctl restart float       # restart after git pull
sudo journalctl -u float -f        # live logs
```

---

## Web UI

Open `http://<pi-ip>:5000` in any browser on the same network.

The dashboard shows live depth and pressure, mission status, packet count,
action buttons (calibrate, start, extend, retract, stop), CSV download,
and the depth plot from the most recent run.

---

## REST API

All endpoints return JSON unless noted.

### Actuator

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/extend?duration=20` | Extend actuator N seconds (default 20) |
| `POST` | `/retract?duration=20` | Retract actuator N seconds (default 20) |
| `POST` | `/stop` | Stop actuator immediately |

### Depth Sensor

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/depth` | Raw depth, bias-corrected depth, and pressure in kPa |

### Bias

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/bias` | Return current bias (meters) |
| `GET` | `/bias?value=-0.0983` | Set bias to value |
| `POST` | `/bias` body `{"value": -0.0983}` | Set bias (programmatic) |
| `POST` | `/calibrate` | Auto-calibrate: 10 readings at surface → stores bias |

Bias is persisted to `bias.json` and survives reboots.
`depthadjust.py` reads it automatically on startup — no need to recalibrate between power cycles.

### Mission

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/start` | Run `depthadjust.py` as a subprocess |
| `GET` | `/status` | Mission running, bias value, packets logged so far |

### Data

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/data` | Download `data.csv` |
| `GET` | `/plot` | Download depth-vs-time JPG plot |

---

## Test Scripts

Run these **before** deploying. Stop the server first (`sudo systemctl stop float`) if GPIO conflicts occur.

| Script | Command | What it checks |
|--------|---------|----------------|
| Piston position | `python test_extend.py` | Extends 10 s at full speed — piston must go **out** (not in) |
| Actuator both ways | `python test_actuator.py [--duty 50] [--duration 8]` | Extend then retract; optional PWM speed |
| Depth sensor | `python test_depth.py [--bias -98.3]` | Continuous depth in mm; should read ≈ 0 in air with correct bias |

**Direction check:** if `test_extend.py` retracts instead of extends, swap `ACTUATOR_PIN_NEGATIVE_BCM` and `ACTUATOR_PIN_POSITIVE_BCM` in `config.py`.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Pi Zero 2 W | BCM pin mode |
| MS5837-02BA | Pressure/depth sensor (I2C bus 1) |
| H-bridge enable | GPIO 27 |
| H-bridge positive | GPIO 23 |
| H-bridge negative | GPIO 24 |

---

## Files

| File | Purpose |
|------|---------|
| `server.py` | Flask REST API + web UI |
| `depthadjust.py` | Mission script — run directly or via `/start` |
| `depthdetect.py` | MS5837 sensor wrapper |
| `actuator.py` | H-bridge / actuator control |
| `config.py` | All pin assignments, mission parameters, file paths |
| `ms5837/` | Vendored Blue Robotics MS5837 library |
| `float.service` | systemd service template (paths filled by `install.sh`) |
| `install.sh` | Full setup: apt packages, venv, systemd service |
| `requirements.txt` | Package list (all via apt) |
| `example_plot.jpg` | Example mission output plot |
| `bias.json` | Persisted depth bias (created at runtime) |
| `data.csv` | Mission data log (created at runtime, overwritten each run) |
