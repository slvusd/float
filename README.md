# float
**Team RN08** — Autonomous vertical profiler for MATE ROV competition.

## Setup

```bash
bash install.sh
```

`install.sh` does everything:
1. Installs system packages via `apt`
2. Creates a Python venv and installs `flask` + `matplotlib`
3. Installs and enables the `float` systemd service so the REST API starts on boot

Requires internet access on the first run (for pip packages).

After setup, manage the server with:
```bash
sudo systemctl status float
sudo systemctl restart float
sudo journalctl -u float -f      # live logs
```

---

## Hardware

| Component | Detail |
|-----------|--------|
| Pi Zero 2 W | BCM pin mode |
| MS5837-02BA | Pressure/depth sensor (I2C bus 1) |
| H-bridge | Enable: GPIO 27, Positive: GPIO 23, Negative: GPIO 24 |

---

## REST API

The server runs on port **5000**. All endpoints return JSON unless noted.

### Actuator

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/extend?duration=20` | Extend actuator for N seconds (default 20) |
| `POST` | `/retract?duration=20` | Retract actuator for N seconds (default 20) |
| `POST` | `/stop` | Stop actuator immediately |

### Depth Sensor

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/depth` | Read current depth and pressure |

Response:
```json
{"raw_m": -0.0983, "bias_m": -0.0983, "depth_m": 0.0, "pressure_kpa": 101.4}
```

### Bias

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/bias` | Return current bias |
| `GET` | `/bias?value=-0.0983` | Set bias to value (meters) |
| `POST` | `/bias` body `{"value": -0.0983}` | Set bias (programmatic) |
| `POST` | `/calibrate` | Auto-calibrate: reads sensor 10× at surface, stores result |

Bias is persisted to `bias.json` and survives reboots.

### Mission

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/start` | Start the mission (runs `depthadjust.py` as subprocess) |
| `GET` | `/status` | Mission running, bias, packets logged so far |

### Data

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/data` | Download `data.csv` |
| `GET` | `/plot` | Download depth-vs-time plot as JPG |

---

## Data Packet Format

One packet every 5 seconds throughout the dive:

```
RN08  HH:MM:SS  XXXX.X kPa  X.XX meters
```

Packets are written to `data.csv` with columns:
`elapsed_s, depth_m, pressure_kpa, packet`

---

## Plot

`GET /plot` returns a JPEG with:
- X axis: elapsed time (seconds)
- Y axis: depth (meters, 0 at top — oceanographic convention)
- Red dashed line at 2.5 m (bottom target)
- Green dashed line at 0.4 m (surface target)
- Should show two "U" shapes across the two profiles

Requires at least 2 data points to generate.

---

## Mission Behavior

### Pre-deployment
1. Run `python test_extend.py` to ensure piston is fully extended (maximum buoyancy)
2. Call `POST /calibrate` while float is at the surface in air
3. Call `POST /start` to begin the mission

### Two Vertical Profiles

**The float does not surface on its own — surfacing is a 5-point penalty.**

Each profile:

1. **Descend to 2.5 m** — valid window 2.17–2.83 m (±33 cm, bias-adjusted)
2. **Hold at 2.5 m for 30 s** — must log 7 packets at 5 s intervals; clock resets if depth drifts out of range
3. **Ascend to 40 cm** — valid window 0.07–0.73 m; must not break surface
4. **Hold at 40 cm for 30 s** — same 7-packet / clock-reset rules

### After Two Profiles
Actuator stops. Float holds position passively and waits for ROV recovery.

### Data Transmission (on deck)
After recovery, download `data.csv` and the plot JPG via the REST API from the pool deck.

---

## Test Scripts

| Script | Purpose |
|--------|---------|
| `python test_extend.py` | Extend actuator 10 s — pre-deployment position check |
| `python test_actuator.py [--duty 50] [--duration 8]` | Exercise actuator both directions with optional PWM |
| `python test_depth.py [--bias -98.3]` | Read depth sensor continuously in mm |

---

## Files

| File | Purpose |
|------|---------|
| `server.py` | Flask REST API server |
| `depthadjust.py` | Mission script — run directly or via `/start` |
| `depthdetect.py` | MS5837 sensor wrapper |
| `actuator.py` | H-bridge / actuator control |
| `config.py` | All pin assignments, mission parameters, file paths |
| `ms5837/` | Vendored Blue Robotics MS5837 library |
| `float.service` | systemd service template |
| `install.sh` | Full setup: apt, venv, pip, systemd |
| `requirements.txt` | pip dependencies |
| `bias.json` | Persisted depth bias (created at runtime) |
| `data.csv` | Mission data log (created at runtime) |
