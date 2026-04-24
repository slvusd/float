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

One packet every 5 seconds throughout the dive:
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

---

## Network Architecture

There are two computers involved on competition day:

```
[Shore laptop / ROV controller]          [Float — inside the float]
  192.168.3.46  port 5001                  192.168.3.120  port 5000
  controller.py                            server.py + depthadjust.py

  Web UI at http://192.168.3.46:5001  ←→  REST API at http://192.168.3.120:5000
  Proxy:  /float/<endpoint>          ────→ forwards to float
  POST /receive  ←────────────────────────  transmitData() after mission
  POST /fetch    ──────────────────────────→ pulls data.csv on demand
```

**Both machines need to be on the same WiFi network** (192.168.3.x).
During the dive the float is underwater and has no WiFi — that is expected and fine.

---

## How Data Gets to the Controller

This is the most important thing to understand about the system.

### During the dive
The float logs packets to `data.csv` continuously throughout the mission.
It cannot transmit yet — it is underwater and WiFi does not reach.

### After profiles complete
The float stops its actuator and immediately begins a **transmission retry loop**:

1. Every **10 seconds**, it tries to POST `data.csv` to the controller.
2. Each attempt has a **5-second timeout** so it fails fast without hanging.
3. The loop prints `no link (ConnectionError) — still underwater?` on every failure — **this is normal and expected while the float is underwater**.
4. When the ROV retrieves the float and brings it to the surface, WiFi reconnects automatically.
5. The very next retry (within 10 seconds) succeeds with `SUCCESS (200)`.
6. The loop then switches to a **30-second heartbeat** — it keeps resending every 30 seconds as long as the float is powered.

**Why keep sending after success?** Debugging confidence. If you don't see data on the controller, you know exactly whether the float tried to send, whether WiFi connected, and whether the controller received it — without guessing.

**Why is sending duplicates okay?** The controller's `/receive` endpoint just overwrites `received_data.csv` each time. Identical data in, identical data out — no harm done.

### Backup: fetch from controller
If for any reason the float's push fails (wrong IP, network blip), a team member can click **"Fetch CSV from float"** on the controller UI. This makes the controller reach out and pull the data directly from the float.

---

## Configuring a Static IP on the Float

The float needs a fixed IP so the controller always knows where to reach it.
Use `nmcli` — it's already installed and manages WiFi on the Pi.

**Step 1 — find the connection name:**
```bash
nmcli connection show
# Look for the wifi connection — e.g. "GL-SFT1200-668"
```

**Step 2 — assign the static IP:**
```bash
sudo nmcli connection modify "YOUR-CONNECTION-NAME" \
  ipv4.method manual \
  ipv4.addresses "192.168.3.120/24" \
  ipv4.gateway "192.168.3.1" \
  ipv4.dns "192.168.3.1"

sudo nmcli connection up "YOUR-CONNECTION-NAME"
```

**Step 3 — verify:**
```bash
ip addr show wlan0   # should show 192.168.3.120
```

This persists across reboots. To revert to DHCP:
```bash
sudo nmcli connection modify "YOUR-CONNECTION-NAME" ipv4.method auto
sudo nmcli connection up "YOUR-CONNECTION-NAME"
```

The float's web UI shows its current IP and whether the controller is reachable — check this before deployment.

---

## Pre-Deployment Checklist

```
□ 1.  git pull on float Pi, sudo systemctl restart float
□ 2.  git pull on controller Pi, sudo systemctl restart float-controller
□ 3.  Open browser → http://192.168.3.46:5001  (controller UI)
□ 4.  Confirm float shows "reachable" in controller UI
□ 5.  Run test_extend.py on float — piston must extend (go out)
□ 6.  On controller UI (or float UI): click "Calibrate bias"
       Float must be at the surface in air. Expect bias ≈ –0.09 to –0.12 m.
□ 7.  Place float in water — should float with some freeboard
□ 8.  Click "Start Mission" — watch packets_logged increment
□ 9.  Float runs both profiles autonomously (~8–10 minutes)
□ 10. Float stops actuator and begins transmission retry loop (normal to see failures)
□ 11. ROV retrieves float, brings to surface
□ 12. Within 10 seconds: controller shows "SUCCESS" in float logs
□ 13. Click "Plot" on controller UI — verify two U-shapes
□ 14. Click "Download CSV" — save for judges
```

---

## Setup

### Float (192.168.3.120)

```bash
git clone git@github.com:slvusd/float.git
cd float
bash install.sh
```

`install.sh` installs all packages via `apt`, creates a Python venv, and enables the `float` systemd service so the REST API starts on every boot.

```bash
sudo systemctl status float          # check
sudo systemctl restart float         # restart after git pull
sudo journalctl -u float -f          # live logs
```

### Controller (192.168.3.46)

```bash
git clone git@github.com:slvusd/float.git
cd float
bash install_controller.sh
```

`install_controller.sh` does the same for the controller service on port 5001.

```bash
sudo systemctl status float-controller
sudo systemctl restart float-controller
sudo journalctl -u float-controller -f
```

---

## Web UIs

| URL | What it is |
|-----|-----------|
| `http://192.168.3.46:5001` | **Controller UI** — use this on competition day |
| `http://192.168.3.46:5001/float/` | Float UI proxied through controller |
| `http://192.168.3.120:5000` | Float UI direct (same network only) |

---

## REST API — Float (port 5000)

### Actuator
| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/extend?duration=20` | Extend actuator N seconds |
| `POST` | `/retract?duration=20` | Retract actuator N seconds |
| `POST` | `/stop` | Stop actuator immediately |

### Depth Sensor
| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/depth` | Raw depth, bias-corrected depth, pressure in kPa |

### Bias
| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/bias` | Return current bias (meters) |
| `GET` | `/bias?value=-0.0983` | Set bias to value |
| `POST` | `/calibrate` | Auto-calibrate: 10 readings at surface |

Bias is persisted to `bias.json` and survives reboots.

### Mission
| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/start` | Run `depthadjust.py` as subprocess |
| `GET` | `/status` | Mission running, bias, packets logged |
| `GET` | `/data` | Download `data.csv` |
| `GET` | `/plot` | Download depth-vs-time JPG |

---

## REST API — Controller (port 5001)

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/` | Controller web UI |
| `GET/POST` | `/float/<path>` | Proxy — forwards to float |
| `POST` | `/receive` | Float pushes CSV here after mission |
| `POST` | `/fetch` | Controller pulls CSV from float on demand |
| `GET` | `/data` | Download received CSV |
| `GET` | `/plot` | Generate and download JPG plot |

---

## Test Scripts

Run these **before** deploying. Stop the float service first if needed:
`sudo systemctl stop float`

| Script | Command | What it checks |
|--------|---------|----------------|
| Piston position | `python test_extend.py` | Piston must go **out** — if it retracts, swap pins in `config.py` |
| Actuator both ways | `python test_actuator.py [--duty 50] [--duration 8]` | Extend then retract, optional PWM |
| Depth sensor | `python test_depth.py [--bias -98.3]` | Continuous mm readings; ≈ 0 mm in air with correct bias |

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
| `server.py` | Float REST API + web UI (port 5000) |
| `controller.py` | Controller proxy + data receiver (port 5001) |
| `depthadjust.py` | Mission script — profiles, logging, transmission retry loop |
| `depthdetect.py` | MS5837 sensor wrapper |
| `actuator.py` | H-bridge / actuator control |
| `config.py` | Pin assignments, mission params, network IPs/ports |
| `ms5837/` | Vendored Blue Robotics MS5837 library |
| `float.service` | systemd template for float |
| `controller.service` | systemd template for controller |
| `install.sh` | Float setup: apt, venv, systemd |
| `install_controller.sh` | Controller setup: apt, venv, systemd |
| `example_plot.jpg` | Example mission output plot |
| `bias.json` | Persisted depth bias (runtime) |
| `data.csv` | Mission data log (runtime, overwritten each run) |
| `received_data.csv` | Data received by controller (runtime) |
