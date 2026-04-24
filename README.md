# float
ROV Float — autonomous vertical profiler for SLVUSD competition.

## Setup

```bash
bash install.sh
source venv/bin/activate
python depthadjust.py
```

## Hardware

- Raspberry Pi (BCM pin mode)
- MS5837-02BA pressure/depth sensor (I2C)
- Linear actuator on GPIO BCM 23 (negative) and 24 (positive)

## Mission Behavior

### Step 0 — Depth Sensor Bias Calibration
Before the float enters the water, it reads the pressure sensor 10 times at the surface
(true depth = 0 m) and computes an offset. All subsequent depth readings subtract this
offset to correct for sensor drift or pressure baseline error.

### Main Loop — Two Vertical Profiles
After deployment the float autonomously executes **two complete vertical profiles**, then
holds position and waits to be recovered by the team's ROV.

**The float does not surface on its own — surfacing is a 5-point penalty.**

Each profile:

1. **Descend to 2.5 m** (bottom of float)
   - Valid window: 2.17 – 2.83 m (±33 cm), adjusted for sensor offset
2. **Hold at 2.5 m for 30 seconds**
   - Must log 7 sequential packets at 5-second intervals while in range
   - If depth drifts outside ±33 cm, the 30-second clock and packet count reset
3. **Ascend to 40 cm** (top of float)
   - Valid window: 0.07 – 0.73 m (±33 cm), adjusted for sensor offset
   - Must not break the surface
4. **Hold at 40 cm for 30 seconds**
   - Same 7-packet / clock-reset rules apply

### After Two Profiles
The float stops its actuator and waits passively for the ROV to physically recover it.

### Data Transmission (on deck)
After recovery and out of the water, the float transmits all logged data packets
wirelessly to the shore receiver. Data transmission happens on the pool deck, not
during the dive.

## Files

| File | Purpose |
|------|---------|
| `depthadjust.py` | Main mission script — bias calibration, profile execution, data logging |
| `depthdetect.py` | MS5837 sensor wrapper — init, depth readings in m / cm / mm |
| `actuator.py` | Linear actuator control — setup, extend, retract, stop |
| `gfpressuresensor.py` | Standalone sensor test/debug script |
| `two_lights.py` | GPIO LED test script |
| `install.sh` | Creates venv and installs dependencies |
| `requirements.txt` | Python dependencies |
