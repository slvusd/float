# Float Tuning Guide
**Team RN08** — How to test, tune, and validate the float before competition.

---

## How the Float Works (Quick Physics)

The float uses a syringe connected to the water. A linear actuator pushes or pulls the plunger:

| Action | Effect | Float does |
|--------|--------|-----------|
| **Retract** (pull plunger in) | More water enters → float gets heavier | Sinks ↓ |
| **Extend** (push plunger out) | Water pushed out → float gets lighter | Rises ↑ |

The control loop runs every 100 ms:
1. Read depth from pressure sensor
2. Subtract bias offset to get true depth
3. If more than `CONTROL_DEADBAND_M` above target → retract (sink)
4. If more than `CONTROL_DEADBAND_M` below target → extend (rise)
5. If within deadband → stop actuator

---

## Finding the Tuning Page

All tuning happens from **`http://192.168.3.46:5001/tuning`** (controller).
You can also reach it from the float directly at `http://192.168.3.120:5000/tuning`.

The tuning page has sliders and inputs for every parameter and three run buttons:

| Button | What it does |
|--------|-------------|
| **🔬 Sim Run (dry)** | Mock sensor + real actuator. No water needed. Use this first. |
| **🧪 Test Run** | Real sensor + real actuator in water. Float surfaces at end. |
| **▶ Start mission** | Competition run. Float waits at depth for ROV recovery. |

Parameters set on the tuning page apply **only to that run** — they do not change `config.py`.
`config.py` holds the permanent defaults.

---

## Simulation Mode (No Water Needed)

**Use this before every pool session** to verify the full system works on the bench.

### What it does
- **Depth sensor is replaced** with a virtual model that tracks depth based on actuator direction
- **Real actuator GPIO still fires** — the dry syringe actually moves
- All data logging, archiving, and transmission to the controller run exactly as in a real mission
- Float surfaces at the end (sim always implies test mode)

### How to run it
1. Go to the tuning page (`/tuning`)
2. Adjust the **Sim rate** slider (see below)
3. Click **🔬 Sim Run (dry)**
4. Watch the live depth change on the right side of the page
5. When done, the plot appears and data flows to the controller

### Sim Rate (m/s)

| Rate | Time to reach 2.5 m | Use when |
|------|---------------------|---------|
| 0.08 (default) | ~30 s | Normal bench testing |
| 0.2 | ~12 s | Quick data-flow verification |
| 0.5 | ~5 s | Just checking the UI works |

### What a good sim run looks like
- Depth increases steadily during descent (slight noise is normal and intentional)
- Depth holds near the target with small corrections — you can hear the motor ticking
- 7 packets logged at each hold, no clock resets
- Controller receives data, plot shows two U-shapes
- `float.log` at `/log` shows `SIM MODE` at the top and clean progression

### What sim cannot tell you
- Whether the float is neutrally buoyant in water
- Whether the syringe displaces enough water to reach 2.5 m
- How the actuator behaves under water pressure
- Actual descent/ascent speed

---

## Parameters to Tune

### `ACTUATOR_DUTY_CYCLE` (default: 80)

Motor speed. Set on the tuning page slider or in `config.py`.

| Value | Effect | Use when |
|-------|--------|---------|
| 100 | Full speed | Bench/sim only |
| 80 | Fast but controllable | Good starting point |
| 50–60 | Slower, less overshoot | Float blows past the target |
| < 50 | Very slow | May stall under water pressure |

**Signs you need to lower it:**
- Float blows past 2.5 m and takes a long time to correct
- Plot shows dips well below 2.5 m before settling
- Motor is still running when it should be holding

**Signs you need to raise it:**
- Descent takes more than 3 minutes
- Motor stalls partway (hums but float doesn't move)

### `CONTROL_DEADBAND_M` (default: 0.03 — 3 cm)

The stop zone around the target. Tune this after duty cycle.

| Value | Effect |
|-------|--------|
| 0.01 m | Very tight — chases target aggressively; may hunt |
| 0.03 m | Default — good balance |
| 0.08 m | Relaxed — less motor activity, more depth variation |

**Signs it is too small (hunting):**
- Actuator switches between extend and retract rapidly
- Motor is constantly running during the hold phase
- Depth trace zigzags around the target

**Signs it is too large:**
- Float drifts 10+ cm from target and hold clock keeps resetting
- Packets logged infrequently

### `SENSOR_DEPTH_OFFSET_M` (default: 0.0)

Physical distance from the pressure sensor to the float's bottom face (the competition reference point). Measure this once with a ruler and set it in `config.py` permanently.

**Example:** sensor is 15 cm above the bottom → `SENSOR_DEPTH_OFFSET_M = 0.15`

When set, the float automatically targets `competition_depth - offset` so the bottom of the float reaches the correct competition depth.

You can also override it per-run on the tuning page under **Depth Settings**.

### Target Bottom / Target Surface (tuning page only)

Sliders on the tuning page let you override the competition target depths for a single run. Use these when:
- Your test pool is not deep enough for 2.5 m → lower the bottom target
- You want a quick test at a shallower depth

The page shows the computed **effective sensor targets** (competition target − sensor offset) so you always know exactly what the sensor will aim for.

### `TEST_SURFACE_DELAY_S` (default: 60)

Seconds to wait at the final position before surfacing (test/sim mode).
Keep at 60–120 s — long enough to confirm the float is holding, short enough to not wait forever.

### `TEST_SURFACE_EXTEND_S` (default: 30)

Seconds to run the extend motor when surfacing. Increase if the float doesn't fully surface.

---

## What a Good Run Looks Like

**In the log** (`/log` page or `sudo journalctl -u float -f`):
```
14:23:05 INFO  === Profile 1 of 2 ===
14:23:05 INFO  Moving to 2.35 m...
14:23:07 INFO    sinking ↓  sensor 0.162 m  target 2.35 m  diff -2.188 m
14:23:09 INFO    sinking ↓  sensor 0.631 m  target 2.35 m  diff -1.719 m
14:25:10 INFO  Reached target 2.35 m  (sensor 2.347 m)
14:25:10 INFO  Holding at 2.35 m (need 7 packets)...
14:25:15 INFO    Packet 1/7: RN08  14:25:15  126.4 kPa  2.35 meters
14:25:45 INFO    Packet 7/7: RN08  14:25:45  126.6 kPa  2.36 meters
```

**In the plot** (`/comp` or `/plot`):
- Two clean U-shapes, one per profile
- Flat hold segments at the target depths
- No wide excursions during holds

**Red flags:**
- `restarting hold clock (reset #2)` — deadband too large or float drifting passively
- Descent takes > 3 minutes — duty too low or syringe stalling
- Hold phase zigzags — deadband too small (hunting)
- `Sensor read returned None` — check I2C wiring

---

## Suggested Tuning Sequence

```
Step 1 — Bench (no water)
  □ python test_extend.py      — piston goes OUT (not in)
  □ python test_actuator.py    — both directions, listen for stall
  □ python test_depth.py       — sensor reads ≈ −98 mm in air

Step 2 — Sim run (dry bench)
  □ Open /tuning on controller
  □ Set sim rate 0.1 m/s
  □ Click 🔬 Sim Run
  □ Confirm: depth changes on page, syringe moves, plot appears on controller
  □ Check /log — no errors, 28 packets, data received

Step 3 — Shallow pool test
  □ Set target bottom to pool depth minus 0.3 m (e.g. 1.5 m for 1.8 m pool)
  □ Calibrate bias at surface
  □ Click 🧪 Test Run — watch descent rate and hold behaviour
  □ Adjust ACTUATOR_DUTY_CYCLE if overshoot or stall

Step 4 — Tune CONTROL_DEADBAND_M
  □ If hunting (motor never stops): raise by 0.01
  □ If clock resets often: lower by 0.01 or raise duty cycle
  □ Repeat pool test

Step 5 — Full depth test (pool ≥ 3 m, targets back to 2.5 m / 0.4 m)
  □ Confirm two complete profiles, no clock resets
  □ Plot shows two clean U-shapes
  □ 28 packets logged

Step 6 — Competition rehearsal
  □ Click ▶ Start Mission (not test run)
  □ ROV retrieves float — confirm data arrives at controller within 10 s
  □ Open /comp — verify plot and data for judges
```

---

## Diagnostics

### Float log
`http://192.168.3.120:5000/log` — last 200 lines of `float.log`

The log file (`float.log` on the Pi) rotates at 500 KB and keeps 5 files — about 2.5 MB of history. It persists across reboots.

### Run history
`http://192.168.3.120:5000/runs` — every run is archived to `runs/run_TIMESTAMP.csv`.
The last 10 are kept. You can download or plot any past run from this page.

### Controller journal
`sudo journalctl -u float-controller -f` shows every `/receive` and `/fetch` call with packet count and data ID.

---

## Quick Reference

| Parameter | Default | Tuning page? | Effect |
|-----------|---------|:---:|--------|
| `ACTUATOR_DUTY_CYCLE` | 80 | ✓ | Motor speed |
| `CONTROL_DEADBAND_M` | 0.03 | ✓ | Stop zone around target |
| `SENSOR_DEPTH_OFFSET_M` | 0.0 | ✓ | Sensor-to-bottom offset |
| Target bottom depth | 2.50 m | ✓ | Per-run override |
| Target surface depth | 0.40 m | ✓ | Per-run override |
| `TEST_MODE` | False | — | Auto-surface (set in config.py) |
| `TEST_SURFACE_DELAY_S` | 60 | ✓ | Wait before surfacing |
| `TEST_SURFACE_EXTEND_S` | 30 | ✓ | Extend duration when surfacing |
| Sim rate | 0.08 m/s | ✓ | Virtual descent speed |
