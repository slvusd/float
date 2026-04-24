# Float Tuning Guide
**Team RN08** — How to test, tune, and validate the float before competition.

---

## How the Float Works (Quick Physics)

The float uses a syringe connected to the water. A linear actuator pushes or pulls
the plunger:

| Action | Effect | Float does |
|--------|--------|-----------|
| **Retract** (pull plunger in) | More water enters syringe → float gets heavier | Sinks ↓ |
| **Extend** (push plunger out) | Water pushed out → float gets lighter | Rises ↑ |

The control loop runs every 100ms:
1. Read depth from pressure sensor
2. Subtract bias to get true depth
3. If more than `CONTROL_DEADBAND_M` above target → retract (sink)
4. If more than `CONTROL_DEADBAND_M` below target → extend (rise)
5. If within deadband → stop actuator

---

## Running a Test

Runs are started from the **controller UI at `http://192.168.3.46:5001`**.

| Button | What happens |
|--------|-------------|
| **▶ Start mission** | Competition run — float waits at depth for ROV recovery after profiles |
| **🧪 Test run** | Same profiles, but float surfaces automatically after `TEST_SURFACE_DELAY_S` seconds — no ROV needed |

Both buttons are also on the float UI at `http://192.168.3.120:5000`.

You can also trigger via curl:
```bash
curl -X POST http://192.168.3.46:5001/float/start           # competition
curl -X POST http://192.168.3.46:5001/float/start?test=true # test run
```

---

## Parameters to Tune

All tuning parameters are in `config.py`. Edit the file, `git push`, `git pull` on
the Pi, and `sudo systemctl restart float` to apply.

### `ACTUATOR_DUTY_CYCLE` (default: 80)

Controls how fast the piston moves. Applies to all mission and test script runs.

| Value | Effect | Use when |
|-------|--------|---------|
| 100 | Full speed | Bench testing only |
| 80 | Fast but controllable | Good starting point |
| 50–60 | Slower, smoother | Float overshoots target badly |
| < 50 | Very slow | Float barely moves, may stall under load |

**Signs you need to lower it:**
- The float blows past the target depth and takes a long time to correct
- The depth trace on the plot shows big U-shaped dips below 2.5 m before settling
- The float is still moving when it should be holding

**Signs you need to raise it:**
- The float takes forever to reach 2.5 m (more than ~2 minutes)
- The float stalls partway (motor hums but doesn't move — too much water pressure)

### `CONTROL_DEADBAND_M` (default: 0.03 — 3 cm)

The zone around the target where the actuator stops. Think of it as the
float's tolerance for "close enough".

| Value | Effect |
|-------|--------|
| 0.01 m (1 cm) | Very tight — chases target aggressively; may hunt (oscillate) |
| 0.03 m (3 cm) | Default — good balance |
| 0.08 m (8 cm) | Relaxed — less actuator movement, more depth variation |

**Signs it is too small:**
- The actuator keeps switching between extend and retract rapidly ("hunting")
- The motor is constantly running during the hold phase
- The depth trace zigzags up and down around the target

**Signs it is too large:**
- The float drifts 10+ cm from target and the hold clock keeps resetting
- Packets are rarely logged because it keeps drifting out of the valid window

### `TEST_SURFACE_DELAY_S` (default: 60)

How many seconds the float waits at its final position before surfacing in test mode.
Keep at 60–120 s — long enough that you can see it holding, short enough that
you're not waiting too long at the pool.

### `TEST_SURFACE_EXTEND_S` (default: 30)

How long the extend motor runs when surfacing. Should be longer than the full stroke
time measured during bench testing. If the float doesn't fully surface, increase this.

---

## What a Good Test Run Looks Like

**In the logs** (`sudo journalctl -u float -f`):
```
=== Profile 1 of 2 ===
Moving to 2.50 m...
Holding at 2.50 m...
  Packet 1/7: RN08  14:23:18  126.4 kPa  2.51 meters
  Packet 2/7: RN08  14:23:23  126.3 kPa  2.50 meters
  ...
  Packet 7/7: RN08  14:23:48  126.5 kPa  2.52 meters
Moving to 0.40 m...
Holding at 0.40 m...
  Packet 1/7: ...
```

**In the plot** (`http://192.168.3.46:5001/plot`):
- Two clean U-shapes, one per profile
- Flat hold segments at 2.5 m and 0.4 m
- No repeated "clock reset" messages in logs during holds

**Red flags:**
- `Depth drifted to X.XX m — restarting hold clock` appears more than once per hold
  → deadband may be too large, or actuator too slow to respond
- The descent takes more than 3 minutes → duty cycle too low, or water resistance
  is higher than expected
- The hold phase shows wild depth swings → deadband too small (hunting)

---

## Common Problems

### Float sinks too fast and overshoots 2.5 m
Lower `ACTUATOR_DUTY_CYCLE` (try 60). The float has momentum — slowing the piston
lets you stop closer to the target.

### Float won't reach 2.5 m
- Check that `BIAS_FILE` (`bias.json`) was calibrated at the surface before the run.
  If bias is wrong, the float thinks it is deeper than it is.
- Increase `ACTUATOR_DUTY_CYCLE` if the motor stalls.
- Check that the syringe can physically displace enough water — if the stroke is too
  short the float may be neutrally buoyant before reaching 2.5 m.

### Hold clock keeps resetting
The float drifted outside the ±33 cm valid window during the hold.
- Widen `CONTROL_DEADBAND_M` slightly — the float may be hunting past the boundary.
- Slow the actuator (`ACTUATOR_DUTY_CYCLE`) so corrections are gentler.
- Check that the syringe has no air leaks causing slow passive drift.

### Float won't surface in test mode
Increase `TEST_SURFACE_EXTEND_S` (try 45). The extend motor needs enough time to
push out all the water.

### Bias is far off (depth reads wrong in air)
Normal: the sensor reads ≈ −0.10 m in air due to atmospheric pressure. Always
run "Calibrate bias" from the UI with the float at the surface **in air** before
each session. Bias is saved to `bias.json` and reloaded on reboot — you only need
to recalibrate if the weather changes significantly or you move to a different
altitude.

---

## Suggested Tuning Sequence

```
1. Bench test (no water)
   □ python test_extend.py          — verify piston goes out
   □ python test_actuator.py        — verify both directions, listen for stall
   □ python test_depth.py           — verify sensor reads ≈ −98 mm in air

2. Shallow pool test (TEST_MODE = True)
   □ Calibrate bias at surface
   □ Click "🧪 Test run" on controller
   □ Watch logs — note how long descent takes and whether hold clock resets
   □ Download plot — check U-shape quality

3. Tune ACTUATOR_DUTY_CYCLE
   □ If overshoot > 20 cm: lower by 10 (e.g. 80 → 70)
   □ If descent > 3 min: raise by 10
   □ Repeat shallow pool test

4. Tune CONTROL_DEADBAND_M
   □ If hunting (motor never stops during hold): raise by 0.01
   □ If clock resets often: lower by 0.01 or raise duty cycle
   □ Repeat shallow pool test

5. Full depth test (TEST_MODE = True, pool must be ≥ 3 m)
   □ Confirm two complete profiles with no clock resets
   □ Plot shows two clean U-shapes
   □ All 28 packets logged

6. Competition run rehearsal (TEST_MODE = False)
   □ Full run — float waits for "ROV" (teammate with a rope) to recover it
   □ Confirm data transmits to controller within 10s of retrieval
   □ Plot and CSV look correct
```

---

## Quick Reference

| Parameter | Default | Typical range | Effect |
|-----------|---------|--------------|--------|
| `ACTUATOR_DUTY_CYCLE` | 80 | 50–100 | Speed — lower = slower |
| `CONTROL_DEADBAND_M` | 0.03 | 0.01–0.10 | Dead zone — larger = less hunting |
| `TEST_MODE` | False | True/False | Auto-surface after profiles |
| `TEST_SURFACE_DELAY_S` | 60 | 30–120 | Wait before surfacing |
| `TEST_SURFACE_EXTEND_S` | 30 | 20–45 | Extend duration when surfacing |
