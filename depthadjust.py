import argparse
import csv
import glob
import json
import logging
import logging.handlers
import os
import shutil
import time
from datetime import datetime
import actuator
import depthdetect as depth
from config import (CALL_SIGN, TARGET_BOTTOM_M, TARGET_SURFACE_M, TOLERANCE_M,
                    HOLD_SECONDS, PACKET_INTERVAL_S, PACKETS_REQUIRED,
                    NUM_PROFILES, CONTROL_DEADBAND_M, CALIBRATION_SAMPLES,
                    BIAS_FILE, DATA_FILE, CONTROLLER_IP, CONTROLLER_PORT,
                    TEST_MODE, TEST_SURFACE_DELAY_S, TEST_SURFACE_EXTEND_S,
                    SENSOR_DEPTH_OFFSET_M, FLOAT_HEIGHT_M, ACTUATOR_DUTY_CYCLE,
                    APPROACH_ZONE_M, MIN_DUTY_PCT)

# ── CLI args ──────────────────────────────────────────────────────────────────

_parser = argparse.ArgumentParser(add_help=False)
_parser.add_argument('--test',            action='store_true')
_parser.add_argument('--sim',             action='store_true')
_parser.add_argument('--sim-rate',        type=float, default=None, dest='sim_rate')
_parser.add_argument('--duty',            type=float, default=None)
_parser.add_argument('--deadband',        type=float, default=None)
_parser.add_argument('--surface-delay',   type=float, default=None, dest='surface_delay')
_parser.add_argument('--surface-extend',  type=float, default=None, dest='surface_extend')
_parser.add_argument('--target-bottom',   type=float, default=None, dest='target_bottom')
_parser.add_argument('--target-surface',  type=float, default=None, dest='target_surface')
_parser.add_argument('--sensor-offset',   type=float, default=None, dest='sensor_offset')
_parser.add_argument('--approach-zone',   type=float, default=None, dest='approach_zone')
_parser.add_argument('--min-duty',        type=float, default=None, dest='min_duty')
_parser.add_argument('--float-height',    type=float, default=None, dest='float_height')
_args, _ = _parser.parse_known_args()

# Runtime values — CLI args override config, config is the fallback
_sim_mode       = _args.sim
_sim_rate       = _args.sim_rate if _args.sim_rate is not None else 0.08
_test_mode      = _args.test or _args.sim or TEST_MODE
_deadband_m     = _args.deadband        if _args.deadband        is not None else CONTROL_DEADBAND_M
_surface_delay  = _args.surface_delay   if _args.surface_delay   is not None else TEST_SURFACE_DELAY_S
_surface_extend = _args.surface_extend  if _args.surface_extend  is not None else TEST_SURFACE_EXTEND_S
_sensor_offset  = _args.sensor_offset   if _args.sensor_offset   is not None else SENSOR_DEPTH_OFFSET_M
_float_height   = _args.float_height    if _args.float_height    is not None else FLOAT_HEIGHT_M
_full_duty      = _args.duty            if _args.duty            is not None else ACTUATOR_DUTY_CYCLE
_approach_zone  = _args.approach_zone   if _args.approach_zone   is not None else APPROACH_ZONE_M
_min_duty       = _args.min_duty        if _args.min_duty        is not None else MIN_DUTY_PCT
_comp_bottom    = _args.target_bottom   if _args.target_bottom   is not None else TARGET_BOTTOM_M
_comp_surface   = _args.target_surface  if _args.target_surface  is not None else TARGET_SURFACE_M
# Descent: bottom of float at competition depth → sensor is sensor_offset above bottom
_target_bottom  = _comp_bottom  - _sensor_offset
# Ascent: top of float at competition surface depth → sensor is (height - offset) below top
_target_surface = _comp_surface + _float_height - _sensor_offset

# ── paths ─────────────────────────────────────────────────────────────────────

BASE_DIR     = os.path.dirname(os.path.abspath(__file__))
BIAS_PATH    = os.path.join(BASE_DIR, BIAS_FILE)
DATA_PATH    = os.path.join(BASE_DIR, DATA_FILE)
RUNS_DIR     = os.path.join(BASE_DIR, 'runs')
LOG_PATH     = os.path.join(BASE_DIR, 'float.log')
STATUS_PATH  = os.path.join(BASE_DIR, 'mission_status.json')

# ── simulation ───────────────────────────────────────────────────────────────
# In sim mode the depth sensor is replaced with a virtual model driven by
# actuator direction. The real actuator GPIO still fires so the dry syringe
# moves and all hardware paths are exercised.

if _sim_mode:
    import random

    class _SimSensor:
        """Physics-based depth model.

        The syringe position (+1 = retracted/heavy, -1 = extended/light) integrates
        from motor direction × duty cycle.  Buoyancy from syringe drives velocity;
        drag limits terminal speed.  The float has slight natural positive buoyancy
        so it drifts up slowly when the motor is off and the syringe is near neutral.

        rate = terminal descent speed at full syringe deflection (maps to sim_rate slider).
        """
        _DRAG        = 0.5    # velocity damping 1/s
        _STROKE_RATE = 0.18   # syringe units/s at 100% duty — full 2-unit stroke in ~14 s at 80%
        _NATURAL     = -0.006 # m/s²: slight upward bias (float prefers to surface)

        def __init__(self, rate):
            # buoy_gain chosen so terminal velocity at syringe=1 equals rate
            self._buoy_gain = rate * self._DRAG - self._NATURAL
            self._depth    = 0.0
            self._velocity = 0.0
            self._syringe  = -1.0  # start fully extended (empty) — maximum positive buoyancy
            self._dir      = 0     # +1 retract/sink, -1 extend/rise, 0 stop
            self._duty     = 1.0   # 0..1 fraction
            self._t        = time.time()

        def set_dir(self, d):
            self._tick()
            self._dir = d

        def set_duty(self, pct):
            self._duty = max(0.0, min(1.0, pct / 100.0))

        def _tick(self):
            now = time.time()
            dt  = min(now - self._t, 0.5)   # cap to avoid big jumps on pause
            self._t = now

            # Syringe moves only while motor is running, scaled by duty
            stroke        = self._dir * self._STROKE_RATE * self._duty
            self._syringe = max(-1.0, min(1.0, self._syringe + stroke * dt))

            # Net vertical acceleration: buoyancy + natural drift − drag
            buoyancy       = self._syringe * self._buoy_gain + self._NATURAL
            self._velocity += (buoyancy - self._DRAG * self._velocity) * dt
            self._depth    += self._velocity * dt
            self._depth     = max(-0.15, min(6.0, self._depth))

        def read(self):
            self._tick()
            pressure = 101.3 + max(0.0, self._depth) * 9.794  # kPa freshwater
            return self._depth + random.gauss(0, 0.003), pressure

    _sim_sensor = _SimSensor(_sim_rate)

    # Wrap actuator calls: sim sensor tracks direction/duty, real GPIO still fires
    _real_retract  = actuator.retractActuator
    _real_extend   = actuator.extendActuator
    _real_stop     = actuator.stopActuator
    _real_set_duty = actuator.setDutyCycle

    def _sim_retract():
        _real_retract()
        _sim_sensor.set_dir(1)    # retract → syringe fills → heavy → sinking

    def _sim_extend():
        _real_extend()
        _sim_sensor.set_dir(-1)   # extend  → syringe empties → light → rising

    def _sim_stop():
        _real_stop()
        _sim_sensor.set_dir(0)    # motor off — buoyancy unchanged, float coasts

    def _sim_set_duty(pct):
        _real_set_duty(pct)
        _sim_sensor.set_duty(pct)

    actuator.retractActuator = _sim_retract
    actuator.extendActuator  = _sim_extend
    actuator.stopActuator    = _sim_stop
    actuator.setDutyCycle    = _sim_set_duty


# ── logging ───────────────────────────────────────────────────────────────────

log = logging.getLogger('float')
log.setLevel(logging.DEBUG)

_fmt = logging.Formatter('%(asctime)s %(levelname)-8s %(message)s',
                         datefmt='%H:%M:%S')

# Rotating log file — 500 KB, keep 5 files (~2.5 MB total)
_fh = logging.handlers.RotatingFileHandler(LOG_PATH, maxBytes=500_000, backupCount=5)
_fh.setFormatter(_fmt)
log.addHandler(_fh)

# Also to stdout → systemd journal
_sh = logging.StreamHandler()
_sh.setFormatter(_fmt)
log.addHandler(_sh)

# ── state ─────────────────────────────────────────────────────────────────────

_depth_bias_m = 0.0


_live_t           = 0.0   # throttle live-status writes to ~400 ms
_mission_start_t  = None  # set in main() for elapsed-time calculation
_prev_live_depth  = None  # for velocity estimate in live status


def _write_status(update):
    """Read-modify-write STATUS_PATH so stage and live fields coexist."""
    try:
        st = {}
        if os.path.exists(STATUS_PATH):
            with open(STATUS_PATH) as f:
                st = json.load(f)
        st.update(update)
        with open(STATUS_PATH, 'w') as f:
            json.dump(st, f)
    except Exception:
        pass


def _set_stage(stage):
    _write_status({'stage': stage, 'time': datetime.now().strftime('%H:%M:%S')})
    log.info(f"Stage: {stage}")


def _live_update(depth_m, actuator):
    """Push current depth/actuator/velocity to STATUS_PATH at ~400 ms intervals."""
    global _live_t, _prev_live_depth
    now = time.time()
    dt  = now - _live_t
    if dt < 0.4:
        return
    velocity = round((depth_m - _prev_live_depth) / dt, 4) if _prev_live_depth is not None else 0.0
    _prev_live_depth = depth_m
    _live_t = now
    elapsed = round(now - _mission_start_t, 1) if _mission_start_t else 0.0
    update = {'depth_m': round(depth_m, 3), 'elapsed_s': elapsed,
              'actuator': actuator, 'velocity_ms': velocity}
    if _sim_mode:
        # syringe_pct: 0 = extended/empty/light (rising), 100 = retracted/full/heavy (sinking)
        update['syringe_pct'] = round((1 + _sim_sensor._syringe) / 2 * 100, 1)
    _write_status(update)


def _proportional_duty(dist_m):
    """Return duty cycle based on distance from target.
    Full speed when far away; ramps down linearly to _min_duty within
    _approach_zone of the target to reduce overshoot."""
    if _approach_zone <= 0:
        return _full_duty
    factor = min(1.0, abs(dist_m) / _approach_zone)
    duty = int(_min_duty + (_full_duty - _min_duty) * factor)
    return max(_min_duty, min(100, duty))


# ── helpers ───────────────────────────────────────────────────────────────────

def loadBias():
    global _depth_bias_m
    if os.path.exists(BIAS_PATH):
        with open(BIAS_PATH) as f:
            _depth_bias_m = float(json.load(f).get('bias_m', 0.0))
        log.info(f"Bias loaded from file: {_depth_bias_m:.4f} m")
        return True
    return False


def calibrateBias():
    global _depth_bias_m
    log.info("Calibrating bias — float must be at the surface...")
    samples = [depth.readDepthM() for _ in range(CALIBRATION_SAMPLES) if not time.sleep(0.2)]
    _depth_bias_m = sum(samples) / len(samples)
    log.info(f"Bias set to {_depth_bias_m:.4f} m")


def readTrueDepthAndPressure():
    """Read sensor with one retry. Returns (depth_m, pressure_kpa) or (None, None)."""
    if _sim_mode:
        d, p = _sim_sensor.read()
        return d, p   # bias is 0 in sim; depth is already in absolute metres
    depth_m, pressure_kpa = depth.readSensor()
    if depth_m is None:
        log.warning("Sensor read returned None — retrying once")
        time.sleep(0.05)
        depth_m, pressure_kpa = depth.readSensor()
    if depth_m is None:
        log.error("Sensor read failed twice in a row")
        return None, None
    return depth_m - _depth_bias_m, pressure_kpa


def createDataPacket(depth_m, pressure_kpa):
    ts = datetime.now().strftime("%H:%M:%S")
    return f"{CALL_SIGN}  {ts}  {pressure_kpa:.1f} kPa  {depth_m:.2f} meters"


def logPacket(elapsed_s, depth_m, pressure_kpa, packet_str):
    write_header = not os.path.exists(DATA_PATH)
    with open(DATA_PATH, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(['elapsed_s', 'depth_m', 'pressure_kpa', 'packet'])
        writer.writerow([f'{elapsed_s:.1f}', f'{depth_m:.4f}',
                         f'{pressure_kpa:.1f}', packet_str])


def archivePreviousRun():
    """Copy data.csv to runs/ with a timestamp. Keep last 10 runs."""
    if not os.path.exists(DATA_PATH):
        return
    os.makedirs(RUNS_DIR, exist_ok=True)
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    dest = os.path.join(RUNS_DIR, f'run_{ts}.csv')
    shutil.copy2(DATA_PATH, dest)
    log.info(f"Archived previous run → runs/run_{ts}.csv")
    # Trim: keep only the 10 most recent
    all_runs = sorted(glob.glob(os.path.join(RUNS_DIR, 'run_*.csv')))
    for old in all_runs[:-10]:
        os.remove(old)
        log.debug(f"Removed old run archive: {os.path.basename(old)}")
    os.remove(DATA_PATH)


# ── mission logic ─────────────────────────────────────────────────────────────

def moveToDepth(targetM):
    log.info(f"Moving to {targetM:.2f} m...")
    prev_depth  = None
    prev_t      = time.time()
    velocity    = 0.0
    last_log    = prev_t - 2
    burst_start = None   # when the current burst began (None = motor off)
    burst_end   = None   # when the last burst ended

    BURST_MAX = 2.0   # max seconds per actuation burst
    OBSERVE_S = 1.5   # min seconds to observe after each burst
    COAST_V   = 0.005 # m/s: already drifting toward target — don't interrupt
    BRAKE_V   = 0.025 # m/s: in approach zone, too fast — actively brake

    while True:
        current, _ = readTrueDepthAndPressure()
        if current is None:
            time.sleep(0.1)
            continue

        now  = time.time()
        diff = current - targetM

        if abs(diff) <= TOLERANCE_M:
            actuator.stopActuator()
            log.info(f"Reached target {targetM:.2f} m  (sensor {current:.3f} m)")
            return

        # Velocity estimate — updated every ~0.4 s to reduce noise
        if prev_depth is None:
            prev_depth, prev_t = current, now
        elif (now - prev_t) >= 0.4:
            velocity   = (current - prev_depth) / (now - prev_t)
            prev_depth, prev_t = current, now

        going_toward = (diff < 0 and velocity >  0) or (diff > 0 and velocity < 0)
        in_approach  = abs(diff) <= _approach_zone

        # 1. Brake: overspeed inside approach zone
        if in_approach and going_toward and abs(velocity) > BRAKE_V:
            if burst_start is not None:
                burst_end = now; burst_start = None
            actuator.setDutyCycle(max(_min_duty, _proportional_duty(diff) // 2))
            if diff < 0: actuator.extendActuator();  phase = 'braking ↑'
            else:        actuator.retractActuator(); phase = 'braking ↓'

        # 2. Coast: already drifting toward target at useful speed
        elif going_toward and abs(velocity) > COAST_V:
            if burst_start is not None:
                burst_end = now; burst_start = None
            actuator.stopActuator()
            phase = 'coasting'

        # 3. Continue active burst (up to BURST_MAX seconds)
        elif burst_start is not None and (now - burst_start) < BURST_MAX:
            actuator.setDutyCycle(_proportional_duty(diff))
            if diff < 0: actuator.retractActuator(); phase = f'burst ↓ {now-burst_start:.1f}s'
            else:        actuator.extendActuator();  phase = f'burst ↑ {now-burst_start:.1f}s'

        # 4. Burst time up — stop motor and observe
        elif burst_start is not None:
            burst_end = now; burst_start = None
            actuator.stopActuator()
            phase = 'observing'

        # 5. Still in observe window — wait for velocity to settle
        elif burst_end is not None and (now - burst_end) < OBSERVE_S:
            actuator.stopActuator()
            phase = f'observing {now-burst_end:.1f}s'

        # 6. Need more — start a new burst
        else:
            burst_start = now
            actuator.setDutyCycle(_proportional_duty(diff))
            if diff < 0: actuator.retractActuator(); phase = 'burst ↓ start'
            else:        actuator.extendActuator();  phase = 'burst ↑ start'

        _live_update(current, phase)

        if now - last_log >= 2.0:
            direction = "sinking ↓" if diff < 0 else "rising ↑"
            log.info(f"  {direction}  sensor {current:.3f} m  target {targetM:.2f} m"
                     f"  diff {diff:+.3f} m  vel {velocity:+.4f} m/s  [{phase}]")
            last_log = now
        time.sleep(0.1)


def holdDepthAndLog(targetM, mission_start):
    packets    = []
    holdStart  = time.time()
    lastPacket = holdStart
    resetCount = 0

    log.info(f"Holding at {targetM:.2f} m (need {PACKETS_REQUIRED} packets)...")

    while len(packets) < PACKETS_REQUIRED:
        now = time.time()
        current, pressure_kpa = readTrueDepthAndPressure()
        if current is None:
            log.warning("Skipping control cycle — sensor read failed")
            time.sleep(0.1)
            continue
        diff = current - targetM

        if abs(diff) > TOLERANCE_M:
            resetCount += 1
            log.warning(f"Depth drifted to {current:.3f} m (±{TOLERANCE_M} m limit) — "
                        f"restarting hold clock (reset #{resetCount})")
            packets    = []
            holdStart  = now
            lastPacket = now
            moveToDepth(targetM)
            continue

        if diff < -_deadband_m:
            actuator.setDutyCycle(_proportional_duty(diff))
            actuator.retractActuator()
            _live_update(current, 'retracting ↓')
        elif diff > _deadband_m:
            actuator.setDutyCycle(_proportional_duty(diff))
            actuator.extendActuator()
            _live_update(current, 'extending ↑')
        else:
            actuator.stopActuator()
            _live_update(current, 'holding ◼')

        if now - lastPacket >= PACKET_INTERVAL_S:
            elapsed = now - mission_start
            packet  = createDataPacket(current, pressure_kpa)
            packets.append(packet)
            logPacket(elapsed, current, pressure_kpa, packet)
            log.info(f"  Packet {len(packets)}/{PACKETS_REQUIRED}: {packet}")
            lastPacket = now

        time.sleep(0.1)

    if resetCount:
        log.info(f"Hold complete — {resetCount} clock reset(s) during this hold")
    return packets


def executeProfile(number, mission_start):
    log.info(f"\n=== Profile {number} of {NUM_PROFILES} ===")
    log.info(f"Sensor targets: bottom {_target_bottom:.2f} m, surface {_target_surface:.2f} m"
             + (f"  (offset {_sensor_offset:+.3f} m, height {_float_height:.3f} m)"
                if _sensor_offset or _float_height else ""))
    packets = []
    _set_stage(f"profile_{number}_descent")
    moveToDepth(_target_bottom)
    _set_stage(f"profile_{number}_hold_bottom")
    packets += holdDepthAndLog(_target_bottom, mission_start)
    _set_stage(f"profile_{number}_ascent")
    moveToDepth(_target_surface)
    _set_stage(f"profile_{number}_hold_surface")
    packets += holdDepthAndLog(_target_surface, mission_start)
    return packets


def transmitData():
    """Retry loop: send data.csv to controller every 10s until first 200/204,
    then every 30s as a heartbeat. Fails fast (5s timeout) while underwater."""
    if not os.path.exists(DATA_PATH):
        log.warning("transmitData: no data file found")
        return

    import requests

    log.info("\n--- Logged packets ---")
    with open(DATA_PATH) as f:
        for row in csv.DictReader(f):
            log.info("  " + row.get('packet', ''))

    url = f"http://{CONTROLLER_IP}:{CONTROLLER_PORT}/receive"
    log.info(f"\n--- Transmission loop → {url} (10s retry, 30s heartbeat after success) ---\n")

    first_success = False
    attempt       = 0

    while True:
        attempt += 1
        try:
            with open(DATA_PATH, 'rb') as f:
                r = requests.post(url, data=f,
                                  headers={'Content-Type': 'text/csv'}, timeout=5)
            if r.status_code in (200, 204):
                if not first_success:
                    log.info(f"Attempt {attempt}: SUCCESS ({r.status_code}) — "
                             f"controller confirmed. Switching to 30s heartbeat.")
                    first_success = True
                else:
                    log.info(f"Attempt {attempt}: heartbeat confirmed ({r.status_code})")
            else:
                log.warning(f"Attempt {attempt}: unexpected status {r.status_code}")
        except Exception as e:
            log.info(f"Attempt {attempt}: no link ({type(e).__name__}) — "
                     f"still underwater?")

        time.sleep(30 if first_success else 10)


def main():
    log.info("=" * 60)
    log.info(f"Float mission starting  call={CALL_SIGN}  test={_test_mode}")
    log.info(f"  duty={_args.duty or 'config'}  deadband={_deadband_m:.3f} m"
             f"  offset={_sensor_offset:+.3f} m")
    log.info(f"  targets: bottom {_target_bottom:.2f} m  surface {_target_surface:.2f} m")
    log.info("=" * 60)

    if _sim_mode:
        log.info(f"SIM MODE — depth sensor mocked at {_sim_rate} m/s")
        log.info("Real actuator GPIO still fires — dry syringe will move")
    else:
        if not depth.initSensor():
            log.error("Sensor init failed — aborting mission")
            return
        if not loadBias():
            calibrateBias()

    global _mission_start_t
    _set_stage('starting')
    archivePreviousRun()

    actuator.setupActuator()
    actuator.setDutyCycle(_full_duty)

    mission_start    = time.time()
    _mission_start_t = mission_start
    all_packets   = []

    for i in range(NUM_PROFILES):
        all_packets.extend(executeProfile(i + 1, mission_start))

    elapsed = time.time() - mission_start
    log.info(f"\nBoth profiles complete in {elapsed:.0f}s. "
             f"{len(all_packets)} packets logged. Actuator stopped.")
    actuator.stopActuator()
    _set_stage('profiles_complete')

    if _test_mode:
        _set_stage('surfacing')
        log.info(f"\nTEST MODE: waiting {_surface_delay:.0f}s then surfacing...")
        time.sleep(_surface_delay)
        log.info(f"Surfacing — extending for {_surface_extend:.0f}s...")
        actuator.extendActuator()
        time.sleep(_surface_extend)
        actuator.stopActuator()
        log.info("Surfaced. Waiting for retrieval.")
    else:
        log.info("Waiting for ROV recovery...")

    _set_stage('transmitting')
    transmitData()
    _set_stage('done')


if __name__ == "__main__":
    main()
