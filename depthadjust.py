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
                    SENSOR_DEPTH_OFFSET_M, ACTUATOR_DUTY_CYCLE,
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
_args, _ = _parser.parse_known_args()

# Runtime values — CLI args override config, config is the fallback
_sim_mode       = _args.sim
_sim_rate       = _args.sim_rate if _args.sim_rate is not None else 0.08
_test_mode      = _args.test or _args.sim or TEST_MODE
_deadband_m     = _args.deadband        if _args.deadband        is not None else CONTROL_DEADBAND_M
_surface_delay  = _args.surface_delay   if _args.surface_delay   is not None else TEST_SURFACE_DELAY_S
_surface_extend = _args.surface_extend  if _args.surface_extend  is not None else TEST_SURFACE_EXTEND_S
_sensor_offset  = _args.sensor_offset   if _args.sensor_offset   is not None else SENSOR_DEPTH_OFFSET_M
_target_bottom  = (_args.target_bottom  if _args.target_bottom  is not None else TARGET_BOTTOM_M)  - _sensor_offset
_target_surface = (_args.target_surface if _args.target_surface is not None else TARGET_SURFACE_M) - _sensor_offset
_full_duty      = _args.duty            if _args.duty            is not None else ACTUATOR_DUTY_CYCLE
_approach_zone  = _args.approach_zone   if _args.approach_zone   is not None else APPROACH_ZONE_M
_min_duty       = _args.min_duty        if _args.min_duty        is not None else MIN_DUTY_PCT

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
        """Virtual depth sensor. Direction: +1 sinking, -1 rising, 0 stopped."""
        def __init__(self, rate):
            self._depth = 0.0
            self._dir   = 0
            self._rate  = rate
            self._t     = time.time()

        def set_dir(self, d):
            self._tick()   # flush elapsed time before changing direction
            self._dir = d

        def _tick(self):
            now = time.time()
            dt  = now - self._t
            self._t = now
            # Active movement + slight positive-buoyancy drift when stopped
            drift = -0.001 if self._dir == 0 else 0
            self._depth += (self._dir * self._rate + drift) * dt
            self._depth += random.gauss(0, 0.004)    # sensor noise
            self._depth  = max(-0.15, min(6.0, self._depth))

        def read(self):
            self._tick()
            pressure = 101.3 + max(0.0, self._depth) * 9.794  # kPa freshwater
            return self._depth, pressure

    _sim_sensor = _SimSensor(_sim_rate)

    # Wrap actuator calls so the sim sensor tracks direction while GPIO still fires
    _real_retract = actuator.retractActuator
    _real_extend  = actuator.extendActuator
    _real_stop    = actuator.stopActuator

    def _sim_retract():
        _real_retract()
        _sim_sensor.set_dir(1)   # retract → sinking

    def _sim_extend():
        _real_extend()
        _sim_sensor.set_dir(-1)  # extend  → rising

    def _sim_stop():
        _real_stop()
        _sim_sensor.set_dir(0)

    actuator.retractActuator = _sim_retract
    actuator.extendActuator  = _sim_extend
    actuator.stopActuator    = _sim_stop


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


def _set_stage(stage):
    """Write current mission stage to a file so server.py can display it."""
    try:
        with open(STATUS_PATH, 'w') as f:
            json.dump({'stage': stage,
                       'time':  datetime.now().strftime('%H:%M:%S')}, f)
    except Exception:
        pass
    log.info(f"Stage: {stage}")


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
    last_log = time.time() - 2
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
        duty = _proportional_duty(diff)
        actuator.setDutyCycle(duty)
        if diff < 0:
            actuator.retractActuator()
        else:
            actuator.extendActuator()
        if now - last_log >= 2.0:
            direction = "sinking ↓" if diff < 0 else "rising ↑"
            log.info(f"  {direction}  sensor {current:.3f} m  target {targetM:.2f} m"
                     f"  diff {diff:+.3f} m  duty {duty}%")
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
        elif diff > _deadband_m:
            actuator.setDutyCycle(_proportional_duty(diff))
            actuator.extendActuator()
        else:
            actuator.stopActuator()

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
             + (f"  (offset {_sensor_offset:+.3f} m)" if _sensor_offset else ""))
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

    _set_stage('starting')
    archivePreviousRun()

    actuator.setupActuator()
    actuator.setDutyCycle(_full_duty)

    mission_start = time.time()
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
