#!/usr/bin/env python3
"""
depthadjust.py  — SEL Technologies float buoyancy engine
Depth control with PID governor replacing the velocity bang-bang approach.

PID replaces the old velocity-governor moveToDepth() logic.
holdDepthAndLog() keeps its proportional nudge for fine trim during the hold.

Tuning cheat-sheet (config.py or CLI):
  PID_KP  — proportional gain: too high → oscillates; too low → slow
  PID_KI  — integral gain:     accumulates steady-state offset (helps stopping)
  PID_KD  — derivative gain:   damps oscillation / anticipates overshoot
  PID_IMAX — integral anti-windup clamp (metres·s)
"""

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
from config import (
    CALL_SIGN, TARGET_BOTTOM_M, TARGET_SURFACE_M, TOLERANCE_M,
    HOLD_SECONDS, PACKET_INTERVAL_S, PACKETS_REQUIRED,
    NUM_PROFILES, CONTROL_DEADBAND_M, CALIBRATION_SAMPLES,
    BIAS_FILE, DATA_FILE, CONTROLLER_IP, CONTROLLER_PORT,
    TEST_MODE, TEST_SURFACE_DELAY_S, TEST_SURFACE_EXTEND_S,
    SENSOR_DEPTH_OFFSET_M, FLOAT_HEIGHT_M, ACTUATOR_DUTY_CYCLE,
    APPROACH_ZONE_M, MIN_DUTY_PCT,
)

# ── PID defaults (add these to config.py if not already present) ──────────────
# Import gracefully so existing config.py files without PID keys still work.
try:
    from config import PID_KP, PID_KI, PID_KD, PID_IMAX
except ImportError:
    PID_KP   = 40.0   # duty-% per metre of error
    PID_KI   = 8.0    # duty-% per metre·second of accumulated error
    PID_KD   = 15.0   # duty-% per m/s of error rate
    PID_IMAX = 0.5    # anti-windup: clamp integral to ±0.5 m·s

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
# PID overrides
_parser.add_argument('--kp',   type=float, default=None)
_parser.add_argument('--ki',   type=float, default=None)
_parser.add_argument('--kd',   type=float, default=None)
_parser.add_argument('--imax', type=float, default=None)
_args, _ = _parser.parse_known_args()

# Runtime values — CLI args override config, config is the fallback
_sim_mode       = _args.sim
_sim_rate       = _args.sim_rate if _args.sim_rate is not None else 0.08
_test_mode      = _args.test or _args.sim or TEST_MODE
_deadband_m     = _args.deadband       if _args.deadband       is not None else CONTROL_DEADBAND_M
_surface_delay  = _args.surface_delay  if _args.surface_delay  is not None else TEST_SURFACE_DELAY_S
_surface_extend = _args.surface_extend if _args.surface_extend is not None else TEST_SURFACE_EXTEND_S
_sensor_offset  = _args.sensor_offset  if _args.sensor_offset  is not None else SENSOR_DEPTH_OFFSET_M
_float_height   = _args.float_height   if _args.float_height   is not None else FLOAT_HEIGHT_M
_full_duty      = _args.duty           if _args.duty           is not None else ACTUATOR_DUTY_CYCLE
_approach_zone  = _args.approach_zone  if _args.approach_zone  is not None else APPROACH_ZONE_M
_min_duty       = _args.min_duty       if _args.min_duty       is not None else MIN_DUTY_PCT
_comp_bottom    = _args.target_bottom  if _args.target_bottom  is not None else TARGET_BOTTOM_M
_comp_surface   = _args.target_surface if _args.target_surface is not None else TARGET_SURFACE_M

# Sensor-referenced targets (corrected for float geometry)
_target_bottom  = _comp_bottom  - _sensor_offset
_target_surface = _comp_surface + _float_height - _sensor_offset

# PID gains — CLI wins, then config, then hardcoded defaults above
_kp   = _args.kp   if _args.kp   is not None else PID_KP
_ki   = _args.ki   if _args.ki   is not None else PID_KI
_kd   = _args.kd   if _args.kd   is not None else PID_KD
_imax = _args.imax if _args.imax is not None else PID_IMAX

# ── paths ─────────────────────────────────────────────────────────────────────

BASE_DIR    = os.path.dirname(os.path.abspath(__file__))
BIAS_PATH   = os.path.join(BASE_DIR, BIAS_FILE)
DATA_PATH   = os.path.join(BASE_DIR, DATA_FILE)
RUNS_DIR    = os.path.join(BASE_DIR, 'runs')
LOG_PATH    = os.path.join(BASE_DIR, 'float.log')
STATUS_PATH = os.path.join(BASE_DIR, 'mission_status.json')

# ── simulation ────────────────────────────────────────────────────────────────

if _sim_mode:
    import random

    class _SimSensor:
        """Physics-based depth model (unchanged from original)."""
        _DRAG        = 0.5
        _STROKE_RATE = 0.18
        _NATURAL     = -0.001

        def __init__(self, rate):
            self._depth    = 0.0
            self._velocity = 0.0
            self._syringe  = -1.0
            self._dir      = 0
            self._duty     = 80
            self._rate     = rate
            self._t        = time.time()

        def set_dir(self, d):   self._dir  = d
        def set_duty(self, p):  self._duty = p

        def read(self):
            now = time.time()
            dt  = now - self._t
            self._t = now
            stroke_speed  = self._STROKE_RATE * (self._duty / 100)
            self._syringe = max(-1.0, min(1.0,
                self._syringe + self._dir * stroke_speed * dt))
            buoyancy_acc  = self._syringe * self._rate / 0.08
            drag_force    = -self._velocity * self._DRAG
            net_acc       = buoyancy_acc + drag_force + self._NATURAL
            self._velocity += net_acc * dt
            self._depth     = max(0.0, self._depth + self._velocity * dt)
            noise = random.gauss(0, 0.001)
            return round(self._depth + noise, 4)

    _sim_sensor = _SimSensor(_sim_rate)

    def _sim_retract(): _sim_sensor.set_dir(+1)
    def _sim_extend():  _sim_sensor.set_dir(-1)
    def _sim_stop():    _sim_sensor.set_dir(0)
    def _sim_set_duty(pct): _sim_sensor.set_duty(pct)

    actuator.retractActuator = _sim_retract
    actuator.extendActuator  = _sim_extend
    actuator.stopActuator    = _sim_stop
    actuator.setDutyCycle    = _sim_set_duty

# ── logging ───────────────────────────────────────────────────────────────────

log = logging.getLogger('float')
log.setLevel(logging.DEBUG)

_fmt = logging.Formatter('%(asctime)s %(levelname)-8s %(message)s', datefmt='%H:%M:%S')
_fh  = logging.handlers.RotatingFileHandler(LOG_PATH, maxBytes=500_000, backupCount=5)
_fh.setFormatter(_fmt)
log.addHandler(_fh)
_sh = logging.StreamHandler()
_sh.setFormatter(_fmt)
log.addHandler(_sh)

# ── state ─────────────────────────────────────────────────────────────────────

_depth_bias_m    = 0.0
_live_t          = 0.0
_mission_start_t = None
_prev_live_depth = None


def _write_status(update):
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


def _live_update(depth_m, actuator_phase):
    global _live_t, _prev_live_depth
    now = time.time()
    dt  = now - _live_t
    if dt < 0.4:
        return
    velocity = (
        round((depth_m - _prev_live_depth) / dt, 4)
        if _prev_live_depth is not None else 0.0
    )
    _prev_live_depth = depth_m
    _live_t          = now
    elapsed = round(now - _mission_start_t, 1) if _mission_start_t else 0.0
    update  = {
        'depth_m': round(depth_m, 3),
        'elapsed_s': elapsed,
        'actuator': actuator_phase,
        'velocity_ms': velocity,
    }
    if _sim_mode:
        update['syringe_pct'] = round((1 + _sim_sensor._syringe) / 2 * 100, 1)
    _write_status(update)


# ── PID controller ────────────────────────────────────────────────────────────

class DepthPID:
    """
    Proportional-Integral-Derivative controller for buoyancy-engine depth control.

    Sign convention (matches the rest of the file):
      error > 0  → float is DEEPER than target → need to RISE  → extend actuator
      error < 0  → float is SHALLOWER than target → need to SINK → retract actuator

    Output is a signed duty-cycle percentage:
      positive → extend (rise)
      negative → retract (sink)
    The magnitude is clamped to [_min_duty, _full_duty] when nonzero,
    and set to 0 when inside the deadband.

    Anti-windup: integral term is clamped to ±imax (in metre·seconds).
    Integral is also reset to 0 whenever the actuator is stopped (inside deadband)
    so it doesn't pre-wind before a new move.
    """

    def __init__(self, kp, ki, kd, imax, deadband, min_duty, max_duty):
        self.kp       = kp
        self.ki       = ki
        self.kd       = kd
        self.imax     = imax
        self.deadband = deadband
        self.min_duty = min_duty
        self.max_duty = max_duty

        self._integral   = 0.0
        self._prev_error = None
        self._prev_t     = None

    def reset(self):
        """Call when starting a new move or after a forced hold-clock reset."""
        self._integral   = 0.0
        self._prev_error = None
        self._prev_t     = None

    def compute(self, current_depth, target_depth):
        """
        Compute the next actuator command.

        Returns:
            (direction, duty, phase_label)
            direction: 'extend' | 'retract' | 'stop'
            duty: int, duty cycle percent (meaningful only when direction != 'stop')
            phase_label: str for logging / live status
        """
        now   = time.time()
        error = target_depth - current_depth  # + = need to rise

        # ── deadband: stop and zero the integral ──────────────────────────────
        if abs(error) <= self.deadband:
            self._integral = 0.0
            self._prev_error = error
            self._prev_t     = now
            return 'stop', 0, f'PID hold  err={error:+.3f}m'

        # ── time delta ────────────────────────────────────────────────────────
        if self._prev_t is None:
            dt = 0.1   # first tick — assume 100 ms loop
        else:
            dt = max(0.001, now - self._prev_t)   # guard against /0
        self._prev_t = now

        # ── proportional ──────────────────────────────────────────────────────
        p_term = self.kp * error

        # ── integral (anti-windup clamp) ──────────────────────────────────────
        self._integral += error * dt
        self._integral  = max(-self.imax, min(self.imax, self._integral))
        i_term = self.ki * self._integral

        # ── derivative (on error, not measurement — avoids derivative kick) ───
        if self._prev_error is None:
            d_term = 0.0
        else:
            d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # ── sum and clamp ─────────────────────────────────────────────────────
        raw_output = p_term + i_term + d_term   # signed duty-%

        if abs(raw_output) < self.min_duty:
            # Output too small to overcome stiction — use minimum duty
            clamped = self.min_duty if raw_output >= 0 else -self.min_duty
        else:
            clamped = max(-self.max_duty, min(self.max_duty, raw_output))

        duty      = int(abs(clamped))
        direction = 'extend' if clamped > 0 else 'retract'

        phase = (f'PID {direction}  err={error:+.3f}m  '
                 f'P={p_term:+.1f} I={i_term:+.1f} D={d_term:+.1f}  duty={duty}%')
        return direction, duty, phase


# Module-level PID instance (reset at each moveToDepth call)
_pid = DepthPID(
    kp=_kp, ki=_ki, kd=_kd, imax=_imax,
    deadband=_deadband_m,
    min_duty=_min_duty,
    max_duty=_full_duty,
)


# ── sensor helpers ────────────────────────────────────────────────────────────

def readTrueDepthAndPressure():
    """Return (depth_m, pressure_kpa) corrected for bias, or (None, None) on error."""
    try:
        raw_depth, pressure_kpa = depth.readDepthAndPressure()
        return raw_depth - _depth_bias_m, pressure_kpa
    except Exception as e:
        log.warning(f"Sensor read error: {e}")
        return None, None


def calibrateBias():
    global _depth_bias_m
    log.info(f"Calibrating bias over {CALIBRATION_SAMPLES} samples...")
    samples = []
    for _ in range(CALIBRATION_SAMPLES):
        d, _ = depth.readDepthAndPressure()
        if d is not None:
            samples.append(d)
        time.sleep(0.1)
    if samples:
        _depth_bias_m = sum(samples) / len(samples)
        log.info(f"Bias set to {_depth_bias_m:.4f} m")
        try:
            with open(BIAS_PATH, 'w') as f:
                json.dump({'bias_m': _depth_bias_m}, f)
        except Exception as e:
            log.warning(f"Could not save bias: {e}")
    else:
        log.warning("No valid samples — bias stays 0.0")


def loadBias():
    global _depth_bias_m
    try:
        with open(BIAS_PATH) as f:
            _depth_bias_m = json.load(f)['bias_m']
        log.info(f"Loaded bias {_depth_bias_m:.4f} m from {BIAS_PATH}")
    except Exception:
        log.info("No bias file found — using 0.0")


# ── data logging ──────────────────────────────────────────────────────────────

def createDataPacket(depth_m, pressure_kpa):
    elapsed = round(time.time() - _mission_start_t, 1) if _mission_start_t else 0.0
    return f"{CALL_SIGN},{elapsed},{pressure_kpa:.2f},{depth_m:.4f}"


def logPacket(elapsed, depth_m, pressure_kpa, packet):
    file_exists = os.path.exists(DATA_PATH)
    with open(DATA_PATH, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['company', 'time', 'pressure_kpa', 'depth_m', 'packet'])
        if not file_exists:
            writer.writeheader()
        writer.writerow({
            'company':      CALL_SIGN,
            'time':         round(elapsed, 1),
            'pressure_kpa': round(pressure_kpa, 2),
            'depth_m':      round(depth_m, 4),
            'packet':        packet,
        })


def archiveLastRun():
    """Copy data.csv to runs/ with timestamp. Keep last 10 runs."""
    if not os.path.exists(DATA_PATH):
        return
    os.makedirs(RUNS_DIR, exist_ok=True)
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    dest = os.path.join(RUNS_DIR, f'run_{ts}.csv')
    shutil.copy2(DATA_PATH, dest)
    log.info(f"Archived previous run → runs/run_{ts}.csv")
    all_runs = sorted(glob.glob(os.path.join(RUNS_DIR, 'run_*.csv')))
    for old in all_runs[:-10]:
        os.remove(old)
    os.remove(DATA_PATH)


# ── mission logic ─────────────────────────────────────────────────────────────

def moveToDepth(targetM):
    """
    Drive the float to targetM using the PID controller.

    The PID replaces the old velocity-governor / burst logic:
      - P term: responds instantly to distance from target
      - I term: eliminates steady-state offset (the main cause of not stopping)
      - D term: damps oscillation / brakes as float approaches
    The integral is reset at the start of each move to avoid windup from the
    previous hold.
    """
    log.info(f"Moving to {targetM:.2f} m  (PID kp={_kp} ki={_ki} kd={_kd})")
    _pid.reset()

    last_log = time.time() - 2.0

    while True:
        current, _ = readTrueDepthAndPressure()
        if current is None:
            time.sleep(0.1)
            continue

        # Arrival check — independent of PID so TOLERANCE_M is authoritative
        if abs(current - targetM) <= TOLERANCE_M:
            actuator.stopActuator()
            log.info(f"Arrived at {targetM:.2f} m  (sensor {current:.3f} m)")
            return

        direction, duty, phase = _pid.compute(current, targetM)

        if direction == 'stop':
            actuator.stopActuator()
        else:
            actuator.setDutyCycle(duty)
            if direction == 'extend':
                actuator.extendActuator()
            else:
                actuator.retractActuator()

        _live_update(current, phase)

        now = time.time()
        if now - last_log >= 2.0:
            log.info(f"  {phase}  sensor={current:.3f}m  target={targetM:.2f}m")
            last_log = now

        time.sleep(0.1)


def holdDepthAndLog(targetM, mission_start):
    """
    Hold at targetM for HOLD_SECONDS, logging a packet every PACKET_INTERVAL_S.

    Fine trim during hold still uses proportional nudge (same as original) so
    PID windup from the transit doesn't carry into the hold phase.
    The PID integral was already zeroed when we entered the deadband at arrival.
    """
    packets    = []
    holdStart  = time.time()
    lastPacket = holdStart
    resetCount = 0

    log.info(f"Holding at {targetM:.2f} m  (need {PACKETS_REQUIRED} packets)...")

    while len(packets) < PACKETS_REQUIRED:
        now = time.time()
        current, pressure_kpa = readTrueDepthAndPressure()
        if current is None:
            log.warning("Skipping control cycle — sensor read failed")
            time.sleep(0.1)
            continue

        diff = current - targetM

        # Drift outside tolerance → restart hold clock
        if abs(diff) > TOLERANCE_M:
            resetCount += 1
            log.warning(
                f"Depth drifted to {current:.3f} m (±{TOLERANCE_M} m limit) — "
                f"restarting hold clock (reset #{resetCount})"
            )
            packets    = []
            holdStart  = now
            lastPacket = now
            moveToDepth(targetM)
            continue

        # Fine trim inside tolerance: proportional nudge only
        # (PID integral is 0 here — we're in the deadband)
        if diff < -_deadband_m:
            actuator.setDutyCycle(_min_duty)
            actuator.retractActuator()
            _live_update(current, 'hold trim ↓')
        elif diff > _deadband_m:
            actuator.setDutyCycle(_min_duty)
            actuator.extendActuator()
            _live_update(current, 'hold trim ↑')
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
    log.info(
        f"Sensor targets: bottom {_target_bottom:.2f} m, surface {_target_surface:.2f} m"
        + (f"  (offset {_sensor_offset:+.3f} m, height {_float_height:.3f} m)"
           if _sensor_offset or _float_height else "")
    )
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
    then every 30s as a heartbeat."""
    if not os.path.exists(DATA_PATH):
        log.warning("transmitData: no data file found")
        return

    import requests

    log.info("\n--- Logged packets ---")
    with open(DATA_PATH) as f:
        for row in csv.DictReader(f):
            log.info("  " + row.get('packet', ''))

    url = f"http://{CONTROLLER_IP}:{CONTROLLER_PORT}/receive"
    log.info(f"\n--- Transmission loop → {url} ---\n")

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
                    log.info(
                        f"Attempt {attempt}: SUCCESS ({r.status_code}) — "
                        f"controller confirmed. Switching to 30s heartbeat."
                    )
                    first_success = True
                time.sleep(30)
            else:
                log.warning(f"Attempt {attempt}: HTTP {r.status_code} — retrying in 10s")
                time.sleep(10)
        except Exception as e:
            log.warning(f"Attempt {attempt}: {e} — retrying in 10s")
            time.sleep(10)


# ── surface recovery (test mode) ──────────────────────────────────────────────

def surfaceFloat():
    log.info(f"Test mode: waiting {_surface_delay}s then surfacing...")
    time.sleep(_surface_delay)
    log.info(f"Extending actuator fully for {_surface_extend}s to surface.")
    actuator.setDutyCycle(_full_duty)
    actuator.extendActuator()
    time.sleep(_surface_extend)
    actuator.stopActuator()
    log.info("Surfacing complete.")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    global _mission_start_t

    log.info("=" * 60)
    log.info(f"SEL Float — {CALL_SIGN}  {'[SIM]' if _sim_mode else ''}"
             f"  {'[TEST]' if _test_mode else '[COMPETITION]'}")
    log.info(f"PID gains  kp={_kp}  ki={_ki}  kd={_kd}  imax={_imax}")
    log.info(f"Targets    bottom={_comp_bottom} m  surface={_comp_surface} m")
    log.info(f"Sensor tgts bottom={_target_bottom:.3f} m  surface={_target_surface:.3f} m")
    log.info("=" * 60)

    actuator.setupActuator()

    archiveLastRun()
    loadBias() if os.path.exists(BIAS_PATH) else calibrateBias()

    _mission_start_t = time.time()
    _set_stage('calibrated')

    all_packets = []
    for p in range(1, NUM_PROFILES + 1):
        all_packets += executeProfile(p, _mission_start_t)

    _set_stage('profiles_complete')
    log.info(f"\nAll profiles done — {len(all_packets)} packets logged.")

    if _test_mode:
        surfaceFloat()

    _set_stage('transmitting')
    transmitData()
    _set_stage('done')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        log.info("Interrupted — stopping actuator.")
        actuator.stopActuator()
    finally:
        actuator.cleanupActuator()
