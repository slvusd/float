#!/usr/bin/env python3
"""
depthadjust.py  — SEL Technologies float buoyancy engine
 
Control flow per profile:
  1. moveToDepth()        — PID transit with approach-zone duty ramping
  2. findNeutral()        — stop motor, observe drift, nudge until stable,
                            record neutral syringe % for this depth
  3. holdDepthAndLog()    — motor off; nudge only if tolerance broken;
                            clock resets only on large / sustained excursions
 
Neutral buoyancy is re-estimated at every hold via drift observation rather
than trusting accumulated on-time integration alone.  The estimated syringe
position (0 % = fully retracted, 100 % = fully extended) is tracked by
integrating actuator on-time against ACTUATOR_FULL_STROKE_S and is used to
guide nudges toward the neutral point.
 
Tuning constants (config.py or CLI):
  ACTUATOR_FULL_STROKE_S  — seconds for a full retract→extend stroke at 80 %
  NEUTRAL_SEARCH_WINDOW_S — seconds to observe drift before deciding direction
  NEUTRAL_NUDGE_S         — duration of each neutral-seeking nudge
  NEUTRAL_STABLE_COUNT    — consecutive stable windows needed to declare neutral
  HOLD_EXCURSION_S        — seconds outside tolerance before clock resets
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
 
# ── PID defaults ──────────────────────────────────────────────────────────────
try:
    from config import PID_KP, PID_KI, PID_KD, PID_IMAX
except ImportError:
    PID_KP   = 20.0
    PID_KI   = 8.0
    PID_KD   = 20.0
    PID_IMAX = 0.2
 
# ── neutral buoyancy defaults ─────────────────────────────────────────────────
# Add these to config.py to override permanently.
try:
    from config import (
        ACTUATOR_FULL_STROKE_S,
        NEUTRAL_SEARCH_WINDOW_S,
        NEUTRAL_NUDGE_S,
        NEUTRAL_STABLE_COUNT,
        HOLD_EXCURSION_S,
    )
except ImportError:
    ACTUATOR_FULL_STROKE_S  = 12.0   # seconds for full stroke at 80 % duty
    NEUTRAL_SEARCH_WINDOW_S = 2.0    # seconds to watch drift before nudging
    NEUTRAL_NUDGE_S         = 0.4    # seconds per neutral-seeking nudge
    NEUTRAL_STABLE_COUNT    = 3      # stable windows in a row → neutral found
    HOLD_EXCURSION_S        = 2.0    # seconds continuously outside tolerance
                                     # before hold clock resets
 
# ── CLI args ──────────────────────────────────────────────────────────────────
 
_parser = argparse.ArgumentParser(add_help=False)
_parser.add_argument('--test',            action='store_true')
_parser.add_argument('--sim',             action='store_true')
_parser.add_argument('--sim-rate',        type=float, default=None,  dest='sim_rate')
_parser.add_argument('--duty',            type=float, default=None)
_parser.add_argument('--deadband',        type=float, default=None)
_parser.add_argument('--surface-delay',   type=float, default=None,  dest='surface_delay')
_parser.add_argument('--surface-extend',  type=float, default=None,  dest='surface_extend')
_parser.add_argument('--target-bottom',   type=float, default=None,  dest='target_bottom')
_parser.add_argument('--target-surface',  type=float, default=None,  dest='target_surface')
_parser.add_argument('--sensor-offset',   type=float, default=None,  dest='sensor_offset')
_parser.add_argument('--approach-zone',   type=float, default=None,  dest='approach_zone')
_parser.add_argument('--min-duty',        type=float, default=None,  dest='min_duty')
_parser.add_argument('--float-height',    type=float, default=None,  dest='float_height')
_parser.add_argument('--kp',   type=float, default=None)
_parser.add_argument('--ki',   type=float, default=None)
_parser.add_argument('--kd',   type=float, default=None)
_parser.add_argument('--imax', type=float, default=None)
_args, _ = _parser.parse_known_args()
 
# Runtime values — CLI overrides config
_sim_mode       = _args.sim
_sim_rate       = _args.sim_rate       if _args.sim_rate       is not None else 0.08
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
 
# PID gains
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
        """Physics-based depth model."""
        _DRAG        = 0.5
        _STROKE_RATE = 0.18
        _NATURAL     = -0.001
 
        def __init__(self, rate):
            self._depth    = 0.0
            self._velocity = 0.0
            self._syringe  = -1.0   # -1 = fully extended, +1 = fully retracted
            self._dir      = 0
            self._duty     = 80
            self._rate     = rate
            self._t        = time.time()
 
        def set_dir(self, d):  self._dir  = d
        def set_duty(self, p): self._duty = p
 
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
            return round(self._depth + random.gauss(0, 0.001), 4)
 
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
 
# ── syringe position tracker ──────────────────────────────────────────────────
 
class SyringeTracker:
    """
    Estimates syringe fill level by integrating actuator on-time.
 
    Position is expressed as a percentage:
      0 %   = fully retracted (most water in, most ballast, sinks)
      100 % = fully extended  (least water in, most buoyant, rises)
 
    Starts at 100 % because test_extend.py is run before every deployment,
    leaving the piston fully out.
 
    Call override() after findNeutral() to anchor the estimate against a
    drift-validated observation, preventing integration drift from
    accumulating across the full mission.
    """
 
    def __init__(self, full_stroke_s, initial_pct=100.0):
        self._stroke_s  = full_stroke_s
        self._position  = initial_pct
        self._last_t    = None
        self._direction = 0      # +1 extend, -1 retract, 0 stop
        self._duty      = 100.0
 
    def set_extending(self, duty_pct):
        self._flush()
        self._direction = +1
        self._duty      = duty_pct
 
    def set_retracting(self, duty_pct):
        self._flush()
        self._direction = -1
        self._duty      = duty_pct
 
    def set_stopped(self):
        self._flush()
        self._direction = 0
 
    def _flush(self):
        """Integrate elapsed time into position before a direction change."""
        now = time.time()
        if self._last_t is not None and self._direction != 0:
            dt    = now - self._last_t
            # Speed in %/s, scaled by duty ratio relative to _full_duty
            speed = 100.0 / self._stroke_s * (self._duty / _full_duty)
            self._position = max(0.0, min(100.0,
                self._position + self._direction * speed * dt))
        self._last_t = now
 
    @property
    def position(self):
        self._flush()
        return round(self._position, 1)
 
    def override(self, pct):
        """Anchor position to a drift-validated neutral observation."""
        self._flush()
        self._position = max(0.0, min(100.0, pct))
        log.info(f"Syringe position anchored to {self._position:.1f} %  (drift-validated)")
 
 
_syringe = SyringeTracker(ACTUATOR_FULL_STROKE_S, initial_pct=100.0)
 
# Neutral syringe % per sensor target depth — populated by findNeutral().
# Stored separately for surface and bottom because pressure shifts neutral.
_neutral_map: dict[float, float] = {}
 
 
def _neutral_key(target_m: float) -> float:
    return round(target_m, 2)
 
 
def _get_neutral(target_m: float):
    return _neutral_map.get(_neutral_key(target_m))
 
 
def _set_neutral(target_m: float, pct: float):
    key = _neutral_key(target_m)
    _neutral_map[key] = pct
    log.info(f"Neutral syringe for {target_m:.2f} m locked at {pct:.1f} %")
 
 
# ── state ─────────────────────────────────────────────────────────────────────
 
_depth_bias_m    = 0.9
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
    _write_status({
        'depth_m':     round(depth_m, 3),
        'elapsed_s':   elapsed,
        'actuator':    actuator_phase,
        'velocity_ms': velocity,
        'syringe_pct': _syringe.position,
    })
 
 
# ── actuator wrappers (keep syringe tracker in sync) ─────────────────────────
 
def _extend(duty=None):
    d = int(duty if duty is not None else _full_duty)
    actuator.setDutyCycle(d)
    actuator.extendActuator()
    _syringe.set_extending(d)
 
 
def _retract(duty=None):
    d = int(duty if duty is not None else _full_duty)
    actuator.setDutyCycle(d)
    actuator.retractActuator()
    _syringe.set_retracting(d)
 
 
def _stop():
    actuator.stopActuator()
    _syringe.set_stopped()
 
 
# ── PID controller ────────────────────────────────────────────────────────────
 
class DepthPID:
    """
    PID depth controller with approach-zone duty ramping.
 
    Sign convention:
      error > 0  → float is DEEPER than target → extend (rise)
      error < 0  → float is SHALLOWER than target → retract (sink)
 
    The approach zone linearly caps the maximum allowed duty from max_duty
    (at the zone boundary) down to min_duty (at the deadband edge), bleeding
    momentum before the float reaches the target.
    """
 
    def __init__(self, kp, ki, kd, imax, deadband, min_duty, max_duty, approach_zone):
        self.kp            = kp
        self.ki            = ki
        self.kd            = kd
        self.imax          = imax
        self.deadband      = deadband
        self.min_duty      = min_duty
        self.max_duty      = max_duty
        self.approach_zone = approach_zone
        self._integral     = 0.0
        self._prev_error   = None
        self._prev_t       = None
 
    def reset(self):
        self._integral   = 0.0
        self._prev_error = None
        self._prev_t     = None
 
    def compute(self, current_depth, target_depth):
        """Returns (direction, duty, phase_label)."""
        now   = time.time()
        error = target_depth - current_depth   # + = need to rise
 
        if abs(error) <= self.deadband:
            self._integral   = 0.0
            self._prev_error = error
            self._prev_t     = now
            return 'stop', 0, f'PID hold  err={error:+.3f}m'
 
        dt = 0.1 if self._prev_t is None else max(0.001, now - self._prev_t)
        self._prev_t = now
 
        p_term = self.kp * error
 
        self._integral += error * dt
        self._integral  = max(-self.imax, min(self.imax, self._integral))
        i_term = self.ki * self._integral
 
        d_term = (0.0 if self._prev_error is None
                  else self.kd * (error - self._prev_error) / dt)
        self._prev_error = error
 
        raw_output = p_term + i_term + d_term
 
        # Approach-zone duty cap
        dist = abs(error)
        if dist <= self.approach_zone:
            span     = max(self.approach_zone - self.deadband, 0.001)
            t        = (dist - self.deadband) / span
            zone_cap = int(self.min_duty + t * (self.max_duty - self.min_duty))
        else:
            zone_cap = self.max_duty
 
        if abs(raw_output) < self.min_duty:
            clamped = self.min_duty if raw_output >= 0 else -self.min_duty
        else:
            clamped = max(-zone_cap, min(zone_cap, raw_output))
 
        duty      = int(abs(clamped))
        direction = 'extend' if clamped > 0 else 'retract'
        zone_note = f' [cap={zone_cap}%]' if dist <= self.approach_zone else ''
        phase = (
            f'PID {direction}  err={error:+.3f}m  '
            f'P={p_term:+.1f} I={i_term:+.1f} D={d_term:+.1f}  '
            f'duty={duty}%{zone_note}'
        )
        return direction, duty, phase
 
 
_pid = DepthPID(
    kp=_kp, ki=_ki, kd=_kd, imax=_imax,
    deadband=_deadband_m,
    min_duty=_min_duty,
    max_duty=_full_duty,
    approach_zone=_approach_zone,
)
 
# ── sensor helpers ────────────────────────────────────────────────────────────
 
def readTrueDepthAndPressure():
    """Return (depth_m, pressure_kpa) corrected for bias, or (None, None)."""
    try:
        raw_depth, pressure_kpa = depth.readSensor()
        if raw_depth is None:
            return None, None
        return raw_depth - _depth_bias_m, pressure_kpa
    except Exception as e:
        log.warning(f"Sensor read error: {e}")
        return None, None
 
 
def _read_depth_only():
    d, _ = readTrueDepthAndPressure()
    return d
 
 
def calibrateBias():
    global _depth_bias_m
    log.info(f"Calibrating bias over {CALIBRATION_SAMPLES} samples...")
    samples = []
    for _ in range(CALIBRATION_SAMPLES):
        d, _ = depth.readSensor()
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
        writer = csv.DictWriter(
            f, fieldnames=['company', 'elapsed_s', 'pressure_kpa', 'depth_m', 'packet'])
        if not file_exists:
            writer.writeheader()
        writer.writerow({
            'company':      CALL_SIGN,
            'elapsed_s':    round(elapsed, 1),
            'pressure_kpa': round(pressure_kpa, 2),
            'depth_m':      round(depth_m, 4),
            'packet':       packet,
        })
 
 
def archiveLastRun():
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
 
 
# ── neutral buoyancy search ───────────────────────────────────────────────────
 
def findNeutral(targetM):
    """
    Determine and lock onto the neutral buoyancy syringe position at targetM.
 
    Algorithm (runs after every moveToDepth arrival):
      1. Stop motor. Observe depth over NEUTRAL_SEARCH_WINDOW_S seconds.
      2. If drift > DRIFT_THRESHOLD_M:
           sinking → extend nudge (add buoyancy)
           rising  → retract nudge (add ballast)
         Reset stable counter.
      3. If drift ≤ threshold → increment stable counter.
      4. Repeat until NEUTRAL_STABLE_COUNT consecutive stable windows.
      5. Record syringe position as neutral for this depth and anchor
         the SyringeTracker to that observation.
 
    Warm-start: if a prior neutral for this depth exists (second profile),
    nudge toward it first to shorten the search.
 
    Safety: if depth escapes TOLERANCE_M during the search, moveToDepth()
    is called to re-acquire before resuming.
    """
    log.info(f"Finding neutral buoyancy at {targetM:.2f} m ...")
    _set_stage(f"finding_neutral_{targetM:.2f}m")
 
    # ~1 cm drift per observation window is considered non-neutral
    DRIFT_THRESHOLD_M = 0.008
 
    # Warm-start toward prior neutral estimate (second profile only)
    prior = _get_neutral(targetM)
    if prior is not None:
        current_pos = _syringe.position
        diff        = prior - current_pos
        if abs(diff) > 5.0:
            nudge_s = min(abs(diff) / 100.0 * ACTUATOR_FULL_STROKE_S, 3.0)
            direction = 'extend' if diff > 0 else 'retract'
            log.info(
                f"  Warm-start: nudging {direction} {nudge_s:.1f} s "
                f"toward prior neutral {prior:.1f} %"
            )
            if diff > 0:
                _extend(_min_duty)
            else:
                _retract(_min_duty)
            time.sleep(nudge_s)
            _stop()
 
    stable_count = 0
    attempt      = 0
 
    while stable_count < NEUTRAL_STABLE_COUNT:
        attempt += 1
        _stop()
 
        d_start = _read_depth_only()
        if d_start is None:
            time.sleep(0.2)
            continue
 
        time.sleep(NEUTRAL_SEARCH_WINDOW_S)
 
        d_end = _read_depth_only()
        if d_end is None:
            time.sleep(0.2)
            continue
 
        drift = d_end - d_start   # + = sinking, - = rising
        _live_update(d_end, f'neutral search  drift={drift:+.4f}m')
        log.info(
            f"  Attempt {attempt}: depth={d_end:.3f}m  drift={drift:+.4f}m  "
            f"syringe={_syringe.position:.1f}%  "
            f"stable={stable_count}/{NEUTRAL_STABLE_COUNT}"
        )
 
        # Safety: re-acquire if we've drifted well outside tolerance
        if abs(d_end - targetM) > TOLERANCE_M:
            log.warning(
                f"  Drifted to {d_end:.3f} m during neutral search — "
                f"re-acquiring {targetM:.2f} m"
            )
            stable_count = 0
            moveToDepth(targetM)
            continue
 
        if abs(drift) <= DRIFT_THRESHOLD_M:
            stable_count += 1
        else:
            stable_count = 0
            if drift > 0:
                log.info(f"  Sinking — nudging extend {NEUTRAL_NUDGE_S} s")
                _extend(_min_duty)
            else:
                log.info(f"  Rising — nudging retract {NEUTRAL_NUDGE_S} s")
                _retract(_min_duty)
            time.sleep(NEUTRAL_NUDGE_S)
            _stop()
 
    neutral_pct = _syringe.position
    _set_neutral(targetM, neutral_pct)
    _syringe.override(neutral_pct)
    log.info(
        f"Neutral found at {targetM:.2f} m after {attempt} attempts — "
        f"syringe {neutral_pct:.1f} %"
    )
 
 
# ── mission logic ─────────────────────────────────────────────────────────────
 
def moveToDepth(targetM):
    """
    Drive the float to targetM using the PID controller.
 
    Overshoot protection:
      1. Approach-zone duty ramping inside DepthPID.compute() bleeds
         momentum as the float closes in.
      2. Overshoot reversal guard: if the float coasts past TOLERANCE_M
         in the wrong direction, min-duty correction is applied directly
         instead of letting the full PID fire into a reversed error.
    """
    log.info(
        f"Moving to {targetM:.2f} m  "
        f"(approach zone {_approach_zone:.2f} m, "
        f"PID kp={_kp} ki={_ki} kd={_kd})"
    )
    _pid.reset()
    last_log = time.time() - 2.0
 
    while True:
        current, _ = readTrueDepthAndPressure()
        if current is None:
            time.sleep(0.1)
            continue
 
        error = targetM - current   # + = need to rise
 
        # Arrival
        if abs(error) <= TOLERANCE_M:
            _stop()
            log.info(f"Arrived at {targetM:.2f} m  (sensor {current:.3f} m)")
            return
 
        # Overshoot reversal guard — gentle min-duty correction only
        if error > TOLERANCE_M:
            _retract(_min_duty)
            _live_update(current, f'overshoot retract  err={error:+.3f}m')
            time.sleep(0.1)
            continue
        elif error < -TOLERANCE_M:
            _extend(_min_duty)
            _live_update(current, f'overshoot extend  err={error:+.3f}m')
            time.sleep(0.1)
            continue
 
        # Normal PID path
        direction, duty, phase = _pid.compute(current, targetM)
        if direction == 'stop':
            _stop()
        elif direction == 'extend':
            _extend(duty)
        else:
            _retract(duty)
 
        _live_update(current, phase)
 
        now = time.time()
        if now - last_log >= 2.0:
            log.info(f"  {phase}  sensor={current:.3f}m  target={targetM:.2f}m")
            last_log = now
 
        time.sleep(0.1)
 
 
def holdDepthAndLog(targetM, mission_start):
    """
    Hold at targetM for HOLD_SECONDS, logging PACKETS_REQUIRED packets
    at PACKET_INTERVAL_S intervals.
 
    The float drifts freely at neutral buoyancy — motor is off by default.
    A single brief nudge at min duty is applied only when depth escapes the
    deadband; the motor is stopped again immediately after.
 
    Clock behaviour:
      - Keeps running through transient excursions within TOLERANCE_M.
      - Resets only if depth stays outside TOLERANCE_M continuously for
        HOLD_EXCURSION_S seconds (sustained loss of depth, not a blip).
      - On reset: packets cleared, findNeutral() re-run, clock restarts.
    """
    packets         = []
    holdStart       = time.time()
    lastPacket      = holdStart
    resetCount      = 0
    excursion_start = None
 
    log.info(
        f"Holding at {targetM:.2f} m  "
        f"(need {PACKETS_REQUIRED} packets, "
        f"clock resets after {HOLD_EXCURSION_S} s outside ±{TOLERANCE_M} m)"
    )
 
    while len(packets) < PACKETS_REQUIRED:
        now = time.time()
        current, pressure_kpa = readTrueDepthAndPressure()
        if current is None:
            log.warning("Skipping hold cycle — sensor read failed")
            time.sleep(0.1)
            continue
 
        diff   = current - targetM   # + = deeper, - = shallower
        in_tol = abs(diff) <= TOLERANCE_M
 
        # ── sustained-excursion clock reset ───────────────────────────────────
        if in_tol:
            excursion_start = None
        else:
            if excursion_start is None:
                excursion_start = now
            elif now - excursion_start >= HOLD_EXCURSION_S:
                resetCount += 1
                log.warning(
                    f"Sustained excursion at {current:.3f} m for "
                    f"{HOLD_EXCURSION_S:.0f} s — "
                    f"resetting hold clock (#{resetCount})"
                )
                packets         = []
                holdStart       = now
                lastPacket      = now
                excursion_start = None
                moveToDepth(targetM)
                findNeutral(targetM)
                continue
 
        # ── motor control: brief nudge then stop ──────────────────────────────
        if abs(diff) <= _deadband_m:
            # Inside deadband — coast at neutral, motor off
            _stop()
            _live_update(current, 'holding ◼  (neutral drift)')
        elif diff < -_deadband_m:
            # Shallower than target — brief retract nudge
            _retract(_min_duty)
            time.sleep(0.15)
            _stop()
            _live_update(current, 'hold nudge ↓')
        else:
            # Deeper than target — brief extend nudge
            _extend(_min_duty)
            time.sleep(0.15)
            _stop()
            _live_update(current, 'hold nudge ↑')
 
        # ── packet logging ────────────────────────────────────────────────────
        if now - lastPacket >= PACKET_INTERVAL_S:
            elapsed = now - mission_start
            packet  = createDataPacket(current, pressure_kpa)
            packets.append(packet)
            logPacket(elapsed, current, pressure_kpa, packet)
            log.info(
                f"  Packet {len(packets)}/{PACKETS_REQUIRED}: {packet}  "
                f"syringe={_syringe.position:.1f}%"
            )
            lastPacket = now
 
        time.sleep(0.1)
 
    if resetCount:
        log.info(f"Hold complete — {resetCount} clock reset(s) during this hold")
    return packets
 
 
def executeProfile(number, mission_start):
    log.info(f"\n=== Profile {number} of {NUM_PROFILES} ===")
    log.info(
        f"Sensor targets: bottom {_target_bottom:.2f} m, "
        f"surface {_target_surface:.2f} m"
        + (f"  (offset {_sensor_offset:+.3f} m, height {_float_height:.3f} m)"
           if _sensor_offset or _float_height else "")
    )
    packets = []
 
    # ── bottom hold ───────────────────────────────────────────────────────────
    _set_stage(f"profile_{number}_descent")
    moveToDepth(2.5)
 
    _set_stage(f"profile_{number}_neutral_bottom")
    findNeutral(2.5)
 
    _set_stage(f"profile_{number}_hold_bottom")
    packets += holdDepthAndLog(2.5, mission_start)
 
    # ── surface hold ──────────────────────────────────────────────────────────
    _set_stage(f"profile_{number}_ascent")
    moveToDepth(0.4)
 
    _set_stage(f"profile_{number}_neutral_surface")
    findNeutral(0.4)
 
    _set_stage(f"profile_{number}_hold_surface")
    packets += holdDepthAndLog(0.4, mission_start)
 
    return packets
 
 
def transmitData():
    """
    POST data.csv to controller every 10 s until first success,
    then 30 s heartbeat.  Duplicate sends are safe — controller deduplicates
    on MD5.
    """
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
                r = requests.post(
                    url, data=f,
                    headers={'Content-Type': 'text/csv'},
                    timeout=5,
                )
            if r.status_code in (200, 204):
                if not first_success:
                    log.info(
                        f"Attempt {attempt}: SUCCESS ({r.status_code}) — "
                        f"controller confirmed. Switching to 30 s heartbeat."
                    )
                    first_success = True
                time.sleep(30)
            else:
                log.warning(f"Attempt {attempt}: HTTP {r.status_code} — retrying in 10 s")
                time.sleep(10)
        except Exception as e:
            log.warning(f"Attempt {attempt}: {e} — retrying in 10 s")
            time.sleep(10)
 
 
# ── surface recovery (test mode) ──────────────────────────────────────────────
 
def surfaceFloat():
    log.info(f"Test mode: waiting {_surface_delay} s then surfacing...")
    time.sleep(_surface_delay)
    log.info(f"Extending actuator for {_surface_extend} s to surface.")
    _extend(_full_duty)
    time.sleep(_surface_extend)
    _stop()
    log.info("Surfacing complete.")
 
 
# ── main ──────────────────────────────────────────────────────────────────────
 
def main():
    global _mission_start_t
 
    log.info("=" * 60)
    log.info(
        f"SEL Float — {CALL_SIGN}  "
        f"{'[SIM] ' if _sim_mode else ''}"
        f"{'[TEST]' if _test_mode else '[COMPETITION]'}"
    )
    log.info(f"PID gains     kp={_kp}  ki={_ki}  kd={_kd}  imax={_imax}")
    log.info(
        f"Duty          full={_full_duty}%  min={_min_duty}%  "
        f"approach={_approach_zone} m"
    )
    log.info(
        f"Neutral       stroke={ACTUATOR_FULL_STROKE_S} s  "
        f"window={NEUTRAL_SEARCH_WINDOW_S} s  nudge={NEUTRAL_NUDGE_S} s  "
        f"stable={NEUTRAL_STABLE_COUNT}  excursion_reset={HOLD_EXCURSION_S} s"
    )
    log.info(f"Targets       bottom={_comp_bottom} m  surface={_comp_surface} m")
    log.info(f"Sensor tgts   bottom={_target_bottom:.3f} m  surface={_target_surface:.3f} m")
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
    if _neutral_map:
        for depth_m, pct in sorted(_neutral_map.items()):
            log.info(f"  Neutral syringe at {depth_m:.2f} m → {pct:.1f} %")
 
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
        _stop()
    finally:
        actuator.cleanupActuator()
