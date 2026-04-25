import ms5837
import time
import math
import csv
from datetime import datetime, timezone
import RPi.GPIO as GPIO
from simple_pid import PID

# ------------------------------------------------------------------
# TB6612FNG PIN SETUP
# Pi Zero hardware PWM pins are GPIO 12 and 13 only
# ------------------------------------------------------------------
AIN1_PIN = 16   # BCM — direction pin 1
AIN2_PIN = 18   # BCM — direction pin 2
PWMA_PIN = 13   # BCM — hardware PWM (must be 12 or 13 on Pi Zero)

PWM_FREQ = 1000  # Hz

GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN1_PIN, GPIO.OUT)
GPIO.setup(AIN2_PIN, GPIO.OUT)
GPIO.setup(PWMA_PIN, GPIO.OUT)


motor_pwm = GPIO.PWM(PWMA_PIN, PWM_FREQ)
motor_pwm.start(0)

# ------------------------------------------------------------------
# BUOYANCY CONSTANTS
# ------------------------------------------------------------------
SYRINGE_MAX_ML = 200
NEUTRAL_ML     = 160   # syringe volume at neutral buoyancy

# Asymmetric authority from neutral point:
#   extend  headroom: 200 - 160 = 40  mL → ascent  → +20% of range
#   retract headroom: 160 - 0   = 160 mL → descent → -80% of range
ASCENT_LIMIT  = (40  / SYRINGE_MAX_ML) * 100   # 20.0
DESCENT_LIMIT = (160 / SYRINGE_MAX_ML) * 100   # 80.0

MAX_DUTY = 55    # starting point for 3V H-bridge — tune in ±5% steps
DEADBAND = 1.5   # PID outputs within ±DEADBAND stop the actuator

# ------------------------------------------------------------------
# TRAJECTORY CONSTANTS
# Top = 0.4m (near surface hold), Bottom = 2.5m (deep hold)
# ------------------------------------------------------------------
DEPTH_TOP    = 0.40   # meters
DEPTH_BOTTOM = 2.50   # meters
HOLD_TIME    = 30.0   # seconds to hold at each extreme
CYCLES       = 2      # number of full up/down cycles
TRANSIT_TIME = 90.0   # seconds to travel between depths — tune this first

# Sine parameters derived from task depths
AMPLITUDE = (DEPTH_BOTTOM - DEPTH_TOP) / 2   # 1.05 m
OFFSET    = (DEPTH_BOTTOM + DEPTH_TOP)  / 2  # 1.45 m

# Total mission duration
CYCLE_DURATION = 2 * HOLD_TIME + 2 * TRANSIT_TIME   # 240 s per cycle
TOTAL_DURATION = CYCLES * CYCLE_DURATION              # 480 s total

# ------------------------------------------------------------------
# LOGGING
# ------------------------------------------------------------------
COMPANY      = "RN08"
LOG_INTERVAL = 5.0
LOG_FILE     = f"RN08_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}.csv"

def init_log():
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["company", "utc_time", "pressure_mbar", "depth_m", "setpoint_m"])
    print(f"Logging to {LOG_FILE}")

def log_data(depth: float, pressure: float, setpoint: float):
    utc = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'
    with open(LOG_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([COMPANY, utc, f"{pressure:.4f}", f"{depth:.4f}", f"{setpoint:.4f}"])

# ------------------------------------------------------------------
# SENSOR
# ------------------------------------------------------------------
sensor = None

def init_sensor() -> bool:
    global sensor
    sensor = ms5837.MS5837_02BA()
    if sensor is None:
        print("Failed to create sensor")
        return False
    if not sensor.init():
        print("Failed to init sensor")
        return False
    sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
    return True

def read_sensor() -> tuple[float, float] | tuple[None, None]:
    """Returns (depth_m, pressure_mbar) or (None, None) on failure."""
    if sensor is None:
        print("Sensor not initialized")
        return None, None
    if sensor.read(ms5837.OSR_8192):
        return sensor.depth(), sensor.pressure()
    return None, None

# ------------------------------------------------------------------
# ACTUATOR — TB6612FNG
# Direction: AIN1/AIN2 HIGH/LOW or LOW/HIGH
# Speed:     PWMA duty cycle
# ------------------------------------------------------------------
def set_actuator(command: float):
    """
    command > 0 → extend (fill syringe → ascend)   max +20 from PID
    command < 0 → retract (empty syringe → descend) max -80 from PID
    """
    if command > DEADBAND:
        duty = (command / ASCENT_LIMIT) * MAX_DUTY
        GPIO.output(AIN1_PIN, GPIO.HIGH)
        GPIO.output(AIN2_PIN, GPIO.LOW)
        motor_pwm.ChangeDutyCycle(min(duty, MAX_DUTY))

    elif command < -DEADBAND:
        duty = (abs(command) / DESCENT_LIMIT) * MAX_DUTY
        GPIO.output(AIN1_PIN, GPIO.LOW)
        GPIO.output(AIN2_PIN, GPIO.HIGH)
        motor_pwm.ChangeDutyCycle(min(duty, MAX_DUTY))

    else:
        # Coast stop — both LOW
        GPIO.output(AIN1_PIN, GPIO.LOW)
        GPIO.output(AIN2_PIN, GPIO.LOW)
        motor_pwm.ChangeDutyCycle(0)

def stop_actuator():
    GPIO.output(AIN1_PIN, GPIO.LOW)
    GPIO.output(AIN2_PIN, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(0)

def cleanup():
    stop_actuator()
    GPIO.output(STBY_PIN, GPIO.LOW)
    motor_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up.")

# ------------------------------------------------------------------
# SINE TRAJECTORY
# Piecewise: flat hold at extremes, cosine-eased transit between them
#
# One cycle (CYCLE_DURATION seconds):
#   [0,              HOLD_TIME]                → hold at DEPTH_TOP
#   [HOLD_TIME,      HOLD_TIME + TRANSIT_TIME] → cosine descent to DEPTH_BOTTOM
#   [HT + TT,        2*HT + TT]               → hold at DEPTH_BOTTOM
#   [2*HT + TT,      2*HT + 2*TT]             → cosine ascent to DEPTH_TOP
# ------------------------------------------------------------------
def get_setpoint(elapsed: float) -> float:
    """Returns target depth in meters for a given elapsed time."""
    elapsed = min(elapsed, TOTAL_DURATION)
    cycle_t = elapsed % CYCLE_DURATION

    HT = HOLD_TIME
    TT = TRANSIT_TIME

    if cycle_t < HT:
        # Hold at top
        return DEPTH_TOP

    elif cycle_t < HT + TT:
        # Cosine descent: top → bottom
        # cos goes from 1→-1 over π, so -cos goes 0→1 (top→bottom)
        t     = cycle_t - HT
        phase = (t / TT) * math.pi
        return OFFSET - AMPLITUDE * math.cos(phase)

    elif cycle_t < 2 * HT + TT:
        # Hold at bottom
        return DEPTH_BOTTOM

    else:
        # Cosine ascent: bottom → top
        t     = cycle_t - (2 * HT + TT)
        phase = (t / TT) * math.pi
        return OFFSET + AMPLITUDE * math.cos(phase)

# ------------------------------------------------------------------
# PID
# output_limits reflect asymmetric syringe authority: (-80, +20)
# differential_on_measurement prevents derivative kick on setpoint changes
# ------------------------------------------------------------------
pid = PID(
    Kp=20.0,
    Ki=1.0,
    Kd=5.0,
    setpoint=DEPTH_TOP,
    output_limits=(-DESCENT_LIMIT, ASCENT_LIMIT),
    sample_time=0.05,
    differential_on_measurement=True,
)

# ------------------------------------------------------------------
# MAIN CONTROL LOOP
# ------------------------------------------------------------------
def run_profile():
    init_log()
    print(f"Mission start — {CYCLES} cycles, {TOTAL_DURATION:.0f}s total")

    start_time = time.time()
    last_log   = -LOG_INTERVAL   # trigger immediate first log at t=0
    last_print = 0.0

    try:
        while True:
            elapsed = time.time() - start_time

            if elapsed >= TOTAL_DURATION:
                print("Mission complete.")
                break

            # Trajectory setpoint for this moment
            setpoint     = get_setpoint(elapsed)
            pid.setpoint = setpoint

            # Read sensor
            depth, pressure = read_sensor()
            if depth is None:
                print("Bad sensor read — holding actuator")
                stop_actuator()
                time.sleep(0.1)
                continue

            # PID → actuator
            command = pid(depth)
            if command is not None:
                set_actuator(command)

            # Log every 5 seconds
            if elapsed - last_log >= LOG_INTERVAL:
                log_data(depth, pressure, setpoint)
                last_log = elapsed

            # Console output every 1 second
            if elapsed - last_print >= 1.0:
                p, i, d = pid.components
                print(
                    f"t={elapsed:6.1f}s | "
                    f"set={setpoint:.3f}m | "
                    f"depth={depth:.3f}m | "
                    f"err={setpoint - depth:+.3f}m | "
                    f"cmd={command:+.1f} "
                    f"(P={p:+.1f} I={i:+.1f} D={d:+.1f})"
                )
                last_print = elapsed

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
        raise
    finally:
        cleanup()

# ------------------------------------------------------------------
# ENTRY POINT
# ------------------------------------------------------------------
if __name__ == "__main__":
    if not init_sensor():
        raise SystemExit("Sensor init failed — check I2C wiring.")
    run_profile()
