# GPIO pin assignments (BCM numbering)
ACTUATOR_PIN_NEGATIVE_BCM   = 24
ACTUATOR_PIN_POSITIVE_BCM   = 23
ACTUATOR_PIN_ENABLE_BCM     = 27   # H-bridge enable pin

# GPIO pin assignments (BOARD numbering)
ACTUATOR_PIN_NEGATIVE_BOARD = 16
ACTUATOR_PIN_POSITIVE_BOARD = 18
ACTUATOR_PIN_ENABLE_BOARD   = 13   # H-bridge enable pin

# Depth sensor — "02BA" for Bar02, "30BA" for Bar30
SENSOR_MODEL   = "02BA"
SENSOR_I2C_BUS = 1      # default Raspberry Pi I2C bus

# Competition call sign
CALL_SIGN = "RN08"

# Persistent files (relative to repo root)
BIAS_FILE = "bias.json"
DATA_FILE = "data.csv"

# Mission parameters
TARGET_BOTTOM_M     = 2.50
TARGET_SURFACE_M    = 0.40
TOLERANCE_M         = 0.33   # ±33 cm valid window around each target
CONTROL_DEADBAND_M  = 0.03   # stop actuator when within this of target
HOLD_SECONDS        = 30
PACKET_INTERVAL_S   = 5
PACKETS_REQUIRED    = 7
NUM_PROFILES        = 2
CALIBRATION_SAMPLES = 10

# ── Tuning ────────────────────────────────────────────────────────────────────
# Reduce ACTUATOR_DUTY_CYCLE to slow the piston and reduce overshoot.
# Reduce CONTROL_DEADBAND_M to chase the target more aggressively (may oscillate).
# Widen CONTROL_DEADBAND_M if the float hunts (constantly extends/retracts).
ACTUATOR_DUTY_CYCLE  = 80    # H-bridge PWM duty % (100 = full speed, ~80 recommended)
# CONTROL_DEADBAND_M is already defined above in Mission parameters

# Proportional speed: slow down as float approaches target depth.
# Within APPROACH_ZONE_M of the target, duty ramps linearly from
# ACTUATOR_DUTY_CYCLE down to MIN_DUTY_PCT. Never goes below 25% — below
# that the motor may stall under water pressure.
APPROACH_ZONE_M  = 0.80   # metres: start braking/coasting this far from target
MIN_DUTY_PCT     = 30     # minimum duty % when very close to target

# Physical float dimensions for accurate competition depth targeting.
#
# SENSOR_DEPTH_OFFSET_M: distance from the pressure sensor to the BOTTOM of the
#   float body.  For descent the competition measures to the bottom face.
#   Descent sensor target = competition_bottom - SENSOR_DEPTH_OFFSET_M
#
# FLOAT_HEIGHT_M: total height of the float body.  For ascent the competition
#   measures to the TOP face, so the sensor (which is near the bottom) will be
#   DEEPER than the top by (FLOAT_HEIGHT_M - SENSOR_DEPTH_OFFSET_M).
#   Ascent sensor target = competition_surface + FLOAT_HEIGHT_M - SENSOR_DEPTH_OFFSET_M
#
# Both default to 0.0 until measured.  Set them once in config.py.
SENSOR_DEPTH_OFFSET_M = 0.0   # metres sensor is above float bottom face
FLOAT_HEIGHT_M        = 0.0   # metres total float body height

# ── Test mode ─────────────────────────────────────────────────────────────────
# Set TEST_MODE = True for pool tests without the ROV.
# After both profiles complete, the float waits TEST_SURFACE_DELAY_S seconds
# then extends the piston fully so it floats to the surface for manual retrieval.
TEST_MODE            = False
TEST_SURFACE_DELAY_S = 60    # seconds to wait after profiles before surfacing
TEST_SURFACE_EXTEND_S = 30   # seconds to run the extend motor when surfacing

# Actuator test — seconds to run each direction
ACTUATOR_TEST_DURATION_S = 8

# Network
FLOAT_IP        = "192.168.3.120"
FLOAT_PORT      = 5000
CONTROLLER_IP   = "192.168.3.46"
CONTROLLER_PORT = 5001
