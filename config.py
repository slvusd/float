# GPIO pin assignments (BCM numbering)
ACTUATOR_PIN_NEGATIVE_BCM   = 23
ACTUATOR_PIN_POSITIVE_BCM   = 24
ACTUATOR_PIN_ENABLE_BCM     = 27   # H-bridge enable pin

# GPIO pin assignments (BOARD numbering)
ACTUATOR_PIN_NEGATIVE_BOARD = 16
ACTUATOR_PIN_POSITIVE_BOARD = 18
ACTUATOR_PIN_ENABLE_BOARD   = 13   # H-bridge enable pin

# Depth sensor — "02BA" for Bar02, "30BA" for Bar30
SENSOR_MODEL   = "02BA"
SENSOR_I2C_BUS = 1      # default Raspberry Pi I2C bus

# Competition call sign — update before each run
CALL_SIGN = "PLACEHOLDER01"

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

# Actuator test — seconds to run each direction
ACTUATOR_TEST_DURATION_S = 8
