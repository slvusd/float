#!/usr/bin/env python3
"""
Actuator H-bridge test.

Runs the actuator fully in each direction so it exercises full travel
regardless of starting position, then leaves it stopped.

Usage:
    python test_actuator.py                  # uses ACTUATOR_DUTY_CYCLE from config.py
    python test_actuator.py --duty 50        # override duty cycle for this run
    python test_actuator.py --duration 10    # run each direction 10s
"""
import argparse
import time
import RPi.GPIO as GPIO
import actuator
from config import ACTUATOR_TEST_DURATION_S, ACTUATOR_DUTY_CYCLE

parser = argparse.ArgumentParser(description="Test linear actuator in both directions.")
parser.add_argument("--duration", type=float, default=ACTUATOR_TEST_DURATION_S,
                    help=f"Seconds to run each direction (default: {ACTUATOR_TEST_DURATION_S})")
parser.add_argument("--duty", type=float, default=ACTUATOR_DUTY_CYCLE,
                    help=f"PWM duty cycle 0-100 (default: {ACTUATOR_DUTY_CYCLE} from config)")
args = parser.parse_args()

if not 0 < args.duty <= 100:
    print("--duty must be between 1 and 100")
    raise SystemExit(1)

GPIO.setwarnings(False)
actuator.setupActuator()
actuator.setDutyCycle(args.duty)

try:
    print(f"Extending for {args.duration}s at {args.duty:.0f}% duty...")
    actuator.extendActuator()
    time.sleep(args.duration)

    print("Stopping (1s pause)...")
    actuator.stopActuator()
    time.sleep(1)

    print(f"Retracting for {args.duration}s at {args.duty:.0f}% duty...")
    actuator.retractActuator()
    time.sleep(args.duration)

    print("Stopping.")
    actuator.stopActuator()

except KeyboardInterrupt:
    print("\nInterrupted — stopping actuator.")
    actuator.stopActuator()

finally:
    actuator.cleanupActuator()
    print("Done.")
