#!/usr/bin/env python3
"""
Actuator H-bridge test.

Runs the actuator fully in each direction so it exercises full travel
regardless of starting position, then leaves it stopped.

Usage:
    python test_actuator.py [--duration SECONDS]
"""
import argparse
import time
import RPi.GPIO as GPIO
import actuator
from config import ACTUATOR_TEST_DURATION_S

parser = argparse.ArgumentParser(description="Test linear actuator in both directions.")
parser.add_argument("--duration", type=float, default=ACTUATOR_TEST_DURATION_S,
                    help=f"Seconds to run each direction (default: {ACTUATOR_TEST_DURATION_S})")
args = parser.parse_args()

GPIO.setwarnings(False)
actuator.setupActuator()

try:
    print(f"Extending for {args.duration}s...")
    actuator.extendActuator()
    time.sleep(args.duration)

    print("Stopping (1s pause)...")
    actuator.stopActuator()
    time.sleep(1)

    print(f"Retracting for {args.duration}s...")
    actuator.retractActuator()
    time.sleep(args.duration)

    print("Stopping.")
    actuator.stopActuator()

except KeyboardInterrupt:
    print("\nInterrupted — stopping actuator.")
    actuator.stopActuator()

finally:
    GPIO.cleanup()
    print("Done.")
