#!/usr/bin/env python3
"""
Pre-deployment check — extends the actuator for 10 seconds at full speed
to ensure the piston is fully out (maximum buoyancy) before entering water.

If the actuator retracts instead, either swap the wires on the H-bridge
or swap ACTUATOR_PIN_NEGATIVE_BCM and ACTUATOR_PIN_POSITIVE_BCM in config.py.
"""
import time
import RPi.GPIO as GPIO
import actuator

GPIO.setwarnings(False)
actuator.setupActuator()

print("Extending actuator for 10 seconds...")
actuator.extendActuator()
time.sleep(10)
actuator.stopActuator()

actuator.cleanupActuator()
print("Done — piston should be fully extended (maximum buoyancy).")
