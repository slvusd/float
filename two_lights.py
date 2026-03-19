import RPi.GPIO as GPIO
import time


# Pin Definitions
LED_PIN_1 = 23
LED_PIN_2 = 24

# Suppress warnings
GPIO.setwarnings(False)

# Pin Setup
# Use the Broadcom (BCM) pin-numbering scheme
GPIO.setmode(GPIO.BCM)

# Set up both GPIO pins as outputs
GPIO.setup(LED_PIN_1, GPIO.OUT)
GPIO.setup(LED_PIN_2, GPIO.OUT)

print("Turning on both LEDs for 5 seconds. Press CTRL+C to exit.")

def f(state):
    if state == 0:
        GPIO.output(LED_PIN_1, GPIO.HIGH)
        GPIO.output(LED_PIN_2, GPIO.HIGH)
    elif state == 1:
        GPIO.output(LED_PIN_1, GPIO.HIGH)

f(0)
time.sleep(1)
f(1)
time.sleep(1)
f(2)
time.sleep(1)

    # Resets all GPIO ports used by this program to their default state
GPIO.cleanup()

