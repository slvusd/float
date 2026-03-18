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

try:
    # Turn on both LEDs (set to HIGH/True)
    GPIO.output(LED_PIN_1, GPIO.HIGH)
    GPIO.output(LED_PIN_2, GPIO.HIGH)
    
    # Wait for 5 seconds
    time.sleep(5)
    
    # Turn off both LEDs (set to LOW/False)
    GPIO.output(LED_PIN_1, GPIO.LOW)
    GPIO.output(LED_PIN_2, GPIO.LOW)

except KeyboardInterrupt:
    # Trap a CTRL+C keyboard interrupt
    pass

finally:
    # Resets all GPIO ports used by this program to their default state
    GPIO.cleanup()

