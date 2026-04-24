import RPi.GPIO as GPIO
import time
from config import (ACTUATOR_PIN_NEGATIVE_BCM, ACTUATOR_PIN_POSITIVE_BCM, ACTUATOR_PIN_ENABLE_BCM,
                    ACTUATOR_PIN_NEGATIVE_BOARD, ACTUATOR_PIN_POSITIVE_BOARD, ACTUATOR_PIN_ENABLE_BOARD)

def setupActuator():
    mode = GPIO.getmode()
    if mode == GPIO.BCM:
        GPIO.setup((ACTUATOR_PIN_NEGATIVE_BCM, ACTUATOR_PIN_POSITIVE_BCM, ACTUATOR_PIN_ENABLE_BCM), GPIO.OUT)
        GPIO.output(ACTUATOR_PIN_ENABLE_BCM, GPIO.HIGH)
    elif mode == GPIO.BOARD:
        GPIO.setup((ACTUATOR_PIN_NEGATIVE_BOARD, ACTUATOR_PIN_POSITIVE_BOARD, ACTUATOR_PIN_ENABLE_BOARD), GPIO.OUT)
        GPIO.output(ACTUATOR_PIN_ENABLE_BOARD, GPIO.HIGH)
    else:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup((ACTUATOR_PIN_NEGATIVE_BCM, ACTUATOR_PIN_POSITIVE_BCM, ACTUATOR_PIN_ENABLE_BCM), GPIO.OUT)
        GPIO.output(ACTUATOR_PIN_ENABLE_BCM, GPIO.HIGH)

def retractActuator():
    mode = GPIO.getmode()
    if mode == GPIO.BCM:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BCM, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BCM, GPIO.LOW)
    elif mode == GPIO.BOARD:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BOARD, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BOARD, GPIO.LOW)
    else:
        print("GPIO is not setup when retracting actuator")

def extendActuator():
    mode = GPIO.getmode()
    if mode == GPIO.BCM:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BCM, GPIO.LOW)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BCM, GPIO.HIGH)
    elif mode == GPIO.BOARD:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BOARD, GPIO.LOW)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BOARD, GPIO.HIGH)
    else:
        print("GPIO is not setup when extending actuator")

def stopActuator():
    mode = GPIO.getmode()
    if mode == GPIO.BCM:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BCM, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BCM, GPIO.HIGH)
    elif mode == GPIO.BOARD:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BOARD, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BOARD, GPIO.HIGH)
    else:
        print("GPIO is not setup when stopping actuator")


#setupActuator()
#retractActuator()
#time.sleep(3)
#for i in range(0, 10):
#    extendActuator()
#    time.sleep(0.1)
#    stopActuator()
#    time.sleep(1)
