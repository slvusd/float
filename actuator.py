import RPi.GPIO as GPIO
import time

ACTUATOR_PIN_NEGATIVE_BCM = 23
ACTUATOR_PIN_POSITIVE_BCM = 24

ACTUATOR_PIN_NEGATIVE_BOARD = 16
ACTUATOR_PIN_POSITIVE_BOARD = 18

def setupActuator():
    mode = GPIO.getmode()
    if (mode == GPIO.BCM):
        GPIO.setup((ACTUATOR_PIN_NEGATIVE_BCM, ACTUATOR_PIN_POSITIVE_BCM), GPIO.OUT)
    if (mode == GPIO.BOARD):
        GPIO.setup((ACTUATOR_PIN_NEGATIVE_BOARD, ACTUATOR_PIN_POSITIVE_BOARD), GPIO.OUT)
    if (mode == None):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup((ACTUATOR_PIN_NEGATIVE_BCM, ACTUATOR_PIN_POSITIVE_BCM), GPIO.OUT)

def retractActuator():
    mode = GPIO.getmode()
    if (mode == GPIO.BCM):
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BCM, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BCM, GPIO.LOW)
    if (mode == GPIO.BOARD):
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BOARD, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BOARD, GPIO.LOW)
    if (mode == None):
         print("GPIO is not setup when retracting actuator")

def extendActuator():
    mode = GPIO.getmode()
    if (mode == GPIO.BCM):
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BCM, GPIO.LOW)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BCM, GPIO.HIGH)
    if (mode == GPIO.BOARD):
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BOARD, GPIO.LOW)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BOARD, GPIO.HIGH)
    if (mode == None):
         print("GPIO is not setup when extending actuator")

def stopActuator():
    mode = GPIO.getmode()
    if (mode == GPIO.BCM):
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BCM, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BCM, GPIO.HIGH)
    if (mode == GPIO.BOARD):
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BOARD, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BOARD, GPIO.HIGH)
    if (mode == None):
         print("GPIO is not setup when stopping actuator")


#setupActuator()
#retractActuator()
#time.sleep(3)
#for i in range(0, 10):
#    extendActuator()
#    time.sleep(0.1)
#    stopActuator()
#    time.sleep(1)
