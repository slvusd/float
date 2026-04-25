import gc
try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except ImportError:
    GPIO = None
    _HAS_GPIO = False

from config import (ACTUATOR_PIN_NEGATIVE_BCM, ACTUATOR_PIN_POSITIVE_BCM, ACTUATOR_PIN_ENABLE_BCM,
                    ACTUATOR_PIN_NEGATIVE_BOARD, ACTUATOR_PIN_POSITIVE_BOARD, ACTUATOR_PIN_ENABLE_BOARD,
                    ACTUATOR_DUTY_CYCLE)

_pwm = None


def _enable_pin():
    return (ACTUATOR_PIN_ENABLE_BCM if GPIO.getmode() == GPIO.BCM
            else ACTUATOR_PIN_ENABLE_BOARD)


def setupActuator():
    global _pwm
    if not _HAS_GPIO:
        return
    mode = GPIO.getmode()
    if mode is None:
        GPIO.setmode(GPIO.BCM)
        mode = GPIO.BCM
    if mode == GPIO.BCM:
        GPIO.setup((ACTUATOR_PIN_NEGATIVE_BCM, ACTUATOR_PIN_POSITIVE_BCM,
                    ACTUATOR_PIN_ENABLE_BCM), GPIO.OUT)
    else:
        GPIO.setup((ACTUATOR_PIN_NEGATIVE_BOARD, ACTUATOR_PIN_POSITIVE_BOARD,
                    ACTUATOR_PIN_ENABLE_BOARD), GPIO.OUT)
    if ACTUATOR_DUTY_CYCLE < 100:
        _pwm = GPIO.PWM(_enable_pin(), 1000)
        _pwm.start(ACTUATOR_DUTY_CYCLE)
    else:
        GPIO.output(_enable_pin(), GPIO.HIGH)


def setDutyCycle(pct):
    """Change actuator speed after setup. Call after setupActuator()."""
    global _pwm
    if not _HAS_GPIO:
        return
    if pct >= 100:
        if _pwm is not None:
            _pwm.stop()
            del _pwm
            gc.collect()
            _pwm = None
        GPIO.output(_enable_pin(), GPIO.HIGH)
    else:
        if _pwm is None:
            _pwm = GPIO.PWM(_enable_pin(), 1000)
            _pwm.start(pct)
        else:
            _pwm.ChangeDutyCycle(pct)


def retractActuator():
    if not _HAS_GPIO:
        return
    mode = GPIO.getmode()
    if mode == GPIO.BCM:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BCM, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BCM, GPIO.LOW)
    elif mode == GPIO.BOARD:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BOARD, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BOARD, GPIO.LOW)


def extendActuator():
    if not _HAS_GPIO:
        return
    mode = GPIO.getmode()
    if mode == GPIO.BCM:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BCM, GPIO.LOW)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BCM, GPIO.HIGH)
    elif mode == GPIO.BOARD:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BOARD, GPIO.LOW)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BOARD, GPIO.HIGH)


def stopActuator():
    if not _HAS_GPIO:
        return
    mode = GPIO.getmode()
    if mode == GPIO.BCM:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BCM, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BCM, GPIO.HIGH)
    elif mode == GPIO.BOARD:
        GPIO.output(ACTUATOR_PIN_NEGATIVE_BOARD, GPIO.HIGH)
        GPIO.output(ACTUATOR_PIN_POSITIVE_BOARD, GPIO.HIGH)


def cleanupActuator():
    """Stop actuator and release GPIO cleanly. Call at end of test scripts."""
    global _pwm
    if not _HAS_GPIO:
        return
    stopActuator()
    if _pwm is not None:
        _pwm.stop()
        del _pwm
        gc.collect()
        _pwm = None
    GPIO.cleanup()
