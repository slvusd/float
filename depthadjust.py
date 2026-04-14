
import math
import time
from datetime import datetime
import actuator
import depthdetect as depth

CALL_SIGN = "PLACEHOLDER01"
TIME_INCREMENT = 0.1
DEPTH_OFFSET_CM = 0
MAX_DISTANCE_FROM_TARGET_DEPTH_CM = 30 # technically is 33 but 30 to be safe
TARGET_DISTANCE_FROM_TARGET_DEPTH_CM = 3 # how close to the taget depth the float will aim to be


def moveToDepth(targetDepthCM):
    currentDepth = depth.readDepthCM()
    difference = currentDepth - targetDepthCM
    while (math.abs(difference) > TARGET_DISTANCE_FROM_TARGET_DEPTH_CM):
        print("Moving to target depth, Depth: %f, Target: %f, Distance: %f", currentDepth, targetDepthCM, difference)

        if (difference < 0):
            actuator.retractActuator()
        elif (difference > 0):
            actuator.extendActuator()
        else:
            print("Float doesn't know if it should go up or down (no idea how this could end up happening)")

        time.sleep(TIME_INCREMENT)

        currentDepth = depth.readDepthCM()
        difference = currentDepth - targetDepthCM

def createDataPacket(depthCM):
    formatedDateTime = datetime.now().strftime("%Y-%m-%d %H:%M:%S %Z%z")
    return CALL_SIGN + "  " + formatedDateTime + f"  {depthCM} cm"

def holdDepthAndGatherData(targetDepthCM):
    startHoldTime = time.time()

    datapackets = []

    while (time.time() - startHoldTime < 30):
        currentDepth = depth.readDepthCM()
        difference = currentDepth - targetDepthCM
        print("Holding target depth, Depth: %f, Target: %f, Distance: %f", currentDepth, targetDepthCM, difference)

        if (math.abs(difference) > MAX_DISTANCE_FROM_TARGET_DEPTH_CM):
            print("Failed depth hold")
            return None

        if (difference < TARGET_DISTANCE_FROM_TARGET_DEPTH_CM):
            actuator.retractActuator()
        elif (difference > TARGET_DISTANCE_FROM_TARGET_DEPTH_CM):
            actuator.extendActuator()
        else:
            actuator.stopActuator()

        time.sleep(TIME_INCREMENT)


while True:
    print(createDataPacket(50))
    time.sleep(1)
