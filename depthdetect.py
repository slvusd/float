import ms5837
import time
from config import SENSOR_MODEL, SENSOR_I2C_BUS

sensor = None

def initSensor():
    global sensor
    if SENSOR_MODEL == "02BA":
        sensor = ms5837.MS5837_02BA(SENSOR_I2C_BUS)
    else:
        sensor = ms5837.MS5837_30BA(SENSOR_I2C_BUS)
    if sensor is None:
        print("Failed to create sensor")
        return False

    if not sensor.init():
        print("Failed to init sensor")
        return False

    return True

def readDepthM():
    if (sensor == None):
        print("Attempted to read from sensor before it was initialized")
        return None

    sensor.read(ms5837.OSR_8192)
    return sensor.depth()

def readDepthCM():
    if (sensor == None):
        print("Attempted to read from sensor before it was initialized")
        return None

    sensor.read(ms5837.OSR_8192)
    return sensor.depth() * 100

def readDepthMM():
    if (sensor == None):
        print("Attempted to read from sensor before it was initialized")
        return None

    sensor.read(ms5837.OSR_8192)
    return sensor.depth() * 1000

def readLoop():
    if (sensor == None):
        print("Attempted to read from sensor before it was initialized")
        pass

    while True:
        sensor.read(ms5837.OSR_8192)
        print(sensor.depth() * 1000)
        time.sleep(1)



#initSensor()
#readLoop()


