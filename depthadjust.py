import time
from datetime import datetime
import actuator
import depthdetect as depth

CALL_SIGN = "PLACEHOLDER01"

TARGET_BOTTOM_M     = 2.50
TARGET_SURFACE_M    = 0.40
TOLERANCE_M         = 0.33   # ±33 cm valid window

HOLD_SECONDS        = 30
PACKET_INTERVAL_S   = 5
PACKETS_REQUIRED    = 7
NUM_PROFILES        = 2

CONTROL_DEADBAND_M  = 0.03   # stop actuator when within this of target
CALIBRATION_SAMPLES = 10

_depth_bias_m = 0.0
_all_packets  = []


def calibrateBias():
    """Read sensor at surface (true depth = 0 m) and store offset."""
    global _depth_bias_m
    print("Calibrating — float must be at the surface...")
    samples = [depth.readDepthM() for _ in range(CALIBRATION_SAMPLES) if not time.sleep(0.2)]
    _depth_bias_m = sum(samples) / len(samples)
    print(f"Depth bias: {_depth_bias_m:.4f} m")


def readTrueDepthM():
    return depth.readDepthM() - _depth_bias_m


def createDataPacket(depthM):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return f"{CALL_SIGN}  {ts}  {depthM:.4f} m"


def moveToDepth(targetM):
    print(f"Moving to {targetM:.2f} m...")
    while True:
        current = readTrueDepthM()
        diff = current - targetM
        if abs(diff) <= TOLERANCE_M:
            actuator.stopActuator()
            return
        # diff < 0 → too shallow → retract to sink
        # diff > 0 → too deep   → extend  to rise
        if diff < 0:
            actuator.retractActuator()
        else:
            actuator.extendActuator()
        time.sleep(0.1)


def holdDepthAndLog(targetM):
    """Hold depth for HOLD_SECONDS, collecting PACKETS_REQUIRED packets at PACKET_INTERVAL_S.
    Resets the clock and packet list if depth drifts outside tolerance."""
    packets = []
    holdStart = time.time()
    lastPacketTime = holdStart

    print(f"Holding at {targetM:.2f} m...")

    while len(packets) < PACKETS_REQUIRED:
        now = time.time()
        current = readTrueDepthM()
        diff = current - targetM

        if abs(diff) > TOLERANCE_M:
            print(f"Depth drifted to {current:.3f} m — restarting hold clock")
            packets = []
            holdStart = now
            lastPacketTime = now
            moveToDepth(targetM)
            continue

        if diff < -CONTROL_DEADBAND_M:
            actuator.retractActuator()
        elif diff > CONTROL_DEADBAND_M:
            actuator.extendActuator()
        else:
            actuator.stopActuator()

        if now - lastPacketTime >= PACKET_INTERVAL_S:
            packet = createDataPacket(current)
            packets.append(packet)
            print(f"  Packet {len(packets)}/{PACKETS_REQUIRED}: {packet}")
            lastPacketTime = now

        time.sleep(0.1)

    return packets


def executeProfile(number):
    print(f"\n=== Profile {number} of {NUM_PROFILES} ===")
    packets = []
    moveToDepth(TARGET_BOTTOM_M)
    packets += holdDepthAndLog(TARGET_BOTTOM_M)
    moveToDepth(TARGET_SURFACE_M)
    packets += holdDepthAndLog(TARGET_SURFACE_M)
    return packets


def transmitData(packets):
    # Placeholder — send packets to shore receiver over wireless link
    print("\n--- Transmitting data ---")
    for p in packets:
        print(" ", p)


def main():
    if not depth.initSensor():
        print("Sensor init failed — aborting")
        return

    calibrateBias()
    actuator.setupActuator()

    for i in range(NUM_PROFILES):
        _all_packets.extend(executeProfile(i + 1))

    print("\nProfiles complete. Holding for ROV recovery (Ctrl+C to transmit data)...")
    actuator.stopActuator()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    transmitData(_all_packets)


if __name__ == "__main__":
    main()
