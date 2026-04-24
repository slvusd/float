import csv
import json
import os
import time
from datetime import datetime
import actuator
import depthdetect as depth
from config import (CALL_SIGN, TARGET_BOTTOM_M, TARGET_SURFACE_M, TOLERANCE_M,
                    HOLD_SECONDS, PACKET_INTERVAL_S, PACKETS_REQUIRED,
                    NUM_PROFILES, CONTROL_DEADBAND_M, CALIBRATION_SAMPLES,
                    BIAS_FILE, DATA_FILE, CONTROLLER_IP, CONTROLLER_PORT)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
BIAS_PATH = os.path.join(BASE_DIR, BIAS_FILE)
DATA_PATH = os.path.join(BASE_DIR, DATA_FILE)

_depth_bias_m = 0.0


def loadBias():
    global _depth_bias_m
    if os.path.exists(BIAS_PATH):
        with open(BIAS_PATH) as f:
            _depth_bias_m = float(json.load(f).get('bias_m', 0.0))
        print(f"Loaded bias from file: {_depth_bias_m:.4f} m")
        return True
    return False


def calibrateBias():
    global _depth_bias_m
    print("Calibrating — float must be at the surface...")
    samples = [depth.readDepthM() for _ in range(CALIBRATION_SAMPLES) if not time.sleep(0.2)]
    _depth_bias_m = sum(samples) / len(samples)
    print(f"Depth bias: {_depth_bias_m:.4f} m")


def readTrueDepthAndPressure():
    depth_m, pressure_kpa = depth.readSensor()
    return depth_m - _depth_bias_m, pressure_kpa


def createDataPacket(depth_m, pressure_kpa):
    ts = datetime.now().strftime("%H:%M:%S")
    return f"{CALL_SIGN}  {ts}  {pressure_kpa:.1f} kPa  {depth_m:.2f} meters"


def logPacket(elapsed_s, depth_m, pressure_kpa, packet_str):
    write_header = not os.path.exists(DATA_PATH)
    with open(DATA_PATH, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(['elapsed_s', 'depth_m', 'pressure_kpa', 'packet'])
        writer.writerow([f'{elapsed_s:.1f}', f'{depth_m:.4f}',
                         f'{pressure_kpa:.1f}', packet_str])


def moveToDepth(targetM):
    print(f"Moving to {targetM:.2f} m...")
    while True:
        current, _ = readTrueDepthAndPressure()
        diff = current - targetM
        if abs(diff) <= TOLERANCE_M:
            actuator.stopActuator()
            return
        if diff < 0:
            actuator.retractActuator()
        else:
            actuator.extendActuator()
        time.sleep(0.1)


def holdDepthAndLog(targetM, mission_start):
    packets = []
    holdStart = time.time()
    lastPacketTime = holdStart

    print(f"Holding at {targetM:.2f} m...")

    while len(packets) < PACKETS_REQUIRED:
        now = time.time()
        current, pressure_kpa = readTrueDepthAndPressure()
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
            elapsed = now - mission_start
            packet = createDataPacket(current, pressure_kpa)
            packets.append(packet)
            logPacket(elapsed, current, pressure_kpa, packet)
            print(f"  Packet {len(packets)}/{PACKETS_REQUIRED}: {packet}")
            lastPacketTime = now

        time.sleep(0.1)

    return packets


def executeProfile(number, mission_start):
    print(f"\n=== Profile {number} of {NUM_PROFILES} ===")
    packets = []
    moveToDepth(TARGET_BOTTOM_M)
    packets += holdDepthAndLog(TARGET_BOTTOM_M, mission_start)
    moveToDepth(TARGET_SURFACE_M)
    packets += holdDepthAndLog(TARGET_SURFACE_M, mission_start)
    return packets


def transmitData(packets):
    print("\n--- Transmitting data ---")
    for p in packets:
        print(" ", p)

    if not os.path.exists(DATA_PATH):
        print("No data file to transmit.")
        return

    try:
        import requests
        url = f"http://{CONTROLLER_IP}:{CONTROLLER_PORT}/receive"
        with open(DATA_PATH, 'rb') as f:
            r = requests.post(url, data=f,
                              headers={'Content-Type': 'text/csv'}, timeout=10)
        print(f"Transmitted to controller: {r.status_code} {r.text}")
    except Exception as e:
        print(f"Could not reach controller ({CONTROLLER_IP}:{CONTROLLER_PORT}): {e}")


def main():
    if not depth.initSensor():
        print("Sensor init failed — aborting")
        return

    if not loadBias():
        calibrateBias()

    # Clear previous run data
    if os.path.exists(DATA_PATH):
        os.remove(DATA_PATH)

    actuator.setupActuator()
    mission_start = time.time()
    all_packets = []

    for i in range(NUM_PROFILES):
        all_packets.extend(executeProfile(i + 1, mission_start))

    print("\nProfiles complete. Holding for ROV recovery (Ctrl+C to transmit data)...")
    actuator.stopActuator()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    transmitData(all_packets)


if __name__ == "__main__":
    main()
