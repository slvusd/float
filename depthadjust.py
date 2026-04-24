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
                    BIAS_FILE, DATA_FILE, CONTROLLER_IP, CONTROLLER_PORT,
                    TEST_MODE, TEST_SURFACE_DELAY_S, TEST_SURFACE_EXTEND_S)

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


def transmitData():
    """Transmit data.csv to the controller, retrying until successful.

    The float is likely underwater with no WiFi when this starts — that is
    expected and fine.  Each attempt uses a 5-second timeout so it fails fast
    instead of hanging.  Once the ROV brings the float above water, WiFi
    reconnects and the next retry succeeds.

    Schedule:
      - Every 10 seconds until the first 200/204 response.
      - Every 30 seconds after that, indefinitely.

    Sending duplicates is harmless — the controller just overwrites the file
    each time, which is exactly what we want for debugging confidence.
    """
    if not os.path.exists(DATA_PATH):
        print("No data file found — nothing to transmit.")
        return

    import requests

    # Print all logged packets to stdout for local record
    print("\n--- Logged packets ---")
    with open(DATA_PATH) as f:
        for row in csv.DictReader(f):
            print(" ", row.get('packet', ''))

    url = f"http://{CONTROLLER_IP}:{CONTROLLER_PORT}/receive"
    print(f"\n--- Transmission loop → {url} ---")
    print("Retrying every 10s until first success, then every 30s. Ctrl+C to stop.\n")

    first_success = False
    attempt = 0

    while True:
        attempt += 1
        try:
            with open(DATA_PATH, 'rb') as f:
                r = requests.post(url, data=f,
                                  headers={'Content-Type': 'text/csv'}, timeout=5)
            if r.status_code in (200, 204):
                if not first_success:
                    print(f"Attempt {attempt}: SUCCESS ({r.status_code}) — "
                          f"controller confirmed receipt. Switching to 30s heartbeat.")
                    first_success = True
                else:
                    print(f"Attempt {attempt}: heartbeat confirmed ({r.status_code})")
            else:
                print(f"Attempt {attempt}: unexpected status {r.status_code}")
        except Exception as e:
            print(f"Attempt {attempt}: no link ({type(e).__name__}) — "
                  f"still underwater? retrying in 10s")

        time.sleep(30 if first_success else 10)


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

    print("\nProfiles complete. Actuator stopped.")
    actuator.stopActuator()

    if TEST_MODE:
        print(f"\nTEST MODE: waiting {TEST_SURFACE_DELAY_S}s then surfacing for manual retrieval...")
        time.sleep(TEST_SURFACE_DELAY_S)
        print(f"Surfacing — extending for {TEST_SURFACE_EXTEND_S}s...")
        actuator.extendActuator()
        time.sleep(TEST_SURFACE_EXTEND_S)
        actuator.stopActuator()
        print("Surfaced. Waiting for retrieval.")
    else:
        print("Waiting for ROV recovery...")

    # Immediately begin transmission loop — will keep retrying until the float
    # is above water and WiFi connects, then continues as a 30s heartbeat.
    transmitData()


if __name__ == "__main__":
    main()
