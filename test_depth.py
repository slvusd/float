#!/usr/bin/env python3
"""
Depth sensor test — reads depth continuously and prints in mm.

Usage:
    python test_depth.py              # no bias correction
    python test_depth.py --bias 12.5  # subtract 12.5 mm from every reading
"""
import argparse
import time
import depthdetect as depth

parser = argparse.ArgumentParser(description="Read depth sensor in mm.")
parser.add_argument("--bias", type=float, default=0.0,
                    help="Bias offset in mm to subtract from raw readings (default: 0)")
args = parser.parse_args()

if not depth.initSensor():
    print("Sensor init failed — check wiring and I2C bus.")
    raise SystemExit(1)

print(f"Depth sensor ready. Bias: {args.bias:.1f} mm  |  Ctrl+C to stop.\n")
print(f"{'Raw (mm)':>12}  {'Corrected (mm)':>16}")
print("-" * 32)

try:
    while True:
        raw_mm = depth.readDepthMM()
        if raw_mm is None:
            print("Read error")
        else:
            corrected_mm = raw_mm - args.bias
            print(f"{raw_mm:>12.1f}  {corrected_mm:>16.1f}")
        time.sleep(1)

except KeyboardInterrupt:
    print("\nDone.")
