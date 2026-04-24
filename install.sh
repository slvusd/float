#!/bin/bash
set -e

# Install system dependencies (no internet required — uses local apt cache)
# python3-rpi-lgpio is the modern drop-in replacement for python3-rpi.gpio
sudo apt-get install -y python3-rpi-lgpio python3-smbus2

# Create venv with access to the apt-installed system packages
python3 -m venv --system-site-packages venv

echo "Setup complete. Activate with: source venv/bin/activate"
