#!/bin/bash
set -e

# Install system dependencies (no internet required — uses local apt cache)
sudo apt-get install -y python3-rpi.gpio python3-smbus2

# Create venv with access to the apt-installed system packages
python3 -m venv --system-site-packages venv

echo "Setup complete. Activate with: source venv/bin/activate"
