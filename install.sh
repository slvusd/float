#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CURRENT_USER="$(whoami)"

echo "Installing float in $SCRIPT_DIR as user $CURRENT_USER..."

# All dependencies via apt — no pip/internet needed
sudo apt-get install -y \
    python3-rpi-lgpio \
    python3-smbus2 \
    python3-flask \
    python3-matplotlib \
    python3-numpy \
    python3-requests

# Venv with access to apt-installed packages (no pip install needed)
python3 -m venv --system-site-packages "$SCRIPT_DIR/venv"

# Install systemd service
sed "s|FLOAT_USER|$CURRENT_USER|g; s|FLOAT_DIR|$SCRIPT_DIR|g" \
    "$SCRIPT_DIR/float.service" \
    | sudo tee /etc/systemd/system/float.service > /dev/null

sudo systemctl daemon-reload
sudo systemctl enable float
sudo systemctl restart float

IP=$(hostname -I | awk '{print $1}')
echo ""
echo "Setup complete."
echo "  Server: http://$IP:5000"
echo "  Manage: sudo systemctl status float"
echo "  Logs:   sudo journalctl -u float -f"
