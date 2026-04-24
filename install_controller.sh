#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CURRENT_USER="$(whoami)"

echo "Installing float controller in $SCRIPT_DIR as user $CURRENT_USER..."

# All dependencies via apt
sudo apt-get install -y \
    python3-flask \
    python3-matplotlib \
    python3-numpy \
    python3-requests

# Venv with access to apt-installed packages
python3 -m venv --system-site-packages "$SCRIPT_DIR/venv"

# Install systemd service
sed "s|FLOAT_USER|$CURRENT_USER|g; s|FLOAT_DIR|$SCRIPT_DIR|g" \
    "$SCRIPT_DIR/controller.service" \
    | sudo tee /etc/systemd/system/float-controller.service > /dev/null

sudo systemctl daemon-reload
sudo systemctl enable float-controller
sudo systemctl restart float-controller

IP=$(hostname -I | awk '{print $1}')
echo ""
echo "Setup complete."
echo "  Controller: http://$IP:5001"
echo "  Float UI:   http://$IP:5001/float/"
echo "  Manage:     sudo systemctl status float-controller"
echo "  Logs:       sudo journalctl -u float-controller -f"
