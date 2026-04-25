#!/usr/bin/env bash
# Mac setup for the Float simulator — no Pi hardware required.
# Runs server.py in mock mode: GPIO stubs out, sim sensor provides virtual depth.
set -e

echo "=== RN08 Float — Mac Simulator Setup ==="
echo

# Require Python 3.9+
PYTHON=$(command -v python3 || true)
if [ -z "$PYTHON" ]; then
  echo "ERROR: python3 not found."
  echo "Install it with:  brew install python3"
  exit 1
fi
PY_VER=$($PYTHON -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "Python: $PYTHON  ($PY_VER)"

# Create venv
if [ ! -d ".venv" ]; then
  echo "Creating virtual environment..."
  $PYTHON -m venv .venv
fi

source .venv/bin/activate
echo "Virtual environment: $(which python)"
echo

echo "Installing dependencies..."
pip install --quiet --upgrade pip
# smbus2 is pure-Python; its import works on Mac even though I2C calls will fail.
pip install --quiet flask matplotlib numpy requests smbus2
echo "Done."

echo
echo "============================================================"
echo "  Setup complete!"
echo
echo "  To start the simulator:"
echo "    source .venv/bin/activate"
echo "    python server.py"
echo "  Then open:  http://localhost:5000/tuning"
echo
echo "  Click '🔬 Sim Run (dry)' to run the full physics simulation."
echo "  The live gauge and plot will show virtual depth in real time."
echo "  GPIO and real sensor calls are automatically stubbed out."
echo "============================================================"
