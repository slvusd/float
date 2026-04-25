#!/usr/bin/env bash
# Mac setup for the Float simulator — no Pi hardware required.
# Runs server.py in mock mode: GPIO stubs out, sim sensor provides virtual depth.
set -e

echo "=== RN08 Float — Mac Simulator Setup ==="
echo

# Prefer a stable Python (3.13 → 3.12 → 3.11 → 3.10 → generic python3).
# Python 3.14 is pre-release and ensurepip may not work yet.
PYTHON=""
for candidate in python3.13 python3.12 python3.11 python3.10 python3; do
  if command -v "$candidate" &>/dev/null; then
    ver=$($candidate -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
    major=${ver%%.*}; minor=${ver##*.}
    if [ "$major" -eq 3 ] && [ "$minor" -ge 10 ]; then
      PYTHON=$(command -v "$candidate")
      PY_VER="$ver"
      break
    fi
  fi
done

if [ -z "$PYTHON" ]; then
  echo "ERROR: Python 3.10+ not found."
  echo "Install it with:  brew install python@3.13"
  exit 1
fi
echo "Python: $PYTHON  ($PY_VER)"

# Warn but continue on pre-release Python
PY_MINOR=${PY_VER##*.}
if [ "$PY_MINOR" -gt 13 ]; then
  echo "NOTE: Python $PY_VER is pre-release — using it, but 3.13 is recommended."
  echo "      brew install python@3.13   (if you hit issues)"
fi

# Create venv — fall back to --without-pip + bootstrap if ensurepip fails
if [ ! -f ".venv/bin/activate" ]; then
  echo "Creating virtual environment..."
  rm -rf .venv
  if ! "$PYTHON" -m venv .venv; then
    echo "  (ensurepip failed — retrying without pip, then bootstrapping)"
    rm -rf .venv
    "$PYTHON" -m venv .venv --without-pip
    echo "  Downloading get-pip.py..."
    curl -sS https://bootstrap.pypa.io/get-pip.py | .venv/bin/python
  fi
fi

source .venv/bin/activate
echo "Virtual environment: $(which python)  ($(python --version))"
echo

echo "Installing dependencies..."
pip install --quiet --upgrade pip
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
