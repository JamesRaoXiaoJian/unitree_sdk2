#!/bin/bash
# Setup script for G1 CSV motion replay
# Installs Python dependencies for MuJoCo simulation

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "Setting up G1 CSV motion replay environment..."
echo "Script directory: $SCRIPT_DIR"

# Find Python (prefer conda, fallback to system)
PYTHON=""
if command -v conda &> /dev/null; then
    PYTHON="$(conda run which python 2>/dev/null || true)"
fi
if [ -z "$PYTHON" ]; then
    PYTHON="$(which python3 2>/dev/null || which python 2>/dev/null || true)"
fi
if [ -z "$PYTHON" ]; then
    echo "ERROR: Python not found"
    exit 1
fi

echo "Python: $PYTHON ($($PYTHON --version 2>&1))"

# Install dependencies
echo "Installing Python dependencies..."
$PYTHON -m pip install -r "$SCRIPT_DIR/requirements.txt"

# Verify installation
echo "Verifying installation..."
$PYTHON -c "import mujoco; print(f'  mujoco {mujoco.__version__} OK')"
$PYTHON -c "import numpy; print(f'  numpy {numpy.__version__} OK')"

# Verify model assets
XML_PATH="$SCRIPT_DIR/assets/g1/g1_sim2sim_29dof.xml"
if [ -f "$XML_PATH" ]; then
    echo "  Model: $XML_PATH OK"
    MESH_COUNT=$(ls "$SCRIPT_DIR/assets/g1/meshes/"*.STL 2>/dev/null | wc -l)
    echo "  Meshes: $MESH_COUNT STL files"
else
    echo "WARNING: Model not found at $XML_PATH"
fi

echo ""
echo "Setup complete! Usage:"
echo "  $PYTHON $SCRIPT_DIR/csv_replay_mujoco.py --csv path/to/motion.csv"
