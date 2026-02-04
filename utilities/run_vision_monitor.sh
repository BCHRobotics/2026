#!/bin/bash
# Setup and launch NetworkTables Vision Monitor
# This script creates a virtual environment, installs dependencies, and runs the monitor

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"

echo "=========================================="
echo "NetworkTables Vision Monitor Setup"
echo "=========================================="
echo ""

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo "❌ Error: Python 3 is not installed"
    echo "Please install Python 3 and try again"
    exit 1
fi

echo "✓ Python 3 found: $(python3 --version)"
echo ""

# Create virtual environment if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
    echo "✓ Virtual environment created"
else
    echo "✓ Virtual environment already exists"
fi
echo ""

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source "$VENV_DIR/bin/activate"
echo "✓ Virtual environment activated"
echo ""

# Upgrade pip
echo "⬆️  Upgrading pip..."
pip install --upgrade pip --quiet
echo "✓ pip upgraded"
echo ""

# Install requirements
echo "📥 Installing dependencies..."
pip install -r "$SCRIPT_DIR/requirements.txt" --quiet
echo "✓ Dependencies installed"
echo ""

# Launch the monitor
echo "=========================================="
echo "🚀 Launching Vision Monitor"
echo "=========================================="
echo ""
echo "Usage examples:"
echo "  Team number: Add --team 2026"
echo "  IP address:  Add --ip 10.20.26.2"
echo "  Simulator:   Add --simulator"
echo ""

# Pass all command line arguments to the Python script
python "$SCRIPT_DIR/nt_vision_monitor.py" "$@"

# Deactivate virtual environment on exit
deactivate
