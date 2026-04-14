#!/bin/bash

set -e

VENV_DIR="venv"
PYTHON=$(command -v python3 || command -v python)

echo ""
echo "╔══════════════════════════════════════╗"
echo "║     AIS Tracker — venv setup         ║"
echo "╚══════════════════════════════════════╝"
echo ""

# Check Python
if [ -z "$PYTHON" ]; then
    echo "✗ Python not found. Install python3 first."
    exit 1
fi

PYVER=$($PYTHON --version 2>&1)
echo "✓ Using $PYVER"

# Create venv
if [ -d "$VENV_DIR" ]; then
    echo "✓ venv already exists, skipping creation"
else
    echo "→ Creating virtual environment..."
    $PYTHON -m venv "$VENV_DIR"
    echo "✓ venv created at ./$VENV_DIR"
fi

# Activate
source "$VENV_DIR/bin/activate"
echo "✓ venv activated"

# Upgrade pip silently
echo "→ Upgrading pip..."
pip install --upgrade pip --quiet

# Install dependencies
echo "→ Installing dependencies..."
echo ""

pip install \
    pyserial \
    pyais \
    websockets \
    aiohttp

echo ""
echo "╔══════════════════════════════════════╗"
echo "║         All packages installed       ║"
echo "╚══════════════════════════════════════╝"
echo ""
echo "  pyserial   — USB serial (EM-Track B100)"
echo "  pyais      — NMEA / AIS sentence parser"
echo "  websockets — WebSocket server for browser"
echo "  aiohttp    — optional HTTP server"
echo ""
echo "──────────────────────────────────────"
echo "  To activate the venv in any terminal:"
echo "    source venv/bin/activate"
echo ""
echo "  To run the stack:"
echo "    python server.py          # terminal 1"
echo "    python ais_forwarder.py   # terminal 2"
echo "    python -m http.server 8000 # terminal 3"
echo "──────────────────────────────────────"
echo ""
