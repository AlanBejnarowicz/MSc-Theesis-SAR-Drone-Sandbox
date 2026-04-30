#!/bin/bash

PORT=9000
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "MAROPS // Starting local server..."
echo "Serving: $SCRIPT_DIR"
echo "URL:     http://localhost:$PORT/maritime_display.html"
echo "Press Ctrl+C to stop."
echo ""

cd "$SCRIPT_DIR"

# Try python3 first, fall back to python
if command -v python3 &>/dev/null; then
    python3 -m http.server $PORT
elif command -v python &>/dev/null; then
    python -m http.server $PORT
else
    echo "ERROR: Python not found. Please install Python 3."
    exit 1
fi
