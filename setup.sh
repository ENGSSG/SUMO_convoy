#!/bin/bash
# Setup script for convoy handoff demo
# Run this once before running escort_handoff.py

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Setting up convoy handoff demo in: $SCRIPT_DIR"

# Check for SUMO
if [ -z "$SUMO_HOME" ]; then
    # Try common locations
    if [ -d "/usr/share/sumo" ]; then
        export SUMO_HOME="/usr/share/sumo"
    elif [ -d "$HOME/sumo" ]; then
        export SUMO_HOME="$HOME/sumo"
    else
        echo "ERROR: SUMO_HOME not set. Please set it to your SUMO installation directory."
        echo "Example: export SUMO_HOME=/usr/share/sumo"
        exit 1
    fi
fi

echo "Using SUMO_HOME: $SUMO_HOME"

# Check for netconvert
if ! command -v netconvert &> /dev/null; then
    echo "ERROR: netconvert not found. Make sure SUMO is installed and in PATH."
    exit 1
fi

# Generate network file
echo "Generating network file..."
netconvert --node-files=network.nod.xml --edge-files=network.edg.xml --output-file=network.net.xml

echo ""
echo "Setup complete! Files created:"
ls -la *.xml

echo ""
echo "To run the simulation:"
echo "  python3 escort_handoff.py"
