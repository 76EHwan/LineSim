#!/usr/bin/env bash
# Runs interactive 3D simulation session inside Codespaces (renders to desktop-lite noVNC display)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

export DISPLAY=${DISPLAY:-:1}

echo "=== Launching 3D Simulation Session ==="
echo "Display target: $DISPLAY"
echo "Open port 6080 on your iPad / browser (VS Code Ports tab) to see the 3D window!"

./linesim view --session --track tracks/all_elements.json --set base_mps=2.04 "$@"
