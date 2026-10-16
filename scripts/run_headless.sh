#!/usr/bin/env bash
# Runs headless simulation scenario and serves the interactive HTML report
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

echo "=== Running Headless Simulation Scenario ==="
./linesim run --track tracks/all_elements.json --set base_mps=2.04 "$@"

echo ""
echo "=== Simulation Finished ==="
echo "Starting Telemetry Web Server on port 8080 for iPad / browser..."
python3 tools/serve_report.py
