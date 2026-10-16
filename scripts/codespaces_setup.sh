#!/usr/bin/env bash
set -e

echo "=== [LineSim] Initializing GitHub Codespaces Environment ==="

# 1. Update package list & install Wine, X11 OpenGL, and MinGW cross-compiler
sudo dpkg --add-architecture i386 || true
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    wine \
    wine64 \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    xvfb \
    x11-xserver-utils \
    gcc-mingw-w64-x86-64 \
    g++-mingw-w64-x86-64 \
    cmake \
    ninja-build

# 2. Initialize Wine prefix in headless mode
echo "=== Initializing Wine prefix ==="
export WINEARCH=win64
WINEDEBUG=-all wineboot --init || true

# 3. Setup Python virtual environment
if [ ! -d "tools/.venv" ]; then
    echo "=== Setting up tools/.venv ==="
    python3 -m venv tools/.venv
    tools/.venv/bin/pip install --upgrade pip setuptools
    tools/.venv/bin/pip install trimesh numpy || true
fi

# 4. Make scripts executable
chmod +x linesim scripts/*.sh || true

# 5. Check or build linesim.exe
if [ ! -f "build/Release/linesim.exe" ] && [ ! -f "build-linux/linesim.exe" ]; then
    echo "=== Building linesim.exe using MinGW ==="
    bash scripts/build_mingw.sh || true
fi

echo "=== [LineSim] Codespaces Setup Complete! ==="
echo "1) 3D GUI Viewer: Open port 6080 (Desktop noVNC) on your iPad Safari, then run ./scripts/run_view.sh"
echo "2) Headless Simulator: Run ./scripts/run_headless.sh (view reports on port 8080)"
