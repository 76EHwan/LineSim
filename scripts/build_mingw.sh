#!/usr/bin/env bash
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

echo "=== Building LineSim with MinGW-w64 cross-compiler ==="

mkdir -p build-linux
cd build-linux

cmake .. \
    -DCMAKE_SYSTEM_NAME=Windows \
    -DCMAKE_C_COMPILER=x86_64-w64-mingw32-gcc \
    -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++ \
    -DCMAKE_BUILD_TYPE=Release

cmake --build . --config Release -j"$(nproc 2>/dev/null || echo 2)"

echo "=== MinGW Build Complete: build-linux/linesim.exe ==="
