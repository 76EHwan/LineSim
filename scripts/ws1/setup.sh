#!/usr/bin/env bash
# ws1 (Ubuntu) one-time setup: MinGW cross-build of linesim.exe + a shared Wine prefix.
# Run once from the ws1 login shell (no GPU, no Slurm needed):
#   cd ~/LineSim && bash scripts/ws1/setup.sh
set -euo pipefail

LINESIM_DIR="${LINESIM_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WINEPREFIX="${WINEPREFIX:-$HOME/.wine-linesim}"
cd "$LINESIM_DIR"

echo "=== [1/4] packages ==="
# ws1-lab-admin install refuses already-installed packages, so try them one at a time
# and keep going; whatever is already there is fine.
for pkg in gcc-mingw-w64-x86-64 g++-mingw-w64-x86-64 wine64 cmake ninja-build; do
    if sudo ws1-lab-admin install "$pkg"; then
        echo "  installed: $pkg"
    else
        echo "  skipped (already present or refused): $pkg"
    fi
done

command -v wine >/dev/null || { echo "ERROR: wine not on PATH - ask the admin to install it" >&2; exit 1; }
command -v x86_64-w64-mingw32-g++ >/dev/null || { echo "ERROR: MinGW cross-compiler missing" >&2; exit 1; }

echo "=== [2/4] shared Wine prefix at $WINEPREFIX ==="
# ONE prefix for every job. A prefix costs ~300 MB, so never make one per array task:
# /home is shared and has no quota. Concurrent wine processes share a wineserver fine;
# only the initial creation races, which is why it happens here and not in the jobs.
if [ ! -f "$WINEPREFIX/system.reg" ]; then
    WINEARCH=win64 WINEPREFIX="$WINEPREFIX" WINEDEBUG=-all wineboot --init
    # wineboot returns before the prefix is fully written
    WINEPREFIX="$WINEPREFIX" wineserver -w || true
else
    echo "  already initialized"
fi

echo "=== [3/4] build (MinGW -> build-linux/linesim.exe) ==="
bash scripts/build_mingw.sh

echo "=== [4/4] smoke test: one 1st run on the practice board ==="
out="$(mktemp -d)"
WINEPREFIX="$WINEPREFIX" WINEDEBUG=-all \
    wine build-linux/linesim.exe run --track tracks/practice_board.json \
        --drives 1 --quiet --max-time 60 --out "$out"
if [ -s "$out/summary.json" ]; then
    echo "  OK - summary.json written"
    rm -rf "$out"
else
    echo "ERROR: no summary.json; the Wine run failed" >&2
    echo "       output kept at $out" >&2
    exit 1
fi

cat <<MSG

=== setup complete ===
  binary : $LINESIM_DIR/build-linux/linesim.exe
  prefix : $WINEPREFIX
  next   : sbatch scripts/ws1/first_run_array.sh      (CPU, parallel 1st runs)
           sbatch scripts/ws1/gpu_batch.sh            (GPU, DirectCompute sweep)
MSG
