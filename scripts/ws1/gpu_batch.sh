#!/usr/bin/env bash
# LineSim GPU 1st-run sweep (DirectCompute) on ws1, through Wine + DXVK.
#
#   sbatch scripts/ws1/gpu_batch.sh
#   CANDIDATES=4096 MAPS=8 sbatch scripts/ws1/gpu_batch.sh
#
# One job, one dispatch: tracks x candidates 1st runs at once. No per-run files are
# written - the ranking table goes to the job log.
#
#SBATCH -J linesim-gpu
#SBATCH -p compute
#SBATCH --gres=shard:4
#SBATCH -c 4
#SBATCH --mem=8G
#SBATCH -t 1:00:00
#SBATCH -o linesim-gpu-%j.log
set -uo pipefail

LINESIM_DIR="${LINESIM_DIR:-$HOME/LineSim}"
LINESIM_EXE="${LINESIM_EXE:-build-linux/linesim.exe}"
TRACK="${TRACK:-tracks/finals_1_5x_championship.json}"
CANDIDATES="${CANDIDATES:-1024}"
MAPS="${MAPS:-8}"
SEED="${SEED:-42}"
SETS="${SETS:-}"
export WINEPREFIX="${WINEPREFIX:-$HOME/.wine-linesim}"
export WINEDEBUG="${WINEDEBUG:--all}"
export DXVK_LOG_LEVEL="${DXVK_LOG_LEVEL:-none}"
# DXVK talks Vulkan, which ignores CUDA_VISIBLE_DEVICES: Slurm's shard allocation does NOT
# pin the Vulkan device. If both GPUs are visible, DXVK may pick the one Slurm did not give
# you. Set DXVK_FILTER_DEVICE_NAME to a substring of the right adapter when they differ.
[ -n "${DXVK_FILTER_DEVICE_NAME:-}" ] && export DXVK_FILTER_DEVICE_NAME

cd "$LINESIM_DIR" || { echo "no such directory: $LINESIM_DIR" >&2; exit 1; }
[ -f "$LINESIM_EXE" ] || { echo "$LINESIM_EXE missing - run scripts/ws1/setup.sh first" >&2; exit 1; }

# The shader cache lives at build/gpu_cs_<hash>.cso relative to the working directory.
# Wine's d3dcompiler_47 may fail to compile the cs_5_0 shader, so a .cso built on Windows
# is copied in here: with a cache hit, D3DCompile is never called.
mkdir -p build
if compgen -G "scripts/ws1/shader-cache/*.cso" >/dev/null; then
    cp -n scripts/ws1/shader-cache/*.cso build/ 2>/dev/null || true
    echo "shader cache: $(ls build/*.cso 2>/dev/null | wc -l) file(s) available"
fi

echo "=== Slurm gave this job ==="
echo "  CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-<unset>}  (informational only - DXVK ignores it)"
nvidia-smi --query-gpu=index,name,memory.used,memory.total --format=csv 2>/dev/null || true

cmd=(wine "$LINESIM_EXE" gpu --track "$TRACK" --candidates "$CANDIDATES"
     --random-maps "$MAPS" --seed "$SEED")
for kv in $SETS; do cmd+=(--set "$kv"); done

echo "=== running: ${cmd[*]} ==="
start=$(date +%s)
"${cmd[@]}"
rc=$?
echo "=== exit=$rc wall=$(( $(date +%s) - start ))s ==="
# Check the "Adapter:" line above: it must name the NVIDIA GPU, not a software or
# integrated device. "D3D11CreateDevice failed" means DXVK/Vulkan is not reachable here.
exit $rc
