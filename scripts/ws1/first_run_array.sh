#!/usr/bin/env bash
# Parallel LineSim 1st runs on ws1 as a Slurm array job (CPU only - no GPU is requested,
# so this never takes a shard away from anyone else).
#
#   sbatch scripts/ws1/first_run_array.sh                    # 64 runs, 8 at a time
#   sbatch -a 0-199%16 scripts/ws1/first_run_array.sh        # 200 runs, 16 at a time
#   TRACK=tracks/practice_board.json sbatch scripts/ws1/first_run_array.sh
#   REPORT=full CSV_HZ=100 sbatch scripts/ws1/first_run_array.sh   # keep plots, 2.3 MB per run
#
# Each task = one 1st run (one core, single threaded). Logs land in the submit directory.
#
#SBATCH -J linesim-first
#SBATCH -p compute
#SBATCH -a 0-63%8
#SBATCH -c 1
#SBATCH --mem=2G
#SBATCH -t 0:30:00
#SBATCH -o linesim-first-%A_%a.log
set -uo pipefail

LINESIM_DIR="${LINESIM_DIR:-$HOME/LineSim}"
LINESIM_EXE="${LINESIM_EXE:-build-linux/linesim.exe}"
OUTROOT="${OUTROOT:-$HOME/linesim-runs}"
TRACK="${TRACK:-tracks/finals_1_5x_championship.json}"
ROBOT="${ROBOT:-}"
MAXTIME="${MAXTIME:-60}"
SEED_BASE="${SEED_BASE:-0}"
REPORT="${REPORT:-light}"          # light = summary.json + marker log (~15 KB) | full (~9.9 MiB) | none (~11 KB)
CSV_HZ="${CSV_HZ:-0}"              # with REPORT=full: thin the telemetry csv to this rate (0 = every sample)
MIN_FREE_GIB="${MIN_FREE_GIB:-20}" # same floor ComfyUI uses; /home is shared and has no quota
SETS="${SETS:-}"                   # extra firmware params, e.g. SETS="base_mps=2.1 steer_kp=17"
export WINEPREFIX="${WINEPREFIX:-$HOME/.wine-linesim}"
export WINEDEBUG="${WINEDEBUG:--all}"

task="${SLURM_ARRAY_TASK_ID:-0}"
jobid="${SLURM_ARRAY_JOB_ID:-local}"
seed=$((SEED_BASE + task))
out="$OUTROOT/$jobid/$(printf '%04d' "$task")"

cd "$LINESIM_DIR" || { echo "no such directory: $LINESIM_DIR" >&2; exit 1; }
[ -f "$LINESIM_EXE" ] || { echo "$LINESIM_EXE missing - run scripts/ws1/setup.sh first" >&2; exit 1; }

# Disk guard: /home is everyone's. REPORT=light keeps a run to ~15 KB, REPORT=full to ~9.9 MiB.
free_gib=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')
if [ -n "$free_gib" ] && [ "$free_gib" -lt "$MIN_FREE_GIB" ]; then
    echo "/home has only ${free_gib} GiB free (floor ${MIN_FREE_GIB}) - refusing to start" >&2
    exit 1
fi

mkdir -p "$out"
cmd=(wine "$LINESIM_EXE" run --track "$TRACK" --drives 1 --quiet
     --seed "$seed" --max-time "$MAXTIME" --report "$REPORT" --out "$out")
# The heavy files are never written at all, so nothing has to be deleted afterwards.
[ "${CSV_HZ:-0}" != "0" ] && cmd+=(--csv-hz "$CSV_HZ")
[ -n "$ROBOT" ] && cmd+=(--robot "$ROBOT")
for kv in $SETS; do cmd+=(--set "$kv"); done

echo "[task $task] seed=$seed track=$TRACK out=$out"
start=$(date +%s)
"${cmd[@]}"
rc=$?
echo "[task $task] exit=$rc wall=$(( $(date +%s) - start ))s"

if [ ! -s "$out/summary.json" ]; then
    echo "[task $task] FAILED: no summary.json" >&2
    exit 1
fi

du -sh "$out"
