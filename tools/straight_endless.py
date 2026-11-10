#!/usr/bin/env python3
"""Straight-Line Section Dedicated Endless Auto-Tuner for LineSim.

Features:
- Dedicated to the high-speed straight-line scenario (scenarios/highspeed_10s_test.json).
- Multi-run validation (default: 5 consecutive runs across seeds 1..5 of Drive 3).
- FFT-based instantaneous & cruising weave analysis on straight cruise segments:
    Straight 1: s in [25.0, 65.0] m (cruise @ 7.0 m/s)
    Straight 2: s in [110.0, 160.0] m (cruise @ 7.0 m/s)
- Strictly saves ONLY configurations that pass all 5 Drive 3 runs with:
    1) outcome == "finished"
    2) marker_check == "OK" (no missed/ghost markers)
    3) Straight-line weave ratio ~1% (avg weave <= max_weave across all 5 runs).
- Parallel simulation execution across seeds for maximum throughput.
- Saves results to runs/tune/straight_endless/ (accepted_gains.csv, accepted_gains.json, best.json).
- Optional --apply to update firmware/Main/Src/drive.c with the best configuration.
- Standalone: Zero mandatory external dependencies (pure Python FFT fallback if numpy is not installed).
"""

import argparse
import cmath
import concurrent.futures
import csv
import functools
import json
import math
import os
import random
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid

# Unbuffered print
print = functools.partial(print, flush=True)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Auto re-exec with venv python if available and numpy is missing
try:
    import numpy as np
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False
    venv_py = os.path.join(ROOT, "tools", ".venv", "Scripts", "python.exe")
    if not os.path.exists(venv_py):
        venv_py = os.path.join(ROOT, "tools", ".venv", "bin", "python")
    if os.path.exists(venv_py) and os.path.abspath(sys.executable).lower() != os.path.abspath(venv_py).lower():
        try:
            os.execv(venv_py, [venv_py] + sys.argv)
        except Exception:
            pass

EXE = os.path.join(ROOT, "build", "Release", "linesim.exe")
DRIVE_C = os.path.join(ROOT, "firmware", "Main", "Src", "drive.c")
DEFAULT_SCENARIO = os.path.join(ROOT, "scenarios", "highspeed_10s_test.json")
DEFAULT_OUT_DIR = os.path.join(ROOT, "runs", "tune", "straight_endless")
TMP_RUN_BASE = os.path.join(ROOT, "runs", "tune", "straight_endless", "tmp")

g_stop_requested = False


def sigint_handler(sig, frame):
    global g_stop_requested
    print("\n[STOP REQUESTED] Gracefully finishing current round and shutting down...", flush=True)
    g_stop_requested = True


signal.signal(signal.SIGINT, sigint_handler)


def pure_fft(sig):
    """Cooley-Tukey radix-2 FFT in pure Python (exact match to np.fft.fft)."""
    n = len(sig)
    if n <= 1:
        return sig
    even = pure_fft(sig[0::2])
    odd = pure_fft(sig[1::2])
    factor = [cmath.exp(-2j * math.pi * k / n) for k in range(n // 2)]
    return [even[k] + factor[k] * odd[k] for k in range(n // 2)] + \
           [even[k] - factor[k] * odd[k] for k in range(n // 2)]


def update_drive_c(kp, kd_fw, yaw_kp=None):
    """Safely updates firmware/Main/Src/drive.c with newly discovered best parameters."""
    if not os.path.exists(DRIVE_C):
        return False
    try:
        with open(DRIVE_C, "r", encoding="utf-8") as fh:
            text = fh.read()
        text = re.sub(r"(\.steer_gain_p\s*=\s*)[0-9.]+f?", rf"\g<1>{kp:.3f}f", text)
        text = re.sub(r"(\.steer_gain_d\s*=\s*)[0-9.]+f?", rf"\g<1>{kd_fw:.1f}f", text)
        if yaw_kp is not None:
            text = re.sub(r"(\.yaw_gain_p\s*=\s*)[0-9.]+f?", rf"\g<1>{yaw_kp:.3f}f", text)
        with open(DRIVE_C, "w", encoding="utf-8") as fh:
            fh.write(text)
        return True
    except Exception as e:
        print(f"Warning: failed to update {DRIVE_C}: {e}")
        return False


def compute_weave_series(csv_path, n_fft=256, fs=100.0):
    """Computes instantaneous weave % (2.0 - 8.0 Hz energy / total power) across 10ms samples."""
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) < 1000:
        return []

    try:
        with open(csv_path, "r", encoding="utf-8", errors="ignore") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
    except Exception:
        return []

    t_wz = []
    last_t = -1.0
    for r in rows:
        try:
            t = float(r["t"])
            if t - last_t >= 0.0099:  # ~10 ms sample
                wz = float(r["wz_dps"])
                v = float(r["v"])
                s = float(r["s"])
                lat = float(r["lat_mm"])
                t_wz.append((t, s, v, lat, wz))
                last_t = t
        except (ValueError, KeyError):
            continue

    if len(t_wz) < n_fft:
        return []

    results = []
    # Precompute Hann window
    hann_win = [0.5 * (1.0 - math.cos(2.0 * math.pi * k / (n_fft - 1))) for k in range(n_fft)]

    for i in range(n_fft, len(t_wz)):
        window = [t_wz[j][4] for j in range(i - n_fft, i)]
        mean_wz = sum(window) / n_fft
        sig = [(window[k] - mean_wz) * hann_win[k] for k in range(n_fft)]

        if HAVE_NUMPY:
            fft_res = np.fft.fft(sig)
        else:
            fft_res = pure_fft(sig)

        total_power = 1e-9
        weave_power = 0.0
        for k in range(1, n_fft // 2):
            freq = k * fs / n_fft
            amp = 2.0 * abs(fft_res[k]) / n_fft
            pwr = amp * amp
            total_power += pwr
            if 2.0 <= freq <= 8.0:
                weave_power += pwr

        weave_pct = max(0.0, min(100.0, 100.0 * weave_power / total_power))
        curr_t = t_wz[i - 1][0]
        curr_s = t_wz[i - 1][1]
        curr_v = t_wz[i - 1][2]
        curr_lat = t_wz[i - 1][3]
        results.append((curr_t, curr_s, curr_v, curr_lat, weave_pct))

    return results


def evaluate_single_seed(scenario_path, params, seed, max_weave_threshold=1.25, timeout=75):
    """Executes a single simulation run (drives 1,3) with a given seed and extracts metrics."""
    run_id = f"r_{uuid.uuid4().hex[:8]}_s{seed}"
    out_dir = os.path.join(TMP_RUN_BASE, run_id)
    os.makedirs(out_dir, exist_ok=True)

    cmd = [
        EXE, "run", scenario_path,
        "--drives", "1,3",
        "--seed", str(seed),
        "--quiet",
        "--out", out_dir
    ]
    for k, v in params.items():
        cmd.extend(["--set", f"{k}={v}"])

    try:
        subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        shutil.rmtree(out_dir, ignore_errors=True)
        return {"seed": seed, "ok": False, "error": "timeout"}

    summary_path = os.path.join(out_dir, "summary.json")
    csv_path = os.path.join(out_dir, "drive3.csv")

    if not os.path.exists(summary_path) or not os.path.exists(csv_path):
        shutil.rmtree(out_dir, ignore_errors=True)
        return {"seed": seed, "ok": False, "error": "missing output"}

    try:
        with open(summary_path, "r", encoding="utf-8") as f:
            summary = json.load(f)
    except Exception as e:
        shutil.rmtree(out_dir, ignore_errors=True)
        return {"seed": seed, "ok": False, "error": f"summary json error: {e}"}

    drives = summary.get("drives", [])
    if len(drives) < 2:
        shutil.rmtree(out_dir, ignore_errors=True)
        return {"seed": seed, "ok": False, "error": "drive 3 was not executed"}

    d3 = drives[1]
    outcome = d3.get("outcome", "")
    marker_check = d3.get("marker_check", "")
    lap_time = float(d3.get("lap_time_fw", -1.0))
    rms_lat = float(d3.get("rms_lat_mm", 999.0))
    max_lat = float(d3.get("max_lat_mm", 999.0))
    max_beta = float(d3.get("max_beta_deg", 999.0))

    if outcome != "finished" or marker_check != "OK":
        shutil.rmtree(out_dir, ignore_errors=True)
        return {
            "seed": seed,
            "ok": False,
            "outcome": outcome,
            "marker_check": marker_check,
            "lap_time": lap_time,
            "error": f"{outcome} | {marker_check}"
        }

    # Extract weave series
    weaves = compute_weave_series(csv_path)
    shutil.rmtree(out_dir, ignore_errors=True)

    if not weaves:
        return {"seed": seed, "ok": False, "error": "insufficient samples for FFT"}

    # Cruise straights for highspeed_10s_test
    c1 = [r[4] for r in weaves if 25.0 <= r[1] <= 65.0]
    c2 = [r[4] for r in weaves if 110.0 <= r[1] <= 160.0]

    if not c1 or not c2:
        hs = [r[4] for r in weaves if r[2] >= 3.5]
        avg_w = float(sum(hs) / len(hs)) if hs else 99.0
        max_w = float(max(hs)) if hs else 99.0
    else:
        avg_w = float((sum(c1) / len(c1) + sum(c2) / len(c2)) / 2.0)
        max_w = float(max(max(c1), max(c2)))

    # Reject if single run weave exceeds threshold
    if avg_w > max_weave_threshold:
        return {
            "seed": seed,
            "ok": False,
            "avg_weave": avg_w,
            "max_weave": max_w,
            "lap_time": lap_time,
            "error": f"weave {avg_w:.2f}% > {max_weave_threshold:.2f}%"
        }

    return {
        "seed": seed,
        "ok": True,
        "outcome": outcome,
        "marker_check": marker_check,
        "avg_weave": avg_w,
        "max_weave": max_w,
        "lap_time": lap_time,
        "rms_lat": rms_lat,
        "max_lat": max_lat,
        "max_beta": max_beta
    }


def evaluate_candidate(scenario_path, params, num_runs=5, max_workers=5, max_weave_threshold=1.25):
    """Evaluates candidate across num_runs seeds (1..num_runs) in parallel with fail-fast."""
    seeds = list(range(1, num_runs + 1))
    results = []

    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        futs = {
            executor.submit(
                evaluate_single_seed, scenario_path, params, s, max_weave_threshold
            ): s
            for s in seeds
        }
        for fut in concurrent.futures.as_completed(futs):
            res = fut.result()
            results.append(res)
            # Fail-fast: if any seed fails or weave > threshold, cancel remaining
            if not res.get("ok", False):
                for f in futs:
                    f.cancel()
                return False, results

    passed_all = all(r.get("ok", False) for r in results) and len(results) == num_runs
    return passed_all, results


def run_straight_endless(
    scenario_path=DEFAULT_SCENARIO,
    num_runs=5,
    max_weave_threshold=1.25,
    jobs=5,
    out_dir=DEFAULT_OUT_DIR,
    hours=None,
    apply_c=False,
):
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(TMP_RUN_BASE, exist_ok=True)

    csv_log_path = os.path.join(out_dir, "accepted_gains.csv")
    json_log_path = os.path.join(out_dir, "accepted_gains.json")
    best_json_path = os.path.join(out_dir, "best.json")
    history_log_path = os.path.join(out_dir, "straight_endless.log")

    if not os.path.exists(csv_log_path):
        with open(csv_log_path, "w", encoding="utf-8") as f:
            f.write("timestamp,round,steer_kp,steer_kd,yaw_kp,avg_weave_pct,max_weave_pct,lap_time_s,rms_lat_mm,max_lat_mm\n")

    accepted_list = []
    if os.path.exists(json_log_path):
        try:
            with open(json_log_path, "r", encoding="utf-8") as f:
                accepted_list = json.load(f)
        except Exception:
            accepted_list = []

    # Elite anchor configurations
    elites = [
        {"steer_kp": 16.338, "steer_kd": 602.7, "yaw_kp": 0.450},
        {"steer_kp": 16.000, "steer_kd": 650.0, "yaw_kp": 0.450},
        {"steer_kp": 15.500, "steer_kd": 620.0, "yaw_kp": 0.420},
        {"steer_kp": 16.500, "steer_kd": 700.0, "yaw_kp": 0.480},
        {"steer_kp": 15.000, "steer_kd": 580.0, "yaw_kp": 0.450},
        {"steer_kp": 17.000, "steer_kd": 720.0, "yaw_kp": 0.500},
        {"steer_kp": 16.200, "steer_kd": 680.0, "yaw_kp": 0.460},
        {"steer_kp": 15.800, "steer_kd": 630.0, "yaw_kp": 0.440},
        {"steer_kp": 14.500, "steer_kd": 550.0, "yaw_kp": 0.400},
        {"steer_kp": 17.500, "steer_kd": 750.0, "yaw_kp": 0.480},
    ]
    for acc in accepted_list:
        elites.append({
            "steer_kp": acc["steer_kp"],
            "steer_kd": acc["steer_kd"],
            "yaw_kp": acc["yaw_kp"]
        })

    best_record = None
    if os.path.exists(best_json_path):
        try:
            with open(best_json_path, "r", encoding="utf-8") as f:
                best_record = json.load(f)
        except Exception:
            pass

    t_start = time.time()
    duration_sec = (hours * 3600.0) if hours else None
    round_no = 0

    print("=" * 82)
    print("   LINE SIM STRAIGHT-SECTION ENDLESS TUNER (HIGH SPEED 7.0 m/s)")
    print(f"   Target: Drive 3 x {num_runs} consecutive runs with Weave <= {max_weave_threshold:.2f}%")
    print(f"   Scenario: {os.path.basename(scenario_path)} | Parallel Workers: {jobs}")
    print(f"   Output Directory: {out_dir}")
    if hours:
        print(f"   Duration: {hours:g} hour(s) ({hours*60:.0f} mins)")
    else:
        print("   Running Indefinitely (Press Ctrl+C to Stop)")
    print("=" * 82 + "\n")

    try:
        while not g_stop_requested:
            t_now = time.time()
            elapsed = t_now - t_start
            if duration_sec and elapsed >= duration_sec:
                print(f"\n[TIME'S UP] Target duration of {hours:g} hour(s) has elapsed.")
                break

            round_no += 1
            elapsed_str = time.strftime("%H:%M:%S", time.gmtime(elapsed))

            # Choose candidate parameters
            if round_no <= len(elites):
                cand = dict(elites[round_no - 1])
            else:
                mode = random.random()
                if mode < 0.75 and elites:
                    # Mutate around an elite
                    parent = random.choice(elites)
                    kp = max(11.0, min(23.0, parent["steer_kp"] + random.gauss(0.0, 0.35)))
                    kd = max(300.0, min(950.0, parent["steer_kd"] + random.gauss(0.0, 30.0)))
                    yaw_kp = max(0.18, min(0.62, parent["yaw_kp"] + random.gauss(0.0, 0.035)))
                else:
                    # Random exploration across plausible range
                    kp = random.uniform(13.0, 20.0)
                    kd = random.uniform(400.0, 850.0)
                    yaw_kp = random.uniform(0.22, 0.55)

                cand = {
                    "steer_kp": round(kp, 3),
                    "steer_kd": round(kd, 1),
                    "yaw_kp": round(yaw_kp, 3),
                }

            best_banner = (
                f"BEST: {best_record['avg_weave']:.3f}% weave (Lap: {best_record['lap_time']:.2f}s) | "
                f"kp={best_record['steer_kp']:.3f}, kd={best_record['steer_kd']:.1f}, yaw_kp={best_record['yaw_kp']:.3f}"
                if best_record else "BEST: searching initial..."
            )

            print(
                f"[{elapsed_str}] Round #{round_no:04d} | Testing: kp={cand['steer_kp']:6.3f}, "
                f"kd={cand['steer_kd']:5.1f}, yaw_kp={cand['yaw_kp']:5.3f} | {best_banner}",
                flush=True
            )

            passed_all, run_results = evaluate_candidate(
                scenario_path, cand, num_runs=num_runs, max_workers=jobs, max_weave_threshold=max_weave_threshold
            )

            if g_stop_requested:
                break

            if not passed_all:
                failed_res = next((r for r in run_results if not r.get("ok", False)), None)
                fail_reason = failed_res.get("error", "unknown") if failed_res else "incomplete"
                fail_seed = failed_res.get("seed", 0) if failed_res else 0
                log_line = (
                    f"[{elapsed_str}] Round #{round_no:04d} FAIL (seed {fail_seed}: {fail_reason}) "
                    f"[kp={cand['steer_kp']:.3f}, kd={cand['steer_kd']:.1f}, yaw_kp={cand['yaw_kp']:.3f}]"
                )
                print(f"  --> {log_line}\n")
                with open(history_log_path, "a", encoding="utf-8") as f_hist:
                    f_hist.write(log_line + "\n")
                continue

            # Compute statistics across all runs
            avg_weaves = [r["avg_weave"] for r in run_results]
            max_weaves = [r["max_weave"] for r in run_results]
            lap_times = [r["lap_time"] for r in run_results]
            rms_lats = [r["rms_lat"] for r in run_results]
            max_lats = [r["max_lat"] for r in run_results]

            mean_avg_weave = sum(avg_weaves) / len(avg_weaves)
            peak_max_weave = max(max_weaves)
            mean_lap = sum(lap_times) / len(lap_times)
            mean_rms_lat = sum(rms_lats) / len(rms_lats)
            mean_max_lat = max(max_lats)

            # Qualification check: weave must be ~1%
            qualified = mean_avg_weave <= max_weave_threshold

            weaves_str = ", ".join(f"{w:.3f}%" for w in avg_weaves)
            res_summary = (
                f"Avg Weave: {mean_avg_weave:.3f}% (runs: [{weaves_str}]), "
                f"Max Weave: {peak_max_weave:.2f}%, Lap: {mean_lap:.2f}s, Lat RMS: {mean_rms_lat:.2f}mm"
            )

            if qualified:
                tag = "*** [QUALIFIED 1% WEAVE] ***"
                print(f"\n{tag}")
                print(f"  Gains: kp={cand['steer_kp']:.3f}, kd={cand['steer_kd']:.1f}, yaw_kp={cand['yaw_kp']:.3f}")
                print(f"  {res_summary}")
                print(f"  All {num_runs} runs PASSED with markers OK!")

                record = {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "round": round_no,
                    "steer_kp": cand["steer_kp"],
                    "steer_kd": cand["steer_kd"],
                    "yaw_kp": cand["yaw_kp"],
                    "avg_weave": mean_avg_weave,
                    "max_weave": peak_max_weave,
                    "lap_time": mean_lap,
                    "rms_lat": mean_rms_lat,
                    "max_lat": mean_max_lat,
                    "all_runs": run_results,
                }

                accepted_list.append(record)
                elites.append(cand)

                # Append to CSV
                with open(csv_log_path, "a", encoding="utf-8") as f_csv:
                    f_csv.write(
                        f"{record['timestamp']},{round_no},{cand['steer_kp']:.3f},{cand['steer_kd']:.1f},"
                        f"{cand['yaw_kp']:.3f},{mean_avg_weave:.4f},{peak_max_weave:.3f},{mean_lap:.3f},"
                        f"{mean_rms_lat:.3f},{mean_max_lat:.3f}\n"
                    )

                # Save updated JSON
                with open(json_log_path, "w", encoding="utf-8") as f_json:
                    json.dump(accepted_list, f_json, indent=2)

                # Update best if lowest weave or superior lap
                is_new_best = False
                if best_record is None:
                    is_new_best = True
                else:
                    score_new = mean_avg_weave * 10.0 + mean_lap * 0.2
                    score_old = best_record["avg_weave"] * 10.0 + best_record["lap_time"] * 0.2
                    if score_new < score_old:
                        is_new_best = True

                if is_new_best:
                    best_record = record
                    with open(best_json_path, "w", encoding="utf-8") as f_best:
                        json.dump(best_record, f_best, indent=2)
                    print(f"  --> NEW ALL-TIME BEST RECORD SAVED TO {best_json_path}")
                    if apply_c:
                        update_drive_c(cand["steer_kp"], cand["steer_kd"], cand["yaw_kp"])
                        print(f"  --> Auto-applied best gains to {DRIVE_C}")

                print("*" * 82 + "\n")
            else:
                log_line = (
                    f"[{elapsed_str}] Round #{round_no:04d} PASS but weave {mean_avg_weave:.3f}% > {max_weave_threshold:.2f}%: "
                    f"kp={cand['steer_kp']:.3f}, kd={cand['steer_kd']:.1f}, yaw_kp={cand['yaw_kp']:.3f}"
                )
                print(f"  --> {log_line}\n")
                with open(history_log_path, "a", encoding="utf-8") as f_hist:
                    f_hist.write(log_line + "\n")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        shutil.rmtree(TMP_RUN_BASE, ignore_errors=True)
        total_time_str = time.strftime("%H:%M:%S", time.gmtime(time.time() - t_start))
        print("\n" + "=" * 82)
        print(f"   STRAIGHT ENDLESS TUNING SUMMARY (Total Time: {total_time_str}, Rounds: {round_no})")
        print(f"   Total Qualified Gain Sets (~1% Weave): {len(accepted_list)}")
        if best_record:
            print(f"   Optimal Configuration: kp={best_record['steer_kp']:.3f}, kd={best_record['steer_kd']:.1f}, yaw_kp={best_record['yaw_kp']:.3f}")
            print(f"   Best Average Weave: {best_record['avg_weave']:.3f}% | Lap Time: {best_record['lap_time']:.2f}s | RMS Lat: {best_record['rms_lat']:.2f}mm")
            print(f"   Log Files: {csv_log_path} and {json_log_path}")
        print("=" * 82 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="LineSim Straight-Section Dedicated Endless Tuner (5-run Drive 3 verification for ~1% weave)"
    )
    parser.add_argument("--scenario", default=DEFAULT_SCENARIO, help="Scenario JSON path (default: scenarios/highspeed_10s_test.json)")
    parser.add_argument("--runs", type=int, default=5, help="Number of consecutive Drive 3 runs per candidate (default: 5)")
    parser.add_argument("--max-weave", type=float, default=1.20, help="Maximum allowable straight weave %% threshold (default: 1.20%%)")
    parser.add_argument("--jobs", type=int, default=5, help="Number of parallel simulation threads (default: 5)")
    parser.add_argument("--hours", type=float, default=None, help="Running duration in hours (default: run indefinitely until Ctrl+C)")
    parser.add_argument("--out", default=DEFAULT_OUT_DIR, help="Output directory for saved results")
    parser.add_argument("--apply", action="store_true", help="Automatically write newly found best gains to firmware/Main/Src/drive.c")

    args = parser.parse_args()
    run_straight_endless(
        scenario_path=args.scenario,
        num_runs=args.runs,
        max_weave_threshold=args.max_weave,
        jobs=args.jobs,
        out_dir=args.out,
        hours=args.hours,
        apply_c=args.apply,
    )


if __name__ == "__main__":
    main()
