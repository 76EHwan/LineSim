#!/usr/bin/env python3
"""Live 3D Continuous Tuner for LineSim
Runs 3D visual drives infinitely while fine-tuning parameters:
- Shows the robot driving in real-time in the 3D OpenGL viewer
- Explores fine decimal parameter tweaks around the best baseline
- Strictly checks Oversteer / Sideslip Angle (beta <= max_beta)
- Verifies marker accuracy (marker_check == "OK")
- Automatically updates the current best record whenever a faster lap is found
- Press Ctrl+C anytime to stop and keep the best parameters
"""

import argparse
import csv
import functools
import json
import math
import os
import random
import re
import shutil
import subprocess
import sys
import time

print = functools.partial(print, flush=True)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.join(ROOT, "build", "Release", "linesim.exe")
DRIVE_C = os.path.join(ROOT, "firmware", "Main", "Src", "drive.c")
TMP_DIR = os.path.join(ROOT, "runs", "live_tune_tmp")

def compute_max_sideslip_beta(csv_path, min_v=0.6):
    if not os.path.exists(csv_path):
        return 999.0
    try:
        rows = list(csv.DictReader(open(csv_path)))
        if len(rows) < 10:
            return 999.0
        max_beta = 0.0
        for i in range(1, len(rows)):
            dt = float(rows[i]["t"]) - float(rows[i-1]["t"])
            if dt < 1e-5:
                continue
            dx = float(rows[i]["x"]) - float(rows[i-1]["x"])
            dy = float(rows[i]["y"]) - float(rows[i-1]["y"])
            v = math.hypot(dx, dy) / dt
            if v < min_v:
                continue
            th_traj = math.degrees(math.atan2(dy, dx))
            th_veh = float(rows[i]["yaw_deg"])
            diff = abs((th_veh - th_traj + 180.0) % 360.0 - 180.0)
            if diff > max_beta:
                max_beta = diff
        return max_beta
    except Exception:
        return 999.0

def run_3d_drive(track, robot, params, speed=1.5, drives="1,4"):
    os.makedirs(TMP_DIR, exist_ok=True)
    cmd = [EXE, "view", "--track", track, "--robot", robot, "--drives", drives, "--out", TMP_DIR, "--speed", str(speed)]
    for k, v in params.items():
        cmd.extend(["--set", f"{k}={v}"])
        
    try:
        proc = subprocess.run(cmd, cwd=ROOT, text=True)
    except Exception as e:
        return None
        
    summary_path = os.path.join(TMP_DIR, "summary.json")
    if not os.path.exists(summary_path):
        return None
        
    try:
        with open(summary_path) as f:
            return json.load(f)
    except Exception:
        return None

def apply_to_drive_c(cand):
    if not os.path.exists(DRIVE_C):
        return False
    with open(DRIVE_C, "r", encoding="utf-8") as f:
        content = f.read()
    pattern = r"(DriveParam_t\s+driveData\s*=\s*\{)(.*?)(\};)"
    new_body = f"""
    .base_mps = {cand['base_mps']:.2f}f,
    .max_mps = {cand['max_mps']:.2f}f,
    .accel = {cand['accel']:.1f}f,
    .decel = {cand['decel']:.1f}f,
    .steer_gain_p = {cand['steer_kp']:.2f}f,
    .steer_gain_d = {cand['steer_kd']:.1f}f,
    .pos_atten_gain = 0.0f,
    .pit_in_distance_m = 0.15f,
    .fan_en = 0,
	.target_shift_val = {cand['target_shift']:.2f}f,
	.yaw_gain_p = {cand['yaw_kp']:.2f}f,
	.yaw_gain_i = {cand['yaw_ki']:.1f}f,
	.lat_acc_max = {cand['lat_acc_max']:.1f}f,
	.target_curve_gain = {cand['target_curve_gain']:.2f}f,
	.slalom_ratio = {cand['slalom_ratio']:.2f}f,
"""
    new_content = re.sub(pattern, r"\g<1>" + new_body + r"\g<3>", content, flags=re.DOTALL)
    with open(DRIVE_C, "w", encoding="utf-8") as f:
        f.write(new_content)
    print(f"  [SAVED] Applied new best parameters to {os.path.basename(DRIVE_C)}.")
    return True

def main():
    parser = argparse.ArgumentParser(description="Live 3D Continuous Tuner for LineSim")
    parser.add_argument("--track", default=os.path.join(ROOT, "tracks", "practice_board.json"), help="Track json")
    parser.add_argument("--robot", default=os.path.join(ROOT, "robots", "full.json"), help="Robot json")
    parser.add_argument("--speed", type=float, default=1.5, help="3D View playback speed multiplier (default: 1.5x)")
    parser.add_argument("--max-beta", type=float, default=35.0, help="Max allowed sideslip beta angle (default: 35.0 deg)")
    parser.add_argument("--apply", action="store_true", help="Auto-save new records to drive.c")
    args = parser.parse_args()

    print("=" * 70)
    print("  LIVE 3D CONTINUOUS TUNER")
    print(f"  Track: {os.path.basename(args.track)} | View Speed: {args.speed}x")
    print(f"  Max Sideslip Beta: {args.max_beta} deg (Oversteer Reject Filter)")
    print("  Press Ctrl+C in this terminal anytime to stop tuning.")
    print("=" * 70)

    # Initial Best baseline
    best = {
        "base_mps": 1.90,
        "max_mps": 10.0,
        "accel": 10.0,
        "decel": 10.0,
        "steer_kp": 28.0,
        "steer_kd": 425.0,
        "lat_acc_max": 13.6,
        "target_shift": 0.25,
        "target_curve_gain": 1.15,
        "slalom_ratio": 0.06,
        "pos_atten_gain": 0.0,
        "yaw_kp": 0.35,
        "yaw_ki": 1.0
    }
    best_lap4 = 9.970
    trial = 0

    try:
        while True:
            trial += 1
            # Generate a fine perturbation around the best
            cand = dict(best)
            if trial > 1:
                # Perturb 1 or 2 parameters with fine decimal steps
                tweak_type = random.choice(["kp", "kd", "base", "lat", "crv", "combo"])
                if tweak_type == "kp":
                    cand["steer_kp"] = round(best["steer_kp"] + random.choice([-0.8, -0.4, -0.2, 0.2, 0.4, 0.8]), 2)
                elif tweak_type == "kd":
                    cand["steer_kd"] = round(best["steer_kd"] + random.choice([-25, -15, 15, 25]), 1)
                elif tweak_type == "base":
                    cand["base_mps"] = round(best["base_mps"] + random.choice([-0.05, -0.02, 0.02, 0.05]), 3)
                elif tweak_type == "lat":
                    cand["lat_acc_max"] = round(best["lat_acc_max"] + random.choice([-0.4, -0.2, 0.2, 0.4]), 2)
                elif tweak_type == "crv":
                    cand["target_curve_gain"] = round(best["target_curve_gain"] + random.choice([-0.05, 0.05]), 2)
                elif tweak_type == "combo":
                    cand["steer_kp"] = round(best["steer_kp"] + random.choice([-0.4, 0.4]), 2)
                    cand["lat_acc_max"] = round(best["lat_acc_max"] + random.choice([-0.2, 0.2]), 2)

                # Clamp within sensible safe boundaries
                cand["base_mps"] = max(1.70, min(2.20, cand["base_mps"]))
                cand["steer_kp"] = max(22.0, min(34.0, cand["steer_kp"]))
                cand["steer_kd"] = max(350.0, min(500.0, cand["steer_kd"]))
                cand["lat_acc_max"] = max(12.5, min(15.0, cand["lat_acc_max"]))
                cand["target_curve_gain"] = max(1.0, min(1.3, cand["target_curve_gain"]))

            print(f"\n[Trial #{trial:03d}] Running 3D View: base={cand['base_mps']:.2f}, kp={cand['steer_kp']:.2f}, kd={cand['steer_kd']:.1f}, lat={cand['lat_acc_max']:.1f}...")
            
            res = run_3d_drive(args.track, args.robot, cand, speed=args.speed, drives="1,4")
            if not res or not res.get("ok"):
                print("  -> Run aborted or failed.")
                continue

            drives = {d["drive"]: d for d in res.get("drives", [])}
            d1 = drives.get(1)
            d4 = drives.get(4)

            if not d1 or d1.get("marker_check") != "OK":
                print(f"  -> Drive 1 markers wrong: {d1.get('marker_check', '?') if d1 else 'missing'}")
                continue
            if not d4 or d4.get("outcome") != "finished" or d4.get("marker_check") != "OK":
                print(f"  -> Drive 4 failed/wrong markers: {d4.get('marker_check', '?') if d4 else 'missing'}")
                continue

            # Sideslip beta check
            b4 = compute_max_sideslip_beta(os.path.join(TMP_DIR, "drive4.csv"))
            lap4 = d4["lap_time_fw"]
            print(f"  -> Drive 4 Finish: lap = {lap4:.3f} s (Best: {best_lap4:.3f} s) | Sideslip Beta = {b4:.1f}°")

            if b4 > args.max_beta:
                print(f"  [REJECTED] Oversteer detected (beta={b4:.1f}° > {args.max_beta}°)! Discarded.")
                continue

            if lap4 < best_lap4 - 0.005:
                gain = best_lap4 - lap4
                best_lap4 = lap4
                best = cand
                print("=" * 70)
                print(f"  *** NEW RECORD! Drive 4: {best_lap4:.3f} s (-{gain*1000:.1f} ms) ***")
                print(f"  Parameters: base_mps={best['base_mps']}, kp={best['steer_kp']}, kd={best['steer_kd']}, lat={best['lat_acc_max']}")
                print("=" * 70)
                if args.apply:
                    apply_to_drive_c(best)
            else:
                print(f"  -> No gain (current best: {best_lap4:.3f} s)")

    except KeyboardInterrupt:
        print("\n" + "=" * 70)
        print("  LIVE 3D TUNING STOPPED BY USER")
        print(f"  Final Best Drive 4 Lap Time: {best_lap4:.3f} s")
        print(f"  Final Best Parameters: {best}")
        print("=" * 70)
        if args.apply:
            apply_to_drive_c(best)

if __name__ == "__main__":
    main()
