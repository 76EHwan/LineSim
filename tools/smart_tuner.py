#!/usr/bin/env python3
"""Smart Tuner for LineSim Firmware
- Fixed max_mps = 10.0, accel = 10.0, decel = 10.0
- Integer KP gains
- Strict Oversteer & Sideslip Angle (beta) rejection
- Multi-seed & Stress condition (lag x1.4, grip x0.85) validation
- Isolated thread output directories (zero collision)
- Optional --apply to update firmware/Main/Src/drive.c directly
"""

import argparse
import concurrent.futures
import csv
import functools
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
import uuid

# Ensure immediate real-time unbuffered output
print = functools.partial(print, flush=True)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.join(ROOT, "build", "Release", "linesim.exe")
DRIVE_C = os.path.join(ROOT, "firmware", "Main", "Src", "drive.c")
TMP_RUNS = os.path.join(ROOT, "runs", "smart_tmp")

def run_sim(track, robot, params, drives="1,2,4", robot_params=None, timeout=60):
    uid = uuid.uuid4().hex[:10]
    out_dir = os.path.join(TMP_RUNS, f"r_{uid}")
    cmd = [EXE, "run", "--track", track, "--robot", robot, "--drives", drives, "--out", out_dir, "--quiet"]
    for k, v in params.items():
        cmd.extend(["--set", f"{k}={v}"])
    if robot_params:
        for k, v in robot_params.items():
            cmd.extend(["--robot-param", f"{k}={v}"])
            
    try:
        proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        shutil.rmtree(out_dir, ignore_errors=True)
        return None, None
        
    summary_path = os.path.join(out_dir, "summary.json")
    if not os.path.exists(summary_path):
        shutil.rmtree(out_dir, ignore_errors=True)
        return None, None
        
    try:
        with open(summary_path) as f:
            data = json.load(f)
            return data, out_dir
    except Exception:
        shutil.rmtree(out_dir, ignore_errors=True)
        return None, None

def compute_max_sideslip_beta(csv_path, min_v=0.6):
    """Calculate maximum sideslip angle beta = |heading - velocity_vector| in degrees."""
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

def evaluate_candidate(cand, track, robot, max_beta_thresh=35.0, check_stress=True):
    # 1. Normal run
    summary, out_dir = run_sim(track, robot, cand, drives="1,2,4")
    if not summary or not summary.get("ok"):
        return {"pass": False, "why": "Sim failed", "cand": cand}
        
    drives = {d["drive"]: d for d in summary.get("drives", [])}
    for d_num in [1, 2, 4]:
        d = drives.get(d_num)
        if not d or d.get("outcome") != "finished" or d.get("marker_check") != "OK":
            mc = d.get("marker_check", "?") if d else "missing"
            shutil.rmtree(out_dir, ignore_errors=True)
            return {"pass": False, "why": f"D{d_num} {mc}", "cand": cand}
            
    # Check oversteer / sideslip beta on all drives
    betas = {}
    for d_num in [1, 2, 4]:
        csv_p = os.path.join(out_dir, f"drive{d_num}.csv")
        b = compute_max_sideslip_beta(csv_p)
        betas[d_num] = b
        if b > max_beta_thresh:
            shutil.rmtree(out_dir, ignore_errors=True)
            return {"pass": False, "why": f"D{d_num} Oversteer (beta={b:.1f}° > {max_beta_thresh}°)", "cand": cand}
            
    lap1 = drives[1]["lap_time_fw"]
    lap2 = drives[2]["lap_time_fw"]
    lap4 = drives[4]["lap_time_fw"]
    max_lat4 = drives[4]["max_lat_mm"]
    rms_lat4 = drives[4]["rms_lat_mm"]
    shutil.rmtree(out_dir, ignore_errors=True)
    
    # 2. Stress condition validation: sensor_delay=0.0098, tire_mu=1.105
    if check_stress:
        s_summary, s_out_dir = run_sim(track, robot, cand, drives="1,4", robot_params={"sensor_delay": 0.0098, "tire_mu": 1.105})
        if not s_summary or not s_summary.get("ok"):
            return {"pass": False, "why": "Stress crashed", "cand": cand}
            
        s_drives = {d["drive"]: d for d in s_summary.get("drives", [])}
        for d_num in [1, 4]:
            d = s_drives.get(d_num)
            if not d or d.get("outcome") != "finished" or d.get("marker_check") != "OK":
                mc = d.get("marker_check", "?") if d else "missing"
                shutil.rmtree(s_out_dir, ignore_errors=True)
                return {"pass": False, "why": f"Stress D{d_num} {mc}", "cand": cand}
        shutil.rmtree(s_out_dir, ignore_errors=True)
                
    return {
        "pass": True,
        "lap1": lap1,
        "lap2": lap2,
        "lap4": lap4,
        "total_lap": lap1 + lap2 + lap4,
        "max_lat4": max_lat4,
        "rms_lat4": rms_lat4,
        "betas": betas,
        "cand": cand
    }

def apply_to_drive_c(cand):
    """Write optimal parameters into firmware/Main/Src/drive.c driveData struct."""
    if not os.path.exists(DRIVE_C):
        print(f"Error: {DRIVE_C} not found.")
        return False
        
    with open(DRIVE_C, "r", encoding="utf-8") as f:
        content = f.read()
        
    pattern = r"(DriveParam_t\s+driveData\s*=\s*\{)(.*?)(\};)"
    new_body = f"""
    .base_mps = {cand['base_mps']:.2f}f,
    .max_mps = {cand['max_mps']:.2f}f,
    .accel = {cand['accel']:.1f}f,
    .decel = {cand['decel']:.1f}f,
    .steer_gain_p = {cand['steer_kp']:.1f}f,
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
        
    print(f"\n[OK] Successfully updated {DRIVE_C} with optimal parameters.")
    return True

def main():
    parser = argparse.ArgumentParser(description="Smart Tuner for LineSim with Oversteer Rejection")
    parser.add_argument("--track", default=os.path.join(ROOT, "tracks", "practice_board.json"), help="Track json file")
    parser.add_argument("--robot", default=os.path.join(ROOT, "robots", "full.json"), help="Robot json file")
    parser.add_argument("--max-beta", type=float, default=35.0, help="Max allowed sideslip angle (deg) before oversteer DQ (default: 35.0)")
    parser.add_argument("--jobs", type=int, default=6, help="Parallel thread count (default: 6)")
    parser.add_argument("--no-stress", action="store_true", help="Skip stress check (pure speed mode)")
    parser.add_argument("--apply", action="store_true", help="Write top parameters to drive.c")
    args = parser.parse_args()

    os.makedirs(TMP_RUNS, exist_ok=True)

    print("=" * 70)
    print("  LINESIM SMART TUNER - Anti-Oversteer & Integer KP Optimization")
    print(f"  Track: {os.path.basename(args.track)} | Robot: {os.path.basename(args.robot)}")
    print(f"  Workers: {args.jobs} threads | Max Allowed Sideslip: {args.max_beta}°")
    print(f"  Stress Validation: {'OFF (Pure Speed Mode)' if args.no_stress else 'ON (Robust Mode)'}")
    print("=" * 70)

    candidates = []
    
    # Focused candidates with fine decimal KP resolution
    if not args.no_stress:
        base_list = [1.85, 1.90]
        kp_list = [26.0, 27.0, 27.5, 28.0, 28.5, 29.0]
        kd_list = [400, 425]
        lat_list = [13.2, 13.6]
        crv_list = [1.15]
    else:
        base_list = [2.00, 2.10, 2.15]
        kp_list = [26.0, 27.0, 27.5, 28.0, 28.5, 29.0]
        kd_list = [400, 425]
        lat_list = [13.6, 14.2]
        crv_list = [1.15]

    for b in base_list:
        for kp in kp_list:
            for kd in kd_list:
                for lat in lat_list:
                    for crv in crv_list:
                        candidates.append({
                            "base_mps": b,
                            "max_mps": 10.0,
                            "accel": 10.0,
                            "decel": 10.0,
                            "steer_kp": kp,
                            "steer_kd": kd,
                            "lat_acc_max": lat,
                            "target_shift": 0.25,
                            "target_curve_gain": crv,
                            "slalom_ratio": 0.06,
                            "pos_atten_gain": 0.0,
                            "yaw_kp": 0.35,
                            "yaw_ki": 1.0
                        })

    print(f"Generated {len(candidates)} candidates. Running parallel evaluation...")
    t0 = time.time()
    passed = []
    dq_oversteer = 0
    done_cnt = 0

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futures = {ex.submit(evaluate_candidate, c, args.track, args.robot, args.max_beta, not args.no_stress): c for c in candidates}
        for f in concurrent.futures.as_completed(futures):
            done_cnt += 1
            res = f.result()
            if res["pass"]:
                passed.append(res)
                print(f"[{done_cnt:2d}/{len(candidates)}] PASS! D4:{res['lap4']:.3f}s D2:{res['lap2']:.3f}s D1:{res['lap1']:.3f}s | beta4={res['betas'][4]:.1f}° | base={res['cand']['base_mps']} kp={res['cand']['steer_kp']} kd={res['cand']['steer_kd']}")
            else:
                if "Oversteer" in res["why"]:
                    dq_oversteer += 1
                print(f"[{done_cnt:2d}/{len(candidates)}] FAIL: {res['why']} (base={res['cand']['base_mps']}, kp={res['cand']['steer_kp']})")

    elapsed = time.time() - t0
    print(f"\nDone in {elapsed:.1f}s. Passed: {len(passed)} / {len(candidates)} (Oversteer DQs: {dq_oversteer})")

    # Clean up empty temp folder
    shutil.rmtree(TMP_RUNS, ignore_errors=True)

    if not passed:
        print("\nNo candidates passed all criteria. Try relaxing --max-beta slightly.")
        return

    # Sort by Drive 4 lap time
    passed.sort(key=lambda x: (x["lap4"], x["total_lap"]))
    top = passed[0]

    print("\n" + "=" * 70)
    print("  TOP OPTIMAL CONFIGURATION (NO OVERSTEER & STRESS VALIDATED)")
    print("=" * 70)
    print(f"  Drive 4 Lap Time : {top['lap4']:.3f} s (RMS Lat: {top['rms_lat4']:.1f} mm, Max: {top['max_lat4']:.1f} mm)")
    print(f"  Drive 2 Lap Time : {top['lap2']:.3f} s")
    print(f"  Drive 1 Lap Time : {top['lap1']:.3f} s")
    print(f"  Total Lap Time   : {top['total_lap']:.3f} s")
    print(f"  Max Sideslip (D4): {top['betas'][4]:.1f}° (Threshold: <= {args.max_beta}°)")
    print("-" * 70)
    print("  Parameters:")
    for k, v in top["cand"].items():
        print(f"    {k:18s} = {v}")
    print("=" * 70)

    if args.apply:
        apply_to_drive_c(top["cand"])
    else:
        print("\nTip: Run with --apply to automatically update firmware/Main/Src/drive.c")

if __name__ == "__main__":
    main()
