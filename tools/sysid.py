#!/usr/bin/env python3
"""
sysid.py - Sim2Real System Identification & Physical Property Extractor for LineSim

Reads SD card logs collected from the real robot (or a simulated SD card folder)
and extracts physical parameters for `robots/full.json` (or `robots/default.json`).

Supported input files inside the SD card / log directory:
  1. SysID_Data/sensor_sysid.csv        (from `Sensor Menu -> SysID Log`)
  2. SysID_Data/motor_sysid.csv         (from `Motor Menu  -> SysID Test`)
  3. SysID_Data/drive_sysid.csv         (from `Drive Menu  -> SysID Drive` or auto-saved after `1st Drive`)
  4. Sensor_Data/calibration_result.txt (fallback for IR white/black/spread)
  5. Drive_Data/vib_log_*.csv           (fallback for gyro noise & lateral acceleration)
  6. Drive_Data/save_slot_*.txt         (fallback for marker odometry & lateral acceleration settings)

Usage:
  python tools/sysid.py <sdcard_or_log_dir> [--robot robots/full.json] [--out runs/sysid_report.json] [--apply]
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
from typing import Any, Dict, List, Optional, Tuple


G_ACCEL = 9.80665
DEG2RAD = math.pi / 180.0


def _mean(xs: List[float]) -> float:
    return sum(xs) / len(xs) if xs else 0.0


def _std(xs: List[float]) -> float:
    if len(xs) < 2:
        return 0.0
    m = _mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def _percentile(xs: List[float], q: float) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    idx = max(0.0, min(float(len(s) - 1), (q / 100.0) * (len(s) - 1)))
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return s[lo]
    return s[lo] + (idx - lo) * (s[hi] - s[lo])


def _find_file(base_dir: str, rel_candidates: List[str]) -> Optional[str]:
    for rel in rel_candidates:
        p = os.path.join(base_dir, rel)
        if os.path.isfile(p):
            return p
    target_names = {os.path.basename(r).lower() for r in rel_candidates}
    for dp, _, fns in os.walk(base_dir):
        for fn in fns:
            if fn.lower() in target_names:
                return os.path.join(dp, fn)
    return None


def _find_all_files(base_dir: str, prefix: str, suffix: str) -> List[str]:
    found: List[str] = []
    for dp, _, fns in os.walk(base_dir):
        for fn in fns:
            fl = fn.lower()
            if fl.startswith(prefix.lower()) and fl.endswith(suffix.lower()):
                found.append(os.path.join(dp, fn))
    return sorted(found)


def parse_sensor_sysid(path: str) -> Dict[str, Any]:
    """Parse /SysID_Data/sensor_sysid.csv."""
    calib_white: List[int] = []
    calib_black: List[int] = []
    meta: Dict[str, float] = {}
    ir_cols: List[List[float]] = [[] for _ in range(18)]
    gyro_z_samples: List[float] = []

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        header: Optional[List[str]] = None
        for row in reader:
            if not row:
                continue
            first = row[0].strip()
            if first.startswith("#"):
                line = ",".join(row)
                low = line.lower()
                if "whitemax" in low or "calib_white_max" in low:
                    rhs = line.split("=", 1)[1] if "=" in line else line.split(":", 1)[1]
                    nums = [int(x) for x in re.findall(r"\d+", rhs)]
                    if len(nums) >= 16:
                        calib_white = nums
                elif "blackmax" in low or "calib_black_max" in low:
                    rhs = line.split("=", 1)[1] if "=" in line else line.split(":", 1)[1]
                    nums = [int(x) for x in re.findall(r"\d+", rhs)]
                    if len(nums) >= 16:
                        calib_black = nums
                elif "gyro_z_offset" in low:
                    rhs = line.split("=", 1)[1] if "=" in line else line.split(":", 1)[1]
                    try:
                        meta["GYRO_Z_OFFSET_DPS"] = float(rhs.strip().split()[0])
                    except ValueError:
                        pass
                else:
                    for key in ("GYRO_Z_OFFSET_DPS", "GYRO_Z_STD_DPS", "IR_NOISE_AVG_LSB"):
                        if key in line and ":" in line:
                            try:
                                meta[key] = float(line.split(":", 1)[1].strip().split()[0])
                            except ValueError:
                                pass
                continue
            if header is None:
                header = [c.strip().lower() for c in row]
                continue
            if len(row) < 20:
                continue
            try:
                ir_start = 2 if (header and header[0] == "sample") else 1
                gz_idx = ir_start + 18
                for ch in range(18):
                    ir_cols[ch].append(float(row[ir_start + ch]))
                if gz_idx < len(row):
                    gyro_z_samples.append(float(row[gz_idx]))
            except ValueError:
                continue

    ch_stds = [_std(col) for col in ir_cols[:16] if len(col) >= 2]
    ir_noise = round(_mean(ch_stds), 2) if ch_stds else meta.get("IR_NOISE_AVG_LSB", 6.0)
    gyro_noise_dps = round(_std(gyro_z_samples), 4) if len(gyro_z_samples) >= 2 else meta.get("GYRO_Z_STD_DPS", 0.08)
    # Use actual stationary samples for sensor bias; fallback to header offset only if samples missing
    gyro_bias_dps = round(abs(_mean(gyro_z_samples)), 4) if len(gyro_z_samples) >= 2 else round(abs(meta.get("GYRO_Z_OFFSET_DPS", 0.4)), 4)

    result: Dict[str, Any] = {
        "source": path,
        "samples": len(gyro_z_samples),
        "ir_noise": ir_noise,
        "gyro_noise_dps": gyro_noise_dps,
        "gyro_bias_dps": gyro_bias_dps,
    }

    if len(calib_white) >= 16 and len(calib_black) >= 16:
        w16 = [float(x) for x in calib_white[:16]]
        b16 = [float(x) for x in calib_black[:16]]
        # In LineSim (sensors.cpp), calibration white[i] = white_[i] + 3 * ir_noise
        w_mean = max(100.0, _mean(w16) - 3.0 * ir_noise)
        b_mean = max(10.0, _mean(b16) - 3.0 * ir_noise)
        w_spread = _std(w16) / w_mean if w_mean > 1e-3 else 0.06
        result["ir_white"] = int(round(w_mean))
        result["ir_black"] = int(round(b_mean))
        result["ir_spread"] = round(max(0.01, min(0.25, w_spread)), 4)

    return result


def parse_calibration_result_txt(path: str, ir_noise: float = 6.0) -> Dict[str, Any]:
    """Parse /Sensor_Data/calibration_result.txt (`whitemax_00=...` or block format)."""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()

    white_kv = re.findall(r"whitemax_\d+\s*=\s*(\d+)", text, flags=re.IGNORECASE)
    black_kv = re.findall(r"blackmax_\d+\s*=\s*(\d+)", text, flags=re.IGNORECASE)
    white_vals: List[float] = [float(x) for x in white_kv]
    black_vals: List[float] = [float(x) for x in black_kv]

    if len(white_vals) < 16 or len(black_vals) < 16:
        m_w = re.search(r"White Max Values[^\n]*\n([\d\s]+)", text)
        m_b = re.search(r"Black Max Values[^\n]*\n([\d\s]+)", text)
        if m_w:
            white_vals = [float(x) for x in re.findall(r"\d+", m_w.group(1))]
        if m_b:
            black_vals = [float(x) for x in re.findall(r"\d+", m_b.group(1))]

    out: Dict[str, Any] = {"source": path}
    if len(white_vals) >= 16 and len(black_vals) >= 16:
        w16 = white_vals[:16]
        b16 = black_vals[:16]
        w_mean = max(100.0, _mean(w16) - 3.0 * ir_noise)
        b_mean = max(10.0, _mean(b16) - 3.0 * ir_noise)
        w_spread = _std(w16) / w_mean if w_mean > 1e-3 else 0.06
        out["ir_white"] = int(round(w_mean))
        out["ir_black"] = int(round(b_mean))
        out["ir_spread"] = round(max(0.01, min(0.25, w_spread)), 4)
    return out


def parse_motor_sysid(path: str, kt: float = 0.00646, efficiency: float = 0.9) -> Dict[str, Any]:
    """Parse /SysID_Data/motor_sysid.csv and fit motor friction, viscous damping, current_tau, and vbat."""
    rows: List[Dict[str, float]] = []

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        header: Optional[List[str]] = None
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            if header is None:
                header = [c.strip().lower() for c in row]
                continue
            if len(row) < 9:
                continue
            try:
                if len(row) >= 12 and header and header[0] == "step":
                    rows.append(
                        {
                            "phase": float(row[0]),
                            "t_ms": float(row[1]),
                            "target_omega": float(row[2]),
                            "omega_L": float(row[3]),
                            "omega_R": float(row[4]),
                            "iq_L": float(row[7]),
                            "iq_R": float(row[8]),
                            "vq_L": float(row[9]),
                            "vbus": float(row[11]),
                        }
                    )
                else:
                    rows.append(
                        {
                            "t_ms": float(row[0]),
                            "phase": float(row[1]),
                            "target_omega": float(row[2]),
                            "omega_L": float(row[3]),
                            "omega_R": float(row[4]),
                            "iq_L": float(row[5]),
                            "iq_R": float(row[6]),
                            "vq_L": float(row[7]),
                            "vbus": float(row[8]),
                        }
                    )
            except ValueError:
                continue

    by_phase: Dict[int, List[Dict[str, float]]] = {}
    for r in rows:
        p = int(round(r["phase"]))
        by_phase.setdefault(p, []).append(r)

    omega_pts_L: List[float] = []
    tau_pts_L: List[float] = []
    omega_pts_R: List[float] = []
    tau_pts_R: List[float] = []
    vbus_samples: List[float] = [r["vbus"] for r in rows if r["vbus"] > 5.0]

    for p in range(1, 8):
        pts = by_phase.get(p, [])
        if len(pts) < 10:
            continue
        steady = pts[int(len(pts) * 0.55) :]
        wL = _mean([x["omega_L"] for x in steady])
        wR = _mean([x["omega_R"] for x in steady])
        iL = _mean([x["iq_L"] for x in steady])
        iR = _mean([x["iq_R"] for x in steady])
        if wL > 50.0 and iL > 0.0:
            omega_pts_L.append(wL)
            # In LineSim robot.cpp: t_wheel = kt * iq * G * efficiency = friction * G + viscous * G^2 * omega_wheel
            # => kt * efficiency * iq = friction + viscous * omega_motor
            tau_pts_L.append(kt * efficiency * iL)
        if wR > 50.0 and iR > 0.0:
            omega_pts_R.append(wR)
            tau_pts_R.append(kt * efficiency * iR)

    def _linfit(xs: List[float], ys: List[float]) -> Tuple[float, float]:
        if len(xs) < 2:
            return 0.00025, 2e-8
        mx = _mean(xs)
        my = _mean(ys)
        num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        den = sum((x - mx) ** 2 for x in xs)
        slope = max(1e-9, num / den) if den > 1e-9 else 2e-8
        intercept = max(1e-5, my - slope * mx)
        return intercept, slope

    fric_L, visc_L = _linfit(omega_pts_L, tau_pts_L)
    fric_R, visc_R = _linfit(omega_pts_R, tau_pts_R)
    friction = 0.5 * (fric_L + fric_R)
    viscous = 0.5 * (visc_L + visc_R)

    p0 = [r for r in by_phase.get(0, []) if 6.0 <= r["t_ms"] <= 80.0 and r["iq_L"] > 0.3]
    j_rotor_est: Optional[float] = None
    if len(p0) >= 6:
        dt = (p0[-1]["t_ms"] - p0[0]["t_ms"]) * 1e-3
        dw_L = p0[-1]["omega_L"] - p0[0]["omega_L"]
        dw_R = p0[-1]["omega_R"] - p0[0]["omega_R"]
        iq_avg = 0.5 * (_mean([x["iq_L"] for x in p0]) + _mean([x["iq_R"] for x in p0]))
        alpha = 0.5 * (dw_L + dw_R) / max(1e-3, dt)
        if alpha > 500.0:
            tau_net = max(1e-5, kt * efficiency * iq_avg - friction)
            j_total_motor_side = tau_net / alpha
            j_rotor_est = round(j_total_motor_side, 9)

    result: Dict[str, Any] = {
        "source": path,
        "samples": len(rows),
        "friction": round(friction, 7),
        "viscous": round(viscous, 10),
        "friction_L": round(fric_L, 7),
        "friction_R": round(fric_R, 7),
        "viscous_L": round(visc_L, 10),
        "viscous_R": round(visc_R, 10),
    }
    if vbus_samples:
        result["vbat"] = round(_percentile(vbus_samples, 90.0), 2)
    if j_rotor_est is not None:
        result["j_rotor_total_motor_side_est"] = j_rotor_est
    return result


def parse_drive_sysid(
    path: str,
    track_width: float = 0.187,
    timestep_default: float = 0.002,
) -> Dict[str, Any]:
    """
    Parse /SysID_Data/drive_sysid.csv and extract:
      - tire.mu (from peak lateral & longitudinal acceleration)
      - tire.slip_stiffness (from yaw rate ratio gyro_z vs encoder differential speed in curves)
      - tire.relaxation_length (from lag between wheel differential speed and gyro yaw rate)
      - sensor_delay (from line_pos rate vs gyro yaw rate phase lag)
      - vbat (from vbus column)
    """
    rows: List[Dict[str, float]] = []

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        header: Optional[List[str]] = None
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            if header is None:
                header = [c.strip() for c in row]
                continue
            if len(row) < 13:
                continue
            try:
                rows.append(
                    {
                        "t_ms": float(row[0]),
                        "line_pos": float(row[1]),
                        "target_mps": float(row[2]),
                        "mps_L": float(row[3]),
                        "mps_R": float(row[4]),
                        "target_iq_L": float(row[5]),
                        "target_iq_R": float(row[6]),
                        "iq_L": float(row[7]),
                        "iq_R": float(row[8]),
                        "gyro_z_dps": float(row[9]),
                        "odom_m": float(row[10]),
                        "marker_cnt": float(row[11]),
                        "vbus": float(row[12]),
                    }
                )
            except ValueError:
                continue

    if len(rows) < 20:
        return {"source": path, "samples": len(rows), "error": "Not enough drive samples"}

    dts = [
        (rows[i]["t_ms"] - rows[i - 1]["t_ms"]) * 1e-3
        for i in range(1, len(rows))
        if 0.0005 <= (rows[i]["t_ms"] - rows[i - 1]["t_ms"]) * 1e-3 <= 0.05
    ]
    dt = _mean(dts) if dts else timestep_default

    v_list: List[float] = []
    w_enc_list: List[float] = []
    w_gyro_list: List[float] = []
    ay_list: List[float] = []
    ax_list: List[float] = []
    atot_list: List[float] = []
    yaw_slip_ratios: List[float] = []
    vbus_list: List[float] = []

    for i, r in enumerate(rows):
        v = 0.5 * (r["mps_L"] + r["mps_R"])
        w_enc = (r["mps_R"] - r["mps_L"]) / max(0.05, track_width)
        w_gyro = r["gyro_z_dps"] * DEG2RAD
        ay = abs(v * w_gyro)
        if i >= 3:
            dt_w = max(1e-3, (rows[i]["t_ms"] - rows[i - 3]["t_ms"]) * 1e-3)
            v_prev = 0.5 * (rows[i - 3]["mps_L"] + rows[i - 3]["mps_R"])
            ax = (v - v_prev) / dt_w
        else:
            ax = 0.0

        v_list.append(v)
        w_enc_list.append(w_enc)
        w_gyro_list.append(w_gyro)
        ay_list.append(ay)
        ax_list.append(ax)
        atot_list.append(math.hypot(ax, ay))
        if r["vbus"] > 5.0:
            vbus_list.append(r["vbus"])

        if v > 0.8 and abs(w_enc) > 2.0 and abs(w_gyro) > 1.5 and (w_enc * w_gyro > 0):
            slip_ratio = (abs(w_enc) - abs(w_gyro)) / abs(w_enc)
            if -0.1 <= slip_ratio <= 0.6:
                yaw_slip_ratios.append(slip_ratio)

    ay_p98 = _percentile(ay_list, 98.0)
    ay_max = max(ay_list) if ay_list else 0.0
    ax_p98 = _percentile([abs(x) for x in ax_list], 98.0)
    atot_p98 = _percentile(atot_list, 98.0)

    mu_from_accel = (atot_p98 / G_ACCEL) * 1.12
    tire_mu = round(max(0.75, min(2.2, mu_from_accel if mu_from_accel > 0.6 else 1.3)), 3)

    if len(yaw_slip_ratios) >= 10:
        mean_slip = max(0.015, _mean(yaw_slip_ratios))
        mean_ay_g = max(0.15, _mean([ay for ay in ay_list if ay > 1.0]) / G_ACCEL)
        c_alpha = (mean_ay_g / mean_slip) * 1.15
        slip_stiffness = round(max(3.0, min(25.0, c_alpha)), 2)
    else:
        slip_stiffness = 8.0

    best_lag_steps = 0
    best_corr = -1e9
    max_lag_steps = min(10, max(1, int(round(0.020 / dt))))
    for lag in range(0, max_lag_steps + 1):
        n = len(w_enc_list) - lag
        if n < 20:
            break
        s = sum(w_enc_list[i] * w_gyro_list[i + lag] for i in range(n)) / n
        if s > best_corr:
            best_corr = s
            best_lag_steps = lag

    v_moving = _mean([v for v in v_list if v > 0.5]) or 1.5
    relax_len = max(0.002, min(0.015, max(0.5, float(best_lag_steps)) * dt * v_moving * 0.5))

    dline_dt = [0.0] + [
        -(rows[i]["line_pos"] - rows[i - 1]["line_pos"]) / dt for i in range(1, len(rows))
    ]
    best_s_lag = 3
    best_s_corr = -1e9
    max_s_steps = min(15, max(2, int(round(0.025 / dt))))
    for lag in range(1, max_s_steps + 1):
        n = len(w_gyro_list) - lag
        if n < 20:
            break
        s = sum(w_gyro_list[i] * dline_dt[i + lag] for i in range(n)) / n
        if s > best_s_corr:
            best_s_corr = s
            best_s_lag = lag
    sensor_delay = round(max(0.002, min(0.020, best_s_lag * dt)), 4)

    result: Dict[str, Any] = {
        "source": path,
        "samples": len(rows),
        "dt_s": round(dt, 5),
        "max_speed_mps": round(max(v_list), 3),
        "ay_p98_mps2": round(ay_p98, 3),
        "ay_max_mps2": round(ay_max, 3),
        "ax_p98_mps2": round(ax_p98, 3),
        "atot_p98_mps2": round(atot_p98, 3),
        "tire_mu": tire_mu,
        "slip_stiffness": slip_stiffness,
        "relaxation_length": round(relax_len, 4),
        "sensor_delay": sensor_delay,
        "recommended_lat_acc_max": round(ay_p98 * 0.90, 2),
    }
    if vbus_list:
        result["vbat"] = round(_percentile(vbus_list, 90.0), 2)
    return result


def parse_vib_logs(paths: List[str]) -> Dict[str, Any]:
    """Fallback parser for existing /Drive_Data/vib_log_*.csv files."""
    ay_list: List[float] = []
    v_list: List[float] = []
    stationary_gyro: List[float] = []

    for p in paths:
        with open(p, "r", encoding="utf-8", errors="ignore") as f:
            reader = csv.reader(f)
            header: Optional[List[str]] = None
            for row in reader:
                if not row or row[0].strip().startswith("#"):
                    continue
                if header is None:
                    header = [c.strip() for c in row]
                    continue
                if len(row) < 6:
                    continue
                try:
                    v = float(row[2])
                    gz_dps = float(row[4])
                    v_list.append(v)
                    ay_list.append(abs(v * gz_dps * DEG2RAD))
                    if abs(v) < 0.03 and abs(gz_dps) < 5.0:
                        stationary_gyro.append(gz_dps)
                except ValueError:
                    continue

    if not v_list:
        return {}
    ay_p98 = _percentile(ay_list, 98.0)
    mu_est = round(max(0.8, min(2.2, (ay_p98 / G_ACCEL) * 1.15)), 3)
    out: Dict[str, Any] = {
        "sources": paths,
        "samples": len(v_list),
        "max_speed_mps": round(max(v_list), 3),
        "ay_p98_mps2": round(ay_p98, 3),
        "tire_mu_lower_bound": mu_est,
    }
    if len(stationary_gyro) >= 5:
        out["gyro_noise_dps"] = round(_std(stationary_gyro), 4)
    return out


def parse_save_slots(paths: List[str]) -> Dict[str, Any]:
    """Fallback parser for /Drive_Data/save_slot_*.txt files."""
    lat_accs: List[float] = []
    accels: List[float] = []
    corner_90_deg_ratios: List[float] = []

    for p in paths:
        with open(p, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read()
        m_lat = re.search(r"lat acc max\s*=\s*([0-9.]+)", text)
        m_acc = re.search(r"accel\s*=\s*([0-9.]+)", text)
        if m_lat:
            lat_accs.append(float(m_lat.group(1)))
        if m_acc:
            accels.append(float(m_acc.group(1)))
        for m_row in re.finditer(r"^\d+\s+[LR]\s+([0-9.]+)\s*m\s+(-?[0-9.]+)\s*deg", text, flags=re.MULTILINE):
            dist = float(m_row.group(1))
            deg = abs(float(m_row.group(2)))
            if 0.33 <= dist <= 0.41 and 75.0 <= deg <= 98.0:
                corner_90_deg_ratios.append(deg / 90.0)

    if not lat_accs and not corner_90_deg_ratios:
        return {}

    out: Dict[str, Any] = {
        "slot_files_count": len(paths),
    }
    if lat_accs:
        max_lat = max(lat_accs)
        out["max_configured_lat_acc_mps2"] = round(max_lat, 2)
        out["tire_mu_from_slots"] = round(max(0.85, min(2.0, (max_lat / G_ACCEL) * 1.12)), 3)
    if accels:
        out["max_configured_accel_mps2"] = round(max(accels), 2)
    if corner_90_deg_ratios:
        mean_ratio = _mean(corner_90_deg_ratios)
        out["corner_90_gyro_to_true_ratio"] = round(mean_ratio, 4)
    return out


def run_sysid(log_dir: str, robot_json_path: str) -> Dict[str, Any]:
    with open(robot_json_path, "r", encoding="utf-8") as f:
        robot_cfg = json.load(f)

    kt = float(robot_cfg.get("motor", {}).get("kt", 0.00646))
    efficiency = float(robot_cfg.get("motor", {}).get("efficiency", 0.9))
    track_width = float(robot_cfg.get("track", 0.187))

    report: Dict[str, Any] = {
        "input_dir": os.path.abspath(log_dir),
        "base_robot_json": os.path.abspath(robot_json_path),
        "sources_found": [],
        "sections": {},
        "extracted_parameters": {},
    }

    extracted: Dict[str, Any] = {}

    # 1. Sensor SysID / Calibration
    sensor_csv = _find_file(
        log_dir,
        [
            os.path.join("SysID_Data", "sensor_sysid.csv"),
            "sensor_sysid.csv",
        ],
    )
    calib_txt = _find_file(
        log_dir,
        [
            os.path.join("Sensor_Data", "calibration_result.txt"),
            "calibration_result.txt",
        ],
    )

    if sensor_csv:
        report["sources_found"].append(sensor_csv)
        s_res = parse_sensor_sysid(sensor_csv)
        report["sections"]["sensor_sysid"] = s_res
        for k in ("ir_white", "ir_black", "ir_spread", "ir_noise", "gyro_noise_dps", "gyro_bias_dps"):
            if k in s_res:
                extracted[k] = s_res[k]

    if calib_txt:
        report["sources_found"].append(calib_txt)
        c_res = parse_calibration_result_txt(calib_txt, ir_noise=float(extracted.get("ir_noise", 6.0)))
        report["sections"]["calibration_result"] = c_res
        for k in ("ir_white", "ir_black", "ir_spread"):
            if k not in extracted and k in c_res:
                extracted[k] = c_res[k]

    # 2. Motor SysID
    motor_csv = _find_file(
        log_dir,
        [
            os.path.join("SysID_Data", "motor_sysid.csv"),
            "motor_sysid.csv",
        ],
    )
    if motor_csv:
        report["sources_found"].append(motor_csv)
        m_res = parse_motor_sysid(motor_csv, kt=kt, efficiency=efficiency)
        report["sections"]["motor_sysid"] = m_res
        extracted.setdefault("motor", {})
        extracted["motor"]["friction"] = m_res["friction"]
        extracted["motor"]["viscous"] = m_res["viscous"]
        if "vbat" in m_res:
            extracted["vbat"] = m_res["vbat"]

    # 3. Drive SysID
    drive_csv = _find_file(
        log_dir,
        [
            os.path.join("SysID_Data", "drive_sysid.csv"),
            "drive_sysid.csv",
        ],
    )
    if drive_csv:
        report["sources_found"].append(drive_csv)
        d_res = parse_drive_sysid(drive_csv, track_width=track_width)
        report["sections"]["drive_sysid"] = d_res
        if "error" not in d_res:
            extracted["tire_mu"] = d_res["tire_mu"]
            extracted.setdefault("tire", {})
            extracted["tire"]["mu"] = d_res["tire_mu"]
            extracted["tire"]["slip_stiffness"] = d_res["slip_stiffness"]
            extracted["tire"]["relaxation_length"] = d_res["relaxation_length"]
            extracted["sensor_delay"] = d_res["sensor_delay"]
            if "vbat" in d_res:
                extracted["vbat"] = d_res["vbat"]

    # 4. Fallback: Drive_Data/vib_log_*.csv
    vib_files = _find_all_files(log_dir, "vib_log_", ".csv")
    if vib_files:
        report["sources_found"].extend(vib_files)
        v_res = parse_vib_logs(vib_files)
        report["sections"]["vib_logs"] = v_res
        if "tire_mu" not in extracted and "tire_mu_lower_bound" in v_res:
            extracted["tire_mu"] = v_res["tire_mu_lower_bound"]
            extracted.setdefault("tire", {})
            extracted["tire"]["mu"] = v_res["tire_mu_lower_bound"]
        if "gyro_noise_dps" not in extracted and "gyro_noise_dps" in v_res:
            extracted["gyro_noise_dps"] = v_res["gyro_noise_dps"]

    # 5. Fallback: Drive_Data/save_slot_*.txt
    slot_files = _find_all_files(log_dir, "save_slot_", ".txt")
    if slot_files:
        report["sources_found"].extend(slot_files[:5])
        sl_res = parse_save_slots(slot_files)
        report["sections"]["save_slots"] = sl_res
        if "tire_mu" not in extracted and "tire_mu_from_slots" in sl_res:
            extracted["tire_mu"] = sl_res["tire_mu_from_slots"]
            extracted.setdefault("tire", {})
            extracted["tire"]["mu"] = sl_res["tire_mu_from_slots"]

    report["extracted_parameters"] = extracted
    return report


def apply_to_robot_json(robot_json_path: str, extracted: Dict[str, Any]) -> None:
    with open(robot_json_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    for top_key in (
        "ir_white",
        "ir_black",
        "ir_noise",
        "ir_spread",
        "sensor_delay",
        "gyro_noise_dps",
        "gyro_bias_dps",
        "vbat",
        "tire_mu",
    ):
        if top_key in extracted:
            cfg[top_key] = extracted[top_key]

    if "tire" in extracted and isinstance(cfg.get("tire"), dict):
        for tk, tv in extracted["tire"].items():
            cfg["tire"][tk] = tv

    if "motor" in extracted and isinstance(cfg.get("motor"), dict):
        for mk, mv in extracted["motor"].items():
            cfg["motor"][mk] = mv

    with open(robot_json_path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract LineSim physical properties from real robot SD card logs.")
    parser.add_argument(
        "log_dir",
        help="Path to SD card root or log directory (containing SysID_Data/, Sensor_Data/, Drive_Data/)",
    )
    parser.add_argument(
        "--robot",
        default="robots/full.json",
        help="Target robot configuration JSON file (default: robots/full.json)",
    )
    parser.add_argument(
        "--out",
        default="runs/sysid_report.json",
        help="Output path for the extracted SysID report JSON (default: runs/sysid_report.json)",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Apply extracted parameters directly to the --robot JSON file",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.log_dir):
        print(f"[ERROR] Directory not found: {args.log_dir}", file=sys.stderr)
        return 1
    if not os.path.isfile(args.robot):
        print(f"[ERROR] Robot config not found: {args.robot}", file=sys.stderr)
        return 1

    report = run_sysid(args.log_dir, args.robot)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print("=" * 64)
    print(f"LineSim SysID Report")
    print(f"  Input Directory : {report['input_dir']}")
    print(f"  Sources Found   : {len(report['sources_found'])} file(s)")
    print(f"  Report Saved To : {os.path.abspath(args.out)}")
    print("-" * 64)
    print("Extracted Parameters for robots/full.json:")
    print(json.dumps(report["extracted_parameters"], indent=2))
    print("=" * 64)

    if args.apply:
        apply_to_robot_json(args.robot, report["extracted_parameters"])
        print(f"[OK] Updated robot configuration in-place: {os.path.abspath(args.robot)}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
