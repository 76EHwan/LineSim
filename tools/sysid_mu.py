#!/usr/bin/env python3
"""
sysid_mu.py - Inverse Tire Friction (tire.mu, tire.mu_viscous) SysID for LineSim

In LineSim's brush tire model (sim/world/robot.cpp):
    fmax_total = fmax_coulomb + mu_viscous * fz * v_slide
               = fz * (mu + mu_viscous * v_slide)

So a single peak-acceleration estimate conflates Coulomb friction `mu` and
viscous sliding friction `mu_viscous * v_slide`. This tool inversely separates
and solves for (mu, mu_viscous) in two modes:

  1. `telemetry`: Physics-based upper-envelope deconvolution from
     `drive_sysid.csv` (real SD card) or `drive*.csv` (LineSim telemetry).
  2. `sim2real`: Closed-loop Sim2Real inverse optimization using `linesim.exe`
     against either a reference telemetry CSV or real `save_slot_*.txt` logs.

Usage examples:
  # 1) Direct inverse deconvolution from telemetry CSV:
  python tools/sysid_mu.py telemetry runs/ref/drive1.csv --robot robots/full.json

  # 1b) Real `SysID Drive` CSV with IMU accel columns (acc_x, acc_y). Give the IMU
  #     position [m] in the robot frame (x forward, y left, origin = robots/*.json origin)
  #     so the w^2*r / dw/dt*r lever-arm terms are removed (or set "imu_pos" in the robot JSON):
  python tools/sysid_mu.py telemetry SysID/drive_sysid.csv --robot robots/full.json --imu-x 0.03 --imu-y 0.0

  # 2) Closed-loop Sim2Real inverse solve against telemetry CSV:
  python tools/sysid_mu.py sim2real --track tracks/finals_1_5x_championship.json ^
      --telemetry runs/ref/drive1.csv --robot robots/full.json --drive 1

  # 3) Closed-loop Sim2Real inverse solve against real save_slot logs:
  python tools/sysid_mu.py sim2real --track tracks/my_track.json ^
      --logs real_logs/주행 로그 --answer real_logs/주행 로그/본선 마커 정답.txt
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from logs import first_deviation, load_all, read_answer  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_EXE = os.path.join(ROOT, "build", "Release", "linesim.exe")
G_ACCEL = 9.80665
DEG2RAD = math.pi / 180.0


def _load_robot(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _save_robot_mu(path: str, mu: float, mu_viscous: float) -> None:
    cfg = _load_robot(path)
    cfg["tire_mu"] = round(float(mu), 4)
    cfg.setdefault("tire", {})
    cfg["tire"]["mu"] = round(float(mu), 4)
    cfg["tire"]["mu_viscous"] = round(float(mu_viscous), 4)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")


def load_telemetry_csv(path: str) -> Dict[str, np.ndarray]:
    """
    Load either LineSim `drive*.csv` or firmware `drive_sysid.csv`
    into unified numpy arrays.
    """
    t_list: List[float] = []
    v_body_list: List[float] = []
    v_l_list: List[float] = []
    v_r_list: List[float] = []
    wz_dps_list: List[float] = []
    lat_mm_list: List[float] = []
    s_list: List[float] = []
    beta_deg_list: List[float] = []
    yaw_deg_list: List[float] = []
    lost_list: List[float] = []
    x_list: List[float] = []
    y_list: List[float] = []
    base_mps_list: List[float] = []
    meta_params: Dict[str, float] = {}
    meta_imu: Dict[str, float] = {}
    meta_lost: Optional[bool] = None
    acc_x_list: List[float] = []
    acc_y_list: List[float] = []
    has_acc_cols = False
    has_v_body = False
    has_beta = False
    has_yaw = False
    has_xy = False
    has_s = False

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        header: Optional[List[str]] = None
        col_idx: Dict[str, int] = {}
        for row in reader:
            if not row:
                continue
            first = row[0].strip()
            if first.startswith("#"):
                line = ",".join(row).lstrip("#").strip()
                if "=" in line:
                    mk, mv = line.split("=", 1)
                    mk = mk.strip()
                    try:
                        val = float(mv.strip().split()[0])
                        if mk == "line_lost":
                            meta_lost = (val > 0.5)
                        elif mk in ("base_mps", "max_mps", "accel", "decel", "steer_kp", "steer_kd", "pos_atten_gain"):
                            meta_params[mk.replace("_", " ")] = val
                        elif mk in ("acc_capture", "acc_bias_x", "acc_bias_y", "fan_en"):
                            meta_imu[mk] = val
                    except ValueError:
                        pass
                continue
            if header is None:
                header = [c.strip() for c in row]
                col_idx = {name: i for i, name in enumerate(header)}
                has_v_body = "v" in col_idx
                has_beta = "beta_deg" in col_idx
                has_yaw = "yaw_deg" in col_idx
                has_xy = ("x" in col_idx) and ("y" in col_idx)
                has_s = ("s" in col_idx) or ("odom_m" in col_idx)
                has_acc_cols = ("acc_x" in col_idx) and ("acc_y" in col_idx)
                continue

            try:
                if "t" in col_idx:
                    t_val = float(row[col_idx["t"]])
                elif "t_ms" in col_idx:
                    t_val = float(row[col_idx["t_ms"]]) * 1e-3
                else:
                    continue

                if "wheel_v_l" in col_idx and "wheel_v_r" in col_idx:
                    vl = float(row[col_idx["wheel_v_l"]])
                    vr = float(row[col_idx["wheel_v_r"]])
                elif "mps_L" in col_idx and "mps_R" in col_idx:
                    vl = float(row[col_idx["mps_L"]])
                    vr = float(row[col_idx["mps_R"]])
                else:
                    continue

                vb = float(row[col_idx["v"]]) if has_v_body else 0.5 * (vl + vr)
                if "wz_dps" in col_idx:
                    wz = float(row[col_idx["wz_dps"]])
                elif "gyro_z_dps" in col_idx:
                    wz = float(row[col_idx["gyro_z_dps"]])
                else:
                    continue

                if "lat_mm" in col_idx:
                    lat = float(row[col_idx["lat_mm"]])
                elif "line_pos" in col_idx:
                    # Normalized [-1, 1] -> approximate mm (half-sensor width ~40mm)
                    lat = float(row[col_idx["line_pos"]]) * 40.0
                else:
                    lat = 0.0

                if "odom_m" in col_idx:
                    s_val = float(row[col_idx["odom_m"]])
                elif "fw_odom" in col_idx:
                    s_val = float(row[col_idx["fw_odom"]])
                elif "s" in col_idx:
                    s_val = float(row[col_idx["s"]])
                else:
                    s_val = 0.0

                beta = float(row[col_idx["beta_deg"]]) if has_beta else float("nan")
                yaw_d = float(row[col_idx["yaw_deg"]]) if has_yaw else float("nan")
                lost = float(row[col_idx["fw_lost"]]) if "fw_lost" in col_idx else 0.0
                if "body_contact" in col_idx and float(row[col_idx["body_contact"]]) > 0.5:
                    lost = 1.0
                x_val = float(row[col_idx["x"]]) if has_xy else 0.0
                y_val = float(row[col_idx["y"]]) if has_xy else 0.0
                ax_val = float(row[col_idx["acc_x"]]) if has_acc_cols else 0.0
                ay_val = float(row[col_idx["acc_y"]]) if has_acc_cols else 0.0
                if "fw_target_base" in col_idx:
                    base_mps_list.append(float(row[col_idx["fw_target_base"]]))
            except (ValueError, IndexError):
                continue

            t_list.append(t_val)
            v_l_list.append(vl)
            v_r_list.append(vr)
            v_body_list.append(vb)
            wz_dps_list.append(wz)
            lat_mm_list.append(lat)
            s_list.append(s_val)
            beta_deg_list.append(beta)
            yaw_deg_list.append(yaw_d)
            lost_list.append(lost)
            x_list.append(x_val)
            y_list.append(y_val)
            acc_x_list.append(ax_val)
            acc_y_list.append(ay_val)

    if len(t_list) < 20:
        raise ValueError(f"Not enough telemetry rows in {path}: {len(t_list)}")

    t_arr = np.asarray(t_list, dtype=np.float64)
    v_arr = np.asarray(v_body_list, dtype=np.float64)
    vl_arr = np.asarray(v_l_list, dtype=np.float64)
    vr_arr = np.asarray(v_r_list, dtype=np.float64)
    wz_arr = np.asarray(wz_dps_list, dtype=np.float64)
    lat_arr = np.asarray(lat_mm_list, dtype=np.float64)
    s_arr = np.asarray(s_list, dtype=np.float64)
    beta_arr = np.asarray(beta_deg_list, dtype=np.float64)

    if not has_s:
        dt = np.diff(t_arr, prepend=t_arr[0])
        s_arr = np.cumsum(np.maximum(0.0, v_arr) * np.clip(dt, 0.0, 0.05))

    if "base mps" not in meta_params and base_mps_list:
        meta_params["base mps"] = float(np.median(base_mps_list))

    # Firmware `SysID Drive` logs raw IMU accel (sensor frame, m/s^2) plus the still-calibration
    # bias in the header. Columns are all zero when the run had acc_capture=0 (e.g. 1st Drive).
    acc_x_arr = np.asarray(acc_x_list, dtype=np.float64)
    acc_y_arr = np.asarray(acc_y_list, dtype=np.float64)
    has_acc = (
        has_acc_cols
        and meta_imu.get("acc_capture", 1.0) > 0.5
        and bool(np.any(np.abs(acc_x_arr) > 1e-6) or np.any(np.abs(acc_y_arr) > 1e-6))
    )
    if has_acc:
        acc_x_arr = acc_x_arr - meta_imu.get("acc_bias_x", 0.0)
        acc_y_arr = acc_y_arr - meta_imu.get("acc_bias_y", 0.0)

    return {
        "t": t_arr,
        "v": v_arr,
        "v_l": vl_arr,
        "v_r": vr_arr,
        "wz_dps": wz_arr,
        "lat_mm": lat_arr,
        "s": s_arr,
        "beta_deg": beta_arr,
        "yaw_deg": np.asarray(yaw_deg_list, dtype=np.float64),
        "lost": np.asarray(lost_list, dtype=np.float64),
        "x": np.asarray(x_list, dtype=np.float64),
        "y": np.asarray(y_list, dtype=np.float64),
        "meta_params": meta_params,
        "meta_imu": meta_imu,
        "acc_x": acc_x_arr,
        "acc_y": acc_y_arr,
        "has_acc": np.array([1.0 if has_acc else 0.0]),
        "meta_lost": np.array([1.0 if meta_lost else 0.0]) if meta_lost is not None else np.array([]),
        "has_v_body": np.array([1.0 if has_v_body else 0.0]),
        "has_beta": np.array([1.0 if has_beta else 0.0]),
        "has_yaw": np.array([1.0 if has_yaw else 0.0]),
        "has_xy": np.array([1.0 if has_xy else 0.0]),
    }


def _smooth_centered(x: np.ndarray, half: int) -> np.ndarray:
    # Centered moving average over 2*half+1 samples, edge-padded (no phase lag).
    if half <= 0 or len(x) < 2 * half + 1:
        return x.astype(np.float64, copy=True)
    k = np.ones(2 * half + 1, dtype=np.float64) / float(2 * half + 1)
    return np.convolve(np.pad(x, half, mode="edge"), k, mode="valid")


def _centered_diff(x: np.ndarray, t: np.ndarray, half: int) -> np.ndarray:
    out = np.zeros_like(x)
    if len(x) > 2 * half + 1 and half > 0:
        span = np.maximum(1e-3, t[2 * half :] - t[: -2 * half])
        out[half:-half] = (x[2 * half :] - x[: -2 * half]) / span
    return out


def _corr(a: np.ndarray, b: np.ndarray) -> float:
    if len(a) < 3 or float(np.std(a)) < 1e-9 or float(np.std(b)) < 1e-9:
        return 0.0
    return float(np.corrcoef(a, b)[0, 1])


def _calibrate_enc_yaw(
    vl: np.ndarray,
    vr: np.ndarray,
    v: np.ndarray,
    w_gyro: np.ndarray,
    track_width: float,
    half: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Dict[str, Any]]:
    # Encoder yaw rate (vR - vL) / track vs gyro yaw rate.
    #
    # The sign convention (wheel L/R labels or gyro axis) and the effective track width are not
    # assumed: w_enc = b * w_gyro is fitted on gentle turns (|v*wz| < 4 m/s^2) where tyre slip is
    # negligible, so only the remaining mismatch in hard cornering is read as slip. Both signals
    # are smoothed over `half` samples each side because the logged wheel speeds are noisy.
    w_e = _smooth_centered((vr - vl) / max(0.05, track_width), half)
    w_g = _smooth_centered(w_gyro, half)
    v_s = _smooth_centered(v, half)
    turning = (np.abs(v_s) > 0.5) & (np.abs(w_g) > 1.0)
    a_t = np.abs(v_s * w_g)
    n_turn = int(np.sum(turning))
    gentle = turning & (a_t < 4.0)
    if int(np.sum(gentle)) < 1000 and n_turn >= 200:
        # Too few gentle samples for a stable fit (smoothed noise is strongly autocorrelated):
        # take the 1000 lowest-|a_lat| turning samples, at most half of all turning samples.
        k = min(1000, n_turn // 2)
        thr = float(np.sort(a_t[turning])[k - 1])
        gentle = turning & (a_t <= max(4.0, thr))
    fit = gentle if int(np.sum(gentle)) >= 100 else turning
    info: Dict[str, Any] = {
        "fit_set": "gentle_turns" if fit is gentle else "all_turns",
        "fit_samples": int(np.sum(fit)),
        "fit_lat_acc_max": round(float(np.max(a_t[fit])), 2) if np.any(fit) else None,
    }
    b, corr, b_se = 1.0, 0.0, float("nan")
    if int(np.sum(fit)) >= 50:
        sgg = max(1e-9, float(np.sum(w_g[fit] ** 2)))
        b = float(np.sum(w_e[fit] * w_g[fit]) / sgg)
        corr = _corr(w_e[fit], w_g[fit])
        resid = w_e[fit] - b * w_g[fit]
        # standard error with the smoothing window as the decorrelation length
        b_se = float(np.sqrt(np.sum(resid ** 2) / max(1, int(np.sum(fit)) - 1) / sgg * (2 * half + 1)))
    warnings: List[str] = []
    sign = 1.0
    scale = 1.0
    if abs(corr) >= 0.3:
        sign = 1.0 if b >= 0.0 else -1.0
        if 0.5 <= abs(b) <= 2.0:
            scale = abs(b)
        else:
            warnings.append(f"Encoder/gyro yaw scale {abs(b):.2f} out of [0.5, 2]; scale left at 1.")
    else:
        warnings.append(
            f"Encoder vs gyro yaw rate barely correlated (|corr|={abs(corr):.2f}); sign/scale not calibrated, "
            "v_slide (and mu_viscous) unreliable."
        )
    info.update({
        "sign": int(sign),
        "scale": round(scale, 4),
        "scale_se": round(b_se, 4) if np.isfinite(b_se) else None,
        "effective_track_m": round(track_width * scale, 4),
        "corr": round(abs(corr), 3),
        "window_ms": None,
    })
    if warnings:
        info["warnings"] = warnings
    return w_e / (sign * scale), w_g, v_s, info


def _imu_planar_accel(
    data: Dict[str, np.ndarray],
    v: np.ndarray,
    w_gyro: np.ndarray,
    win: int,
    dt_med: float,
    imu_r: Optional[Tuple[float, float]],
) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    # Planar CG acceleration from the logged IMU accel (sensor frame, bias removed).
    #
    # The IMU mounting axes are not assumed: the lateral axis/sign is the one that best
    # correlates with the centripetal reference v*wz, and the longitudinal sign is taken from
    # d(v)/dt. Body frame B: x forward, z = gyro-positive axis, y = z x x (so a_y = +v*wz).
    # With the IMU at r = (rx, ry) from the CG (rx forward, ry left; assumes gyro + = CCW
    # as documented in the firmware), the rigid-body lever-arm terms are removed:
    #     a_cg = a_imu - dw/dt (z x r) + w^2 r
    t = data["t"]
    ax_s = _smooth_centered(data["acc_x"], win)
    ay_s = _smooth_centered(data["acc_y"], win)
    w_s = _smooth_centered(w_gyro, win)
    v_s = _smooth_centered(v, win)
    dwdt = _centered_diff(w_s, t, win)
    dvdt = _centered_diff(v_s, t, win)

    moving = np.isfinite(ax_s) & np.isfinite(ay_s) & (np.abs(v_s) > 0.3)
    lat_ref = (v_s * w_s)[moving]
    c_x = _corr(ax_s[moving], lat_ref)
    c_y = _corr(ay_s[moving], lat_ref)
    if abs(c_y) >= abs(c_x):
        lat_name, lon_name, c_lat = "acc_y", "acc_x", c_y
        lat_raw, lon_raw = ay_s, ax_s
    else:
        lat_name, lon_name, c_lat = "acc_x", "acc_y", c_x
        lat_raw, lon_raw = ax_s, ay_s
    lat_sign = 1.0 if c_lat >= 0.0 else -1.0
    c_lon = _corr(lon_raw[moving], dvdt[moving])
    lon_sign = 1.0 if c_lon >= 0.0 else -1.0
    a_lat = lat_sign * lat_raw
    a_lon = lon_sign * lon_raw

    w2 = w_s * w_s
    if imu_r is not None:
        rx, ry = imu_r
        a_lon = a_lon + w2 * rx + dwdt * ry
        a_lat = a_lat + w2 * ry - dwdt * rx

    a_tot = np.clip(np.hypot(a_lon, a_lat), 0.0, 30.0)
    w2_p95 = float(np.percentile(w2[moving], 95.0)) if np.any(moving) else 0.0
    info: Dict[str, Any] = {
        "lat_axis": ("-" if lat_sign < 0 else "+") + lat_name,
        "lon_axis": ("-" if lon_sign < 0 else "+") + lon_name,
        "corr_lat_vs_v_wz": round(abs(c_lat), 3),
        "corr_lon_vs_dvdt": round(abs(c_lon), 3),
        "imu_offset_from_cg_m": None if imu_r is None else [round(imu_r[0], 4), round(imu_r[1], 4)],
        "smooth_window_ms": round((2 * win + 1) * dt_med * 1e3, 1),
    }
    warnings: List[str] = []
    if abs(c_lat) < 0.5:
        warnings.append(f"IMU lateral axis poorly identified (|corr|={abs(c_lat):.2f} < 0.5); check the accel data.")
    if abs(c_lon) < 0.1:
        warnings.append(f"IMU longitudinal sign ambiguous (|corr|={abs(c_lon):.2f}); only matters with an IMU offset.")
    if imu_r is None:
        warnings.append(
            "IMU position unknown (--imu-x/--imu-y or robot JSON imu_pos): no lever-arm correction. "
            f"Each 1 cm of IMU-CG distance adds up to ~{w2_p95 * 0.01:.2f} m/s^2 (p95 w^2) to |a|."
        )
    if warnings:
        info["warnings"] = warnings
    return a_tot, np.abs(dwdt), info


def deconvolve_mu_from_telemetry(
    data: Dict[str, np.ndarray],
    track_width: float = 0.187,
    mass: float = 0.3432,
    fan_force: float = 0.0,
    min_speed: float = 0.7,
    min_mu_eff: float = 0.40,
    com_xy: Tuple[float, float] = (0.0, 0.0),
    imu_xy: Optional[Tuple[float, float]] = None,
    use_imu_accel: bool = True,
) -> Dict[str, Any]:
    """
    Inversely separate Coulomb `mu` and viscous `mu_viscous` from telemetry
    by fitting the upper-envelope traction curve:
        mu_eff(v_slide) = mu + mu_viscous * v_slide
    """
    t = data["t"]
    v = data["v"]
    vl = data["v_l"]
    vr = data["v_r"]
    wz_dps = data["wz_dps"]
    beta_deg = data["beta_deg"]
    yaw_deg = data["yaw_deg"]
    lost = data["lost"]
    has_v_body = bool(data["has_v_body"][0] > 0.5)
    has_beta = bool(data["has_beta"][0] > 0.5)
    has_yaw = bool(data["has_yaw"][0] > 0.5)
    has_xy = bool(data["has_xy"][0] > 0.5)
    has_acc = bool(len(data.get("has_acc", [])) > 0 and data["has_acc"][0] > 0.5)

    dt_med = float(np.median(np.diff(t)))
    dt_med = max(1e-4, min(0.05, dt_med))

    # Centered acceleration over ~20ms window
    win = max(2, int(round(0.010 / dt_med)))
    w_gyro = wz_dps * DEG2RAD
    w_enc = (vr - vl) / max(0.05, track_width)

    if has_xy and len(v) > 4 * win + 2:
        x_pos = data["x"]
        y_pos = data["y"]
        vx_w = np.zeros_like(v)
        vy_w = np.zeros_like(v)
        dt_v = np.maximum(1e-3, t[2 * win :] - t[: -2 * win])
        vx_w[win:-win] = (x_pos[2 * win :] - x_pos[: -2 * win]) / dt_v
        vy_w[win:-win] = (y_pos[2 * win :] - y_pos[: -2 * win]) / dt_v
        a_tot = np.zeros_like(v)
        ax_w = (vx_w[2 * win :] - vx_w[: -2 * win]) / dt_v
        ay_w = (vy_w[2 * win :] - vy_w[: -2 * win]) / dt_v
        a_tot[2 * win : -2 * win] = np.clip(np.hypot(ax_w[win:-win], ay_w[win:-win]), 0.0, 30.0)
        dw_dt = np.zeros_like(v)
        accel_source = "xy"
    elif use_imu_accel and has_acc and len(v) > 2 * win + 1:
        imu_r = None
        if imu_xy is not None:
            imu_r = (float(imu_xy[0]) - float(com_xy[0]), float(imu_xy[1]) - float(com_xy[1]))
        a_tot, dw_dt, imu_info = _imu_planar_accel(data, v, w_gyro, win, dt_med, imu_r)
        accel_source = "imu"
    elif has_yaw and has_beta and len(v) > 2 * win + 1:
        # True world-frame velocity vector acceleration (beta is yaw_vel - yaw_body)
        course_rad = (yaw_deg - np.nan_to_num(beta_deg)) * DEG2RAD
        vx_w = v * np.cos(course_rad)
        vy_w = v * np.sin(course_rad)
        a_tot = np.zeros_like(v)
        dt_span = np.maximum(1e-3, t[2 * win :] - t[: -2 * win])
        ax_w = (vx_w[2 * win :] - vx_w[: -2 * win]) / dt_span
        ay_w = (vy_w[2 * win :] - vy_w[: -2 * win]) / dt_span
        a_tot[win:-win] = np.clip(np.hypot(ax_w, ay_w), 0.0, 30.0)
        dw_dt = np.zeros_like(v)
        accel_source = "yaw_beta"
    else:
        ax = np.zeros_like(v)
        dw_dt = np.zeros_like(v)
        if len(v) > 2 * win + 1:
            dt_span = np.maximum(1e-3, t[2 * win :] - t[: -2 * win])
            ax[win:-win] = np.clip((v[2 * win :] - v[: -2 * win]) / dt_span, -15.0, 15.0)
            dw_dt[win:-win] = np.abs((w_gyro[2 * win :] - w_gyro[: -2 * win]) / dt_span)
        ay = np.abs(v * w_gyro)
        a_tot = np.hypot(ax, ay)
        accel_source = "wheel_diff"

    extra: Dict[str, Any] = {"accel_source": accel_source}
    if accel_source == "imu":
        extra["imu_accel"] = imu_info

    fz_ratio = 1.0 + max(0.0, fan_force) / max(0.05, mass * G_ACCEL)
    mu_eff = a_tot / (G_ACCEL * fz_ratio)

    # Estimate tread sliding velocity v_slide [m/s]
    v_wheel = 0.5 * (vl + vr)
    if has_beta and not np.all(np.isnan(beta_deg)):
        b_rad = np.nan_to_num(beta_deg) * DEG2RAD
        v_long_slip = np.abs(v_wheel - v * np.cos(b_rad)) if has_v_body else np.zeros_like(v)
        v_lat_slip = np.abs(v * np.sin(b_rad))
    else:
        v_long_slip = np.abs(v_wheel - v) if has_v_body else np.zeros_like(v)
        half_slip = max(win, int(round(0.050 / dt_med)))
        w_enc_c, w_gyro_s, v_slip_s, yaw_cal = _calibrate_enc_yaw(vl, vr, v, w_gyro, track_width, half_slip)
        yaw_cal["window_ms"] = round((2 * half_slip + 1) * dt_med * 1e3, 1)
        yaw_slip_ratio = np.abs(w_enc_c - w_gyro_s) / np.maximum(np.maximum(np.abs(w_enc_c), np.abs(w_gyro_s)), 2.0)
        v_lat_slip = np.abs(v_slip_s) * np.clip(yaw_slip_ratio, 0.0, 1.0)
        # Noise floor: the same proxy on near-straight samples, where real slip is ~0.
        low = (np.abs(v_slip_s) >= min_speed) & (np.abs(v_slip_s * w_gyro_s) < 2.0)
        if int(np.sum(low)) >= 30:
            v_floor = np.hypot(v_long_slip[low], v_lat_slip[low])
            yaw_cal["v_slide_floor_p50"] = round(float(np.percentile(v_floor, 50.0)), 4)
            yaw_cal["v_slide_floor_p90"] = round(float(np.percentile(v_floor, 90.0)), 4)
        extra["enc_yaw_cal"] = yaw_cal
    v_slide = np.hypot(v_long_slip, v_lat_slip)

    mask = (
        (v >= min_speed)
        & (mu_eff >= min_mu_eff)
        & (mu_eff <= 3.0)
        & (lost < 0.5)
        & (dw_dt < 25.0)
        & np.isfinite(mu_eff)
        & np.isfinite(v_slide)
    )
    if int(np.sum(mask)) < 30:
        return {
            "error": "Not enough high-traction cornering samples for inverse deconvolution",
            "valid_samples": int(np.sum(mask)),
            **extra,
        }

    vs = v_slide[mask]
    mu_s = mu_eff[mask]

    floor_p90 = extra.get("enc_yaw_cal", {}).get("v_slide_floor_p90")
    if floor_p90 is not None and floor_p90 >= float(np.median(vs)):
        extra.setdefault("warnings", []).append(
            f"Half of the cornering v_slide values are below the straight-line noise floor "
            f"(p90 {floor_p90:.2f} m/s): mu_viscous is poorly identified from this run."
        )

    # Single-constant mu estimate (what naive sysid gets without v_slide separation)
    mu_constant_p95 = float(np.percentile(mu_s, 95.0))
    mu_constant_p98 = float(np.percentile(mu_s, 98.0))

    # Bin by v_slide quantiles to build upper-envelope (92nd percentile) pairs
    n_bins = min(14, max(5, int(len(vs) // 80)))
    edges = np.quantile(vs, np.linspace(0.0, 1.0, n_bins + 1))
    bin_x: List[float] = []
    bin_y: List[float] = []
    bin_w: List[float] = []
    bins_detail: List[Dict[str, float]] = []

    for b in range(n_bins):
        lo, hi = edges[b], edges[b + 1]
        if b < n_bins - 1:
            bm = (vs >= lo) & (vs < hi)
        else:
            bm = (vs >= lo) & (vs <= hi)
        cnt = int(np.sum(bm))
        if cnt < 8:
            continue
        x_med = float(np.median(vs[bm]))
        y_env = float(np.percentile(mu_s[bm], 92.0))
        bin_x.append(x_med)
        bin_y.append(y_env)
        bin_w.append(math.sqrt(float(cnt)))
        bins_detail.append(
            {
                "v_slide_med": round(x_med, 4),
                "mu_eff_p92": round(y_env, 4),
                "count": cnt,
            }
        )

    if len(bin_x) < 3 or (max(bin_x) - min(bin_x)) < 0.05:
        return {
            "mu_coulomb": round(mu_constant_p95, 4),
            "mu_viscous": 0.0,
            "mu_constant_p95": round(mu_constant_p95, 4),
            "mu_constant_p98": round(mu_constant_p98, 4),
            "valid_samples": int(len(vs)),
            "note": "Insufficient v_slide spread to separate mu_viscous; returning p95 constant mu.",
            "bins": bins_detail,
            **extra,
        }

    bx = np.asarray(bin_x, dtype=np.float64)
    by = np.asarray(bin_y, dtype=np.float64)
    bw = np.asarray(bin_w, dtype=np.float64)

    # Separate elastic brush ramp (low v_slide) from sliding region (high v_slide)
    # by fitting the post-knee sliding bins where slope flattens to mu_viscous.
    best_rmse = 1e9
    mu_coulomb = float(np.clip(mu_constant_p95, 0.5, 2.2))
    mu_viscous = 0.0
    max_start = max(1, len(bx) - 3)
    for start in range(max_start):
        sx = bx[start:]
        sy = by[start:]
        sw = bw[start:]
        if (sx[-1] - sx[0]) < 0.15:
            continue
        w_sum = float(np.sum(sw))
        x_bar = float(np.sum(sw * sx) / w_sum)
        y_bar = float(np.sum(sw * sy) / w_sum)
        denom = float(np.sum(sw * (sx - x_bar) ** 2))
        slope = float(np.sum(sw * (sx - x_bar) * (sy - y_bar)) / denom) if denom > 1e-9 else 0.0
        intercept = y_bar - slope * x_bar
        if slope < 0.0:
            slope = 0.0
            intercept = y_bar
        preds = intercept + slope * sx
        rmse_sub = float(np.sqrt(np.mean((sy - preds) ** 2)))
        # Prefer larger sliding span when RMSE is comparable
        score = rmse_sub - 0.015 * float(len(sx))
        if score < best_rmse and 0.4 <= intercept <= 2.5 and 0.0 <= slope <= 1.5:
            best_rmse = score
            mu_coulomb = float(np.clip(intercept, 0.5, 2.2))
            mu_viscous = float(np.clip(slope, 0.0, 1.5))

    rmse = float(np.sqrt(np.mean((by - (mu_coulomb + mu_viscous * bx)) ** 2)))

    return {
        "mu_coulomb": round(mu_coulomb, 4),
        "mu_viscous": round(mu_viscous, 4),
        "envelope_rmse": round(rmse, 4),
        "mu_constant_p95": round(mu_constant_p95, 4),
        "mu_constant_p98": round(mu_constant_p98, 4),
        "v_slide_p50": round(float(np.median(vs)), 4),
        "v_slide_p95": round(float(np.percentile(vs, 95.0)), 4),
        "valid_samples": int(len(vs)),
        "bins": bins_detail,
        **extra,
    }


def _run_linesim(
    exe: str,
    track: str,
    robot: str,
    drive: int,
    mu: float,
    mu_viscous: float,
    out_dir: str,
    fw_params: Optional[Dict[str, float]] = None,
    max_time: float = 45.0,
) -> Tuple[Optional[Dict[str, Any]], str]:
    os.makedirs(out_dir, exist_ok=True)
    cmd = [
        exe,
        "run",
        "--track",
        track,
        "--robot",
        robot,
        "--drives",
        str(drive),
        "--robot-param",
        f"tire_mu={mu:.5f}",
        "--robot-param",
        f"tire_mu_viscous={mu_viscous:.5f}",
        "--max-time",
        f"{max_time:.1f}",
        "--quiet",
        "--out",
        out_dir,
    ]
    if fw_params:
        key_map = {
            "base mps": "base_mps",
            "max mps": "max_mps",
            "accel": "accel",
            "decel": "decel",
            "steer kp": "steer_kp",
            "steer kd": "steer_kd",
            "pos atten gain": "pos_atten_gain",
        }
        for k_src, k_cli in key_map.items():
            if k_src in fw_params:
                cmd.extend(["--set", f"{k_cli}={fw_params[k_src]}"])

    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    summary_path = os.path.join(out_dir, "summary.json")
    if not os.path.isfile(summary_path):
        return None, out_dir
    with open(summary_path, "r", encoding="utf-8") as f:
        return json.load(f), out_dir


def _score_telemetry_sim(
    ref: Dict[str, np.ndarray],
    sim: Dict[str, np.ndarray],
    sim_drive_info: Dict[str, Any],
) -> Tuple[float, Dict[str, float]]:
    """Distance-aligned loss between reference telemetry and simulated drive CSV."""
    outcome = str(sim_drive_info.get("outcome", "unknown"))
    if len(ref["meta_lost"]) > 0:
        ref_lost = bool(ref["meta_lost"][0] > 0.5)
    else:
        ref_lost = bool(ref["lost"][-1] > 0.5)
    sim_lost = not outcome.startswith("finished")
    outcome_pen = 0.0 if (sim_lost == ref_lost) else 40.0
    # Note: drive_sysid.csv buffers up to 15s (7500 samples @ 500Hz);
    # only penalize s_end mismatch when ref is a full run (not a capped 15s buffer)
    is_clipped_buf = (not bool(ref["has_v_body"][0] > 0.5)) and (float(sim["s"][-1]) > float(ref["s"][-1]) + 1.0) and (not ref_lost)
    s_end_pen = 0.0 if is_clipped_buf else abs(float(sim["s"][-1]) - float(ref["s"][-1])) * 1.5

    # Monotonic distance grid over overlapping [s_min, s_max]
    s_ref = np.maximum.accumulate(ref["s"])
    s_sim = np.maximum.accumulate(sim["s"])
    s_lo = max(float(s_ref[0]), float(s_sim[0]), 0.1)
    s_hi = min(float(s_ref[-1]), float(s_sim[-1]))
    if s_hi <= s_lo + 0.5:
        return 1000.0, {"outcome_pen": 1000.0}

    s_grid = np.linspace(s_lo, s_hi, 600)
    use_wheel_v = not bool(ref["has_v_body"][0] > 0.5)
    v_ref_sig = 0.5 * (ref["v_l"] + ref["v_r"]) if use_wheel_v else ref["v"]
    v_sim_sig = 0.5 * (sim["v_l"] + sim["v_r"]) if use_wheel_v else sim["v"]
    v_r = np.interp(s_grid, s_ref, v_ref_sig)
    v_s = np.interp(s_grid, s_sim, v_sim_sig)
    wz_r = np.interp(s_grid, s_ref, ref["wz_dps"])
    wz_s = np.interp(s_grid, s_sim, sim["wz_dps"])
    lat_r = np.interp(s_grid, s_ref, ref["lat_mm"])
    lat_s = np.interp(s_grid, s_sim, sim["lat_mm"])

    v_rmse = float(np.sqrt(np.mean((v_r - v_s) ** 2)))
    wz_rmse = float(np.sqrt(np.mean((wz_r - wz_s) ** 2)))
    lat_rmse = float(np.sqrt(np.mean((lat_r - lat_s) ** 2)))

    t_ref = float(ref["t"][-1] - ref["t"][0])
    t_sim_at_shi = float(np.interp(s_hi, s_sim, sim["t"]) - sim["t"][0])
    t_err = abs(t_sim_at_shi - t_ref) if is_clipped_buf else abs(float(sim["t"][-1] - sim["t"][0]) - t_ref)

    beta_pen = 0.0
    if bool(ref["has_beta"][0] > 0.5) and not np.all(np.isnan(ref["beta_deg"])):
        b_r = float(np.nanmax(np.abs(ref["beta_deg"])))
        b_s = float(np.nanmax(np.abs(sim["beta_deg"])))
        beta_pen = abs(b_s - b_r) / 15.0

    loss = (
        outcome_pen
        + s_end_pen
        + 2.0 * v_rmse
        + 1.0 * (wz_rmse / 100.0)
        + 0.8 * (lat_rmse / 10.0)
        + 0.5 * t_err
        + 0.6 * beta_pen
    )
    metrics = {
        "loss": round(loss, 4),
        "v_rmse": round(v_rmse, 4),
        "wz_rmse_dps": round(wz_rmse, 2),
        "lat_rmse_mm": round(lat_rmse, 2),
        "t_err_s": round(t_err, 3),
        "beta_pen": round(beta_pen, 3),
        "outcome": outcome,
    }
    return loss, metrics


def solve_sim2real_telemetry(
    exe: str,
    track: str,
    robot: str,
    drive: int,
    ref_csv: str,
    mu_range: Tuple[float, float],
    visc_range: Tuple[float, float],
    grid_mu: int,
    grid_visc: int,
    jobs: int,
    max_time: float,
) -> Dict[str, Any]:
    ref_data = load_telemetry_csv(ref_csv)
    fw_params: Optional[Dict[str, float]] = ref_data.get("meta_params") or None
    tmp_root = tempfile.mkdtemp(prefix="sysid_mu_t_")
    eval_cache: Dict[Tuple[float, float], Tuple[float, Dict[str, float]]] = {}

    def eval_pair(mu: float, visc: float, tag: str) -> Tuple[float, Dict[str, float]]:
        key = (round(mu, 4), round(visc, 4))
        if key in eval_cache:
            return eval_cache[key]
        out_dir = os.path.join(tmp_root, tag)
        summary, _ = _run_linesim(
            exe, track, robot, drive, key[0], key[1], out_dir, fw_params=fw_params, max_time=max_time
        )
        drive_csv = os.path.join(out_dir, f"drive{drive}.csv")
        if not summary or not summary.get("drives") or not os.path.isfile(drive_csv):
            res = (1e6, {"outcome": "error"})
        else:
            sim_data = load_telemetry_csv(drive_csv)
            res = _score_telemetry_sim(ref_data, sim_data, summary["drives"][0])
        eval_cache[key] = res
        return res

    try:
        mus = np.linspace(mu_range[0], mu_range[1], grid_mu)
        viscs = np.linspace(visc_range[0], visc_range[1], grid_visc)
        candidates = [(float(m), float(v), f"g_{i}_{j}") for i, m in enumerate(mus) for j, v in enumerate(viscs)]

        with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
            futs = {ex.submit(eval_pair, m, v, tag): (m, v) for m, v, tag in candidates}
            grid_results = []
            for fut in concurrent.futures.as_completed(futs):
                m, v = futs[fut]
                loss, met = fut.result()
                grid_results.append((loss, m, v, met))

        grid_results.sort(key=lambda x: x[0])
        best_loss, best_mu, best_visc, best_met = grid_results[0]

        # Stage 2: coordinate pattern search refinement around best candidate
        step_mu = (mu_range[1] - mu_range[0]) / max(1, grid_mu - 1) * 0.5
        step_visc = (visc_range[1] - visc_range[0]) / max(1, grid_visc - 1) * 0.5
        for it in range(4):
            neighbors = []
            for dm in (-step_mu, 0.0, step_mu):
                for dv in (-step_visc, 0.0, step_visc):
                    if dm == 0.0 and dv == 0.0:
                        continue
                    nm = float(np.clip(best_mu + dm, mu_range[0], mu_range[1]))
                    nv = float(np.clip(best_visc + dv, visc_range[0], visc_range[1]))
                    neighbors.append((nm, nv, f"r_{it}_{len(neighbors)}"))
            with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
                futs = {ex.submit(eval_pair, m, v, tag): (m, v) for m, v, tag in neighbors}
                improved = False
                for fut in concurrent.futures.as_completed(futs):
                    m, v = futs[fut]
                    loss, met = fut.result()
                    if loss < best_loss:
                        best_loss, best_mu, best_visc, best_met = loss, m, v, met
                        improved = True
            if not improved:
                step_mu *= 0.5
                step_visc *= 0.5

        return {
            "mode": "sim2real_telemetry",
            "mu_coulomb": round(best_mu, 4),
            "mu_viscous": round(best_visc, 4),
            "loss": round(best_loss, 4),
            "metrics": best_met,
            "evaluations": len(eval_cache),
            "top_grid": [
                {"mu": round(m, 4), "mu_viscous": round(v, 4), "loss": round(l, 4)}
                for l, m, v, _ in grid_results[:5]
            ],
        }
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)


def _group_real_logs(folder: str, answer_path: str, max_cases: int) -> Tuple[List[str], List[Dict[str, Any]]]:
    ans = read_answer(answer_path)
    runs = load_all(folder, ans)
    if not runs:
        raise ValueError(f"No save_slot_*.txt found in {folder}")

    groups: Dict[Tuple[float, ...], List[Dict[str, Any]]] = {}
    for r in runs:
        p = r["params"]
        k = (
            p.get("base mps", 1.8),
            p.get("accel", 5.5),
            p.get("decel", 5.5),
            p.get("steer kp", 20.0),
            p.get("steer kd", 600.0),
            p.get("pos atten gain", 0.0),
        )
        groups.setdefault(k, []).append(r)

    cases: List[Dict[str, Any]] = []
    for k, rs in sorted(groups.items()):
        ok_count = sum(1 for r in rs if r["dev"] < 0)
        real_ok = ok_count > 0
        devs = [r["dev"] for r in rs if r["dev"] >= 0]
        med_dev = float(np.median(devs)) if devs else -1.0
        dists_per_seg: Optional[List[float]] = None
        if real_ok:
            good_rs = [r for r in rs if r["dev"] < 0]
            dists_per_seg = [
                float(np.median([r["marks"][i][1] for r in good_rs]))
                for i in range(len(ans))
            ]
        cases.append(
            {
                "params": rs[0]["params"],
                "real_ok": real_ok,
                "med_dev": med_dev,
                "dists": dists_per_seg,
                "base_mps": k[0],
            }
        )

    # Prioritize boundary cases (highest-speed OK and lowest-speed FAIL)
    ok_cases = sorted([c for c in cases if c["real_ok"]], key=lambda c: -c["base_mps"])
    fail_cases = sorted([c for c in cases if not c["real_ok"]], key=lambda c: c["base_mps"])
    selected: List[Dict[str, Any]] = []
    while len(selected) < max_cases and (ok_cases or fail_cases):
        if ok_cases:
            selected.append(ok_cases.pop(0))
        if len(selected) < max_cases and fail_cases:
            selected.append(fail_cases.pop(0))
    return ans, selected


def solve_sim2real_logs(
    exe: str,
    track: str,
    robot: str,
    logs_dir: str,
    answer_path: str,
    mu_range: Tuple[float, float],
    visc_range: Tuple[float, float],
    grid_mu: int,
    grid_visc: int,
    jobs: int,
    max_cases: int,
    max_time: float,
) -> Dict[str, Any]:
    ans, cases = _group_real_logs(logs_dir, answer_path, max_cases=max_cases)
    tmp_root = tempfile.mkdtemp(prefix="sysid_mu_l_")
    eval_cache: Dict[Tuple[float, float], Tuple[float, Dict[str, Any]]] = {}

    def eval_pair(mu: float, visc: float, tag: str) -> Tuple[float, Dict[str, Any]]:
        key = (round(mu, 4), round(visc, 4))
        if key in eval_cache:
            return eval_cache[key]
        total_loss = 0.0
        agreements = 0
        dist_rmses: List[float] = []
        for idx, c in enumerate(cases):
            out_dir = os.path.join(tmp_root, f"{tag}_c{idx}")
            summary, _ = _run_linesim(
                exe, track, robot, 1, key[0], key[1], out_dir, fw_params=c["params"], max_time=max_time
            )
            mfile = os.path.join(out_dir, "drive1_markers.csv")
            sim_marks: List[Tuple[str, float]] = []
            if os.path.isfile(mfile):
                with open(mfile, "r", encoding="utf-8") as mf:
                    for row in csv.DictReader(mf):
                        t = row["type"]
                        sim_marks.append(("U" if t == "STOP" else t, float(row["dist_from_prev_m"])))
            sim_dev = first_deviation([m[0] for m in sim_marks], ans)
            sim_ok = (sim_dev < 0)
            if sim_ok == c["real_ok"]:
                agreements += 1
            else:
                total_loss += 10.0

            if not sim_ok and not c["real_ok"] and c["med_dev"] >= 0:
                total_loss += abs(float(sim_dev) - c["med_dev"]) / max(1.0, float(len(ans)))
            elif sim_ok and c["real_ok"] and c["dists"] is not None and len(sim_marks) == len(ans):
                d_rmse = float(np.sqrt(np.mean([(sim_marks[i][1] - c["dists"][i]) ** 2 for i in range(len(ans))])))
                dist_rmses.append(d_rmse)
                total_loss += d_rmse * 5.0

        info = {
            "agreements": agreements,
            "cases": len(cases),
            "mean_dist_rmse_m": round(float(np.mean(dist_rmses)), 4) if dist_rmses else None,
        }
        eval_cache[key] = (total_loss, info)
        return total_loss, info

    try:
        mus = np.linspace(mu_range[0], mu_range[1], grid_mu)
        viscs = np.linspace(visc_range[0], visc_range[1], grid_visc)
        candidates = [(float(m), float(v), f"g_{i}_{j}") for i, m in enumerate(mus) for j, v in enumerate(viscs)]

        with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
            futs = {ex.submit(eval_pair, m, v, tag): (m, v) for m, v, tag in candidates}
            grid_results = []
            for fut in concurrent.futures.as_completed(futs):
                m, v = futs[fut]
                loss, info = fut.result()
                grid_results.append((loss, m, v, info))

        grid_results.sort(key=lambda x: x[0])
        best_loss, best_mu, best_visc, best_info = grid_results[0]
        return {
            "mode": "sim2real_logs",
            "mu_coulomb": round(best_mu, 4),
            "mu_viscous": round(best_visc, 4),
            "loss": round(best_loss, 4),
            "metrics": best_info,
            "cases_used": len(cases),
            "evaluations": len(eval_cache),
            "top_grid": [
                {"mu": round(m, 4), "mu_viscous": round(v, 4), "loss": round(l, 4), **inf}
                for l, m, v, inf in grid_results[:5]
            ],
        }
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Inverse (tire.mu, tire.mu_viscous) SysID for LineSim")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_tel = sub.add_parser("telemetry", help="Inverse deconvolution from drive_sysid.csv or drive*.csv")
    p_tel.add_argument("csv_path", help="Path to drive_sysid.csv or LineSim drive*.csv")
    p_tel.add_argument("--robot", default="robots/full.json", help="Robot JSON for track/mass/fan_force")
    p_tel.add_argument("--out", default="", help="Optional output JSON report path")
    p_tel.add_argument("--apply", action="store_true", help="Write solved mu & mu_viscous to --robot")
    p_tel.add_argument("--imu-x", type=float, default=None,
                       help="IMU x [m] in the robot frame (forward; robot JSON origin). Default: robot JSON imu_pos")
    p_tel.add_argument("--imu-y", type=float, default=None,
                       help="IMU y [m] in the robot frame (left). Default: robot JSON imu_pos, else 0 when --imu-x is given")
    p_tel.add_argument("--no-imu-accel", action="store_true",
                       help="Ignore logged acc_x/acc_y and fall back to differentiated wheel speed (old behaviour)")

    p_s2r = sub.add_parser("sim2real", help="Closed-loop Sim2Real inverse solver using linesim.exe")
    p_s2r.add_argument("--track", required=True, help="Track JSON file")
    p_s2r.add_argument("--robot", default="robots/full.json", help="Base robot JSON file")
    p_s2r.add_argument("--exe", default=DEFAULT_EXE, help="Path to linesim.exe")
    p_s2r.add_argument("--telemetry", default="", help="Reference telemetry CSV (drive_sysid.csv or drive*.csv)")
    p_s2r.add_argument("--drive", type=int, default=1, help="Drive index (1..5) for --telemetry mode")
    p_s2r.add_argument("--logs", default="", help="Real save_slot_*.txt folder")
    p_s2r.add_argument("--answer", default="", help="Marker answer .txt file (required with --logs)")
    p_s2r.add_argument("--mu-min", type=float, default=0.70)
    p_s2r.add_argument("--mu-max", type=float, default=1.60)
    p_s2r.add_argument("--visc-min", type=float, default=0.00)
    p_s2r.add_argument("--visc-max", type=float, default=0.70)
    p_s2r.add_argument("--grid-mu", type=int, default=5)
    p_s2r.add_argument("--grid-visc", type=int, default=4)
    p_s2r.add_argument("--jobs", type=int, default=4)
    p_s2r.add_argument("--max-cases", type=int, default=6)
    p_s2r.add_argument("--max-time", type=float, default=90.0)
    p_s2r.add_argument("--out", default="", help="Optional output JSON report path")
    p_s2r.add_argument("--apply", action="store_true", help="Write solved mu & mu_viscous to --robot")

    args = parser.parse_args()
    robot_cfg = _load_robot(args.robot)

    if args.cmd == "telemetry":
        data = load_telemetry_csv(args.csv_path)
        com = robot_cfg.get("com", [0.0, 0.0, 0.0])
        imu_xy: Optional[Tuple[float, float]] = None
        imu_pos = robot_cfg.get("imu_pos")
        if args.imu_x is not None or args.imu_y is not None:
            imu_xy = (
                float(args.imu_x) if args.imu_x is not None else float(imu_pos[0] if imu_pos else com[0]),
                float(args.imu_y) if args.imu_y is not None else float(imu_pos[1] if imu_pos else 0.0),
            )
        elif imu_pos:
            imu_xy = (float(imu_pos[0]), float(imu_pos[1]))
        fan_force = float(robot_cfg.get("fan_force", 0.0))
        res = deconvolve_mu_from_telemetry(
            data,
            track_width=float(robot_cfg.get("track", 0.187)),
            mass=float(robot_cfg.get("mass", 0.3432)),
            fan_force=fan_force,
            com_xy=(float(com[0]), float(com[1])),
            imu_xy=imu_xy,
            use_imu_accel=not args.no_imu_accel,
        )
        fan_en = data.get("meta_imu", {}).get("fan_en")
        if fan_en is not None and fan_en > 0.5 and fan_force <= 0.0:
            res.setdefault("warnings", []).append(
                f"Run had fan_en={fan_en:g} but {args.robot} fan_force=0: fan downforce is counted as tire mu "
                "(mu overestimated). Re-run with the fan off or set fan_force."
            )
    else:
        if args.telemetry:
            res = solve_sim2real_telemetry(
                exe=args.exe,
                track=args.track,
                robot=args.robot,
                drive=args.drive,
                ref_csv=args.telemetry,
                mu_range=(args.mu_min, args.mu_max),
                visc_range=(args.visc_min, args.visc_max),
                grid_mu=args.grid_mu,
                grid_visc=args.grid_visc,
                jobs=args.jobs,
                max_time=args.max_time,
            )
        elif args.logs and args.answer:
            res = solve_sim2real_logs(
                exe=args.exe,
                track=args.track,
                robot=args.robot,
                logs_dir=args.logs,
                answer_path=args.answer,
                mu_range=(args.mu_min, args.mu_max),
                visc_range=(args.visc_min, args.visc_max),
                grid_mu=args.grid_mu,
                grid_visc=args.grid_visc,
                jobs=args.jobs,
                max_cases=args.max_cases,
                max_time=args.max_time,
            )
        else:
            print("[ERROR] sim2real requires either --telemetry <csv> or --logs <dir> --answer <txt>", file=sys.stderr)
            return 2

    print(json.dumps(res, indent=2, ensure_ascii=False))
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
            f.write("\n")
    if args.apply and "mu_coulomb" in res and "mu_viscous" in res:
        _save_robot_mu(args.robot, res["mu_coulomb"], res["mu_viscous"])
        print(f"[OK] Updated {args.robot}: mu={res['mu_coulomb']}, mu_viscous={res['mu_viscous']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
