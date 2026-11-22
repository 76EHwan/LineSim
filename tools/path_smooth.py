#!/usr/bin/env python3
"""
path_smooth.py - offline prototype of a path-smoothing planner for the 4th run.

Pipeline (one track, one LineSim run folder):
  1. drive-1 log (drive1.csv) -> line reference every DS metres along s.
     line point = body pose (x, y, yaw) + sensor_x ahead, minus lat_mm along the left normal
     (lat_mm > 0 means the sensor bar sits left of the line).
  2. gradient-descent smoothing  y_i += a (x_i - y_i) + b (y_{i-1} + y_{i+1} - 2 y_i)
     (red-black Gauss-Seidel sweep, both ends fixed) until sum |change| < tol.
     After every iteration the lateral offset from the line is clamped to +-d_max.
  3. The sensor bar follows the smoothed path and the body trails it like a trailer
     (no side slip). The body-path curvature gives the friction-circle limit (mu g);
     forward/backward passes with accel/decel and vmax give the speed profile and a
     lap estimate from the standing start to the goal marker.
  4. The signed lateral offset along s becomes target_pos(s) = offset_mm / mm_per_unit,
     clamped to +-TARGET_POS_LIMIT.
  5. Each logged drive's measured lat_mm(s) goes through the same steps 3, so the
     estimates can be tied to the simulator's own lap times (anchored / scaled).
  6. "filter" variants: the closed-form fixed point of step 2 (a two-pass exponential
     low-pass of curvature), from the true line curvature and from a marker map built
     only from the firmware yaw at each marker event - what an on-board version could run.

Usage (from the repo root):
  tools/.venv/Scripts/python.exe tools/path_smooth.py --track tracks/practice_board.json --out runs/path_smooth/practice
  tools/.venv/Scripts/python.exe tools/path_smooth.py --run-dir runs/path_smooth/finals/sim --out runs/path_smooth/finals

Outputs in --out: summary.json, report.md, offset.svg, speed.svg, map.svg,
target_pos_<variant>.csv (one row every DS metres).
Only numpy is needed.
"""

import argparse
import json
import math
import os
import subprocess
import sys

import numpy as np

G = 9.81
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# Categorical slots from the reference data-viz palette (fixed order), plus neutrals.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e4e3df"
NEUTRAL = "#8a8985"
SURFACE = "#fcfcfb"


# ----------------------------------------------------------------------------- io

def load_csv(path):
    with open(path) as f:
        hdr = f.readline().strip().split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return {h: data[:, i] for i, h in enumerate(hdr)}


def load_markers(path):
    rows = []
    with open(path) as f:
        f.readline()
        for line in f:
            p = line.strip().split(",")
            if len(p) >= 2:
                rows.append(p[1])
    return rows


def load_json_c(path):
    """JSON that may start with // comment lines (track files)."""
    with open(path, encoding="utf-8") as f:
        txt = "".join(l for l in f if not l.lstrip().startswith("//"))
    return json.loads(txt)


def run_sim(track, robot, out_dir, drives, seed):
    exe = os.path.join(REPO, "build", "Release", "linesim.exe")
    cmd = [exe, "run", "--track", track, "--robot", robot, "--drives", drives,
           "--report", "full", "--quiet", "--out", out_dir]
    if seed is not None:
        cmd += ["--seed", str(seed)]
    print("running:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO)


# ----------------------------------------------------------------------------- helpers

def gauss1d(a, sigma):
    """Gaussian filter in samples, edge-replicated."""
    if sigma <= 0:
        return a.copy()
    r = int(math.ceil(4 * sigma))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    pad = np.r_[np.full(r, a[0]), a, np.full(r, a[-1])]
    return np.convolve(pad, k, mode="valid")


def continuous_s(d, track_len, jump=0.02):
    """Line distance without the lap wrap and without crossing glitches.

    The logged s / lat_mm come from a nearest-point projection, which snaps to the other
    branch for a few rows at a crossing (s jumps by metres and back). Steps larger than
    `jump` are bridged with the travelled body distance; lat_mm is interpolated over the
    rows where the raw s disagrees with the continuous one.
    Returns (s_continuous, lat_mm_fixed).
    """
    dsr = np.diff(d["s"])
    step = np.hypot(np.diff(d["x"]), np.diff(d["y"]))
    sc = d["s"][0] + np.r_[0, np.cumsum(np.where(np.abs(dsr) < jump, dsr, step))]
    dev = np.abs((d["s"] - sc + 0.5 * track_len) % track_len - 0.5 * track_len)
    bad = dev > jump
    lat = d["lat_mm"].copy()
    if bad.any() and (~bad).any():
        idx = np.arange(len(lat))
        lat[bad] = np.interp(idx[bad], idx[~bad], lat[~bad])
    return sc, lat


def marker_events(d, types):
    """Rows where the firmware marker counter steps, paired with drive<N>_markers.csv types."""
    idx = np.where(np.diff(d["fw_markers"]) != 0)[0] + 1
    n = min(len(idx), len(types))
    return idx[:n], types[:n]


def tangents_normals(X, span=2):
    n = len(X)
    i0 = np.clip(np.arange(n) - span, 0, n - 1)
    i1 = np.clip(np.arange(n) + span, 0, n - 1)
    t = X[i1] - X[i0]
    t /= np.linalg.norm(t, axis=1)[:, None]
    nrm = np.c_[-t[:, 1], t[:, 0]]  # left normal
    return t, nrm


def path_curvature(P, ds, sigma=2.0):
    """Curvature of a uniformly resampled polyline, Gaussian-smoothed (sigma in samples)."""
    gx, gy = np.gradient(P[:, 0]), np.gradient(P[:, 1])
    h = np.unwrap(np.arctan2(gy, gx))
    return gauss1d(np.gradient(h) / ds, sigma)


# ----------------------------------------------------------------------------- planner core

def line_reference(d, sensor_x, track_len, ds):
    s, lat = continuous_s(d, track_len)
    s = np.maximum.accumulate(s)
    keep = np.r_[True, np.diff(s) > 1e-6]
    yaw = np.unwrap(np.radians(d["yaw_deg"]))
    lat = lat * 1e-3
    lx = d["x"] + sensor_x * np.cos(yaw) + lat * np.sin(yaw)
    ly = d["y"] + sensor_x * np.sin(yaw) - lat * np.cos(yaw)
    sk = s[keep]
    grid = np.arange(sk[0], sk[-1], ds)
    X = np.c_[np.interp(grid, sk, lx[keep]), np.interp(grid, sk, ly[keep])]
    return {"s": grid, "X": X, "v1": np.interp(grid, sk, d["v"][keep]), "s_raw": s}


def smooth(X, nrm, alpha, beta, tol, dmax, max_iter):
    """Gradient-descent smoothing with a lateral clamp; returns (Y, iterations, last change).

    dmax: None (no clamp), a scalar or a per-point array of the allowed |offset| [m].
    The update is the gradient of  a/2 |Y-X|^2 + b/2 sum |Y_{i+1}-Y_i|^2  (a spring chain),
    so with a small data weight and a binding clamp the path is pulled taut against the bound.
    """
    Y = X.copy()
    N = len(X)
    odd = np.arange(1, N - 1, 2)
    even = np.arange(2, N - 1, 2)
    change = float("inf")
    it = 0
    for it in range(1, max_iter + 1):
        Y0 = Y.copy()
        for idx in (odd, even):
            Y[idx] += alpha * (X[idx] - Y[idx]) + beta * (Y[idx - 1] + Y[idx + 1] - 2.0 * Y[idx])
        if dmax is not None:
            dn = np.einsum("ij,ij->i", Y - X, nrm)
            c = np.clip(dn, -dmax, dmax)
            fix = dn != c
            Y[fix] -= (dn[fix] - c[fix])[:, None] * nrm[fix]
        change = float(np.abs(Y - Y0).sum())
        if change < tol:
            break
    return Y, it, change


def filter_offset(kappa, ds, alpha, beta):
    """Small-offset fixed point of the GD rule, in closed form.

    With y = x + d n and x'' = kappa n, the fixed point a (x - y) + b (y- + y+ - 2y) = 0 becomes
    a d - b (d- - 2d + d+) = b ds^2 kappa, i.e. d = (b/a) ds^2 * H kappa with
    H = (1-r)^2 / ((1 - r z^-1)(1 - r z)), a forward and a backward first-order low-pass,
    r from b/a = r / (1-r)^2. Length scale ds / -ln(r) (about 90 mm for a=0.005, b=0.4, ds=10 mm).
    """
    c = beta / alpha
    q = 2.0 + 1.0 / c
    r = (q - math.sqrt(q * q - 4.0)) / 2.0
    k = kappa.tolist()
    f = [k[0]]
    for v in k[1:]:
        f.append((1 - r) * v + r * f[-1])
    b = [f[-1]] * len(f)
    for i in range(len(f) - 2, -1, -1):
        b[i] = (1 - r) * f[i] + r * b[i + 1]
    return c * ds * ds * np.array(b)


def marker_map_curvature(s, s_ev, psi_ev, quant_deg):
    """Piecewise-constant curvature between marker events from the firmware yaw at each event."""
    dp = np.diff(psi_ev)
    if quant_deg > 0:
        qq = math.radians(quant_deg)
        dp = np.round(dp / qq) * qq
    km = np.zeros(len(s))
    j = np.searchsorted(s_ev, s, side="right") - 1
    ok = (j >= 0) & (j < len(dp))
    km[ok] = dp[j[ok]] / np.diff(s_ev)[j[ok]]
    return km


def lateral_offset(s, X, Y, tng, nrm):
    """Signed left offset of Y from the line, re-indexed onto the line distance s."""
    e = Y - X
    dn = np.einsum("ij,ij->i", e, nrm)
    dt = np.einsum("ij,ij->i", e, tng)
    sp = s + dt
    order = np.argsort(sp)
    return np.interp(s, sp[order], dn[order]), float(np.abs(dt).max())


def trailer(P, L):
    """Body path of a rigid robot whose sensor point (L ahead) follows P without side slip."""
    seg = np.diff(P, axis=0)
    dsp = np.hypot(seg[:, 0], seg[:, 1]).tolist()
    phi = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0])).tolist()
    th = [phi[0]]
    for i in range(len(dsp)):
        th.append(th[-1] + dsp[i] * math.sin(phi[i] - th[-1]) / L)
    th = np.array(th)
    Q = P - L * np.c_[np.cos(th), np.sin(th)]
    return Q, th


def body_kinematics(P, L, sigma=2.0):
    """Body path, per-point body curvature and per-segment body distance for sensor path P."""
    Q, th = trailer(P, L)
    dq = np.hypot(*np.diff(Q, axis=0).T)
    k_seg = np.diff(th) / np.maximum(dq, 1e-9)
    k_pt = np.r_[k_seg[0], 0.5 * (k_seg[1:] + k_seg[:-1]), k_seg[-1]]
    return Q, gauss1d(k_pt, sigma), dq


def speed_profile(kappa, dseg, mu, accel, decel, vmax, v0=0.0):
    amu = mu * G
    k = np.abs(kappa)
    vlim = np.minimum(vmax, np.sqrt(amu / np.maximum(k, 1e-9)))
    v = vlim.copy().tolist()
    v[0] = min(v[0], v0)
    kl = k.tolist()
    ds = dseg.tolist()
    n = len(v)
    for i in range(n - 1):
        alat = v[i] * v[i] * kl[i]
        a = min(accel, math.sqrt(max(0.0, amu * amu - alat * alat)))
        v[i + 1] = min(v[i + 1], math.sqrt(v[i] * v[i] + 2 * a * ds[i]))
    for i in range(n - 2, -1, -1):
        alat = v[i + 1] * v[i + 1] * kl[i + 1]
        a = min(decel, math.sqrt(max(0.0, amu * amu - alat * alat)))
        v[i] = min(v[i], math.sqrt(v[i + 1] * v[i + 1] + 2 * a * ds[i]))
    return np.array(v), vlim


def seg_time(v, dseg, i0, i1):
    vm = 0.5 * (v[i0:i1] + v[i0 + 1:i1 + 1])
    return float(np.sum(dseg[i0:i1] / np.maximum(vm, 1e-6)))


# ----------------------------------------------------------------------------- svg

def nice_ticks(lo, hi, n=6):
    span = hi - lo
    if span <= 0:
        return [lo]
    step = 10 ** math.floor(math.log10(span / n))
    for m in (1, 2, 2.5, 5, 10):
        if span / (step * m) <= n:
            step *= m
            break
    start = math.ceil(lo / step) * step
    return [round(v, 10) for v in np.arange(start, hi + step * 1e-6, step)]


def decimate(x, y, max_pts=2500):
    k = max(1, len(x) // max_pts)
    return x[::k], y[::k]


def svg_chart(series, title, xlabel, ylabel, xlim=None, ylim=None, w=980, h=340, bands=()):
    ml, mr, mt, mb = 60, 18, 58, 42
    pw, ph = w - ml - mr, h - mt - mb
    xs = np.concatenate([s["x"] for s in series])
    ys = np.concatenate([s["y"] for s in series])
    x0, x1 = xlim if xlim else (float(xs.min()), float(xs.max()))
    if ylim:
        y0, y1 = ylim
    else:
        y0, y1 = float(np.nanmin(ys)), float(np.nanmax(ys))
        pad = 0.06 * (y1 - y0 or 1)
        y0, y1 = y0 - pad, y1 + pad
    fx = lambda v: ml + (v - x0) / (x1 - x0) * pw
    fy = lambda v: mt + (y1 - v) / (y1 - y0) * ph
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}" '
         f'font-family="system-ui,Segoe UI,sans-serif" font-size="12">',
         f'<rect width="{w}" height="{h}" fill="{SURFACE}"/>',
         f'<text x="{ml}" y="20" font-size="14" font-weight="600" fill="{INK}">{title}</text>']
    for yv in nice_ticks(y0, y1):
        o.append(f'<line x1="{ml}" x2="{ml + pw}" y1="{fy(yv):.1f}" y2="{fy(yv):.1f}" stroke="{GRID}"/>')
        o.append(f'<text x="{ml - 6}" y="{fy(yv) + 4:.1f}" text-anchor="end" fill="{INK2}">{yv:g}</text>')
    for xv in nice_ticks(x0, x1, 10):
        o.append(f'<text x="{fx(xv):.1f}" y="{mt + ph + 16}" text-anchor="middle" fill="{INK2}">{xv:g}</text>')
    for (yv, lab) in bands:
        if y0 <= yv <= y1:
            o.append(f'<line x1="{ml}" x2="{ml + pw}" y1="{fy(yv):.1f}" y2="{fy(yv):.1f}" stroke="{NEUTRAL}" '
                     f'stroke-dasharray="4 4"/>')
            if lab:
                o.append(f'<text x="{ml + pw - 4}" y="{fy(yv) - 4:.1f}" text-anchor="end" fill="{INK2}">{lab}</text>')
    o.append(f'<text x="{ml + pw / 2}" y="{h - 6}" text-anchor="middle" fill="{INK2}">{xlabel}</text>')
    o.append(f'<text transform="translate(14,{mt + ph / 2}) rotate(-90)" text-anchor="middle" fill="{INK2}">{ylabel}</text>')
    o.append(f'<clipPath id="pc"><rect x="{ml}" y="{mt}" width="{pw}" height="{ph}"/></clipPath>')
    lx = ml
    for s in series:
        m = (s["x"] >= x0) & (s["x"] <= x1)
        xx, yy = decimate(s["x"][m], s["y"][m])
        pts = " ".join(f"{fx(a):.1f},{fy(b):.1f}" for a, b in zip(xx, yy) if np.isfinite(b))
        dash = f' stroke-dasharray="{s["dash"]}"' if s.get("dash") else ""
        o.append(f'<polyline clip-path="url(#pc)" fill="none" stroke="{s["color"]}" stroke-width="{s.get("width", 1.6)}"'
                 f'{dash} stroke-linejoin="round" points="{pts}"><title>{s["label"]}</title></polyline>')
        o.append(f'<line x1="{lx}" x2="{lx + 18}" y1="38" y2="38" stroke="{s["color"]}" stroke-width="3"{dash}/>')
        o.append(f'<text x="{lx + 23}" y="42" fill="{INK}">{s["label"]}</text>')
        lx += 30 + 7 * len(s["label"])
    o.append(f'<rect x="{ml}" y="{mt}" width="{pw}" height="{ph}" fill="none" stroke="{GRID}"/>')
    o.append("</svg>")
    return "\n".join(o)


def svg_map(paths, title, box=None, w=480, h=480):
    allp = np.concatenate([p["P"] for p in paths])
    if box:
        x0, x1, y0, y1 = box
    else:
        x0, y0 = allp.min(0) - 0.1
        x1, y1 = allp.max(0) + 0.1
    sc = min((w - 20) / (x1 - x0), (h - 50) / (y1 - y0))
    fx = lambda v: 10 + (v - x0) * sc
    fy = lambda v: 40 + (y1 - v) * sc
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}" '
         f'font-family="system-ui,Segoe UI,sans-serif" font-size="12">',
         f'<rect width="{w}" height="{h}" fill="{SURFACE}"/>',
         f'<text x="10" y="18" font-size="14" font-weight="600" fill="{INK}">{title}</text>',
         f'<clipPath id="mc"><rect x="10" y="40" width="{w - 20}" height="{h - 50}"/></clipPath>']
    lx = 10
    for p in paths:
        m = (p["P"][:, 0] > x0 - 0.05) & (p["P"][:, 0] < x1 + 0.05) & (p["P"][:, 1] > y0 - 0.05) & (p["P"][:, 1] < y1 + 0.05)
        xx, yy = decimate(p["P"][m, 0], p["P"][m, 1], 4000)
        pts = " ".join(f"{fx(a):.1f},{fy(b):.1f}" for a, b in zip(xx, yy))
        o.append(f'<polyline clip-path="url(#mc)" fill="none" stroke="{p["color"]}" stroke-width="{p.get("width", 1.4)}" '
                 f'stroke-linejoin="round" points="{pts}"/>')
        o.append(f'<line x1="{lx}" x2="{lx + 18}" y1="30" y2="30" stroke="{p["color"]}" stroke-width="3"/>')
        o.append(f'<text x="{lx + 23}" y="34" fill="{INK}">{p["label"]}</text>')
        lx += 30 + 7 * len(p["label"])
    if box:
        o.append(f'<text x="{w - 10}" y="{h - 8}" text-anchor="end" fill="{INK2}">'
                 f'{(x1 - x0):.2f} m x {(y1 - y0):.2f} m window</text>')
    o.append("</svg>")
    return "\n".join(o)


# ----------------------------------------------------------------------------- main analysis

def analyse(args):
    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    run_dir = args.run_dir
    if not run_dir:
        if not args.track:
            sys.exit("give --run-dir (existing LineSim output) or --track (to run the sim)")
        run_dir = os.path.join(out, "sim")
        if not os.path.exists(os.path.join(run_dir, "drive1.csv")) or args.rerun:
            run_sim(os.path.abspath(args.track), os.path.abspath(args.robot), run_dir, args.drives, args.seed)
    run_dir = os.path.abspath(run_dir)

    robot = load_json_c(args.robot)
    L = float(robot["sensor_x"])
    mu = args.mu if args.mu is not None else float(robot.get("tire_mu", robot.get("tire", {}).get("mu", 1.0)))
    summ = json.load(open(os.path.join(run_dir, "summary.json")))
    track_len = float(summ["track_length_m"])
    fw_lap = {int(dd["drive"]): dd for dd in summ["drives"]}

    ds = args.ds
    d1 = load_csv(os.path.join(run_dir, "drive1.csv"))
    types1 = load_markers(os.path.join(run_dir, "drive1_markers.csv"))
    ref = line_reference(d1, L, track_len, ds)
    s, X = ref["s"], ref["X"]
    tng, nrm = tangents_normals(X)
    # validation: body path implied by the line reference vs the logged drive-1 body path
    Q1, _ = trailer(X, L)
    ev1, et1 = marker_events(d1, types1)
    s_ev = ref["s_raw"][ev1]
    i_goal = int(np.searchsorted(s, s_ev[-1]))
    i_goal = min(i_goal, len(s) - 1)
    kappa_line = path_curvature(X, ds, sigma=3.0)
    # logged body position at the same line distance vs the no-slip trailer model
    sc1, _ = continuous_s(d1, track_len)
    sc1 = np.maximum.accumulate(sc1)
    k1 = np.r_[True, np.diff(sc1) > 1e-6]
    bxy = np.c_[np.interp(s, sc1[k1], d1["x"][k1]), np.interp(s, sc1[k1], d1["y"][k1])]
    err = np.hypot(*(bxy - Q1).T) * 1e3
    straight, arc = np.abs(kappa_line) < 0.5, np.abs(kappa_line) > 3.5
    trailer_err_mm = {"straight_median": float(np.median(err[straight])), "arc_median": float(np.median(err[arc])),
                      "arc_p95": float(np.percentile(err[arc], 95)),
                      "beta_arc_median_deg": float(np.median(np.abs(np.interp(s, sc1[k1], d1["beta_deg"][k1]))[arc]))}

    # marker positions on the s grid (drive 1) with their side
    marker_rows = [(float(sv), t) for sv, t in zip(s_ev, et1)]
    # curvature map the firmware could build itself: fw yaw at each marker event, snapped to 45 deg
    yaw_fw = np.unwrap(np.radians(d1["fw_yaw"]))
    psi_ev = np.interp(s_ev, sc1[k1], yaw_fw[k1])
    # marker events fire ~40 mm after the curvature step they mark (both test tracks); shift them back
    kappa_map = marker_map_curvature(s, s_ev - args.map_shift, psi_ev, args.map_quant_deg)

    # ---------------------------------------------------------------- variants
    variants = []
    alphas = [float(a) for a in str(args.alpha).split(",")]
    dmaxes = []
    for tok in str(args.dmax).split(","):
        tok = tok.strip()
        dmaxes.append(None if tok in ("inf", "none") else float(tok) * 1e-3)
    mclamps = []
    for tok in str(args.marker_clamp).split(","):
        tok = tok.strip()
        mclamps.append(None if tok in ("", "none", "inf") else float(tok) * 1e-3)
    for a in alphas:
        for dm in dmaxes:
            for mc in mclamps:
                if mc is not None and dm is not None and mc >= dm:
                    continue
                variants.append((a, dm, mc))
    # points within --marker-win of a side marker (L/R) as read in drive 1
    near_marker = np.zeros(len(s), bool)
    for sv, t in zip(s_ev, et1):
        if t in ("L", "R"):
            near_marker |= np.abs(s - sv) <= args.marker_win

    def evaluate(label, P, offset_mm, extra):
        Qb, kb, dq = body_kinematics(P, L)
        v, vlim = speed_profile(kb, dq, mu, args.accel, args.decel, args.vmax)
        lap = seg_time(v, dq, 0, i_goal)
        res = {"label": label, "lap_model_s": lap, "body_len_m": float(dq[:i_goal].sum()),
               "max_abs_kappa_body": float(np.abs(kb[:i_goal]).max()),
               "v": v, "vlim": vlim, "kb": kb, "Q": Qb, "P": P, "offset_mm": offset_mm}
        res.update(extra)
        return res

    base = evaluate("line (d_max 0)", X, np.zeros(len(s)), {"alpha": None, "dmax_mm": 0.0, "iters": 0})
    results = []
    for a, dm, mc in variants:
        bound = np.full(len(s), np.inf if dm is None else dm)
        if mc is not None:
            bound[near_marker] = np.minimum(bound[near_marker], mc)
        use_bound = None if (dm is None and mc is None) else bound
        Y, iters, ch = smooth(X, nrm, a, args.beta, args.tol, use_bound, args.max_iter)
        off, max_dt = lateral_offset(s, X, Y, tng, nrm)
        tag = "inf" if dm is None else f"{dm * 1e3:g}"
        mtag = "" if mc is None else f", markers {mc * 1e3:g}"
        label = "GD " + (f"a={a:g} " if len(alphas) > 1 else "") + f"d_max {tag}{mtag}"
        at_bound = np.abs(off) >= 0.98 * np.minimum(bound, 1e9) - 1e-4
        r = evaluate(label, Y, off * 1e3, {"method": "GD", "alpha": a, "dmax_mm": None if dm is None else dm * 1e3,
                                         "marker_clamp_mm": None if mc is None else mc * 1e3,
                                         "iters": iters, "last_change": ch, "max_tangential_mm": max_dt * 1e3,
                                         "smooth_len_mm": ds * math.sqrt(args.beta / a) * 1e3,
                                         "at_bound_pct": float(np.mean(at_bound) * 100) if use_bound is not None else 0.0,
                                         "export": a == alphas[0]})
        r["tag"] = f"a{a:g}_d{tag}" + ("" if mc is None else f"_m{mc * 1e3:g}")
        print(f"  {label:36s} iters {iters:6d}  lap {r['lap_model_s']:.3f} s  max|off| {np.abs(off).max() * 1e3:5.1f} mm")
        results.append(r)
        if a == alphas[0] and dm is None and mc is None:
            gd_free = off

    # closed-form curvature filter (what an on-board version could run from its marker map)
    a0 = alphas[0]
    filter_check = {}
    for src, kap in (("line", kappa_line), ("map", kappa_map)):
        d_f = filter_offset(kap, ds, a0, args.beta)
        if src == "line" and "gd_free" in locals():
            filter_check["max_abs_diff_vs_unclamped_GD_mm"] = float(np.abs(d_f - gd_free).max() * 1e3)
            filter_check["rms_diff_vs_unclamped_GD_mm"] = float(np.sqrt(np.mean((d_f - gd_free) ** 2)) * 1e3)
        for a_, dm, mc in variants:
            if a_ != a0:
                continue
            bound = np.full(len(s), np.inf if dm is None else dm)
            if mc is not None:
                bound[near_marker] = np.minimum(bound[near_marker], mc)
            off = np.clip(d_f, -bound, bound)
            tag = "inf" if dm is None else f"{dm * 1e3:g}"
            mtag = "" if mc is None else f", markers {mc * 1e3:g}"
            label = f"filter({src} k) d_max {tag}{mtag}"
            r = evaluate(label, X + off[:, None] * nrm, off * 1e3,
                         {"method": f"filter_{src}", "alpha": a0, "dmax_mm": None if dm is None else dm * 1e3,
                          "marker_clamp_mm": None if mc is None else mc * 1e3, "iters": 1,
                          "smooth_len_mm": ds * math.sqrt(args.beta / a0) * 1e3,
                          "at_bound_pct": float(np.mean(np.abs(off) >= 0.98 * np.minimum(bound, 1e9) - 1e-4) * 100),
                          "export": src == "map"})
            r["tag"] = f"filter{src}_a{a0:g}_d{tag}" + ("" if mc is None else f"_m{mc * 1e3:g}")
            print(f"  {label:36s}               lap {r['lap_model_s']:.3f} s  max|off| {np.abs(off).max() * 1e3:5.1f} mm")
            results.append(r)

    # ---------------------------------------------------------------- measured drives
    # Each logged drive's own lateral offset lat_mm(s) is run through the same pipeline
    # (line + offset -> trailer body path -> speed model), so its model lap is directly
    # comparable with the smoothed variants. Its fw lap is what the simulator measured.
    measured = {}
    for k in (1, 2, 3, 4):
        p = os.path.join(run_dir, f"drive{k}.csv")
        if not os.path.exists(p):
            continue
        dk = d1 if k == 1 else load_csv(p)
        mv = dk["v"] > 0.01
        first = int(np.where(mv)[0][0])
        typesk = load_markers(os.path.join(run_dir, f"drive{k}_markers.csv"))
        evk, _ = marker_events(dk, typesk)
        t_goal = float(dk["t"][evk[-1]])
        prof = drive_profile(dk, track_len)
        off_k = np.interp(s, prof["s"], prof["lat"])
        rk = evaluate(f"drive {k} measured", X + (off_k * 1e-3)[:, None] * nrm, off_k, {})
        measured[k] = {"lap_fw_s": float(fw_lap.get(k, {}).get("lap_time_fw", float("nan"))),
                       "t_move_to_goal_s": t_goal - float(dk["t"][first]),
                       "lap_model_s": rk["lap_model_s"],
                       "body_len_m": rk["body_len_m"],
                       "max_v_sim": float(dk["v"].max()),
                       "marker_check": fw_lap.get(k, {}).get("marker_check", ""),
                       "s_line": prof["s"], "v_sim": prof["v"], "lat": prof["lat"]}
        if k in (3, 4):
            # lateral offset seen at each marker event (lat_mm), for the readable-window envelope
            lat_ev = dk["lat_mm"][evk]
            ty = typesk[:len(evk)]
            for side in ("L", "R"):
                vals = [float(lv) for lv, t in zip(lat_ev, ty) if t == side]
                if vals:
                    measured[k][f"marker_lat_{side}_mm"] = [min(vals), max(vals)]

    # marker envelope proven by drive 3 (all its markers were read correctly)
    env = {}
    if 3 in measured and measured[3]["marker_check"] == "OK":
        for side in ("L", "R"):
            if f"marker_lat_{side}_mm" in measured[3]:
                env[side] = measured[3][f"marker_lat_{side}_mm"]

    def marker_stats(off_mm):
        o = np.interp([m[0] for m in marker_rows], s, off_mm)
        worst = float(np.abs(o).max()) if len(o) else 0.0
        outside = 0
        for (sv, t), ov in zip(marker_rows, o):
            if t in env:
                lo, hi = env[t]
                if ov < lo - 1.0 or ov > hi + 1.0:
                    outside += 1
        return worst, outside, int(np.sum(np.abs(o) > 28.0))

    # ---------------------------------------------------------------- tables / exports
    d3 = measured.get(3)
    rows = []
    for r in [base] + results:
        worst, outside, over28 = marker_stats(r["offset_mm"])
        tp = np.clip(r["offset_mm"] / args.mm_per_unit, -args.tp_limit, args.tp_limit)
        row = {k: r.get(k) for k in ("label", "alpha", "dmax_mm", "marker_clamp_mm", "iters", "smooth_len_mm",
                                     "max_tangential_mm", "at_bound_pct", "max_abs_kappa_body")}
        row.update({"lap_model_s": r["lap_model_s"], "body_len_m": r["body_len_m"],
                    "gain_vs_line_s": base["lap_model_s"] - r["lap_model_s"],
                    "max_abs_offset_mm": float(np.abs(r["offset_mm"]).max()),
                    "max_abs_target_pos": float(np.abs(tp).max()),
                    "tp_saturated_pct": float(np.mean(np.abs(r["offset_mm"] / args.mm_per_unit) > args.tp_limit) * 100),
                    "marker_max_abs_offset_mm": worst, "markers_outside_d3_envelope": outside,
                    "markers_over_28mm": over28})
        row["method"] = r.get("method", "line")
        if d3:
            row["sim_anchored_lap_s"] = d3["lap_fw_s"] + (r["lap_model_s"] - d3["lap_model_s"])
            row["sim_scaled_lap_s"] = d3["lap_fw_s"] * r["lap_model_s"] / d3["lap_model_s"]
        rows.append(row)

        if r is not base and r.get("export"):
            seg_idx = np.searchsorted([m[0] for m in marker_rows], s, side="right") - 1
            s_mark = np.array([marker_rows[j][0] if j >= 0 else s[0] for j in seg_idx])
            path = os.path.join(out, f"target_pos_{r['tag']}.csv")
            with open(path, "w") as f:
                f.write("s_line_m,s_from_start_m,marker_idx,s_since_marker_m,offset_mm,target_pos,"
                        "kappa_line_1pm,kappa_body_1pm,v_limit_mps,v_plan_mps\n")
                for i in range(len(s)):
                    f.write(f"{s[i]:.3f},{s[i] - s[0]:.3f},{seg_idx[i]},{s[i] - s_mark[i]:.3f},{r['offset_mm'][i]:.2f},"
                            f"{tp[i]:.4f},{kappa_line[i]:.3f},{r['kb'][i]:.3f},{r['vlim'][i]:.3f},{r['v'][i]:.3f}\n")

    # ---------------------------------------------------------------- plots
    sx = s - s[0]
    prim = [r for r in results if r["method"] == "GD" and r["alpha"] == alphas[0] and r["marker_clamp_mm"] is None]
    if not prim:
        prim = [r for r in results if r["method"] == "GD" and r["alpha"] == alphas[0]]
    off_series = [{"x": sx, "y": r["offset_mm"], "label": r["label"], "color": SERIES[i % 8]}
                  for i, r in enumerate(prim)]
    if d3:
        off_series.append({"x": d3["s_line"] - s[0], "y": d3["lat"], "label": "drive 3 sim lat_mm",
                           "color": NEUTRAL, "width": 1.0})
    dm_lines = []
    for r in prim:
        if r["dmax_mm"]:
            dm_lines += [(r["dmax_mm"], f"+{r['dmax_mm']:g}"), (-r["dmax_mm"], f"-{r['dmax_mm']:g}")]
    zoom = (0, min(sx[-1], args.zoom_len))
    open(os.path.join(out, "offset.svg"), "w").write(
        svg_chart(off_series, f"Lateral offset of the smoothed path from the line (left +), first {zoom[1]:g} m",
                  "distance from start [m]", "offset [mm]", xlim=zoom, bands=dm_lines))
    open(os.path.join(out, "offset_full.svg"), "w").write(
        svg_chart(off_series, "Lateral offset, whole lap", "distance from start [m]", "offset [mm]", bands=dm_lines))
    sp_series = [{"x": sx, "y": base["v"], "label": "model, line", "color": NEUTRAL, "width": 1.2}]
    big = max(prim, key=lambda r: (r["dmax_mm"] is not None, r["dmax_mm"] or 0))
    sp_series.append({"x": sx, "y": big["v"], "label": f"model, {big['label']}", "color": SERIES[0]})
    for k, col in ((3, SERIES[1]), (4, SERIES[2])):
        if k in measured:
            sp_series.append({"x": measured[k]["s_line"] - s[0], "y": measured[k]["v_sim"],
                              "label": f"sim drive {k}", "color": col, "width": 1.2})
    open(os.path.join(out, "speed.svg"), "w").write(
        svg_chart(sp_series, f"Speed profile, first {zoom[1]:g} m", "distance from start [m]", "speed [m/s]", xlim=zoom))
    open(os.path.join(out, "speed_full.svg"), "w").write(
        svg_chart(sp_series, "Speed profile, whole lap", "distance from start [m]", "speed [m/s]"))
    ic = int(np.argmax(np.abs(big["offset_mm"])))
    cx, cy = X[ic]
    hw = args.zoom_box / 2
    maps = [{"P": X, "label": "line", "color": NEUTRAL, "width": 2.4},
            {"P": big["P"], "label": f"sensor path, {big['label']}", "color": SERIES[0]},
            {"P": big["Q"], "label": "body path", "color": SERIES[1], "width": 1.0}]
    open(os.path.join(out, "map.svg"), "w").write(
        svg_map(maps, f"Around the largest offset (s = {sx[ic]:.2f} m)", box=(cx - hw, cx + hw, cy - hw, cy + hw)))
    open(os.path.join(out, "map_full.svg"), "w").write(svg_map(maps[:2], "Whole track"))

    # ---------------------------------------------------------------- report
    summary = {"run_dir": run_dir, "track": summ.get("track"), "track_length_m": track_len,
               "params": {"ds_m": ds, "alpha": alphas, "beta": args.beta, "tol": args.tol, "mu": mu,
                          "accel": args.accel, "decel": args.decel, "vmax": args.vmax, "sensor_x": L,
                          "mm_per_unit": args.mm_per_unit, "tp_limit": args.tp_limit,
                          "map_quant_deg": args.map_quant_deg, "map_shift_m": args.map_shift,
                          "marker_win_m": args.marker_win},
               "goal_s_line_m": float(s[i_goal]), "lap_distance_line_m": float(s[i_goal] - s[0]),
               "trailer_check_mm": trailer_err_mm,
               "filter_check": filter_check,
               "marker_envelope_drive3_mm": env,
               "variants": rows,
               "measured": {k: {kk: vv for kk, vv in m.items() if not isinstance(vv, np.ndarray)}
                            for k, m in measured.items()}}
    json.dump(summary, open(os.path.join(out, "summary.json"), "w"), indent=1)
    write_report(out, summary, measured)
    print("wrote", out)
    return summary


def drive_profile(d, track_len):
    """lat_mm, v and t of one logged drive against the (unwrapped, monotone) line distance s."""
    s, lat = continuous_s(d, track_len)
    s = np.maximum.accumulate(s)
    keep = np.r_[True, np.diff(s) > 1e-6]
    return {"s": s[keep], "lat": lat[keep], "v": d["v"][keep], "t": d["t"][keep]}


def write_report(out, sm, measured):
    p = sm["params"]
    L = []
    L.append(f"# Path-smoothing planner: {sm['track']}\n")
    L.append(f"Run folder: `{sm['run_dir']}`  \nTrack {sm['track_length_m']:.2f} m, timed distance on the line "
             f"{sm['lap_distance_line_m']:.2f} m. Sampling {p['ds_m'] * 1e3:g} mm, beta {p['beta']}, tol {p['tol']:g}, "
             f"mu {p['mu']}, accel/decel {p['accel']}/{p['decel']} m/s^2, vmax {p['vmax']} m/s, "
             f"{p['mm_per_unit']} mm per target unit, limit {p['tp_limit']}.\n")
    tc = sm["trailer_check_mm"]
    L.append(f"Check: logged drive-1 body position vs the no-slip body model at the same s: median "
             f"{tc['straight_median']:.1f} mm on straights, {tc['arc_median']:.1f} mm in arcs (p95 {tc['arc_p95']:.1f} mm); "
             f"the sim body slip angle in arcs is {tc['beta_arc_median_deg']:.0f} deg (median), so absolute model laps are "
             f"optimistic and only differences between variants are used.\n")
    L.append("## Simulator laps, and the speed model on each drive's own measured lat_mm(s)\n")
    L.append("| drive | fw lap [s] | move->goal [s] | model lap on its line [s] | body length [m] | max v [m/s] | markers |")
    L.append("|---|---|---|---|---|---|---|")
    for k, m in sorted(measured.items()):
        L.append(f"| {k} | {m['lap_fw_s']:.2f} | {m['t_move_to_goal_s']:.2f} | {m['lap_model_s']:.2f} | "
                 f"{m['body_len_m']:.2f} | {m['max_v_sim']:.2f} | {m['marker_check']} |")
    L.append("\n## Smoothed variants\n")
    L.append("| variant | iters | smoothing length [mm] | model lap [s] | gain vs line [s] | sim-anchored lap [s] | "
             "sim-scaled lap [s] | max offset [mm] | at clamp [%] | max body curvature [1/m] | max target_pos | "
             "max offset at a marker [mm] | markers outside drive-3 envelope |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in sm["variants"]:
        sl = f"{r['smooth_len_mm']:.0f}" if r.get("smooth_len_mm") else "-"
        sa = f"{r['sim_anchored_lap_s']:.2f}" if "sim_anchored_lap_s" in r else "-"
        sc = f"{r['sim_scaled_lap_s']:.2f}" if "sim_scaled_lap_s" in r else "-"
        L.append(f"| {r['label']} | {r['iters']} | {sl} | {r['lap_model_s']:.2f} | {r['gain_vs_line_s']:.2f} | {sa} | {sc} | "
                 f"{r['max_abs_offset_mm']:.1f} | {r.get('at_bound_pct') or 0:.0f} | {r['max_abs_kappa_body']:.2f} | "
                 f"{r['max_abs_target_pos']:.2f} | {r['marker_max_abs_offset_mm']:.1f} | "
                 f"{r['markers_outside_d3_envelope']} |")
    env = sm["marker_envelope_drive3_mm"]
    if env:
        L.append("\nDrive-3 marker envelope (offsets at which drive 3 read every marker): "
                 + ", ".join(f"{k} [{v[0]:.0f}, {v[1]:.0f}] mm" for k, v in env.items()) + ".")
    L.append("\nSim-anchored lap = drive-3 fw lap + (model lap of the variant - model lap on drive 3's measured lat_mm). "
             "Sim-scaled lap = drive-3 fw lap * (model lap of the variant / model lap on drive 3's lat_mm).")
    fc = sm.get("filter_check") or {}
    if fc:
        L.append(f"\nfilter(line k) vs unclamped GD offset: max difference {fc['max_abs_diff_vs_unclamped_GD_mm']:.2f} mm, "
                 f"rms {fc['rms_diff_vs_unclamped_GD_mm']:.2f} mm. filter(map k) uses only the firmware yaw at each marker "
                 f"event (snapped to {sm['params']['map_quant_deg']:g} deg) and the marker distances, with each event "
                 f"moved back {sm['params']['map_shift_m'] * 1e3:g} mm.\n")
    L.append("![offset](offset.svg)\n\n![offset, whole lap](offset_full.svg)\n\n![speed](speed.svg)\n\n"
             "![speed, whole lap](speed_full.svg)\n\n![map](map.svg) ![map](map_full.svg)\n")
    open(os.path.join(out, "report.md"), "w").write("\n".join(L))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", help="existing LineSim output folder with drive1.csv (and drive3/4.csv)")
    ap.add_argument("--track", help="track json; runs the sim into <out>/sim when --run-dir is not given")
    ap.add_argument("--robot", default=os.path.join(REPO, "robots", "full.json"))
    ap.add_argument("--drives", default="1,2,3,4", help="drives for the sim run (default 1,2,3,4)")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--rerun", action="store_true", help="run the sim even if <out>/sim exists")
    ap.add_argument("--out", required=True)
    ap.add_argument("--ds", type=float, default=0.01, help="resampling step [m] (default 0.01)")
    ap.add_argument("--alpha", default="0.005", help="data weight(s), comma list (default 0.005)")
    ap.add_argument("--beta", type=float, default=0.4, help="smooth weight (default 0.4)")
    ap.add_argument("--tol", type=float, default=1e-5, help="stop when sum |change| < tol [m] (default 1e-5)")
    ap.add_argument("--max-iter", type=int, default=400000)
    ap.add_argument("--dmax", default="20,40,60,inf", help="lateral clamps [mm], comma list; inf = no clamp")
    ap.add_argument("--marker-clamp", default="none,20",
                    help="extra |offset| clamp [mm] near side markers, comma list; none = off (default none,20)")
    ap.add_argument("--marker-win", type=float, default=0.05,
                    help="half length [m] of the marker clamp window around each L/R marker (default 0.05)")
    ap.add_argument("--map-quant-deg", type=float, default=45.0,
                    help="snap the per-marker yaw change of the marker map to this step (ZETIN: 45); 0 = off")
    ap.add_argument("--map-shift", type=float, default=0.04,
                    help="marker map: event-to-curvature-step lag [m] subtracted from each event (default 0.04, "
                         "measured 37-44 mm median in the sim)")
    ap.add_argument("--mu", type=float, help="tire mu (default: robot json tire_mu)")
    ap.add_argument("--accel", type=float, default=6.0)
    ap.add_argument("--decel", type=float, default=6.0)
    ap.add_argument("--vmax", type=float, default=7.0)
    ap.add_argument("--mm-per-unit", type=float, default=52.0, help="robot shift per target_pos unit [mm]")
    ap.add_argument("--tp-limit", type=float, default=1.45, help="TARGET_POS_LIMIT")
    ap.add_argument("--zoom-len", type=float, default=12.0, help="length of the zoomed profile plots [m]")
    ap.add_argument("--zoom-box", type=float, default=1.6, help="side of the zoomed map window [m]")
    analyse(ap.parse_args())


if __name__ == "__main__":
    main()
