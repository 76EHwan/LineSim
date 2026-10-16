"""Base speed and steering gains for the race, found in the simulator.

    python tools/autotune.py 2.0                 best steer kp / kd at base speed 2.0 m/s
    python tools/autotune.py 1.6 --max-speed     highest base speed that passes, from 1.6 up
    python tools/autotune.py 2.2 --max-speed --lat 10,12,14,16   with the grip limit (lat_acc_max) as a 3rd axis

    options: [--track tracks/x.json ...] [--courses 4] [--robot robots/full.json] [--jobs 8]
             [--kp 8,10,...] [--kd 0,0.05,...] [--lat 0] [--set max_mps=4 ...] [--reuse]

Course: the real practice board (tracks/practice_board.json, made from the robot's logs) when it
is there, else --courses random ZETIN courses with fixed seeds. --track picks courses by hand.

Every kp x kd pair of the grid runs drive 1 (learning: markers and distances must come out clean)
and then drive 2 (accelerates on the straights from drive 1's data; max_mps / accel / decel from
--set or the firmware defaults). A pair passes when on every course both drives finish, never more
than MAX_LAT_MM off the line, with every marker read right. Score (lower is better): 5 x drive 2
lap time [s] + drive 1 cleanliness (line rms [mm] + weave [dps] / 20 + wheel slip [%] / 2).
A pass must also hold with more steering lag than the model has: the lag (robot sensor_delay,
fitted to one gyro log) is not known exactly, so drive 1 runs again with it x1.4 (must keep the
line, weave at most 1.5 x nominal + 10 dps) and x1.85 (must keep the line). Gains that only work
for the exact fitted lag count as unstable. Grip is not known exactly either: drive 1 also runs
with the tire friction x0.85 and must still finish with every marker right.
The pick is among the passing points whose grid neighbours are stable too: within 1 % of the
best score, the lowest kp, then the lowest kd, then the lowest lat_acc_max.

--lat adds the grip limit (firmware feat/cascade-steer: driveData.lat_acc_max [m/s^2], slows down
while speed x gyro yaw rate exceeds it) as a third grid axis. 0 = off (firmware without it: 0 only).

Coarse to fine (--refine 3, the default): a coarse grid (kp 12/18/25/32 x kd 0-0.4 s x the --lat
values), then 3 times a 3x3x3 grid around the best point with half the step each time (kp +-3.5,
1.75, 0.9; kd +-0.05, 0.025, 0.0125 s; lat +-1.5 ...). The same stability rules hold on every level.
The pick is then driven with 2 more seeds and must pass every time. --refine 0: the fixed grid.

--max-speed raises the base speed while some pair passes (+0.2, then halving the step on a miss,
down to 0.01 m/s) and reports the highest speed with its gains.

kd is in seconds (derivative gain on de/dt). The firmware's steer_pid is CMSIS arm_pid_f32, called
once per steering period (TIM14, 500 us from Core/Src/tim.c), so its Kd multiplies the error change
per call: the value for the robot's menu (driveData.steer_gain_d) is kd / 0.0005, e.g. 0.3 s -> 600.

Writes runs/tune/<speed>/result.txt and result.json (--max-speed: runs/tune/max_speed.txt too).
--reuse keeps runs that already have a result (to re-score after changing this script).
"""
import concurrent.futures
import csv
import json
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.join(ROOT, "build", "Release", "linesim.exe")


def use_private_exe():
    """Run a copy of the simulator, so a rebuild during a long search does not hit a locked file
    (and the search keeps one build from start to end)."""
    global EXE
    import shutil
    src = os.path.dirname(EXE)
    dst = os.path.join(ROOT, "runs", "tune", f"bin_{os.getpid()}")
    os.makedirs(dst, exist_ok=True)
    for f in ("linesim.exe", "mujoco.dll"):
        shutil.copy2(os.path.join(src, f), os.path.join(dst, f))
    EXE = os.path.join(dst, "linesim.exe")
REAL_COURSE = os.path.join(ROOT, "tracks", "practice_board.json")

KP = [8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40]
KD = [0, 0.05, 0.1, 0.15, 0.225, 0.3, 0.4, 0.5]   # seconds (x 2000 = firmware steer_gain_d)
KP_COARSE = [12, 18, 25, 32]          # first grid of the coarse-to-fine search (--refine)
KD_COARSE = [0, 0.1, 0.2, 0.3, 0.4]
SEED_CHECK = (2, 3)                   # the pick is driven again with these seeds (seed 1 = the grid)
_RAN = set()                          # output folders run in this process (refinement grids overlap)
SEEDS = [101, 102, 103, 104, 105, 106, 107, 108]
STRESS_LAG = (1.4, 1.85)   # sensor_delay factors of the stress drives
STRESS_MU = 0.85           # tire friction factor of the grip stress drive
MAX_LAT_MM = 70.0   # sensor bar this far off counts as losing the line (bar half width 76 mm; 30-40 mm is normal in a curve)


def drive_metrics(csv_path):
    """(weave [dps], wheel slip [%]) of one drive. weave: rms of the yaw rate minus its 60 ms average."""
    wz, slip = [], []
    try:
        with open(csv_path) as fh:
            for r in csv.DictReader(fh):
                v = float(r["v"])
                if v > 0.3:
                    wz.append(float(r["wz_dps"]))
                w = 0.5 * (float(r["wheel_v_l"]) + float(r["wheel_v_r"]))
                if w > 0.3:
                    slip.append(abs(1.0 - v / w))
    except OSError:
        return 0.0, 0.0
    k = 60
    weave = 0.0
    if len(wz) >= 2 * k:
        acc = sum(wz[:k])
        sq = 0.0
        for i in range(k, len(wz)):
            hp = wz[i - k // 2] - acc / k
            sq += hp * hp
            acc += wz[i] - wz[i - k]
        weave = math.sqrt(sq / (len(wz) - k))
    return weave, 100.0 * sum(slip) / len(slip) if slip else 0.0


def robot_delay(robot):
    """sensor_delay of a robot file (the JSON may carry // comments)."""
    import re
    with open(robot, encoding="utf-8") as fh:
        m = re.search(r'"sensor_delay"\s*:\s*([0-9.eE+-]+)', fh.read())
    return float(m.group(1)) if m else 0.0


def robot_mu(robot):
    """tire.mu of a robot file."""
    import re
    with open(robot, encoding="utf-8") as fh:
        m = re.search(r'"tire"\s*:\s*\{[^}]*"mu"\s*:\s*([0-9.eE+-]+)', fh.read())
    return float(m.group(1)) if m else 0.0


def sim(course, robot, drives, speed, kp, kd, extra, out, reuse, robot_params=(), seed=1):
    """Runs linesim, returns summary.json (or None)."""
    cmd = [EXE, "run", "--track", course, "--robot", robot, "--drives", drives, "--quiet", "--out", out,
           "--set", f"base_mps={speed}", "--set", f"steer_kp={kp}", "--set", f"steer_kd_s={kd}", "--max-time", "60",
           "--seed", str(seed)]
    for e in extra:
        cmd += ["--set", e]
    for rp in robot_params:
        cmd += ["--robot-param", rp]
    if not ((reuse or out in _RAN) and os.path.exists(os.path.join(out, "summary.json"))):
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        _RAN.add(out)
    try:
        with open(os.path.join(out, "summary.json"), encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def lost_line(d):
    return not d["outcome"].startswith("finished") or d["max_lat_mm"] > MAX_LAT_MM


def run(course, robot, speed, kp, kd, lat, extra, out, reuse, lag, mu, seed=1):
    extra = list(extra) + ([f"lat_acc_max={lat}"] if lat > 0 else [])
    j = sim(course, robot, "1,2", speed, kp, kd, extra, out, reuse, seed=seed)
    try:
        d1 = j["drives"][0]
        d2 = j["drives"][1] if len(j["drives"]) > 1 else None
        kd_fw = j["params"]["steer_kd"]
    except (TypeError, IndexError, KeyError):
        return {"lost": True, "pass": False, "score": 1000.0, "why": "no result", "kd_fw": 0}
    weave, slip = drive_metrics(os.path.join(out, "drive1.csv"))
    clean1 = d1["rms_lat_mm"] + weave / 20.0 + slip / 2.0
    r = {"kd_fw": kd_fw, "rms_lat": d1["rms_lat_mm"], "max_lat": d1["max_lat_mm"], "weave": weave, "slip": slip,
         "clean1": clean1, "lap2": None, "lost": False, "pass": False}
    if lost_line(d1):
        r.update(lost=True, score=1000.0, why=f"1: {d1['outcome'] if d1['outcome'] != 'finished' else 'line far off'}")
    elif d1.get("marker_check") != "OK":
        r.update(score=500.0 + clean1, why=f"1: markers {d1.get('marker_check')}")
    elif d2 is None:
        r.update(score=400.0 + clean1, why="2: did not run")
    elif lost_line(d2):
        r.update(lost=True, score=300.0 + clean1, why=f"2: {d2['outcome'] if d2['outcome'] != 'finished' else 'line far off'}")
    elif d2.get("marker_check") != "OK" or d2["lap_time_fw"] <= 0:
        r.update(score=200.0 + clean1, why=f"2: markers {d2.get('marker_check')}")
    else:
        r.update({"pass": True, "lap2": d2["lap_time_fw"], "score": 5.0 * d2["lap_time_fw"] + clean1, "why": ""})
        why = ""
        if lag > 0:   # model-error stress: more steering lag than fitted
            for k, f in enumerate(STRESS_LAG):
                sj = sim(course, robot, "1", speed, kp, kd, extra, f"{out}_lag{f:g}", reuse,
                         [f"sensor_delay={lag * f:.6f}"], seed)
                if sj is None or lost_line(sj["drives"][0]):
                    why = f"lag x{f:g}: " + (sj["drives"][0]["outcome"] if sj else "no result")
                    break
                if k == 0:
                    sw, _ = drive_metrics(os.path.join(f"{out}_lag{f:g}", "drive1.csv"))
                    if sw > 1.5 * weave + 10.0:
                        why = f"lag x{f:g}: weave {sw:.0f} dps"
                        break
        if not why and mu > 0:   # grip stress: less friction than the model
            sj = sim(course, robot, "1", speed, kp, kd, extra, f"{out}_mu{STRESS_MU:g}", reuse,
                     [f"tire_mu={mu * STRESS_MU:.4f}"], seed)
            if sj is None or lost_line(sj["drives"][0]):
                why = f"grip x{STRESS_MU:g}: " + (sj["drives"][0]["outcome"] if sj else "no result")
            elif sj["drives"][0].get("marker_check") != "OK":
                why = f"grip x{STRESS_MU:g}: markers {sj['drives'][0].get('marker_check')}"
        if why:
            r.update({"pass": False, "lost": True, "score": 250.0 + clean1, "why": why})
    return r


def tune_at(speed, courses, robot, axes, extra, jobs, reuse, quiet=False):
    """axes = (kps, kds, lats). Returns the grid evaluated at one base speed."""
    kps, kds, lats = axes
    lag, mu = robot_delay(robot), robot_mu(robot)
    out_root = os.path.join(ROOT, "runs", "tune", f"{speed:g}")
    points = [(kp, kd, lat) for lat in lats for kp in kps for kd in kds]
    total = len(points) * len(courses)
    print(f"base speed {speed:g} m/s: {len(kps)} kp x {len(kds)} kd x {len(lats)} lat on {len(courses)} course(s) = "
          f"{total} runs (drive 1 + 2 each), {jobs} at a time", flush=True)
    results = {}
    done = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {}
        for kp, kd, lat in points:
            for ci, c in enumerate(courses):
                name = f"kp{kp:g}_kd{kd:g}s" + (f"_lat{lat:g}" if lat > 0 else "") + f"_c{ci}"
                out = os.path.join(out_root, "runs", name)
                futs[ex.submit(run, c, robot, speed, kp, kd, lat, extra, out, reuse, lag, mu)] = (kp, kd, lat, ci)
        for f in concurrent.futures.as_completed(futs):
            kp, kd, lat, ci = futs[f]
            results.setdefault((kp, kd, lat), {})[ci] = f.result()
            done += 1
            if not quiet and (done % 50 == 0 or done == total):
                print(f"  {done}/{total}", flush=True)

    cell = {}
    for p, rs in results.items():
        rs = [rs[i] for i in range(len(courses))]
        cell[p] = {"pass": all(r["pass"] for r in rs), "lost": any(r["lost"] for r in rs),
                   "score": sum(r["score"] for r in rs) / len(rs), "runs": rs,
                   "why": [r["why"] for r in rs if not r["pass"]]}

    def robust(p):   # no grid neighbour (one step along kp, kd or lat) loses the line / is unstable
        idx = [axes[a].index(p[a]) for a in range(3)]
        for a in range(3):
            for d in (-1, 1):
                i = idx[a] + d
                if 0 <= i < len(axes[a]):
                    q = list(p)
                    q[a] = axes[a][i]
                    if cell[tuple(q)]["lost"]:
                        return False
        return True

    passing = [p for p in points if cell[p]["pass"]]
    safe = [p for p in passing if robust(p)]
    pick = None
    if passing:
        # Within 1 % of the best score (~0.1 s of drive 2 lap) the gentlest wins: lower kp, then lower
        # kd, then lower lat_acc_max. A high kd also amplifies the real sensor's position steps.
        pool = safe or passing
        best = min(cell[p]["score"] for p in pool)
        pick = min((p for p in pool if cell[p]["score"] <= best * 1.01), key=lambda p: (p[0], p[1], p[2]))
    return {"speed": speed, "cell": cell, "points": points, "passing": passing, "safe": safe, "pick": pick,
            "out_root": out_root, "axes": axes}


def describe(p):
    return f"steer kp {p[0]:g}, steer kd {p[1]:g} s" + (f", lat_acc_max {p[2]:g}" if p[2] > 0 else "")


def report(t, courses, robot, extra, with_pick=True):
    cell, pick = t["cell"], t["pick"]
    kps, kds, lats = t["axes"]
    lines = [f"base speed {t['speed']:g} m/s, drive 1 + 2 on {', '.join(os.path.basename(c) for c in courses)}, "
             f"robot {os.path.basename(robot)}" + (f", --set {' '.join(extra)}" if extra else ""),
             "",
             "score = 5 x drive 2 lap [s] + drive 1 (line rms [mm] + weave/20 + slip%/2), lower is better",
             f"x = loses the line, or fails a stress drive (steering lag x{STRESS_LAG[0]:g} / x{STRESS_LAG[1]:g}, "
             f"grip x{STRESS_MU:g}), m = misreads markers, * = picked"]
    for lat in lats:
        lines.append("")
        if len(lats) > 1 or lat > 0:
            lines.append(f"lat_acc_max {lat:g} m/s^2" + (" (grip limit off)" if lat == 0 else ""))
        lines.append("kp\\kd [s] " + "".join(f"{kd:>8g}" for kd in kds))
        for kp in kps:
            row = f"{kp:>8g}  "
            for kd in kds:
                c = cell[(kp, kd, lat)]
                txt = f"{c['score']:.1f}" if c["pass"] else ("x" if c["lost"] else "m")
                if (kp, kd, lat) == pick:
                    txt = "*" + txt
                row += f"{txt:>8}"
            lines.append(row)
    if not with_pick:
        return "\n".join(lines)
    lines.append("")
    if pick:
        rs = cell[pick]["runs"]
        n = len(rs)
        lines.append(f"picked: base {t['speed']:g} m/s, {describe(pick)} (robot menu: steer_gain_d {rs[0]['kd_fw']:.0f})"
                     + ("" if pick in t["safe"] else "   (a grid neighbour fails: at the edge)"))
        lines.append(f"  drive 2 lap {sum(r['lap2'] for r in rs) / n:.2f} s;  drive 1: line rms "
                     f"{sum(r['rms_lat'] for r in rs) / n:.1f} mm, weave {sum(r['weave'] for r in rs) / n:.0f} dps, "
                     f"slip {sum(r['slip'] for r in rs) / n:.1f} %")
        lines.append(f"  passing {len(t['passing'])} of {len(t['points'])}, away from the edge {len(t['safe'])}")
        lines.append("")
        lines.append("try it in the 3D session:")
        lines.append(f"  build\\Release\\linesim.exe view --session --track {os.path.relpath(courses[0], ROOT)} "
                     f"--robot robots/full.json --set base_mps={t['speed']:g} --set steer_kp={pick[0]:g} "
                     f"--set steer_kd_s={pick[1]:g}" + (f" --set lat_acc_max={pick[2]:g}" if pick[2] > 0 else "")
                     + "".join(f" --set {e}" for e in extra))
    else:
        why = {}
        for c in cell.values():
            for w in c["why"]:
                key = w.split(" (")[0] if "markers" in w else w
                why[key] = why.get(key, 0) + 1
        lines.append("nothing passes at this speed. Most common: "
                     + ", ".join(f"{k} x{n}" for k, n in sorted(why.items(), key=lambda kv: -kv[1])[:4]))
    return "\n".join(lines)


def save(t, text):
    os.makedirs(t["out_root"], exist_ok=True)
    with open(os.path.join(t["out_root"], "result.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    cell, pick = t["cell"], t["pick"]
    with open(os.path.join(t["out_root"], "result.json"), "w", encoding="utf-8") as fh:
        json.dump({"speed": t["speed"],
                   "picked": {"steer_kp": pick[0], "steer_kd_s": pick[1], "lat_acc_max": pick[2],
                              "steer_gain_d": cell[pick]["runs"][0]["kd_fw"], "safe": pick in t["safe"]} if pick else None,
                   "grid": [{"kp": p[0], "kd": p[1], "lat_acc_max": p[2], "pass": cell[p]["pass"], "lost": cell[p]["lost"],
                             "score": cell[p]["score"], "why": cell[p]["why"]} for p in t["points"]]},
                  fh, indent=1)


def refine_axes(center, steps):
    """3 x 3 x 3 grid around center (kp, kd, lat): center +- step on each axis (step 0: that axis fixed)."""
    out = []
    for c, s, lo, q in zip(center, steps, (1.0, 0.0, 0.0), (0.05, 0.0025, 0.05)):
        if s <= 0:
            out.append([c])
            continue
        vals = sorted({round(round(max(lo, c + k * s) / q) * q, 4) for k in (-1, 0, 1)})
        out.append(vals)
    return tuple(out)


def search_at(speed, courses, robot, axes, extra, jobs, reuse, refine, quiet=False):
    """Coarse grid, then `refine` times a 3x3x3 grid around the best point with half the step, then the
    pick is driven again with 2 more seeds (sensor noise / gain spread): it must pass every time."""
    coarse = tune_at(speed, courses, robot, axes, extra, jobs, reuse, quiet)
    best = coarse
    log = []

    def score_of(t):
        return t["cell"][t["pick"]]["score"]

    if coarse["pick"]:
        log.append(f"  coarse   {len(coarse['points']):3d} points -> {describe(coarse['pick'])}, score {score_of(coarse):.1f}"
                   + ("" if coarse["pick"] in coarse["safe"] else " (edge)"))
    steps = [(min(b - a for a, b in zip(v, v[1:])) / 2 if len(v) > 1 else 0.0) for v in axes]
    for it in range(refine):
        if not best["pick"]:
            break
        ax = refine_axes(best["pick"], steps)
        t = tune_at(speed, courses, robot, ax, extra, jobs, reuse, quiet=True)
        step_txt = f"+-{steps[0]:g} kp, +-{steps[1]:g} s kd" + (f", +-{steps[2]:g} lat" if steps[2] > 0 else "")
        if t["pick"] and (t["pick"] in t["safe"] or best["pick"] not in best["safe"]):
            best = t
            log.append(f"  refine {it + 1} ({step_txt}) -> {describe(t['pick'])}, score {score_of(t):.1f}"
                       + ("" if t["pick"] in t["safe"] else " (edge)"))
        else:
            log.append(f"  refine {it + 1} ({step_txt}) -> nothing better away from the edge, kept")
        steps = [s / 2 for s in steps]

    verified, seed_txt = False, ""
    if best["pick"]:
        kp, kd, lat = best["pick"]
        lag, mu = robot_delay(robot), robot_mu(robot)
        res = []
        for seed in SEED_CHECK:
            for ci, c in enumerate(courses):
                name = f"kp{kp:g}_kd{kd:g}s" + (f"_lat{lat:g}" if lat > 0 else "") + f"_c{ci}_seed{seed}"
                res.append(run(c, robot, speed, kp, kd, lat, extra, os.path.join(best["out_root"], "runs", name),
                               reuse, lag, mu, seed))
        verified = all(r["pass"] for r in res)
        seed_txt = (f"  seeds {', '.join(map(str, SEED_CHECK))}: "
                    + ("pass" if verified else "FAIL (" + "; ".join(r["why"] for r in res if not r["pass"]) + ")"))
        log.append(seed_txt)
    return {"speed": speed, "coarse": coarse, "final": best, "log": log, "verified": verified}


def report_search(s, courses, robot, extra):
    t = s["final"]
    lines = [report(s["coarse"], courses, robot, extra, with_pick=False), "", "refinement around the best point:"]
    lines += s["log"]
    lines.append("")
    if t["pick"]:
        rs = t["cell"][t["pick"]]["runs"]
        n = len(rs)
        ok = t["pick"] in t["safe"] and s["verified"]
        lines.append(f"{'picked' if ok else 'best found (NOT reliable)'}: base {s['speed']:g} m/s, {describe(t['pick'])}"
                     f"  (robot menu: steer_gain_d {rs[0]['kd_fw']:.0f})")
        lines.append(f"  drive 2 lap {sum(r['lap2'] for r in rs) / n:.2f} s;  drive 1: line rms "
                     f"{sum(r['rms_lat'] for r in rs) / n:.1f} mm, weave {sum(r['weave'] for r in rs) / n:.0f} dps, "
                     f"slip {sum(r['slip'] for r in rs) / n:.1f} %")
        lines.append("")
        lines.append("try it in the 3D session:")
        p = t["pick"]
        lines.append(f"  build\\Release\\linesim.exe view --session --track {os.path.relpath(courses[0], ROOT)} "
                     f"--robot robots/full.json --set base_mps={s['speed']:g} --set steer_kp={p[0]:g} "
                     f"--set steer_kd_s={p[1]:g}" + (f" --set lat_acc_max={p[2]:g}" if p[2] > 0 else "")
                     + "".join(f" --set {e}" for e in extra))
    else:
        lines.append("nothing passes at this speed")
    return "\n".join(lines)


def main():
    a = sys.argv[1:]

    def opt(name, default):
        if name in a:
            i = a.index(name)
            v = a[i + 1]
            del a[i:i + 2]
            return v
        return default

    def flag(name):
        if name in a:
            a.remove(name)
            return True
        return False

    extra, tracks = [], []
    while "--set" in a:
        extra.append(opt("--set", None))
    while "--track" in a:
        tracks.append(os.path.abspath(opt("--track", None)))
    reuse = flag("--reuse")
    max_speed = flag("--max-speed")
    robot = os.path.abspath(opt("--robot", os.path.join(ROOT, "robots", "full.json")))
    if not os.path.exists(robot):
        robot = os.path.join(ROOT, "robots", "default.json")
    jobs = int(opt("--jobs", max(1, (os.cpu_count() or 2) - 1)))
    n_courses = int(opt("--courses", 4))
    refine = int(opt("--refine", "3"))
    kps = [float(x) for x in opt("--kp", ",".join(map(str, KP_COARSE if refine > 0 else KP))).split(",")]
    kds = [float(x) for x in opt("--kd", ",".join(map(str, KD_COARSE if refine > 0 else KD))).split(",")]
    lats = [float(x) for x in opt("--lat", "0").split(",")]
    axes = (kps, kds, lats)
    if not a:
        sys.exit(__doc__)
    speed = float(a[0])
    use_private_exe()

    if not tracks:
        if os.path.exists(REAL_COURSE):
            tracks = [REAL_COURSE]
        else:
            cdir = os.path.join(ROOT, "runs", "tune", "courses")
            os.makedirs(cdir, exist_ok=True)
            for s in SEEDS[:n_courses]:
                c = os.path.join(cdir, f"course_{s}.json")
                subprocess.run([EXE, "gen", "--seed", str(s), "--out", c], stdout=subprocess.DEVNULL, check=True)
                tracks.append(c)
    print(f"courses: {', '.join(os.path.relpath(c, ROOT) for c in tracks)}  robot: {os.path.basename(robot)}")

    def evaluate(v, quiet):
        s = search_at(round(v, 3), tracks, robot, axes, extra, jobs, reuse, refine, quiet)
        text = report_search(s, tracks, robot, extra)
        save(s["final"], text)
        return s, text

    if not max_speed:
        s, text = evaluate(speed, False)
        print()
        print(text)
        print(f"\nwritten to {os.path.relpath(s['final']['out_root'], ROOT)}\\result.txt")
        return

    best, dv, v, log = None, 0.2, speed, []
    while True:
        s, _ = evaluate(v, True)
        t = s["final"]
        ok = t["pick"] is not None and t["pick"] in t["safe"] and s["verified"]
        if ok:
            r = t["cell"][t["pick"]]["runs"]
            line = (f"  {s['speed']:.3f} m/s  pass: {describe(t['pick'])}, "
                    f"drive 2 {sum(x['lap2'] for x in r) / len(r):.2f} s")
            best = s
            v = s["speed"] + dv
        else:
            line = f"  {s['speed']:.3f} m/s  no reliable pass" + (f" ({s['log'][-1].strip()})" if s["log"] else "")
            if best is None:
                if v <= 0.5:
                    break
                v = max(0.5, v - dv)
            else:
                if dv <= 0.01 + 1e-9:
                    print(line, flush=True)
                    log.append(line)
                    break
                dv = max(0.01, dv / 2)
                v = best["speed"] + dv
        print(line, flush=True)
        log.append(line)

    lines = ["max-speed search (drive 1 + 2), " + ", ".join(os.path.basename(c) for c in tracks), ""] + log + [""]
    if best:
        lines.append(f"highest base speed that passes: {best['speed']:.3f} m/s")
        lines.append(report_search(best, tracks, robot, extra))
    else:
        lines.append("no speed passed")
    text = "\n".join(lines)
    with open(os.path.join(ROOT, "runs", "tune", "max_speed.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    print()
    print(text)
    print("\nwritten to runs\\tune\\max_speed.txt")


if __name__ == "__main__":
    main()
