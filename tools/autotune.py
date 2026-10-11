"""Base speed and steering gains for the race, found in the simulator.

    python tools/autotune.py 2.0                 best steer kp / kd at base speed 2.0 m/s
    python tools/autotune.py 1.6 --max-speed     highest base speed that passes, from 1.6 up

    options: [--track tracks/x.json ...] [--courses 4] [--robot robots/full.json] [--jobs 8]
             [--kp 8,10,...] [--kd 0,0.05,...] [--set max_mps=4 ...] [--reuse]

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
for the exact fitted lag count as unstable. The pick is among the passing pairs whose
grid neighbours are stable too: within 1 % of the best score, the lowest kp, then the lowest kd.

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
REAL_COURSE = os.path.join(ROOT, "tracks", "practice_board.json")

KP = [8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40]
KD = [0, 0.05, 0.1, 0.15, 0.225, 0.3, 0.4, 0.5]   # seconds (x 2000 = firmware steer_gain_d)
SEEDS = [101, 102, 103, 104, 105, 106, 107, 108]
STRESS_LAG = (1.4, 1.85)   # sensor_delay factors of the stress drives
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


def sim(course, robot, drives, speed, kp, kd, extra, out, reuse, robot_params=()):
    """Runs linesim, returns summary.json (or None)."""
    cmd = [EXE, "run", "--track", course, "--robot", robot, "--drives", drives, "--quiet", "--out", out,
           "--set", f"base_mps={speed}", "--set", f"steer_kp={kp}", "--set", f"steer_kd_s={kd}", "--max-time", "60"]
    for e in extra:
        cmd += ["--set", e]
    for rp in robot_params:
        cmd += ["--robot-param", rp]
    if not (reuse and os.path.exists(os.path.join(out, "summary.json"))):
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        with open(os.path.join(out, "summary.json"), encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def lost_line(d):
    return d["outcome"] != "finished" or d["max_lat_mm"] > MAX_LAT_MM


def run(course, robot, speed, kp, kd, extra, out, reuse, lag):
    j = sim(course, robot, "1,2", speed, kp, kd, extra, out, reuse)
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
                         [f"sensor_delay={lag * f:.6f}"])
                if sj is None or lost_line(sj["drives"][0]):
                    why = f"lag x{f:g}: " + (sj["drives"][0]["outcome"] if sj else "no result")
                    break
                if k == 0:
                    sw, _ = drive_metrics(os.path.join(f"{out}_lag{f:g}", "drive1.csv"))
                    if sw > 1.5 * weave + 10.0:
                        why = f"lag x{f:g}: weave {sw:.0f} dps"
                        break
        if why:
            r.update({"pass": False, "lost": True, "score": 250.0 + clean1, "why": why})
    return r


def tune_at(speed, courses, robot, kps, kds, extra, jobs, reuse, quiet=False):
    lag = robot_delay(robot)
    out_root = os.path.join(ROOT, "runs", "tune", f"{speed:g}")
    pairs = [(kp, kd) for kp in kps for kd in kds]
    total = len(pairs) * len(courses)
    print(f"base speed {speed:g} m/s: {len(kps)} kp x {len(kds)} kd on {len(courses)} course(s) = {total} runs "
          f"(drive 1 + 2 each), {jobs} at a time", flush=True)
    results = {}
    done = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {}
        for kp, kd in pairs:
            for ci, c in enumerate(courses):
                out = os.path.join(out_root, "runs", f"kp{kp:g}_kd{kd:g}s_c{ci}")
                futs[ex.submit(run, c, robot, speed, kp, kd, extra, out, reuse, lag)] = (kp, kd, ci)
        for f in concurrent.futures.as_completed(futs):
            kp, kd, ci = futs[f]
            results.setdefault((kp, kd), {})[ci] = f.result()
            done += 1
            if not quiet and (done % 50 == 0 or done == total):
                print(f"  {done}/{total}", flush=True)

    cell = {}
    for p, rs in results.items():
        rs = [rs[i] for i in range(len(courses))]
        cell[p] = {"pass": all(r["pass"] for r in rs), "lost": any(r["lost"] for r in rs),
                   "score": sum(r["score"] for r in rs) / len(rs), "runs": rs,
                   "why": [r["why"] for r in rs if not r["pass"]]}

    def robust(kp, kd):
        i, j = kps.index(kp), kds.index(kd)
        nb = [(kps[i + di], kds[j + dj]) for di, dj in ((-1, 0), (1, 0), (0, -1), (0, 1))
              if 0 <= i + di < len(kps) and 0 <= j + dj < len(kds)]
        return not any(cell[n]["lost"] for n in nb)

    passing = [p for p in pairs if cell[p]["pass"]]
    safe = [p for p in passing if robust(*p)]
    pick = None
    if passing:
        # Within 1 % of the best score (~0.1 s of drive 2 lap) the gentlest gains win: lower kp, then
        # lower kd. A high kd also amplifies the real sensor's position steps, which the model smooths.
        pool = safe or passing
        best = min(cell[p]["score"] for p in pool)
        pick = min((p for p in pool if cell[p]["score"] <= best * 1.01), key=lambda p: (p[0], p[1]))
    return {"speed": speed, "cell": cell, "pairs": pairs, "passing": passing, "safe": safe, "pick": pick,
            "out_root": out_root}


def report(t, courses, robot, kps, kds, extra):
    cell, pick = t["cell"], t["pick"]
    lines = [f"base speed {t['speed']:g} m/s, drive 1 + 2 on {', '.join(os.path.basename(c) for c in courses)}, "
             f"robot {os.path.basename(robot)}" + (f", --set {' '.join(extra)}" if extra else ""),
             "",
             "score = 5 x drive 2 lap [s] + drive 1 (line rms [mm] + weave/20 + slip%/2), lower is better",
             "x = loses the line or is unstable with more steering lag (x1.4 / x1.85), m = misreads markers, * = picked",
             "",
             "kp\\kd [s] " + "".join(f"{kd:>8g}" for kd in kds)]
    for kp in kps:
        row = f"{kp:>8g}  "
        for kd in kds:
            c = cell[(kp, kd)]
            txt = f"{c['score']:.1f}" if c["pass"] else ("x" if c["lost"] else "m")
            if (kp, kd) == pick:
                txt = "*" + txt
            row += f"{txt:>8}"
        lines.append(row)
    lines.append("")
    if pick:
        rs = cell[pick]["runs"]
        n = len(rs)
        lines.append(f"picked: base {t['speed']:g} m/s, steer kp {pick[0]:g}, steer kd {pick[1]:g} s "
                     f"(robot menu: steer_gain_d {rs[0]['kd_fw']:.0f})"
                     + ("" if pick in t["safe"] else "   (a grid neighbour loses the line: at the edge)"))
        lines.append(f"  drive 2 lap {sum(r['lap2'] for r in rs) / n:.2f} s;  drive 1: line rms "
                     f"{sum(r['rms_lat'] for r in rs) / n:.1f} mm, weave {sum(r['weave'] for r in rs) / n:.0f} dps, "
                     f"slip {sum(r['slip'] for r in rs) / n:.1f} %")
        lines.append(f"  passing pairs {len(t['passing'])} of {len(t['pairs'])}, away from the edge {len(t['safe'])}")
        lines.append("")
        lines.append("try it in the 3D session:")
        lines.append(f"  build\\Release\\linesim.exe view --session --track {os.path.relpath(courses[0], ROOT)} "
                     f"--robot robots/full.json --set base_mps={t['speed']:g} --set steer_kp={pick[0]:g} "
                     f"--set steer_kd_s={pick[1]:g}" + "".join(f" --set {e}" for e in extra))
    else:
        why = {}
        for c in cell.values():
            for w in c["why"]:
                key = w.split(" (")[0] if "markers" in w else w
                why[key] = why.get(key, 0) + 1
        lines.append("no kp/kd pair passes at this speed. Most common: "
                     + ", ".join(f"{k} x{n}" for k, n in sorted(why.items(), key=lambda kv: -kv[1])[:4]))
    return "\n".join(lines)


def save(t, text):
    os.makedirs(t["out_root"], exist_ok=True)
    with open(os.path.join(t["out_root"], "result.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    cell, pick = t["cell"], t["pick"]
    with open(os.path.join(t["out_root"], "result.json"), "w", encoding="utf-8") as fh:
        json.dump({"speed": t["speed"],
                   "picked": {"steer_kp": pick[0], "steer_kd_s": pick[1], "steer_gain_d": cell[pick]["runs"][0]["kd_fw"],
                              "safe": pick in t["safe"]} if pick else None,
                   "grid": [{"kp": kp, "kd": kd, "pass": cell[(kp, kd)]["pass"], "lost": cell[(kp, kd)]["lost"],
                             "score": cell[(kp, kd)]["score"], "why": cell[(kp, kd)]["why"]} for kp, kd in t["pairs"]]},
                  fh, indent=1)


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
    kps = [float(x) for x in opt("--kp", ",".join(map(str, KP))).split(",")]
    kds = [float(x) for x in opt("--kd", ",".join(map(str, KD))).split(",")]
    if not a:
        sys.exit(__doc__)
    speed = float(a[0])

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

    if not max_speed:
        t = tune_at(speed, tracks, robot, kps, kds, extra, jobs, reuse)
        text = report(t, tracks, robot, kps, kds, extra)
        save(t, text)
        print()
        print(text)
        print(f"\nwritten to {os.path.relpath(t['out_root'], ROOT)}\\result.txt")
        return

    best, dv, v, log = None, 0.2, speed, []
    while True:
        t = tune_at(round(v, 3), tracks, robot, kps, kds, extra, jobs, reuse, quiet=True)
        save(t, report(t, tracks, robot, kps, kds, extra))
        ok = t["pick"] is not None and t["pick"] in t["safe"]
        if ok:
            r = t["cell"][t["pick"]]["runs"]
            line = (f"  {t['speed']:.3f} m/s  pass: kp {t['pick'][0]:g} kd {t['pick'][1]:g} s, "
                    f"drive 2 {sum(x['lap2'] for x in r) / len(r):.2f} s ({len(t['safe'])} safe pairs)")
            best = t
            v = t["speed"] + dv
        else:
            line = f"  {t['speed']:.3f} m/s  no pair passes away from the edge"
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
        lines.append(report(best, tracks, robot, kps, kds, extra))
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
