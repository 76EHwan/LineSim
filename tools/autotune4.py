"""Every drive parameter at once, for the fastest reliable 4th drive.

    python tools/autotune4.py [--track tracks/x.json ...] [--iters 40] [--jobs 11] [--set k=v ...]
                              [--start base_mps=1.7,steer_kp=28,...]

Drive 4 (target_pos shift in curves, acceleration on straights and 45 deg segments) runs from drive
1's marker log, so each evaluation is drive 1 then drive 4 on every course (the real practice board
when it is there). A parameter set passes when on every course:
  - drive 1 reads the markers right (its data is what drive 4 plans with),
  - drive 4 finishes, never more than 70 mm off the line, with every marker right,
  - and the same still holds with more steering lag than the model has (sensor_delay x1.4) and with
    less grip (tire friction x0.85): the model was fitted to one gyro log and a guessed friction.
The result is checked with 2 more seeds (sensor noise, gain spread): drive 1 + 4 must hold on every
seed, the stress drives on at least 2 of the 3.
Score: drive 4 lap time (mean over the courses).

Search: coordinate pattern search. Every round drives the center and each parameter one step up and
one step down (in parallel); the center moves to the fastest passing candidate if it gains more than
10 ms, else every step is halved. It ends when the steps reach their resolution or after --iters
rounds. If the result fails the seed check, the previous center that holds is used.

Parameters (name, range, start, first step, resolution): see PARAMS. steer kd is in seconds (robot
menu value = kd / 0.0005).

Writes runs/tune4/result.txt and result.json.
"""
import concurrent.futures
import hashlib
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune as at   # noqa: E402

ROOT = at.ROOT
#          name                lo     hi    start   step   resolution
PARAMS = [("base_mps",         1.3,   2.2,  1.7,    0.1,   0.02),
          ("max_mps",          2.5,   6.0,  4.5,    0.5,   0.1),
          ("accel",            4.0,  14.0, 10.0,    2.0,   0.5),
          ("decel",            4.0,  14.0, 10.0,    2.0,   0.5),
          ("steer_kp",        12.0,  32.0, 28.0,    3.0,   0.5),
          ("steer_kd_s",       0.0,   0.4,  0.2125, 0.05,  0.01),
          ("lat_acc_max",      9.0,  15.0, 12.6,    1.0,   0.2),
          ("target_shift",     0.0,   0.5,  0.3,    0.1,   0.02),
          ("target_curve_gain", 0.5,  2.5,  1.0,    0.3,   0.05),
          ("slalom_ratio",     0.0,   0.12, 0.05,   0.03,  0.005),
          ("pos_atten_gain",   0.0,   0.6,  0.0,    0.1,   0.02)]
NAMES = [p[0] for p in PARAMS]
MIN_GAIN_S = 0.01          # a move must save more than this much drive-4 lap time
OUT = os.path.join(ROOT, "runs", "tune4")
_cache = {}


def key(vals):
    return tuple(round(v, 4) for v in vals)


def sets(vals, extra):
    return [f"{n}={v:g}" for n, v in zip(NAMES, vals)] + list(extra)


def drive_pair(course, robot, vals, extra, tag, robot_params=(), seed=1):
    """drive 1 then drive 4; returns (ok, why, lap4, d4)."""
    h = hashlib.md5(repr((course, key(vals), tuple(extra), tuple(robot_params), seed)).encode()).hexdigest()[:12]
    out = os.path.join(OUT, "runs", f"{tag}_{h}")
    s = sets(vals, extra)
    # at.sim sets base_mps / steer_kp / steer_kd_s itself: pass the rest as extra --set
    v = dict(zip(NAMES, vals))
    j = at.sim(course, robot, "1,4", v["base_mps"], v["steer_kp"], v["steer_kd_s"],
               [x for x in s if not x.startswith(("base_mps=", "steer_kp=", "steer_kd_s="))], out, False,
               robot_params, seed)
    if not j or not j.get("drives"):
        return False, "no result", None, None
    ds = {d["drive"]: d for d in j["drives"]}
    d1, d4 = ds.get(1), ds.get(4)
    if d1 is None or at.lost_line(d1):
        return False, "1: " + (d1["outcome"] if d1 else "missing"), None, None
    if d1.get("marker_check") != "OK":
        return False, "1: markers " + d1.get("marker_check", "?"), None, None
    if d4 is None or d4["outcome"] == "skipped":
        return False, "4: did not run", None, None
    if at.lost_line(d4):
        return False, "4: " + (d4["outcome"] if d4["outcome"] != "finished" else "line far off"), None, d4
    if d4.get("marker_check") != "OK" or d4["lap_time_fw"] <= 0:
        return False, "4: markers " + d4.get("marker_check", "?"), None, d4
    return True, "", d4["lap_time_fw"], d4


def evaluate(courses, robot, vals, extra, seed=1, stress=True):
    """{'pass', 'score', 'why'} over all courses, with the stress drives. score is the drive 4 lap time
    for a pass; failures get a graded penalty so the search can climb out of them: drive 1 fails 300,
    drive 4 fails 200 (+ how far it got is not known), a stress drive fails 100 + the lap time."""
    k = (key(vals), seed, stress)
    if k in _cache:
        return _cache[k]
    lag, mu = at.robot_delay(robot), at.robot_mu(robot)
    laps, why, penalty = [], "", 0.0
    for ci, c in enumerate(courses):
        ok, w, lap, _ = drive_pair(c, robot, vals, extra, f"c{ci}", seed=seed)
        if not ok:
            why = f"{os.path.basename(c)} {w}"
            penalty = 300.0 if w.startswith("1:") else 200.0
            break
        laps.append(lap)
    if not why and stress:
        for ci, c in enumerate(courses):
            for rp, label in (([f"sensor_delay={lag * 1.4:.6f}"], "lag x1.4"), ([f"tire_mu={mu * 0.85:.4f}"], "grip x0.85")):
                ok, w, _, _ = drive_pair(c, robot, vals, extra, f"c{ci}s", rp, seed)
                if not ok:
                    why = f"{os.path.basename(c)} {label}: {w}"
                    penalty = 100.0
                    break
            if why:
                break
    lap = sum(laps) / len(laps) if laps else 0.0
    r = {"pass": not why, "score": penalty + lap, "why": why}
    _cache[k] = r
    return r


def clip(i, v):
    lo, hi = PARAMS[i][1], PARAMS[i][2]
    return min(hi, max(lo, v))


def fmt(vals):
    return ", ".join(f"{n} {v:g}" for n, v in zip(NAMES, vals))


def main():
    a = sys.argv[1:]

    def opt(name, default):
        if name in a:
            i = a.index(name)
            v = a[i + 1]
            del a[i:i + 2]
            return v
        return default

    extra, tracks = [], []
    while "--set" in a:
        extra.append(opt("--set", None))
    while "--track" in a:
        tracks.append(os.path.abspath(opt("--track", None)))
    robot = os.path.abspath(opt("--robot", os.path.join(ROOT, "robots", "full.json")))
    jobs = int(opt("--jobs", max(1, (os.cpu_count() or 2) - 1)))
    iters = int(opt("--iters", 40))
    center = [p[3] for p in PARAMS]
    for kv in filter(None, opt("--start", "").split(",")):
        n, v = kv.split("=")
        center[NAMES.index(n)] = float(v)
    if not tracks:
        tracks = [at.REAL_COURSE] if os.path.exists(at.REAL_COURSE) else []
    if not tracks:
        sys.exit("no course: give --track")
    at.use_private_exe()
    os.makedirs(OUT, exist_ok=True)
    log = []

    def say(line):
        print(line, flush=True)
        log.append(line)

    say(f"courses: {', '.join(os.path.relpath(c, ROOT) for c in tracks)}, robot {os.path.basename(robot)}, {jobs} jobs")
    steps = [p[4] for p in PARAMS]
    cur = evaluate(tracks, robot, center, extra)
    say(f"start: {'drive 4 %.3f s' % cur['score'] if cur['pass'] else 'FAILS (' + cur['why'] + ')'}  [{fmt(center)}]")
    accepted = [list(center)] if cur["pass"] else []   # passing centers, for the seed check

    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        for it in range(1, iters + 1):
            cands = []
            for i in range(len(PARAMS)):
                for sgn in (1, -1):
                    v = list(center)
                    v[i] = clip(i, round(center[i] + sgn * steps[i], 4))
                    if v[i] != center[i]:
                        cands.append((i, sgn, v))
            res = list(ex.map(lambda c: evaluate(tracks, robot, c[2], extra), cands))
            scored = [(r["score"], c, r) for r, c in zip(res, cands)]
            npass = sum(1 for r in res if r["pass"])
            best = min(scored, key=lambda x: x[0])
            if best[0] < cur["score"] - MIN_GAIN_S:
                score, (i, sgn, v), r = best
                desc = f"drive 4 {score:.3f} s" if r["pass"] else f"still failing: {r['why']}"
                say(f"round {it:2d}: {NAMES[i]} {center[i]:g} -> {v[i]:g}   {desc}   ({npass} of {len(res)} pass)")
                center = v
                cur = r
                if r["pass"]:
                    accepted.append(list(center))
            else:
                steps = [max(p[5], s / 2) for p, s in zip(PARAMS, steps)]
                say(f"round {it:2d}: no gain > {MIN_GAIN_S * 1000:.0f} ms ({npass} of {len(res)} pass), steps halved")
                if all(abs(s - p[5]) < 1e-9 for p, s in zip(PARAMS, steps)):
                    say("steps at their resolution: done")
                    break

    # The pick must hold with other sensor noise / gain spread too: drive 1 + 4 right with seeds 2
    # and 3, and the stress drives (lag x1.4, grip x0.85) on at least 2 of the 3 seeds. (Demanding
    # every stress drive on every seed left nothing: one seed tips the lag x1.4 drive over.)
    pick = None
    for vals in reversed(accepted):
        nominal = [evaluate(tracks, robot, vals, extra, seed, stress=False) for seed in (2, 3)]
        stressed = [evaluate(tracks, robot, vals, extra, seed) for seed in (1, 2, 3)]
        ok = all(r["pass"] for r in nominal) and sum(r["pass"] for r in stressed) >= 2
        why = "; ".join(r["why"] for r in nominal + stressed if r["why"])
        say(f"seed check {'pass' if ok else 'FAIL'}: {fmt(vals)}" + (f"   ({why})" if why else ""))
        if ok:
            pick = vals
            break
    lines = ["autotune4: drive 1 + drive 4, " + ", ".join(os.path.basename(c) for c in tracks), ""] + log + [""]
    if pick:
        r = evaluate(tracks, robot, pick, extra)
        kd_fw = dict(zip(NAMES, pick))["steer_kd_s"] / 0.0005
        lines.append(f"best reliable: drive 4 lap {r['score']:.3f} s")
        lines.append("")
        lines.append(f"{'parameter':20s} {'value':>8s}   robot menu")
        menu = {"base_mps": "Base m/s", "max_mps": "Max m/s", "accel": "Base Accel", "decel": "Base Decel",
                "steer_kp": "Steer KP", "steer_kd_s": "Steer KD", "lat_acc_max": "Lat Acc Max",
                "target_shift": "Target Shift", "target_curve_gain": "Tgt Crv Gain", "slalom_ratio": "Slalom Ratio",
                "pos_atten_gain": "Pos Abs Gain"}
        for n, v in zip(NAMES, pick):
            shown = f"{kd_fw:.0f}" if n == "steer_kd_s" else f"{v:g}"
            lines.append(f"{n:20s} {v:8g}   {menu[n]} = {shown}")
        lines.append("")
        lines.append("try it in the 3D session (drive 1, then 4):")
        lines.append("  build\\Release\\linesim.exe view --session --track " + os.path.relpath(tracks[0], ROOT)
                     + " --robot robots/full.json " + " ".join(f"--set {s}" for s in sets(pick, extra)))
    else:
        lines.append("no parameter set held the seed check")
    text = "\n".join(lines)
    with open(os.path.join(OUT, "result.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    with open(os.path.join(OUT, "result.json"), "w", encoding="utf-8") as fh:
        json.dump({"picked": dict(zip(NAMES, pick)) if pick else None, "log": log}, fh, indent=1)
    print()
    print(text)


if __name__ == "__main__":
    main()
