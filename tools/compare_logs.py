"""Replay the real runs in the simulator and compare marker readings.

    python tools/compare_logs.py <log folder> <answer.txt> <track.json> [--robot robots/full.json] [--jobs 4]

For every distinct parameter set found in the real save-slot logs, runs the 1st drive in
LineSim with the same driveData (base_mps, accel, decel, steer kp/kd, pos_atten_gain) and
compares where the marker sequence first deviates from the answer, and the per-segment
odometry distances against the real median.
"""
import concurrent.futures
import csv
import os
import statistics
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from logs import first_deviation, load_all, read_answer   # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.join(ROOT, "build", "Release", "linesim.exe")


def key_of(p):
    return (p.get("base mps", 0), p.get("accel", 0), p.get("decel", 0), p.get("steer kp", 0),
            p.get("steer kd", 0), p.get("pos atten gain", 0))


def run_sim(track, robot, k, out):
    base, acc, dec, kp, kd, att = k
    cmd = [EXE, "run", "--track", track, "--robot", robot, "--drives", "1", "--quiet", "--out", out,
           "--set", f"base_mps={base}", "--set", f"accel={acc}", "--set", f"decel={dec}",
           "--set", f"steer_kp={kp}", "--set", f"steer_kd={kd}", "--set", f"pos_atten_gain={att}",
           "--max-time", "60"]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    marks = []
    f = os.path.join(out, "drive1_markers.csv")
    if os.path.exists(f):
        for row in csv.DictReader(open(f)):
            t = row["type"]
            marks.append(("U" if t == "STOP" else t, float(row["dist_from_prev_m"])))
    return marks


def main(folder, answer, track, robot, jobs):
    ans = read_answer(answer)
    runs = load_all(folder, ans)
    groups = {}
    for r in runs:
        groups.setdefault(key_of(r["params"]), []).append(r)
    keys = sorted(groups)
    out_root = os.path.join(ROOT, "runs", "compare")
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {k: ex.submit(run_sim, track, robot, k, os.path.join(out_root, "_".join(f"{v:g}" for v in k)))
                for k in keys}
        sim = {k: f.result() for k, f in futs.items()}

    def dev_txt(seq):
        d = first_deviation(seq, ans)
        if d < 0:
            return "OK", d
        got = seq[d] if d < len(seq) else "-"
        return f"#{d} {got}", d

    print(f"{'base':>5} {'acc':>4} {'kp':>5} {'kd':>4}  {'real (each log)':38s} sim")
    agree = total = 0
    for k in keys:
        real = [dev_txt([m[0] for m in r["marks"]])[0] for r in groups[k]]
        s_txt, s_dev = dev_txt([m[0] for m in sim[k]])
        real_ok = sum(1 for t in real if t == "OK")
        total += 1
        agree += (real_ok > 0) == (s_txt == "OK")
        print(f"{k[0]:5.2f} {k[1]:4.1f} {k[3]:5.1f} {k[4]:4.1f}  {', '.join(real):38s} {s_txt}")
    print(f"\nsuccess / failure agrees for {agree} of {total} parameter sets")

    # distances: real median of exact logs vs the simulator's exact runs
    good_real = [r for r in runs if r["dev"] < 0]
    good_sim = [m for m in sim.values() if first_deviation([x[0] for x in m], ans) < 0]
    if good_real and good_sim:
        print(f"\nsegment distance [m] (real median of {len(good_real)} exact logs vs sim median of {len(good_sim)})")
        errs = []
        for i in range(1, len(ans)):
            rd = statistics.median(r["marks"][i][1] for r in good_real)
            sd = statistics.median(m[i][1] for m in good_sim)
            errs.append(sd - rd)
            print(f"  {i:2d} {ans[i]}  real {rd:6.3f}  sim {sd:6.3f}  diff {1000 * (sd - rd):+6.0f} mm")
        print(f"  mean abs diff {1000 * statistics.mean(abs(e) for e in errs):.0f} mm, "
              f"total real {sum(statistics.median(r['marks'][i][1] for r in good_real) for i in range(1, len(ans))):.2f} m")


if __name__ == "__main__":
    a = [x for x in sys.argv[1:]]
    robot = os.path.join(ROOT, "robots", "full.json")
    jobs = 4
    if "--robot" in a:
        robot = a[a.index("--robot") + 1]
        del a[a.index("--robot"):a.index("--robot") + 2]
    if "--jobs" in a:
        jobs = int(a[a.index("--jobs") + 1])
        del a[a.index("--jobs"):a.index("--jobs") + 2]
    main(a[0], a[1], a[2], robot, jobs)
