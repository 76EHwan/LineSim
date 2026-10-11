"""Steering oscillation on the straights, for a LineSim run or a real Vibe Test log.

    python tools/oscillation.py runs/<run>/drive1.csv tracks/<track>.json [--settle 0.15]
    python tools/oscillation.py vib_log_123.csv

Simulator run: for every straight of the course (skipping the first `--settle` metres after
the curve before it) prints the peak-to-peak lateral error of the sensor bar, the yaw-rate
swing, its rms in the first and second half (decaying or sustained) and the weave frequency.

Vibe Test log (/Drive_Data/vib_log_*.csv: Gyro_Z every 1 ms, the IMU updates it every 10 ms):
finds the straights from the yaw rate itself and prints the same yaw-rate numbers. The
simulator's yaw rate is held to 10 ms steps the same way so the two are comparable.
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from course import load, pieces   # noqa: E402


def weave(t, w):
    """(p-p, rms, rms 1st half, rms 2nd half, frequency from zero crossings) of a yaw-rate trace."""
    m = sum(w) / len(w)
    w = [v - m for v in w]
    h = len(w) // 2
    rms = lambda x: math.sqrt(sum(v * v for v in x) / len(x))   # noqa: E731
    zc = sum(1 for a, b in zip(w, w[1:]) if (a < 0) != (b < 0))
    return max(w) - min(w), rms(w), rms(w[:h]), rms(w[h:]), zc / 2 / (t[-1] - t[0])


def hold_10ms(w):
    return [w[(i // 10) * 10] for i in range(len(w))]


def header():
    print(f"{'yaw p-p':>8} {'rms':>5} {'(1st->2nd half)':>16} {'freq':>7}")


def line(w_stats):
    pp, rms, r1, r2, f = w_stats
    return f"{pp:7.0f}  {rms:5.0f} ({r1:5.0f} ->{r2:5.0f})  {f:5.1f}Hz"


def sim_run(csv_path, track_path, settle=0.15):
    j = load(track_path)
    o = j.get("origin", [0, 0, 0])
    ps, _, total = pieces(j["segments"], o[0], o[1], math.radians(o[2]))
    rows = list(csv.DictReader(open(csv_path)))
    t = [float(r["t"]) for r in rows]
    s = [float(r["s"]) for r in rows]
    lat = [float(r["lat_mm"]) for r in rows]
    wz = [float(r["wz_dps"]) for r in rows]
    print(f"{'straight':>9} {'from':>6} {'len':>5}  {'lat p-p':>8}  ", end="")
    header()
    for k, p in enumerate(ps):
        if p[0] != "S" or p[2] < settle + 0.15:
            continue
        a, b = p[1] + settle, p[1] + p[2]
        idx = [i for i in range(len(s)) if a <= s[i] <= b]
        if len(idx) < 30:
            continue
        # keep the first contiguous pass only
        run = [idx[0]]
        for i in idx[1:]:
            if i != run[-1] + 1:
                break
            run.append(i)
        L = [lat[i] for i in run]
        prev = ps[k - 1] if k > 0 else ps[-1]
        after = "arc %+d" % round(math.degrees(prev[2] / 0.25) * prev[6]) if prev[0] == "A" else "straight"
        stats = weave([t[i] for i in run], hold_10ms([wz[i] for i in run]))
        print(f"{k:9d} {p[1]:6.2f} {p[2]:5.2f}  {max(L) - min(L):7.1f}  {line(stats)}  after {after}")


def vib_log(path, quiet_dps=250.0, min_len=0.3):
    rows = list(csv.DictReader(open(path)))
    w = [float(r["Value"]) for r in rows]
    t = [i * 1e-3 for i in range(len(w))]
    # 150 ms average of the yaw rate: a curve turns at ~v/R (450 dps at 2 m/s), a straight averages ~0
    k = 150
    avg = []
    acc = 0.0
    for i, v in enumerate(w):
        acc += v
        if i >= k:
            acc -= w[i - k]
        avg.append(acc / min(i + 1, k))
    # the average lags by k/2: shift it back to the middle of its window
    avg = avg[k // 2:] + [avg[-1]] * (k // 2)
    print(f"{len(w)} samples, {len(w) / 1000:.2f} s")
    print(f"{'straight':>13} {'len':>5}  ", end="")
    header()
    i = 0
    while i < len(w):
        if abs(avg[i]) < quiet_dps:
            j = i
            while j < len(w) and abs(avg[j]) < quiet_dps:
                j += 1
            if (j - i) / 1000 >= min_len:
                stats = weave(t[i:j], w[i:j])
                print(f"{t[i]:5.2f}-{t[j - 1]:5.2f}s {(j - i) / 1000:5.2f}  {line(stats)}")
            i = j
        else:
            i += 1


if __name__ == "__main__":
    first = open(sys.argv[1]).readline()
    if first.startswith("Index,Value"):
        vib_log(sys.argv[1])
    else:
        settle = float(sys.argv[sys.argv.index("--settle") + 1]) if "--settle" in sys.argv else 0.15
        sim_run(sys.argv[1], sys.argv[2], settle)
