"""Firmware save-slot logs (Drive_Data/save_slot_N.txt) vs. the answer marker sequence.

    python tools/logs.py <folder with save_slot_*.txt (searched recursively)> <answer.txt>

For every log: the drive parameters, whether the marker sequence matches the answer,
and where it first deviates. Logs that match exactly are summarised per segment
(median odometry distance and recorded yaw), which is what tools/track_from_logs.py
uses to rebuild the course.
"""
import glob
import os
import re
import statistics
import sys


def read_answer(path):
    seq = []
    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.match(r"\s*(\d+)\.?\s+([LRCUE])\s*$", line)
        if m:
            seq.append("U" if m.group(2) == "E" else m.group(2))
    return seq


def read_log(path):
    params, marks = {}, []
    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.match(r"\s*([a-z ]+?)\s*=\s*([-\d.]+)", line)
        if m and not marks:
            params[m.group(1).strip()] = float(m.group(2))
            continue
        m = re.match(r"\s*(\d+)\s+([LRCU])\s+([-\d.]+)\s*m\s+([-\d.]+)\s*deg", line)
        if m:
            marks.append((m.group(2), float(m.group(3)), float(m.group(4))))
    return params, marks


def first_deviation(seq, ans):
    for i, (a, b) in enumerate(zip(seq, ans)):
        if a != b:
            return i
    return min(len(seq), len(ans)) if len(seq) != len(ans) else -1


def load_all(folder, ans):
    runs = []
    for f in sorted(glob.glob(os.path.join(folder, "**", "save_slot_*.txt"), recursive=True)):
        params, marks = read_log(f)
        seq = [m[0] for m in marks]
        runs.append({"file": os.path.relpath(f, folder), "params": params, "marks": marks,
                     "dev": first_deviation(seq, ans), "n": len(seq)})
    return runs


def main(folder, answer):
    ans = read_answer(answer)
    print(f"answer: {len(ans)} markers  " + "".join(ans))
    runs = load_all(folder, ans)
    good = []
    print(f"\n{'log':34s} {'base':>5s} {'kp':>5s} {'kd':>6s}  markers  result")
    for r in runs:
        p = r["params"]
        if r["dev"] < 0:
            res = "OK"
            good.append(r)
        else:
            i = r["dev"]
            got = r["marks"][i][0] if i < r["n"] else "-"
            exp = ans[i] if i < len(ans) else "-"
            res = f"differs at #{i}: got {got}, expected {exp}"
        print(f"{r['file']:34s} {p.get('base mps', 0):5.2f} {p.get('steer kp', 0):5.1f} {p.get('steer kd', 0):6.1f}"
              f"  {r['n']:3d}/{len(ans)}  {res}")
    print(f"\n{len(good)} of {len(runs)} logs match the answer exactly")
    if good:
        print(f"\n{'#':>3s} type  dist median [m] (min..max)   yaw median [deg]")
        for i in range(len(ans)):
            d = [r["marks"][i][1] for r in good]
            y = [r["marks"][i][2] for r in good]
            print(f"{i:3d}  {ans[i]}    {statistics.median(d):6.3f} ({min(d):.3f}..{max(d):.3f})   {statistics.median(y):7.1f}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
