"""Rebuild practice board track from SysID/save_slot_s2r_1 data."""
import json
import math
import os
import re
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from track_from_logs import build, R
import course

GT = [
    'U', 'L', 'L', 'L', 'L', 'R', 'R', 'R',
    'L', 'R', 'R', 'L', 'L', 'L', 'L', 'R',
    'L', 'L', 'C', 'R', 'R', 'C', 'L', 'L',
    'L', 'R', 'L', 'L', 'L', 'L', 'L', 'R',
    'L', 'R', 'R', 'U'
]

def parse_slot(path):
    lines = open(path, encoding="utf-8", errors="ignore").readlines()
    header, marks = [], []
    in_marks = False
    for l in lines:
        if "====" in l:
            in_marks = True
            header.append(l)
            continue
        if not in_marks:
            header.append(l)
            continue
        m = re.match(r'\s*(\d+)\s+([LRCU])\s+([-\d.]+)\s*m\s+([-\d.]+)\s*deg', l)
        if m:
            marks.append((int(m.group(1)), m.group(2), float(m.group(3)), float(m.group(4))))
    return header, marks

def get_cleaned_runs():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    slot_dir = os.path.join(base_dir, "save_slot_s2r_1")

    # Slot 8
    _, m8 = parse_slot(os.path.join(slot_dir, "save_slot_8.txt"))
    clean_8 = list(m8[:35])
    clean_8.append((35, 'U', m8[35][2] + m8[36][2], m8[35][3] + m8[36][3]))

    # Slot 7
    _, m7 = parse_slot(os.path.join(slot_dir, "save_slot_7.txt"))
    clean_7 = list(m7[:19])
    clean_7.append((19, 'R', m7[19][2] + m7[20][2], m7[19][3] + m7[20][3]))
    for i in range(21, 36):
        clean_7.append((len(clean_7), m7[i][1], m7[i][2], m7[i][3]))
    clean_7.append((35, 'U', m7[36][2] + m7[37][2], m7[36][3] + m7[37][3]))

    return [clean_7, clean_8]

def rebuild(out_path="tracks/practice_board.json"):
    runs = get_cleaned_runs()
    dist = [statistics.median(r[i][2] for r in runs) for i in range(len(GT))]

    # Deducing segment types and angles from ZETIN rules, odometry and gyro:
    segs = [
        ("S", dist[1]), ("A", 180), ("S", dist[3]), ("A", 45), ("S", dist[5]), ("A", -45),
        ("S", dist[7]), ("A", -45), ("A", 45), ("A", -225), ("S", dist[11]), ("A", 45),
        ("S", dist[13]), ("A", 180), ("S", dist[15]), ("A", -225), ("A", 45), ("S", dist[18]),
        ("S", dist[19]), ("A", -270), ("S", dist[21]), ("S", dist[22]), ("A", 90), ("S", dist[24]),
        ("A", 225), ("A", -270), ("A", 135), ("S", dist[28]), ("A", 90), ("S", dist[30]),
        ("A", 45), ("A", -90), ("A", 90), ("A", -45), ("S", dist[35])
    ]

    crosses = [18, 21]
    sidx = [k for k, s in enumerate(segs) if s[0] == "S"]
    lens = [segs[k][1] for k in sidx] + [0.24]

    def residual(v):
        sg = list(segs)
        for j, k in enumerate(sidx):
            sg[k] = ("S", v[j])
        sg.append(("S", v[-1]))
        pts = build(sg)
        r = [pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]]
        r += [pts[crosses[0]][0] - pts[crosses[1]][0], pts[crosses[0]][1] - pts[crosses[1]][1]]
        r += [0.05 * (v[j] - lens[j]) for j in range(len(v))]
        return r

    v = list(lens)
    for _ in range(30):
        r0 = residual(v)
        J = []
        for j in range(len(v)):
            vp = list(v)
            vp[j] += 1e-5
            J.append([(a - b) / 1e-5 for a, b in zip(residual(vp), r0)])
        n = len(v)
        A = [[sum(J[i][m] * J[j][m] for m in range(len(r0))) for j in range(n)] for i in range(n)]
        g = [-sum(J[i][m] * r0[m] for m in range(len(r0))) for i in range(n)]
        for i in range(n): A[i][i] += 1e-9
        for c in range(n):
            p = max(range(c, n), key=lambda q: abs(A[q][c]))
            A[c], A[p] = A[p], A[c]
            g[c], g[p] = g[p], g[c]
            for q in range(c + 1, n):
                f = A[q][c] / A[c][c]
                for w in range(c, n): A[q][w] -= f * A[c][w]
                g[q] -= f * g[c]
        dv = [0.0] * n
        for c in range(n - 1, -1, -1):
            dv[c] = (g[c] - sum(A[c][w] * dv[w] for w in range(c + 1, n))) / A[c][c]
        v = [max(0.02, a + b) for a, b in zip(v, dv)]
        if max(abs(x) for x in dv) < 1e-7: break

    final = list(segs)
    for j, k in enumerate(sidx):
        final[k] = ("S", v[j])
    closing = v[-1]

    out_segs = [{"straight": round(s[1], 4)} if s[0] == "S" else {"arc": int(s[1])} for s in final]
    segs_json = [{"straight": round(closing, 4)}] + out_segs
    track = {
        "name": "practice_board",
        "closed": True,
        "line_width": 0.02,
        "goal_s": 0.0,
        "start_s": round(closing, 4),
        "auto_turn_marks": True,
        "segments": segs_json,
        "_source": "rebuilt from SysID/save_slot_s2r_1 odometry logs and ground truth"
    }

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(track, f, indent=2, ensure_ascii=False)
    print(f"Wrote {out_path}")
    course.main(out_path)

if __name__ == "__main__":
    rebuild()
