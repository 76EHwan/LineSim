"""Rebuild a course from firmware marker logs + the answer sequence + the contest rules.

    python tools/track_from_logs.py <log folder> <answer.txt> <out track.json> [--name practice_board]

Each segment between two consecutive markers is a straight or one arc (R 0.25 m, 45 deg
steps, 45..270). Rules (ZETIN): a turn mark sits where a curve starts or changes direction
(on the side it turns to) and where a curve ends into a straight (side of that curve);
crossings and end marks sit on straights. All labelings that satisfy the marks are
enumerated; the odometry distances (median of the logs that read the answer exactly)
score each one, and the course must close on itself with the two passes over every
crossing meeting at right angles. The odometry measures the axle path, which runs inside
the line on arcs (radius ~ sqrt(R^2 - L^2), L = axle to sensor bar).
"""
import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from logs import load_all, read_answer   # noqa: E402

R = 0.25
L_SENSOR = 0.132
R_AXLE = math.sqrt(R * R - L_SENSOR * L_SENSOR)
ANGLES = [45, 90, 135, 180, 225, 270]


def options(kind_prev_mark, kind_next_mark, d):
    """Candidate segments for odometry distance d: ('S', length) or ('A', +-deg)."""
    out = [("S", d, 0.0)]
    deg = math.degrees(d / R_AXLE)
    for a in ANGLES:
        err = abs(deg - a) / max(a, 45)
        if err < 0.45:
            out.append(("A", +a, err))
            out.append(("A", -a, err))
    return out


def valid_mark(mark, prev, nxt):
    """Is mark (L/R/C/U) consistent with the segments before and after it?"""
    def arc_dir(seg):
        return 0 if seg[0] == "S" else (1 if seg[1] > 0 else -1)
    p, n = arc_dir(prev), arc_dir(nxt)
    if mark in "CU":
        return p == 0 and n == 0
    x = 1 if mark == "L" else -1
    if n == x and p != x:          # curve starts / changes direction to x
        return True
    if n == 0 and p == x:          # curve x ends, straight starts
        return True
    return False


def build(segs):
    """Line path for the segment list -> points of every marker, end pose."""
    x = y = th = 0.0
    marks = [(x, y, th)]
    for s in segs:
        if s[0] == "S":
            x += s[1] * math.cos(th)
            y += s[1] * math.sin(th)
        else:
            turn = 1 if s[1] > 0 else -1
            phi = math.radians(s[1])
            cx, cy = x - turn * R * math.sin(th), y + turn * R * math.cos(th)
            rx, ry = x - cx, y - cy
            x = cx + rx * math.cos(phi) - ry * math.sin(phi)
            y = cy + rx * math.sin(phi) + ry * math.cos(phi)
            th += phi
        marks.append((x, y, th))
    return marks


def main(folder, answer, out_path, name):
    ans = read_answer(answer)
    good = [r for r in load_all(folder, ans) if r["dev"] < 0]
    if not good:
        sys.exit("no log matches the answer exactly")
    dist = [statistics.median(r["marks"][i][1] for r in good) for i in range(len(ans))]
    # segment k runs from marker k to marker k+1; its length is logged at marker k+1.
    # The course is closed: the last segment (goal -> start) is a straight of unknown length.
    nseg = len(ans) - 1
    seg_d = [dist[k + 1] for k in range(nseg)]
    cands = [options(ans[k], ans[k + 1], seg_d[k]) for k in range(nseg)]
    straight_close = ("S", 0.0, 0.0)

    best = []
    seq = []

    def rec(k, cost):
        if k == nseg:
            # closing segment goal -> start is straight; both end marks need straights around them
            if not valid_mark(ans[-1], seq[-1], straight_close) or not valid_mark(ans[0], straight_close, seq[0]):
                return
            best.append((cost, list(seq)))
            return
        for c in cands[k]:
            if k > 0 and not valid_mark(ans[k], seq[-1], c):
                continue
            seq.append(c)
            rec(k + 1, cost + c[2])
            seq.pop()
            if len(best) > 200000:
                return

    rec(0, 0.0)
    print(f"{len(good)} exact logs, {len(best)} labelings satisfy the marks")
    if not best:
        sys.exit("no consistent labeling")

    crosses = [i for i, m in enumerate(ans) if m == "C"]

    def score(segs):
        pts = build(segs)
        end = pts[-1]
        start = pts[0]
        # closing straight goal -> start must point back to the start along the end heading
        dx, dy = start[0] - end[0], start[1] - end[1]
        heading_err = abs(math.remainder(end[2] - start[2], 2 * math.pi))
        along = dx * math.cos(end[2]) + dy * math.sin(end[2])
        across = -dx * math.sin(end[2]) + dy * math.cos(end[2])
        e = across ** 2 + (min(0.0, along - 0.3)) ** 2 + (heading_err * 0.5) ** 2
        # crossings: marks are paired (first pass / second pass)
        half = len(crosses) // 2
        for a, b in zip(crosses[:half], crosses[half:]):
            pa, pb = pts[a], pts[b]
            e += (pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2
            e += (abs(math.cos(pa[2] - pb[2])) * 0.5) ** 2
        return e, along

    def sample(segs, closing):
        """Line points every ~1 cm (with arc-length), closing straight included."""
        pts = []
        x = y = th = 0.0
        sacc = 0.0
        for sg in list(segs) + [("S", closing, 0.0)]:
            if sg[0] == "S":
                n = max(1, int(sg[1] / 0.01))
                for i in range(n):
                    d = sg[1] * i / n
                    pts.append((sacc + d, x + d * math.cos(th), y + d * math.sin(th)))
                x += sg[1] * math.cos(th)
                y += sg[1] * math.sin(th)
                sacc += sg[1]
            else:
                turn = 1 if sg[1] > 0 else -1
                phi = math.radians(sg[1])
                cx, cy = x - turn * R * math.sin(th), y + turn * R * math.cos(th)
                rx, ry = x - cx, y - cy
                L = abs(phi) * R
                n = max(1, int(L / 0.01))
                for i in range(n):
                    a = phi * i / n
                    pts.append((sacc + L * i / n, cx + rx * math.cos(a) - ry * math.sin(a), cy + rx * math.sin(a) + ry * math.cos(a)))
                x = cx + rx * math.cos(phi) - ry * math.sin(phi)
                y = cy + rx * math.sin(phi) + ry * math.cos(phi)
                th += phi
                sacc += L
        return pts, sacc

    def clash(segs, closing, cross_s):
        """Length of line that runs closer than 12 cm to another part of the course,
        excluding +-25 cm around the real crossings."""
        pts, total = sample(segs, closing)
        cell = 0.12
        grid = {}
        for k, (s, x, y) in enumerate(pts):
            grid.setdefault((int(math.floor(x / cell)), int(math.floor(y / cell))), []).append(k)
        bad = 0
        for k, (s, x, y) in enumerate(pts):
            if any(abs(s - c) < 0.25 for c in cross_s):
                continue
            gx, gy = int(math.floor(x / cell)), int(math.floor(y / cell))
            hit = False
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j in grid.get((gx + dx, gy + dy), ()):
                        s2, x2, y2 = pts[j]
                        ds = abs(s - s2)
                        ds = min(ds, total - ds)
                        if ds > 0.4 and (x - x2) ** 2 + (y - y2) ** 2 < 0.12 ** 2:
                            if not any(abs(s2 - c) < 0.25 for c in cross_s):
                                hit = True
                                break
                    if hit:
                        break
                if hit:
                    break
            bad += hit
        return bad * 0.01

    ranked = []
    for cost, segs in best:
        e, along = score(segs)
        ranked.append((e + 0.002 * cost, e, along, segs))
    ranked.sort(key=lambda r: r[0])
    # the geometric score alone allows courses that overlap themselves: re-rank the best
    # few hundred by how much of the line runs into other parts of the course
    rescored = []
    for tot, e, along, segs in ranked[:600]:
        pts = build(segs)
        cs = []
        sacc = 0.0
        for k, sg in enumerate(segs):
            if ans[k] == "C":
                cs.append(sacc)
            sacc += sg[1] if sg[0] == "S" else abs(math.radians(sg[1])) * R
        c = clash(segs, max(along, 0.3), cs)
        rescored.append((tot + 4.0 * c, e, along, segs, c))
    rescored.sort(key=lambda r: r[0])
    ranked = [(a, b, c, d) for a, b, c, d, _ in rescored]
    print("overlap [m] of the best:", ", ".join(f"{r[4]:.2f}" for r in rescored[:5]))
    for tot, e, along, segs in ranked[:5]:
        desc = " ".join(("S%.2f" % s[1]) if s[0] == "S" else ("A%+d" % s[1]) for s in segs)
        print(f"  geometric error {math.sqrt(e) * 1000:7.1f} mm  closing straight {along:.2f} m  {desc}")

    _, e, along, segs = ranked[0]
    # least-squares fix of the straight lengths so the course closes exactly and the
    # crossings coincide: tiny Gauss-Newton on the straights (arcs are fixed by the rules)
    sidx = [k for k, s in enumerate(segs) if s[0] == "S"]
    lens = [segs[k][1] for k in sidx] + [max(along, 0.3)]

    def residual(v):
        sg = list(segs)
        for j, k in enumerate(sidx):
            sg[k] = ("S", v[j], 0.0)
        sg.append(("S", v[-1], 0.0))
        pts = build(sg)
        r = [pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]]
        half = len(crosses) // 2
        for a, b in zip(crosses[:half], crosses[half:]):
            r += [pts[a][0] - pts[b][0], pts[a][1] - pts[b][1]]
        # stay close to the measured lengths
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
        # normal equations (J^T J) dv = -J^T r
        n = len(v)
        A = [[sum(J[i][m] * J[j][m] for m in range(len(r0))) for j in range(n)] for i in range(n)]
        g = [-sum(J[i][m] * r0[m] for m in range(len(r0))) for i in range(n)]
        for i in range(n):
            A[i][i] += 1e-9
        # Gaussian elimination
        for c in range(n):
            p = max(range(c, n), key=lambda q: abs(A[q][c]))
            A[c], A[p] = A[p], A[c]
            g[c], g[p] = g[p], g[c]
            for q in range(c + 1, n):
                f = A[q][c] / A[c][c]
                for w in range(c, n):
                    A[q][w] -= f * A[c][w]
                g[q] -= f * g[c]
        dv = [0.0] * n
        for c in range(n - 1, -1, -1):
            dv[c] = (g[c] - sum(A[c][w] * dv[w] for w in range(c + 1, n))) / A[c][c]
        v = [max(0.02, a + b) for a, b in zip(v, dv)]
        if max(abs(x) for x in dv) < 1e-7:
            break

    final = list(segs)
    for j, k in enumerate(sidx):
        final[k] = ("S", v[j], 0.0)
    closing = v[-1]
    print(f"after adjusting straights: closing straight {closing:.3f} m, "
          f"largest change {max(abs(a - b) for a, b in zip(v, lens)) * 1000:.1f} mm")

    # track JSON: s = 0 at the start mark. Segments from start, then the closing straight.
    out_segs = []
    for s in final:
        out_segs.append({"straight": round(s[1], 4)} if s[0] == "S" else {"arc": int(s[1])})
    # end marks: start mark at s=0 sits inside the closing straight (goal ... start), so put
    # the closing straight first and shift everything: start = closing length - gap_after_goal
    total_close = closing
    segs_json = [{"straight": round(total_close, 4)}] + out_segs
    track = {
        "name": name,
        "closed": True,
        "line_width": 0.02,
        "goal_s": 0.0,   # the closing straight runs goal -> start
        "start_s": round(total_close, 4),
        "auto_turn_marks": True,
        "segments": segs_json,
        "_source": f"rebuilt from {len(good)} logs by tools/track_from_logs.py",
    }
    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(track, fh, indent=2, ensure_ascii=False)
    print("wrote", out_path)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    name = "rebuilt"
    if "--name" in sys.argv:
        name = sys.argv[sys.argv.index("--name") + 1]
    main(args[0], args[1], args[2], name)
