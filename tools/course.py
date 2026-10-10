"""Course geometry helper for ZETIN-rule tracks (R = 0.25 m arcs, 45 deg steps).

    python tools/course.py tracks/zetin_sample.json

Prints the end pose (must equal the start pose for a closed course), the total
length, the bounding box, every self-crossing (angle and straight length on
both sides of the crossing center) and the straight lengths around the end marks.
"""
import json, math, re, sys

R_RULE = 0.25

def load(path):
    text = open(path, encoding="utf-8").read()
    text = re.sub(r"//[^\n]*", "", text)
    return json.loads(text)

def pieces(segs, x=0.0, y=0.0, th=0.0):
    out, s = [], 0.0
    for sg in segs:
        if "straight" in sg:
            L = sg["straight"]
            out.append(("S", s, L, x, y, th, 0))
            x += L * math.cos(th); y += L * math.sin(th); s += L
        else:
            deg = sg["arc"]; R = sg.get("radius", R_RULE); turn = 1 if deg > 0 else -1
            L = abs(deg) * math.pi / 180 * R
            out.append(("A", s, L, x, y, th, turn, R))
            cx = x - turn * R * math.sin(th); cy = y + turn * R * math.cos(th)
            phi = turn * abs(deg) * math.pi / 180
            rx, ry = x - cx, y - cy
            x = cx + rx * math.cos(phi) - ry * math.sin(phi)
            y = cy + rx * math.sin(phi) + ry * math.cos(phi)
            th += phi; s += L
    return out, (x, y, th), s

def sample(ps, step=0.002):
    pts = []
    for p in ps:
        n = max(1, int(p[2] / step))
        for k in range(n):
            d = p[2] * k / n
            if p[0] == "S":
                pts.append((p[1] + d, p[3] + d * math.cos(p[5]), p[4] + d * math.sin(p[5]), p[5], 0))
            else:
                turn, R = p[6], p[7]
                cx = p[3] - turn * R * math.sin(p[5]); cy = p[4] + turn * R * math.cos(p[5])
                phi = turn * d / R
                rx, ry = p[3] - cx, p[4] - cy
                pts.append((p[1] + d, cx + rx * math.cos(phi) - ry * math.sin(phi),
                            cy + rx * math.sin(phi) + ry * math.cos(phi), p[5] + phi, 1))
    return pts

def straight_run(ps, s, total):
    """Straight length before / after arc-length s (0 if s is on an arc)."""
    s %= total
    for i, p in enumerate(ps):
        if p[1] <= s <= p[1] + p[2]:
            if p[0] != "S":
                return 0.0, 0.0
            before, after = s - p[1], p[1] + p[2] - s
            j = i - 1
            while ps[j % len(ps)][0] == "S" and j > i - len(ps):
                before += ps[j % len(ps)][2]; j -= 1
            j = i + 1
            while ps[j % len(ps)][0] == "S" and j < i + len(ps):
                after += ps[j % len(ps)][2]; j += 1
            return before, after
    return 0.0, 0.0

def main(path):
    j = load(path)
    o = j.get("origin", [0, 0, 0])
    ps, end, total = pieces(j["segments"], o[0], o[1], math.radians(o[2]))
    dx, dy = end[0] - o[0], end[1] - o[1]
    dth = (math.degrees(end[2]) - o[2] + 180) % 360 - 180
    print(f"length {total:.3f} m, closure error {1000 * math.hypot(dx, dy):.2f} mm, heading error {dth:.3f} deg")
    pts = sample(ps)
    xs = [p[1] for p in pts]; ys = [p[2] for p in pts]
    print(f"bbox {min(xs):.3f}..{max(xs):.3f} x {min(ys):.3f}..{max(ys):.3f}  ({max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f} m)")
    for sg in j["segments"]:
        if "arc" in sg and (sg.get("radius", R_RULE) != R_RULE or abs(sg["arc"]) % 45 or not 45 <= abs(sg["arc"]) <= 270):
            print("  RULE: arc", sg)
    # self crossings
    found = []
    for a in range(0, len(pts), 5):
        for b in range(a + 50, len(pts), 5):
            if abs(pts[a][0] - pts[b][0]) < 0.3 or total - abs(pts[a][0] - pts[b][0]) < 0.3:
                continue
            if math.hypot(pts[a][1] - pts[b][1], pts[a][2] - pts[b][2]) < 0.006:
                if not any(abs(f[0] - pts[a][0]) < 0.1 for f in found):
                    found.append((pts[a][0], pts[b][0], pts[a][1], pts[a][2], pts[a][3], pts[b][3]))
    for sa, sb, x, y, ta, tb in found:
        ang = abs((math.degrees(ta - tb) + 180) % 360 - 180)
        ba, aa = straight_run(ps, sa, total); bb, ab = straight_run(ps, sb, total)
        print(f"crossing at ({x:.3f},{y:.3f}) s={sa:.3f}/{sb:.3f} angle {ang:.1f} deg, "
              f"straight {ba:.2f}/{aa:.2f} m and {bb:.2f}/{ab:.2f} m")
    for name in ("start_s", "goal_s"):
        if name in j:
            b, a = straight_run(ps, j[name], total)
            print(f"{name} {j[name]}: straight {b:.3f} m before, {a:.3f} m after")

if __name__ == "__main__":
    main(sys.argv[1])
