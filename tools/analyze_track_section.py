import json
import math

with open('tracks/finals_1_5x_championship.json') as f:
    lines = [l for l in f if not l.strip().startswith('//')]
    data = json.loads(''.join(lines))

segs = data['segments']

def analyze_section(start_idx, count):
    print(f"=== Section {start_idx} to {start_idx+count-1} ===")
    x, y, th = 0.0, 0.0, 0.0
    r = 0.25 # ZETIN radius
    for i in range(start_idx, min(len(segs), start_idx + count)):
        sg = segs[i]
        if 'straight' in sg:
            L = sg['straight']
            x += L * math.cos(th)
            y += L * math.sin(th)
            print(f"[{i}] straight {L:.4f}m -> pos: ({x:.3f}, {y:.3f}), th: {math.degrees(th):.1f} deg")
        elif 'arc' in sg:
            deg = sg['arc']
            arc_r = sg.get('radius', r)
            rad = math.radians(deg)
            sign = 1 if deg > 0 else -1
            cx = x - sign * arc_r * math.sin(th)
            cy = y + sign * arc_r * math.cos(th)
            th_new = th + rad
            x = cx + sign * arc_r * math.sin(th_new)
            y = cy - sign * arc_r * math.cos(th_new)
            th = th_new
            print(f"[{i}] arc {deg} deg (R={arc_r}) -> pos: ({x:.3f}, {y:.3f}), th: {math.degrees(th):.1f} deg")

for s in [13, 25, 60, 72, 84, 124]:
    print(f"\n--- Section {s} ---")
    analyze_section(s, 10)
