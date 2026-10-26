"""Full.step -> LineSim robot (meshes + geometry).

    tools\\.venv\\Scripts\\python.exe tools\\step_import.py robots\\Full.step [--total-mass 0.4] [--view-cell 0.6]

Writes
  robots/mesh/<part group>.stl   visual meshes in the simulator's robot frame
  robots/full.json               robot parameters: geometry measured from the CAD model,
                                 dynamic values (mass, friction, ...) from robots/default.json

Robot frame: x forward, y left, z up, origin at the middle of the wheel axles (wheel-center
height), the same frame robot.cpp builds the MuJoCo model in.

The STEP model itself: y up, the line-sensor board at -z (front), wheels left/right along x.
"""
import json
import math
import os
import re
import sys
from collections import defaultdict

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# part-name pattern -> (group, rgba). First match wins.
GROUPS = [
    (r"^(Tire|wheel 13mm)", "tires", [0.08, 0.08, 0.08, 1]),
    (r"^(Wheel|Spur Gear|bearing|Wheel_Axis)", "wheels", [0.72, 0.72, 0.75, 1]),
    (r"^(GearFrame|0 \(5\)|connector)", "drive", [0.55, 0.57, 0.6, 1]),
    (r"^(GNB|battery)", "battery", [0.15, 0.3, 0.75, 1]),
    (r"^plate", "plate", [0.12, 0.12, 0.13, 1]),
    (r"^OPTO-SMD_ITR8307", "ir_sensors", [0.2, 0.2, 0.22, 1]),
    (r"^(Board|EasyEDA PCB Model|PCB|SENSOR NECK|Open CASCADE STEP translator)", "pcb", [0.05, 0.38, 0.18, 1]),
    (r".", "parts", [0.18, 0.18, 0.2, 1]),
]


def base_name(node):
    return re.sub(r"(:\d+.*$|_[0-9a-f]{6}$)", "", node)


def group_of(name):
    for pat, g, rgba in GROUPS:
        if re.search(pat, name):
            return g, rgba
    return "parts", GROUPS[-1][2]


def simplify(m, cell):
    """Vertex clustering: snap vertices to a `cell` grid, merge each cell, drop collapsed faces.
    Only for the visual meshes: the viewer draws them every frame (twice with the sensor view)."""
    q = np.floor(m.vertices / cell).astype(np.int64)
    _, inv, cnt = np.unique(q, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    v = np.zeros((len(cnt), 3))
    np.add.at(v, inv, m.vertices)
    v /= cnt[:, None]
    f = inv[m.faces]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    f = f[np.unique(np.sort(f, axis=1), axis=0, return_index=True)[1]]   # same triangle twice
    out = trimesh.Trimesh(v, f, process=False)
    out.remove_unreferenced_vertices()
    return out


def clusters(points, tol):
    """Greedy clustering of 3D points; returns list of centroid arrays."""
    out = []
    for p in points:
        for c in out:
            if np.linalg.norm(c["sum"] / c["n"] - p) < tol:
                c["sum"] += p
                c["n"] += 1
                break
        else:
            out.append({"sum": p.copy(), "n": 1})
    return [c["sum"] / c["n"] for c in out]


def main(step_path):
    glb = os.path.join(ROOT, "runs", "stepconv", os.path.splitext(os.path.basename(step_path))[0] + ".glb")
    os.makedirs(os.path.dirname(glb), exist_ok=True)
    if not os.path.exists(glb) or os.path.getmtime(glb) < os.path.getmtime(step_path):
        import cascadio
        print("tessellating", step_path, "(takes a few minutes) ...")
        cascadio.step_to_glb(step_path, glb, 0.1, 0.5)   # STEP is in mm: 0.1 mm chord tolerance
    scene = trimesh.load(glb, force="scene")

    parts = []   # (name, mesh in STEP frame)
    for node in scene.graph.nodes_geometry:
        T, g = scene.graph[node]
        m = scene.geometry[g].copy()
        m.apply_transform(T)
        parts.append((base_name(node), m))

    # ---- frame: from the four tires and sensors ----
    tires = [m for n, m in parts if n.startswith("Tire") or "wheel 13mm" in n]
    if len(tires) not in (2, 4):
        sys.exit(f"expected 2 or 4 parts named Tire*, found {len(tires)}")
    tc = np.array([t.bounds.mean(0) for t in tires])
    origin = tc.mean(0)                       # middle of the axles, wheel-center height

    # Automatically detect CAD orientation axes:
    # 1. Axle axis is the one with the largest spread among wheel centers (track ~187mm)
    span = np.ptp(tc, axis=0)
    axle_axis = int(np.argmax(span))

    # 2. Forward axis points towards the IR sensors (or sensor board / neck)
    sens_parts = [m for n, m in parts if "ITR8307" in n or "OPTO" in n or "SENSOR NECK" in n or "Board~p9Sd" in n]
    if sens_parts:
        sensor_center = np.mean([m.bounds.mean(0) for m in sens_parts], axis=0)
        fwd_vec = sensor_center - origin
        fwd_vec[axle_axis] = 0.0
        fwd_axis = int(np.argmax(np.abs(fwd_vec)))
        fwd_sign = 1.0 if fwd_vec[fwd_axis] > 0 else -1.0
    else:
        fwd_axis = 2
        fwd_sign = -1.0

    # 3. Vertical axis is the remaining axis (upwards from tires towards PCB/battery)
    vert_axis = [i for i in (0, 1, 2) if i not in (axle_axis, fwd_axis)][0]
    board_parts = [m for n, m in parts if "Board~ypt3" in n or "GNB" in n or "plate" in n]
    if board_parts:
        vert_vec = np.mean([m.bounds.mean(0) for m in board_parts], axis=0) - origin
        vert_sign = 1.0 if vert_vec[vert_axis] > 0 else -1.0
    else:
        vert_sign = 1.0

    u_fwd = np.zeros(3)
    u_fwd[fwd_axis] = fwd_sign

    u_up = np.zeros(3)
    u_up[vert_axis] = vert_sign

    u_left = np.cross(u_up, u_fwd)

    R3 = np.vstack([u_fwd, u_left, u_up])
    R = np.eye(4)
    R[:3, :3] = R3
    Tm = R @ trimesh.transformations.translation_matrix(-origin)

    def to_robot(p):
        p = np.asarray(p, dtype=float) - origin
        return p @ R3.T

    # Dimensions in robot frame
    tire_radii, tire_widths = [], []
    for t in tires:
        vr = to_robot(t.vertices)
        ex = np.ptp(vr, axis=0)
        tire_radii.append(max(ex[0], ex[2]) / 2.0)
        tire_widths.append(ex[1])
    radius = float(np.mean(tire_radii))
    width = float(np.mean(tire_widths))

    wheels = to_robot(tc)
    wheels = wheels[np.lexsort((-wheels[:, 0], -wheels[:, 1]))]   # left first, front first
    track = float(np.mean(np.abs(wheels[:, 1])) * 2)

    # ---- IR sensors ----
    opto = [m for n, m in parts if n.startswith("OPTO-SMD_ITR8307")]
    sens = []
    if opto:
        cents = clusters([to_robot(m.bounds.mean(0)) for m in opto], 0.004)
        sens = sorted(cents, key=lambda c: -c[1])   # left (+y) first
    ground_z = float(min(to_robot(t.vertices)[:, 2].min() for t in tires))   # tire bottom, robot frame z

    # ---- meshes per group ----
    mesh_dir = os.path.join(ROOT, "robots", "mesh")
    os.makedirs(mesh_dir, exist_ok=True)
    grouped = defaultdict(list)
    rgba_of = {}
    skipped = 0
    for n, m in parts:
        g, rgba = group_of(n)
        if g == "parts" and max(m.extents) < 0.004:   # 0402/0603 chips: invisible, but most of the faces
            skipped += 1
            continue
        mm = m.copy()
        mm.apply_transform(Tm)
        grouped[g].append(mm)
        rgba_of[g] = rgba
    for old in os.listdir(mesh_dir):
        if old.endswith(".stl"):
            os.remove(os.path.join(mesh_dir, old))
    visual = []
    total_drawn = 0
    MAX_FACES = 150000   # MuJoCo's STL reader accepts up to 200k faces per file
    for g, ms in grouped.items():
        chunks, cur, nf = [], [], 0
        for m in ms:
            if cur and nf + len(m.faces) > MAX_FACES:
                chunks.append(cur)
                cur, nf = [], 0
            cur.append(m)
            nf += len(m.faces)
        chunks.append(cur)
        shown = 0
        for k, ch in enumerate(chunks):
            merged = trimesh.util.concatenate(ch)
            if VIEW_CELL > 0:
                merged = simplify(merged, VIEW_CELL)
            f = os.path.join(mesh_dir, g + (f"_{k}" if len(chunks) > 1 else "") + ".stl")
            merged.export(f)
            shown += len(merged.faces)
            visual.append({"file": os.path.relpath(f, os.path.join(ROOT, "robots")).replace("\\", "/"), "rgba": rgba_of[g]})
        print(f"  mesh {g:11s} {len(ms):4d} parts {sum(len(m.faces) for m in ms):7d} faces in {len(chunks)} file(s), "
              f"{shown} drawn")
        total_drawn += shown
    print(f"  ({skipped} tiny chip parts left out of the visual mesh; {total_drawn} faces drawn, "
          f"--view-cell {VIEW_CELL * 1000:g} mm)")

    # ---- collision boxes for parts that can touch the floor ----
    def box_of(pred):
        ms = [m for n, m in parts if pred(n)]
        if not ms:
            return None
        b = np.vstack([to_robot(m.vertices) for m in ms])
        lo, hi = b.min(0), b.max(0)
        c, h = (lo + hi) / 2, (hi - lo) / 2
        return [round(float(v), 5) for v in list(c) + list(h)]

    boxes = [b for b in (box_of(lambda n: n == "plate"),
                         box_of(lambda n: n.startswith("Board~p9Sd")),
                         box_of(lambda n: n.startswith("SENSOR NECK"))) if b]

    # ---- assemble robot json: dynamics from default.json, geometry from CAD ----
    with open(os.path.join(ROOT, "robots", "default.json"), encoding="utf-8") as fh:
        base = json.loads(re.sub(r"//[^\n]*", "", fh.read()))
    base.pop("skids", None)
    out = dict(base)
    out["wheel_radius"] = round(radius, 5)
    out["wheel_width"] = round(width, 5)
    out["track"] = round(track, 5)
    out["wheels"] = [[round(float(w[0]), 5), round(float(w[1]), 5)] for w in wheels]
    out["skids"] = []
    out["collision_boxes"] = boxes
    out["visual_meshes"] = visual
    if sens:
        # line sensors 0..15, left (+y) to right, as the firmware numbers them (sensor.c)
        out["ir_sensors"] = [[round(float(s[0]), 5), round(float(s[1]), 5)] for s in sens]
        out["sensor_x"] = round(float(max(s[0] for s in sens)), 5)
        out["ir_sensor_height"] = round(float(np.mean([s[2] for s in sens]) - ground_z), 5)

    # Mass properties. The STEP has no materials, so each part group gets a typical mass
    # (ASSUMED_MASS) spread over its parts by convex-hull volume. Replace with the measured
    # total mass / balance point when available (robots/full.json "mass", "com").
    ASSUMED_MASS = {"battery": 0.035, "drive": 0.070, "wheels": 0.016, "tires": 0.008,
                    "plate": 0.010, "pcb": 0.018, "ir_sensors": 0.002, "parts": 0.010}
    if TOTAL_MASS:   # measured total: scale every group, keeps the relative distribution
        k = TOTAL_MASS / sum(ASSUMED_MASS.values())
        ASSUMED_MASS = {g: v * k for g, v in ASSUMED_MASS.items()}
    gvol = defaultdict(float)
    pinfo = []
    for n, m in parts:
        g, _ = group_of(n)
        try:
            v = max(float(m.convex_hull.volume), 1e-12)
        except Exception:
            v = 1e-12
        gvol[g] += v
        vr = to_robot(m.vertices)
        pinfo.append((g, v, vr.mean(0), np.ptp(vr, axis=0)))
    pts = [(ASSUMED_MASS.get(g, 0.0) * v / gvol[g], c, e) for g, v, c, e in pinfo if gvol[g] > 0]
    wheel_mass = (ASSUMED_MASS["wheels"] + ASSUMED_MASS["tires"]) / len(tires)
    body = [(m, c, e) for (m, c, e), (g, *_ ) in zip(pts, pinfo) if g not in ("wheels", "tires")]
    mb = sum(m for m, _, _ in body)
    com = sum(m * c for m, c, _ in body) / mb
    I = np.zeros((3, 3))
    for m, c, e in body:
        r = c - com
        I += m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
        I += np.diag([m * (e[1] ** 2 + e[2] ** 2), m * (e[0] ** 2 + e[2] ** 2), m * (e[0] ** 2 + e[1] ** 2)]) / 12
    out["mass"] = round(float(mb), 4)
    out["wheel_mass"] = round(float(wheel_mass), 4)
    com[1] = 0.0   # assume left/right symmetry (part names alone can't place every gear frame)
    out["com"] = [round(float(c), 4) for c in com]
    out["inertia"] = [float(f"{v:.3e}") for v in np.diag(I)]
    out["mass_note"] = ("total mass measured (%.3f kg), split over the parts by typical part masses" % TOTAL_MASS
                        if TOTAL_MASS else "estimated from typical part masses (tools/step_import.py ASSUMED_MASS)")
    out.pop("body_box", None)
    with open(os.path.join(ROOT, "robots", "full.json"), "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)

    print()
    print(f"wheel radius {radius * 1000:.2f} mm, width {width * 1000:.1f} mm, track {track * 1000:.1f} mm")
    for w in wheels:
        print(f"  wheel at x {w[0] * 1000:+7.1f} mm  y {w[1] * 1000:+7.1f} mm  z {w[2] * 1000:+6.1f} mm")
    if sens:
        print(f"IR sensors: {len(sens)} found, {out['ir_sensor_height'] * 1000:.1f} mm above the floor")
        for i, s in enumerate(sens):
            print(f"  sensor {i:2d}: x {s[0] * 1000:6.1f} mm  y {s[1] * 1000:+6.1f} mm")
    if "com" in out:
        print(f"body mass {out['mass'] * 1000:.0f} g (+ 4 x {out['wheel_mass'] * 1000:.0f} g wheels), "
              f"center of mass {np.round(np.array(out['com']) * 1000, 1)} mm, inertia {out['inertia']}  [ESTIMATE]")
    for b in boxes:
        print(f"  collision box center {np.round(np.array(b[:3]) * 1000, 1)} mm, half size {np.round(np.array(b[3:]) * 1000, 1)} mm")
    print("wrote robots/full.json and robots/mesh/*.stl")


TOTAL_MASS = None
VIEW_CELL = 0.0006   # visual mesh simplification grid [m]; 0 keeps the full tessellation

if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if "--total-mass" in sys.argv:   # measured robot mass in kg, e.g. --total-mass 0.4
        TOTAL_MASS = float(sys.argv[sys.argv.index("--total-mass") + 1])
        args = [a for a in args if a != sys.argv[sys.argv.index("--total-mass") + 1]]
    if "--view-cell" in sys.argv:    # mm, e.g. --view-cell 0.4 (finer) or 0 (full detail)
        VIEW_CELL = float(sys.argv[sys.argv.index("--view-cell") + 1]) / 1000
        args = [a for a in args if a != sys.argv[sys.argv.index("--view-cell") + 1]]
    main(args[0] if args else os.path.join(ROOT, "robots", "Full.step"))
