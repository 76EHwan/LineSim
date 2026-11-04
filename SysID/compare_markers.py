ground_truth = [
    (0, "U"), (1, "L"), (2, "L"), (3, "L"), (4, "L"), (5, "R"), (6, "R"), (7, "R"),
    (8, "L"), (9, "R"), (10, "R"), (11, "L"), (12, "L"), (13, "L"), (14, "L"), (15, "R"),
    (16, "L"), (17, "L"), (18, "C"), (19, "R"), (20, "R"), (21, "C"), (22, "L"), (23, "L"),
    (24, "L"), (25, "R"), (26, "L"), (27, "L"), (28, "L"), (29, "L"), (30, "L"), (31, "R"),
    (32, "L"), (33, "R"), (34, "R"), (35, "U")
]

import os

CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drive_sysid.csv")
with open(CSV_PATH, "r", encoding="utf-8", errors="ignore") as f:
    recorded_raw = [line.strip() for line in f if line.startswith("# marker_")]

recorded = []
for r in recorded_raw:
    # Format: # marker_0=U,0.174,3.3
    # key=val,dist,yaw
    parts = r.split("=", 1)[1].split(",")
    mtype = parts[0].strip()
    dist = float(parts[1]) if len(parts) > 1 else 0.0
    yaw = float(parts[2]) if len(parts) > 2 else 0.0
    recorded.append((mtype, dist, yaw))

print(f"Ground Truth count: {len(ground_truth)}")
print(f"Recorded count:     {len(recorded)}")
print("\nRecorded markers:")
for idx, (mtype, dist, yaw) in enumerate(recorded):
    gt = ground_truth[idx] if idx < len(ground_truth) else ("?", "?")
    match = "MATCH" if gt[1] == mtype else f"MISMATCH (expected {gt[1]})"
    print(f"[{idx:02d}] Rec: {mtype:2s} dist={dist:.3f}m yaw={yaw:6.1f} | GT[{gt[0]}]: {gt[1]} -> {match}")
