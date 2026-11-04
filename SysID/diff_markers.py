import difflib
import os

gt = [
    "U", "L", "L", "L", "L", "R", "R", "R",
    "L", "R", "R", "L", "L", "L", "L", "R",
    "L", "L", "C", "R", "R", "C", "L", "L",
    "L", "R", "L", "L", "L", "L", "L", "R",
    "L", "R", "R", "U"
]

CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drive_sysid.csv")
with open(CSV_PATH, "r", encoding="utf-8", errors="ignore") as f:
    recorded_raw = [line.strip() for line in f if line.startswith("# marker_")]

rec = []
rec_details = []
for r in recorded_raw:
    parts = r.split("=", 1)[1].split(",")
    mtype = parts[0].strip()
    dist = float(parts[1]) if len(parts) > 1 else 0.0
    yaw = float(parts[2]) if len(parts) > 2 else 0.0
    rec.append(mtype)
    rec_details.append((mtype, dist, yaw))

matcher = difflib.SequenceMatcher(None, gt, rec)
for tag, i1, i2, j1, j2 in matcher.get_opcodes():
    print(f"\n--- {tag.upper()} ---")
    print(f"GT[{i1}:{i2}]  = {gt[i1:i2]}")
    print(f"REC[{j1}:{j2}] = {[rec_details[k] for k in range(j1, j2)]}")
