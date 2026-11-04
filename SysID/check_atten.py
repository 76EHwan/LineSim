import csv
import math
import os

CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drive_sysid.csv")
with open(CSV_PATH, "r", encoding="utf-8", errors="ignore") as f:
    lines = [l for l in f if not l.startswith("#") and l.strip()]

rows = list(csv.DictReader(lines))

atten_cuts = []
for r in rows:
    v = 0.5 * (float(r["mps_L"]) + float(r["mps_R"]))
    gz = abs(float(r["gyro_z_dps"])) * math.pi / 180.0
    alat = v * gz
    if alat > 12.82:
        atten_cuts.append((float(r["t_ms"]), v, alat, 12.82 / alat))

print(f"Total time steps where a_lat > 12.82: {len(atten_cuts)} / {len(rows)} ({len(atten_cuts)/len(rows)*100:.1f}%)")
if atten_cuts:
    min_cut = min(x[3] for x in atten_cuts)
    print(f"Worst speed cut ratio: {min_cut:.2f} (Speed forced down to {min_cut*100:.1f}%)")
