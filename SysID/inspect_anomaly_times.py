import csv
import os

CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drive_sysid.csv")
with open(CSV_PATH, "r", encoding="utf-8", errors="ignore") as f:
    lines = [l for l in f if not l.startswith("#") and l.strip()]

rows = list(csv.DictReader(lines))

# Find the timestamps where marker_cnt increments
marker_events = []
prev_cnt = 0
for r in rows:
    cnt = int(float(r["marker_cnt"]))
    if cnt > prev_cnt:
        marker_events.append((cnt - 1, float(r["t_ms"]), float(r["odom_m"]), float(r["line_pos"]), float(r["gyro_z_dps"]), float(r["mps_L"]), float(r["mps_R"])))
        prev_cnt = cnt

print("Marker increment events around anomalies:")
for idx, t_ms, odom, lp, gz, vL, vR in marker_events:
    if idx in (17, 18, 19, 22, 23, 24, 32, 33, 34, 35):
        print(f"Marker {idx:2d}: t={t_ms:7.1f}ms odom={odom:6.3f}m line_pos={lp:6.3f} gyro_z={gz:6.1f} v=({vL:.2f},{vR:.2f})")
