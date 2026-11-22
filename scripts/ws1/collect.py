#!/usr/bin/env python3
"""Collect the summary.json files an array job left behind into one CSV.

    python3 scripts/ws1/collect.py ~/linesim-runs/<jobid>            # table on stdout
    python3 scripts/ws1/collect.py ~/linesim-runs/<jobid> -o out.csv

Only drive 1 is read: these jobs run the 1st run only.
"""
import csv
import json
import os
import sys

PARAM_COLS = ["base_mps", "steer_kp", "steer_kd", "pos_atten_gain", "target_shift", "accel", "decel"]
DRIVE_COLS = ["outcome", "lap_time_fw", "distance_m", "max_speed",
              "max_lat_mm", "rms_lat_mm", "max_beta_deg", "marker_check"]


def rows(root):
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name, "summary.json")
        if not os.path.isfile(path):
            continue
        try:
            with open(path, encoding="utf-8") as fh:
                s = json.load(fh)
        except (OSError, ValueError) as e:
            print(f"skipped {name}: {e}", file=sys.stderr)
            continue
        drives = s.get("drives") or [{}]
        d = drives[0]
        row = {
            "task": name,
            "ok": s.get("ok"),
            "track": s.get("track"),
            "track_length_m": s.get("track_length_m"),
            "sim_seconds": s.get("sim_seconds"),
            "wall_seconds": s.get("wall_seconds"),
        }
        row.update({c: d.get(c) for c in DRIVE_COLS})
        row.update({c: (s.get("params") or {}).get(c) for c in PARAM_COLS})
        yield row


def main():
    args = [a for a in sys.argv[1:]]
    out_path = None
    if "-o" in args:
        i = args.index("-o")
        out_path = args[i + 1]
        del args[i:i + 2]
    if not args:
        print(__doc__, file=sys.stderr)
        return 2
    root = os.path.expanduser(args[0])

    data = list(rows(root))
    if not data:
        print(f"no summary.json under {root}", file=sys.stderr)
        return 1
    cols = list(data[0].keys())

    if out_path:
        with open(out_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            w.writerows(data)
        print(f"{len(data)} run(s) -> {out_path}")

    finished = [r for r in data if str(r["outcome"]).startswith("finished")]
    clean = [r for r in finished if r["marker_check"] == "OK"]
    print(f"\n{len(data)} run(s): {len(finished)} finished, {len(clean)} of those with marker_check OK")
    if clean:
        clean.sort(key=lambda r: r["lap_time_fw"] or 9e9)
        print(f"\n{'task':>6} {'lap [s]':>8} {'maxlat':>7} {'maxbeta':>8} {'v_base':>7} {'kp':>6} {'kd':>7}  outcome")
        for r in clean[:20]:
            print(f"{r['task']:>6} {r['lap_time_fw']:>8.2f} {r['max_lat_mm']:>7.1f} {r['max_beta_deg']:>8.1f} "
                  f"{r['base_mps']:>7.2f} {r['steer_kp']:>6.1f} {r['steer_kd']:>7.0f}  {r['outcome']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
