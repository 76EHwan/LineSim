"""Base speed and steering gains for the race, found in the simulator.

    python tools/autotune.py 2.0 --hours 3       endless tuning for 3 hours with rotating random courses
    python tools/autotune.py 2.0 --endless       endless tuning continuously until Ctrl+C
    python tools/autotune.py 1.6 --max-speed     highest base speed that passes, from 1.6 up
    python tools/autotune.py 2.2 --max-speed --lat 10,12,14,16   with the grip limit (lat_acc_max) as a 3rd axis

    options: [--hours <N>] [--endless] [--drives 1,3] [--track tracks/x.json ...] [--courses 4]
             [--robot robots/full.json] [--jobs 8] [--kp 8,10,...] [--kd 0,0.05,...] [--lat 0]
             [--set max_mps=4 ...] [--reuse] [--apply]

Course: the real practice board (tracks/practice_board.json, made from the robot's logs) when it
is there, else --courses random ZETIN courses with fixed seeds. --track picks courses by hand.

Every kp x kd pair of the grid runs drive 1 (learning: markers and distances must come out clean)
and then drive 4 (or drive 2 if specified via --drives 1,2: accelerates on the straights from drive 1's data;
max_mps / accel / decel from --set or the firmware defaults). A pair passes when on every course both drives finish,
never more than MAX_LAT_MM off the line, with every marker read right. Score (lower is better): 5 x fast drive
lap time [s] + drive 1 cleanliness (line rms [mm] + weave [dps] / 20 + wheel slip [%] / 2).
A pass must also hold with more steering lag than the model has: the lag (robot sensor_delay,
fitted to one gyro log) is not known exactly, so drive 1 runs again with it x1.4 (must keep the
line, weave at most 1.5 x nominal + 10 dps) and x1.85 (must keep the line). Gains that only work
for the exact fitted lag count as unstable. Grip is not known exactly either: drive 1 also runs
with the tire friction x0.85 and must still finish with every marker right.
The pick is among the passing points whose grid neighbours are stable too: within 1 % of the
best score, the lowest kp, then the lowest kd, then the lowest lat_acc_max.

--lat adds the grip limit (firmware feat/cascade-steer: driveData.lat_acc_max [m/s^2], slows down
while speed x gyro yaw rate exceeds it) as a third grid axis. 0 = off (firmware without it: 0 only).

Coarse to fine (--refine 3, the default): a coarse grid (kp 12/18/25/32 x kd 0-0.4 s x the --lat
values), then 3 times a 3x3x3 grid around the best point with half the step each time (kp +-3.5,
1.75, 0.9; kd +-0.05, 0.025, 0.0125 s; lat +-1.5 ...). The same stability rules hold on every level.
The pick is then driven with 2 more seeds and must pass every time. --refine 0: the fixed grid.

--max-speed raises the base speed while some pair passes (+0.2, then halving the step on a miss,
down to 0.01 m/s) and reports the highest speed with its gains.

kd is in seconds (derivative gain on de/dt). The firmware's steer_pid is CMSIS arm_pid_f32, called
once per steering period (TIM14, 500 us from Core/Src/tim.c), so its Kd multiplies the error change
per call: the value for the robot's menu (driveData.steer_gain_d) is kd / 0.0005, e.g. 0.3 s -> 600.

Writes runs/tune/<speed>/result.txt and result.json (--max-speed: runs/tune/max_speed.txt too).
--reuse keeps runs that already have a result (to re-score after changing this script).
"""
import concurrent.futures
import csv
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.join(ROOT, "build", "Release", "linesim.exe")


def update_drive_c(base_mps, kp, kd_fw, lat_acc=None):
    """Safely updates firmware/Main/Src/drive.c with newly discovered best parameters."""
    path = os.path.join(ROOT, "firmware", "Main", "Src", "drive.c")
    if not os.path.exists(path):
        return False
    try:
        with open(path, "r", encoding="utf-8") as fh:
            text = fh.read()
        text = re.sub(r"(\.base_mps\s*=\s*)[0-9.]+f?", rf"\g<1>{base_mps:.2f}f", text)
        text = re.sub(r"(\.steer_gain_p\s*=\s*)[0-9.]+f?", rf"\g<1>{kp:.3f}f", text)
        text = re.sub(r"(\.steer_gain_d\s*=\s*)[0-9.]+f?", rf"\g<1>{kd_fw:.0f}.f", text)
        if lat_acc is not None and lat_acc > 0:
            text = re.sub(r"(\.lat_acc_max\s*=\s*)[0-9.]+f?", rf"\g<1>{lat_acc:.2f}f", text)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)
        return True
    except Exception as e:
        print(f"Warning: failed to update {path}: {e}")
        return False


def use_private_exe():
    """Run a copy of the simulator, so a rebuild during a long search does not hit a locked file
    (and the search keeps one build from start to end)."""
    global EXE
    import shutil
    src = os.path.dirname(EXE)
    dst = os.path.join(ROOT, "runs", "tune", f"bin_{os.getpid()}")
    os.makedirs(dst, exist_ok=True)
    for f in ("linesim.exe", "mujoco.dll"):
        shutil.copy2(os.path.join(src, f), os.path.join(dst, f))
    EXE = os.path.join(dst, "linesim.exe")
REAL_COURSE = os.path.join(ROOT, "tracks", "practice_board.json")

KP = [8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40]
KD = [0, 0.05, 0.1, 0.15, 0.225, 0.3, 0.4, 0.5]   # seconds (x 2000 = firmware steer_gain_d)
KP_COARSE = [12, 18, 25, 32]          # first grid of the coarse-to-fine search (--refine)
KD_COARSE = [0, 0.1, 0.2, 0.3, 0.4]
SEED_CHECK = (2, 3)                   # the pick is driven again with these seeds (seed 1 = the grid)
_RAN = set()                          # output folders run in this process (refinement grids overlap)
SEEDS = [101, 102, 103, 104, 105, 106, 107, 108]
STRESS_LAG = (1.4, 1.85)   # sensor_delay factors of the stress drives
STRESS_MU = 0.85           # tire friction factor of the grip stress drive
# Maximum allowable distance off the center line [mm]
MAX_LAT_MM = 70.0
KEEP_RUNS = False


def clean_run_dir(d):
    """Remove heavy simulation artifacts (CSV traces, HTML viewer, SD card logs)
    while keeping summary.json so that disk usage stays minimal."""
    global KEEP_RUNS
    if KEEP_RUNS or not d or not os.path.isdir(d):
        return
    for item in os.listdir(d):
        p = os.path.join(d, item)
        if item.endswith((".csv", ".html", ".xml", ".bmp", ".svg")) or item in ("sd", "sdcard"):
            if os.path.isdir(p):
                shutil.rmtree(p, ignore_errors=True)
            else:
                try:
                    os.remove(p)
                except OSError:
                    pass


def _cache_metrics(out_dir, weave, slip):
    if not out_dir:
        return
    sj_path = os.path.join(out_dir, "summary.json")
    if os.path.exists(sj_path):
        try:
            with open(sj_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            data["_cached_weave"] = weave
            data["_cached_slip"] = slip
            with open(sj_path, "w", encoding="utf-8") as fh:
                json.dump(data, fh)
        except Exception:
            pass


class StreakTracker:
    """Tracks continuous consecutive successes for gain parameter combinations."""
    def __init__(self):
        self.gain_stats = {}  # key (kp, kd, lat) -> dict
        self.current_gain = None
        self.current_streak = 0
        self.best_streak_gain = None
        self.best_streak_count = 0
        self.best_streak_data = None

    def record_result(self, pick, passed, speed=None, score=None, avg_lap=None, kd_fw=0, round_id=None):
        if not pick or not passed:
            self.current_streak = 0
            self.current_gain = None
            return

        key = (round(pick[0], 3), round(pick[1], 4), round(pick[2], 2) if len(pick) > 2 else 0.0)
        if key not in self.gain_stats:
            self.gain_stats[key] = {
                "kp": pick[0],
                "kd_s": pick[1],
                "lat_acc_max": pick[2] if len(pick) > 2 else 0.0,
                "kd_fw": kd_fw,
                "current_streak": 0,
                "max_streak": 0,
                "total_passes": 0,
                "speeds_passed": [],
                "best_score": score if score is not None else 999.0,
                "best_lap": avg_lap if avg_lap is not None else 999.0,
                "rounds_passed": []
            }
        st = self.gain_stats[key]
        st["total_passes"] += 1
        if speed is not None and speed not in st["speeds_passed"]:
            st["speeds_passed"].append(speed)
        if score is not None and score < st["best_score"]:
            st["best_score"] = score
        if avg_lap is not None and avg_lap < st["best_lap"]:
            st["best_lap"] = avg_lap
        if round_id is not None:
            st["rounds_passed"].append(round_id)

        if key == self.current_gain:
            self.current_streak += 1
        else:
            self.current_gain = key
            self.current_streak = 1

        st["current_streak"] = self.current_streak
        if self.current_streak > st["max_streak"]:
            st["max_streak"] = self.current_streak

        if self.current_streak > self.best_streak_count:
            self.best_streak_count = self.current_streak
            self.best_streak_gain = key
            self.best_streak_data = {
                "consecutive_passes": self.best_streak_count,
                "steer_kp": pick[0],
                "steer_kd_s": pick[1],
                "steer_gain_d": kd_fw,
                "lat_acc_max": pick[2] if len(pick) > 2 else 0.0,
                "speeds_passed": list(st["speeds_passed"]),
                "best_lap_time": st["best_lap"],
                "best_score": st["best_score"],
                "round_achieved": round_id
            }

    def get_best_streak(self):
        return self.best_streak_data


class SpeedOptimizer:
    """Tracks base speed vs performance (lap time, wheel slip %, line tracking error) to find
    the true optimal base speed rather than monotonically ratcheting speed upward until failure."""
    def __init__(self, initial_speed, step=0.03, min_speed=1.5, max_speed=3.2):
        self.initial_speed = initial_speed
        self.current_speed = initial_speed
        self.step = step
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.best_speed = initial_speed
        self.best_score = float('inf')
        self.best_lap = float('inf')
        self.best_slip = float('inf')
        self.history = {} # speed -> list of dicts: {"round", "passed", "score", "lap", "slip", "rms"}
        self.failed_ceiling = max_speed

    def record_run(self, speed, passed, score=None, lap=None, slip=None, rms=None, round_id=None):
        speed = round(speed, 3)
        if speed not in self.history:
            self.history[speed] = []
        entry = {
            "round": round_id,
            "passed": passed,
            "score": score,
            "lap": lap,
            "slip": slip,
            "rms": rms
        }
        self.history[speed].append(entry)

        # Penalize excessive slip: beyond 9.5%, slip rapidly increases corner washout and energy loss
        excess_slip_penalty = 0.0
        if slip is not None:
            if slip > 9.5:
                excess_slip_penalty += (slip - 9.5) * 1.8
            if slip > 14.0:
                excess_slip_penalty += (slip - 14.0) * 3.5

        adj_score = (score + excess_slip_penalty) if score is not None else float('inf')

        is_new_optimum = False
        if passed and adj_score < float('inf'):
            # Clear performance improvement (faster lap and/or significantly cleaner line)
            if adj_score < self.best_score - 0.4:
                is_new_optimum = True
            # Similar pace, but lower slip and tighter line (user preference: slip reduction)
            elif abs(adj_score - self.best_score) <= 0.4 and (slip is not None and slip < self.best_slip - 0.4):
                is_new_optimum = True
            elif self.best_score == float('inf'):
                is_new_optimum = True

            if is_new_optimum:
                self.best_score = adj_score
                self.best_speed = speed
                if lap is not None:
                    self.best_lap = lap
                if slip is not None:
                    self.best_slip = slip

        if not passed:
            if speed >= self.best_speed:
                self.failed_ceiling = min(self.failed_ceiling, speed)

        return is_new_optimum, adj_score

    def next_speed(self, round_no):
        """Plans the next speed to evaluate, exploring an adaptive window around the current best speed."""
        # Exploration sequence:
        # 1. Optimal speed (fine-tune gains at best pace)
        # 2. Lower speed (-step): probe for reduced tire slip & tighter lines
        # 3. Higher speed (+step): probe if new gains allow speedup without slip
        # 4. Deeper lower speed (-2*step): test safety & high grip margin
        # 5. Boundary probe (+2*step or fine-tuning)
        candidates = [
            self.best_speed,
            round(self.best_speed - self.step, 3),
            round(self.best_speed + self.step, 3),
            round(self.best_speed - 2 * self.step, 3),
            round(self.best_speed + 2 * self.step, 3)
        ]
        valid_cands = []
        for c in candidates:
            if c < self.min_speed:
                continue
            if c >= self.failed_ceiling and c > self.best_speed:
                continue
            valid_cands.append(c)

        if not valid_cands:
            return self.best_speed

        return valid_cands[round_no % len(valid_cands)]

    def get_summary_table(self):
        speeds = sorted(self.history.keys())
        lines = []
        lines.append("Speed (m/s) | Passes/Total | Best Lap (s) | Avg Slip (%) | Avg Score | Status")
        lines.append("-" * 74)
        for s in speeds:
            runs = self.history[s]
            total = len(runs)
            passes = sum(1 for r in runs if r["passed"])
            laps = [r["lap"] for r in runs if r["lap"] is not None and r["passed"]]
            slips = [r["slip"] for r in runs if r["slip"] is not None]
            scores = [r["score"] for r in runs if r["score"] is not None and r["passed"]]
            best_l = f"{min(laps):.2f}" if laps else "N/A"
            avg_sl = f"{sum(slips)/len(slips):.1f}" if slips else "N/A"
            avg_sc = f"{sum(scores)/len(scores):.1f}" if scores else "N/A"
            tag = "★ OPTIMUM" if abs(s - self.best_speed) < 1e-4 else ("CEILING" if s >= self.failed_ceiling else "")
            lines.append(f"  {s:5.2f}     |    {passes:2d}/{total:2d}     |   {best_l:>6}   |    {avg_sl:>5}   |   {avg_sc:>6}  | {tag}")
        return "\n".join(lines)


def drive_metrics(csv_path):
    """(weave [dps], wheel slip [%]) of one drive. weave: rms of the yaw rate minus its 60 ms average."""
    out_dir = os.path.dirname(csv_path)
    sj_path = os.path.join(out_dir, "summary.json")
    if os.path.exists(sj_path):
        try:
            with open(sj_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
                if "_cached_weave" in data and "_cached_slip" in data:
                    return float(data["_cached_weave"]), float(data["_cached_slip"])
        except Exception:
            pass

    wz, slip = [], []
    try:
        with open(csv_path) as fh:
            for r in csv.DictReader(fh):
                v = float(r["v"])
                if v > 0.3:
                    wz.append(float(r["wz_dps"]))
                w = 0.5 * (float(r["wheel_v_l"]) + float(r["wheel_v_r"]))
                if w > 0.3:
                    slip.append(abs(1.0 - v / w))
    except OSError:
        return 0.0, 0.0
    k = 60
    weave = 0.0
    if len(wz) >= 2 * k:
        acc = sum(wz[:k])
        sq = 0.0
        for i in range(k, len(wz)):
            hp = wz[i - k // 2] - acc / k
            sq += hp * hp
            acc += wz[i] - wz[i - k]
        weave = math.sqrt(sq / (len(wz) - k))
    w_res = weave
    s_res = 100.0 * sum(slip) / len(slip) if slip else 0.0
    _cache_metrics(out_dir, w_res, s_res)
    return w_res, s_res


def robot_delay(robot):
    """sensor_delay of a robot file (the JSON may carry // comments)."""
    import re
    with open(robot, encoding="utf-8") as fh:
        m = re.search(r'"sensor_delay"\s*:\s*([0-9.eE+-]+)', fh.read())
    return float(m.group(1)) if m else 0.0


def robot_mu(robot):
    """tire.mu of a robot file."""
    import re
    with open(robot, encoding="utf-8") as fh:
        m = re.search(r'"tire"\s*:\s*\{[^}]*"mu"\s*:\s*([0-9.eE+-]+)', fh.read())
    return float(m.group(1)) if m else 0.0


def sim(course, robot, drives, speed, kp, kd, extra, out, reuse, robot_params=(), seed=1):
    """Runs linesim, returns summary.json (or None)."""
    cmd = [EXE, "run", "--track", course, "--robot", robot, "--drives", drives, "--quiet", "--out", out,
           "--set", f"base_mps={speed}", "--set", f"steer_kp={kp}", "--set", f"steer_kd_s={kd}", "--max-time", "60",
           "--seed", str(seed)]
    for e in extra:
        cmd += ["--set", e]
    for rp in robot_params:
        cmd += ["--robot-param", rp]
    if not ((reuse or out in _RAN) and os.path.exists(os.path.join(out, "summary.json"))):
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        _RAN.add(out)
    try:
        with open(os.path.join(out, "summary.json"), encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def lost_line(d):
    return not d["outcome"].startswith("finished") or d["max_lat_mm"] > MAX_LAT_MM


def run(course, robot, speed, kp, kd, lat, extra, out, reuse, lag, mu, seed=1, drives="1,3"):
    extra = list(extra) + ([f"lat_acc_max={lat}"] if lat > 0 else [])
    j = sim(course, robot, drives, speed, kp, kd, extra, out, reuse, seed=seed)
    fast_num = drives.split(",")[-1].strip()
    try:
        d1 = j["drives"][0]
        d_fast = j["drives"][1] if len(j["drives"]) > 1 else None
        kd_fw = j["params"]["steer_kd"]
    except (TypeError, IndexError, KeyError):
        clean_run_dir(out)
        return {"lost": True, "pass": False, "score": 1000.0, "why": "no result", "kd_fw": 0}
    weave, slip = drive_metrics(os.path.join(out, "drive1.csv"))
    clean_run_dir(out)
    clean1 = d1["rms_lat_mm"] + weave / 20.0 + slip / 2.0
    r = {"kd_fw": kd_fw, "rms_lat": d1["rms_lat_mm"], "max_lat": d1["max_lat_mm"], "weave": weave, "slip": slip,
         "clean1": clean1, "lap_fast": None, "lost": False, "pass": False}
    if lost_line(d1):
        r.update(lost=True, score=1000.0, why=f"1: {d1['outcome'] if d1['outcome'] != 'finished' else 'line far off'}")
    elif d1.get("marker_check") != "OK":
        r.update(score=500.0 + clean1, why=f"1: markers {d1.get('marker_check')}")
    elif d_fast is None:
        r.update(score=400.0 + clean1, why=f"{fast_num}: did not run")
    elif lost_line(d_fast):
        r.update(lost=True, score=300.0 + clean1, why=f"{fast_num}: {d_fast['outcome'] if d_fast['outcome'] != 'finished' else 'line far off'}")
    elif d_fast.get("marker_check") != "OK" or d_fast["lap_time_fw"] <= 0:
        r.update(score=200.0 + clean1, why=f"{fast_num}: markers {d_fast.get('marker_check')}")
    else:
        r.update({"pass": True, "lap_fast": d_fast["lap_time_fw"], "score": 5.0 * d_fast["lap_time_fw"] + clean1, "why": ""})
        why = ""
        if lag > 0:   # model-error stress: more steering lag than fitted
            for k, f in enumerate(STRESS_LAG):
                lag_out = f"{out}_lag{f:g}"
                sj = sim(course, robot, "1", speed, kp, kd, extra, lag_out, reuse,
                         [f"sensor_delay={lag * f:.6f}"], seed)
                if sj is None or lost_line(sj["drives"][0]):
                    why = f"lag x{f:g}: " + (sj["drives"][0]["outcome"] if sj else "no result")
                    clean_run_dir(lag_out)
                    break
                if k == 0:
                    sw, _ = drive_metrics(os.path.join(lag_out, "drive1.csv"))
                    clean_run_dir(lag_out)
                    if sw > 1.5 * weave + 10.0:
                        why = f"lag x{f:g}: weave {sw:.0f} dps"
                        break
                clean_run_dir(lag_out)
        if not why and mu > 0:   # grip stress: less friction than the model
            mu_out = f"{out}_mu{STRESS_MU:g}"
            sj = sim(course, robot, "1", speed, kp, kd, extra, mu_out, reuse,
                     [f"tire_mu={mu * STRESS_MU:.4f}"], seed)
            if sj is None or lost_line(sj["drives"][0]):
                why = f"grip x{STRESS_MU:g}: " + (sj["drives"][0]["outcome"] if sj else "no result")
            elif sj["drives"][0].get("marker_check") != "OK":
                why = f"grip x{STRESS_MU:g}: markers {sj['drives'][0].get('marker_check')}"
            clean_run_dir(mu_out)
        if why:
            r.update({"pass": False, "lost": True, "score": 250.0 + clean1, "why": why})
    return r


def tune_at(speed, courses, robot, axes, extra, jobs, reuse, quiet=False, drives="1,3"):
    """axes = (kps, kds, lats). Returns the grid evaluated at one base speed."""
    kps, kds, lats = axes
    lag, mu = robot_delay(robot), robot_mu(robot)
    out_root = os.path.join(ROOT, "runs", "tune", f"{speed:g}")
    points = [(kp, kd, lat) for lat in lats for kp in kps for kd in kds]
    total = len(points) * len(courses)
    fast_num = drives.split(",")[-1].strip()
    if not quiet:
        print(f"base speed {speed:g} m/s: {len(kps)} kp x {len(kds)} kd x {len(lats)} lat on {len(courses)} course(s) = "
              f"{total} runs (drive 1 + {fast_num} each), {jobs} at a time", flush=True)
    results = {}
    done = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {}
        for kp, kd, lat in points:
            for ci, c in enumerate(courses):
                name = f"kp{kp:g}_kd{kd:g}s" + (f"_lat{lat:g}" if lat > 0 else "") + f"_c{ci}"
                out = os.path.join(out_root, "runs", name)
                futs[ex.submit(run, c, robot, speed, kp, kd, lat, extra, out, reuse, lag, mu, 1, drives)] = (kp, kd, lat, ci)
        for f in concurrent.futures.as_completed(futs):
            kp, kd, lat, ci = futs[f]
            results.setdefault((kp, kd, lat), {})[ci] = f.result()
            done += 1
            if not quiet and (done % 50 == 0 or done == total):
                print(f"  {done}/{total}", flush=True)

    cell = {}
    for p, rs in results.items():
        rs = [rs[i] for i in range(len(courses))]
        cell[p] = {"pass": all(r["pass"] for r in rs), "lost": any(r["lost"] for r in rs),
                   "score": sum(r["score"] for r in rs) / len(rs), "runs": rs,
                   "why": [r["why"] for r in rs if not r["pass"]]}

    def robust(p):   # no grid neighbour (one step along kp, kd or lat) loses the line / is unstable
        idx = [axes[a].index(p[a]) for a in range(3)]
        for a in range(3):
            for d in (-1, 1):
                i = idx[a] + d
                if 0 <= i < len(axes[a]):
                    q = list(p)
                    q[a] = axes[a][i]
                    if cell[tuple(q)]["lost"]:
                        return False
        return True

    passing = [p for p in points if cell[p]["pass"]]
    safe = [p for p in passing if robust(p)]
    pick = None
    if passing:
        # Within 1 % of the best score (~0.1 s of drive 2 lap) the gentlest wins: lower kp, then lower
        # kd, then lower lat_acc_max. A high kd also amplifies the real sensor's position steps.
        pool = safe or passing
        best = min(cell[p]["score"] for p in pool)
        pick = min((p for p in pool if cell[p]["score"] <= best * 1.01), key=lambda p: (p[0], p[1], p[2]))
    return {"speed": speed, "cell": cell, "points": points, "passing": passing, "safe": safe, "pick": pick,
            "out_root": out_root, "axes": axes}


def describe(p):
    return f"steer kp {p[0]:g}, steer kd {p[1]:g} s" + (f", lat_acc_max {p[2]:g}" if p[2] > 0 else "")


def report(t, courses, robot, extra, with_pick=True, drives="1,3"):
    cell, pick = t["cell"], t["pick"]
    kps, kds, lats = t["axes"]
    fast_num = drives.split(",")[-1].strip()
    lines = [f"base speed {t['speed']:g} m/s, drive 1 + {fast_num} on {', '.join(os.path.basename(c) for c in courses)}, "
             f"robot {os.path.basename(robot)}" + (f", --set {' '.join(extra)}" if extra else ""),
             "",
             f"score = 5 x drive {fast_num} lap [s] + drive 1 (line rms [mm] + weave/20 + slip%/2), lower is better",
             f"x = loses the line, or fails a stress drive (steering lag x{STRESS_LAG[0]:g} / x{STRESS_LAG[1]:g}, "
             f"grip x{STRESS_MU:g}), m = misreads markers, * = picked"]
    for lat in lats:
        lines.append("")
        if len(lats) > 1 or lat > 0:
            lines.append(f"lat_acc_max {lat:g} m/s^2" + (" (grip limit off)" if lat == 0 else ""))
        lines.append("kp\\kd [s] " + "".join(f"{kd:>8g}" for kd in kds))
        for kp in kps:
            row = f"{kp:>8g}  "
            for kd in kds:
                c = cell[(kp, kd, lat)]
                txt = f"{c['score']:.1f}" if c["pass"] else ("x" if c["lost"] else "m")
                if (kp, kd, lat) == pick:
                    txt = "*" + txt
                row += f"{txt:>8}"
            lines.append(row)
    if not with_pick:
        return "\n".join(lines)
    lines.append("")
    if pick:
        rs = cell[pick]["runs"]
        n = len(rs)
        lines.append(f"picked: base {t['speed']:g} m/s, {describe(pick)} (robot menu: steer_gain_d {rs[0]['kd_fw']:.0f})"
                     + ("" if pick in t["safe"] else "   (a grid neighbour fails: at the edge)"))
        lines.append(f"  drive {fast_num} lap {sum(r['lap_fast'] for r in rs) / n:.2f} s;  drive 1: line rms "
                     f"{sum(r['rms_lat'] for r in rs) / n:.1f} mm, weave {sum(r['weave'] for r in rs) / n:.0f} dps, "
                     f"slip {sum(r['slip'] for r in rs) / n:.1f} %")
        lines.append(f"  passing {len(t['passing'])} of {len(t['points'])}, away from the edge {len(t['safe'])}")
        lines.append("")
        lines.append("try it in the 3D session:")
        lines.append(f"  build\\Release\\linesim.exe view --session --track {os.path.relpath(courses[0], ROOT)} "
                     f"--robot robots/full.json --set base_mps={t['speed']:g} --set steer_kp={pick[0]:g} "
                     f"--set steer_kd_s={pick[1]:g}" + (f" --set lat_acc_max={pick[2]:g}" if pick[2] > 0 else "")
                     + "".join(f" --set {e}" for e in extra))
    else:
        why = {}
        for c in cell.values():
            for w in c["why"]:
                key = w.split(" (")[0] if "markers" in w else w
                why[key] = why.get(key, 0) + 1
        lines.append("nothing passes at this speed. Most common: "
                     + ", ".join(f"{k} x{n}" for k, n in sorted(why.items(), key=lambda kv: -kv[1])[:4]))
    return "\n".join(lines)


def save(t, text, drives="1,3", streak_info=None):
    os.makedirs(t["out_root"], exist_ok=True)
    cell, pick = t["cell"], t["pick"]
    if streak_info is None and pick:
        rs = cell[pick]["runs"]
        passing_runs = sum(1 for r in rs if r.get("pass"))
        streak_info = {
            "consecutive_passes": passing_runs,
            "steer_kp": pick[0],
            "steer_kd_s": pick[1],
            "lat_acc_max": pick[2],
            "steer_gain_d": rs[0]["kd_fw"] if rs else 0,
            "best_lap_time": min((r["lap_fast"] for r in rs if r.get("lap_fast")), default=None),
            "best_score": cell[pick]["score"]
        }
    with open(os.path.join(t["out_root"], "result.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
        if streak_info:
            fh.write("\n" + "=" * 50 + "\n")
            fh.write("most consecutive passes (longest streak):\n")
            fh.write(f"  consecutive passes: {streak_info['consecutive_passes']}\n")
            fh.write(f"  steer kp: {streak_info['steer_kp']:.3f}, steer kd: {streak_info['steer_kd_s']:.4f} s (steer_gain_d: {streak_info['steer_gain_d']:.0f})\n")
            if streak_info.get("lat_acc_max", 0) > 0:
                fh.write(f"  lat_acc_max: {streak_info['lat_acc_max']:.2f}\n")
    out_dict = {
        "speed": t["speed"],
        "drives": drives,
        "picked": {"steer_kp": pick[0], "steer_kd_s": pick[1], "lat_acc_max": pick[2],
                   "steer_gain_d": cell[pick]["runs"][0]["kd_fw"], "safe": pick in t["safe"]} if pick else None,
        "grid": [{"kp": p[0], "kd": p[1], "lat_acc_max": p[2], "pass": cell[p]["pass"], "lost": cell[p]["lost"],
                  "score": cell[p]["score"], "why": cell[p]["why"]} for p in t["points"]]
    }
    if streak_info:
        out_dict["most_consecutive_success"] = streak_info
    with open(os.path.join(t["out_root"], "result.json"), "w", encoding="utf-8") as fh:
        json.dump(out_dict, fh, indent=1)


def refine_axes(center, steps):
    """3 x 3 x 3 grid around center (kp, kd, lat): center +- step on each axis (step 0: that axis fixed)."""
    out = []
    for c, s, lo, q in zip(center, steps, (1.0, 0.0, 0.0), (0.001, 0.0025, 0.05)):
        if s <= 0:
            out.append([c])
            continue
        vals = sorted({round(round(max(lo, c + k * s) / q) * q, 4) for k in (-1, 0, 1)})
        out.append(vals)
    return tuple(out)


def search_at(speed, courses, robot, axes, extra, jobs, reuse, refine, quiet=False, drives="1,3"):
    """Coarse grid, then `refine` times a 3x3x3 grid around the best point with half the step, then the
    pick is driven again with 2 more seeds (sensor noise / gain spread): it must pass every time."""
    coarse = tune_at(speed, courses, robot, axes, extra, jobs, reuse, quiet, drives)
    best = coarse
    log = []

    def score_of(t):
        return t["cell"][t["pick"]]["score"]

    if coarse["pick"]:
        log.append(f"  coarse   {len(coarse['points']):3d} points -> {describe(coarse['pick'])}, score {score_of(coarse):.1f}"
                   + ("" if coarse["pick"] in coarse["safe"] else " (edge)"))
    steps = [(min(b - a for a, b in zip(v, v[1:])) / 2 if len(v) > 1 else 0.0) for v in axes]
    for it in range(refine):
        if not best["pick"]:
            break
        ax = refine_axes(best["pick"], steps)
        t = tune_at(speed, courses, robot, ax, extra, jobs, reuse, quiet=True, drives=drives)
        step_txt = f"+-{steps[0]:g} kp, +-{steps[1]:g} s kd" + (f", +-{steps[2]:g} lat" if steps[2] > 0 else "")
        if t["pick"] and (t["pick"] in t["safe"] or best["pick"] not in best["safe"]):
            best = t
            log.append(f"  refine {it + 1} ({step_txt}) -> {describe(t['pick'])}, score {score_of(t):.1f}"
                       + ("" if t["pick"] in t["safe"] else " (edge)"))
        else:
            log.append(f"  refine {it + 1} ({step_txt}) -> nothing better away from the edge, kept")
        steps = [s / 2 for s in steps]

    verified, seed_txt = False, ""
    if best["pick"]:
        kp, kd, lat = best["pick"]
        lag, mu = robot_delay(robot), robot_mu(robot)
        res = []
        for seed in SEED_CHECK:
            for ci, c in enumerate(courses):
                name = f"kp{kp:g}_kd{kd:g}s" + (f"_lat{lat:g}" if lat > 0 else "") + f"_c{ci}_seed{seed}"
                res.append(run(c, robot, speed, kp, kd, lat, extra, os.path.join(best["out_root"], "runs", name),
                               reuse, lag, mu, seed, drives))
        verified = all(r["pass"] for r in res)
        seed_txt = (f"  seeds {', '.join(map(str, SEED_CHECK))}: "
                    + ("pass" if verified else "FAIL (" + "; ".join(r["why"] for r in res if not r["pass"]) + ")"))
        log.append(seed_txt)
    return {"speed": speed, "coarse": coarse, "final": best, "log": log, "verified": verified, "drives": drives}


def report_search(s, courses, robot, extra, drives="1,3"):
    t = s["final"]
    fast_num = drives.split(",")[-1].strip()
    lines = [report(s["coarse"], courses, robot, extra, with_pick=False, drives=drives), "", "refinement around the best point:"]
    lines += s["log"]
    lines.append("")
    if t["pick"]:
        rs = t["cell"][t["pick"]]["runs"]
        n = len(rs)
        ok = t["pick"] in t["safe"] and s["verified"]
        lines.append(f"{'picked' if ok else 'best found (NOT reliable)'}: base {s['speed']:g} m/s, {describe(t['pick'])}"
                     f"  (robot menu: steer_gain_d {rs[0]['kd_fw']:.0f})")
        lines.append(f"  drive {fast_num} lap {sum(r['lap_fast'] for r in rs) / n:.2f} s;  drive 1: line rms "
                     f"{sum(r['rms_lat'] for r in rs) / n:.1f} mm, weave {sum(r['weave'] for r in rs) / n:.0f} dps, "
                     f"slip {sum(r['slip'] for r in rs) / n:.1f} %")
        lines.append("")
        lines.append("try it in the 3D session:")
        p = t["pick"]
        lines.append(f"  build\\Release\\linesim.exe view --session --track {os.path.relpath(courses[0], ROOT)} "
                     f"--robot robots/full.json --set base_mps={s['speed']:g} --set steer_kp={p[0]:g} "
                     f"--set steer_kd_s={p[1]:g}" + (f" --set lat_acc_max={p[2]:g}" if p[2] > 0 else "")
                     + "".join(f" --set {e}" for e in extra))
    else:
        lines.append("nothing passes at this speed")
    return "\n".join(lines)


def run_endless(initial_speed, user_tracks, robot, axes, extra, jobs, reuse, refine, drives, hours=None, n_courses=3, apply_c=False):
    """Continuously tunes base speed and steering gains across rotating random courses for N hours or until Ctrl+C."""
    t_start = time.time()
    duration_sec = (hours * 3600.0) if hours else None
    fast_num = drives.split(",")[-1].strip()

    overall_best = None
    round_no = 0
    current_speed = initial_speed
    consecutive_passes = 0
    consecutive_fails = 0
    streak_tracker = StreakTracker()
    speed_opt = SpeedOptimizer(initial_speed=initial_speed, step=0.03)

    endless_log = []
    out_dir = os.path.join(ROOT, "runs", "tune", "endless")
    os.makedirs(out_dir, exist_ok=True)
    speed_curve_csv = os.path.join(out_dir, "speed_curve.csv")
    with open(speed_curve_csv, "w", encoding="utf-8") as f_csv:
        f_csv.write("round,elapsed_sec,speed,passed,lap_fast,slip_pct,rms_mm,raw_score,adj_score,kp,kd_fw,lat\n")

    print("\n" + "=" * 78)
    if hours:
        print(f"=== LINE SIM ENDLESS AUTO-TUNER: RUNNING FOR {hours:g} HOUR(S) ({hours*60:.0f} MINS) ===")
    else:
        print("=== LINE SIM ENDLESS AUTO-TUNER: RUNNING INDEFINITELY (PRESS CTRL+C TO STOP) ===")
    print(f"=== Start Speed: {initial_speed:.2f} m/s | Drives: 1 + {fast_num} | Parallel Jobs: {jobs} ===")
    print(f"=== Robot: {os.path.basename(robot)} | Courses per round: {len(user_tracks) if user_tracks else n_courses} ===")
    print("=" * 78 + "\n", flush=True)

    try:
        while True:
            t_now = time.time()
            elapsed = t_now - t_start
            if duration_sec and elapsed >= duration_sec:
                print(f"\n[TIME'S UP] Configured duration of {hours:g} hour(s) has elapsed.", flush=True)
                break

            round_no += 1
            elapsed_str = time.strftime("%H:%M:%S", time.gmtime(elapsed))
            if duration_sec:
                remain_sec = max(0.0, duration_sec - elapsed)
                remain_str = time.strftime("%H:%M:%S", time.gmtime(remain_sec))
                progress_hdr = f"[{elapsed_str} / -{remain_str}]"
            else:
                progress_hdr = f"[{elapsed_str} / ENDLESS]"

            # Course selection: use explicit user tracks, or generate fresh random tracks each round
            if user_tracks:
                current_courses = list(user_tracks)
            else:
                current_courses = []
                cdir = os.path.join(out_dir, "courses")
                os.makedirs(cdir, exist_ok=True)
                round_seeds = [1000 * round_no + s for s in SEEDS[:n_courses]]
                for s in round_seeds:
                    c = os.path.join(cdir, f"course_r{round_no}_{s}.json")
                    subprocess.run([EXE, "gen", "--seed", str(s), "--out", c], stdout=subprocess.DEVNULL, check=True)
                    current_courses.append(c)

            best_banner = (f"OPTIMAL: {speed_opt.best_speed:.2f} m/s (Score: {speed_opt.best_score:.1f}, Lap: {speed_opt.best_lap:.2f}s, Slip: {speed_opt.best_slip:.1f}%) | {describe(overall_best['pick'])}"
                           if overall_best else "OPTIMAL: searching initial...")
            print(f"{progress_hdr} Round #{round_no} | Testing base speed {current_speed:.2f} m/s | {best_banner}", flush=True)

            s = search_at(round(current_speed, 3), current_courses, robot, axes, extra, jobs, reuse, refine, quiet=True, drives=drives)
            t = s["final"]
            passed = t["pick"] is not None and t["pick"] in t["safe"] and s["verified"]

            if passed:
                pick = t["pick"]
                rs = t["cell"][pick]["runs"]
                avg_lap = sum(x["lap_fast"] for x in rs) / len(rs)
                score = t["cell"][pick]["score"]
                avg_slip = sum(x.get("slip", 0.0) for x in rs) / len(rs)
                avg_rms = sum(x.get("rms_lat", 0.0) for x in rs) / len(rs)

                is_new_best, adj_score = speed_opt.record_run(
                    current_speed, True, score=score, lap=avg_lap, slip=avg_slip, rms=avg_rms, round_id=round_no
                )

                streak_tracker.record_result(pick, True, current_speed, score, avg_lap, rs[0]["kd_fw"], round_no)
                bs = streak_tracker.get_best_streak()
                streak_str = f" [Streak: {streak_tracker.current_streak}" + (f" / Max: {bs['consecutive_passes']}]" if bs else "]")

                log_entry = (f"{progress_hdr} Round #{round_no} PASS @ {current_speed:.2f} m/s: {describe(pick)}, "
                             f"lap {avg_lap:.2f}s, slip {avg_slip:.1f}%, rms {avg_rms:.1f}mm, score {score:.1f} (adj {adj_score:.1f}){streak_str}")
                print(log_entry, flush=True)
                endless_log.append(log_entry)

                with open(speed_curve_csv, "a", encoding="utf-8") as f_csv:
                    f_csv.write(f"{round_no},{elapsed:.1f},{current_speed:.3f},1,{avg_lap:.3f},{avg_slip:.2f},{avg_rms:.2f},{score:.2f},{adj_score:.2f},{pick[0]:.3f},{rs[0]['kd_fw']:.0f},{pick[2]:.2f}\n")

                if is_new_best:
                    overall_best = {
                        "speed": current_speed,
                        "pick": pick,
                        "score": adj_score,
                        "raw_score": score,
                        "lap_fast": avg_lap,
                        "slip": avg_slip,
                        "rms": avg_rms,
                        "round": round_no,
                        "rs": rs,
                        "kd_fw": rs[0]["kd_fw"],
                        "elapsed_sec": elapsed,
                        "elapsed_str": elapsed_str
                    }
                    print("\n" + "*" * 78)
                    print(f"*** [NEW OPTIMAL SPEED & GAINS] Base Speed: {current_speed:.2f} m/s | {describe(pick)}")
                    print(f"*** Drive {fast_num} Lap: {avg_lap:.2f} s | Slip: {avg_slip:.1f}% | RMS: {avg_rms:.1f} mm | Score: {adj_score:.1f}")
                    print(f"*** Robot Menu Parameter: steer_gain_d {rs[0]['kd_fw']:.0f}")
                    if apply_c:
                        update_drive_c(current_speed, pick[0], rs[0]["kd_fw"], pick[2])
                        print(f"*** Auto-saved optimal configuration to firmware/Main/Src/drive.c and {out_dir}/best.json")
                    else:
                        print(f"*** Saved to {out_dir}/best.json (drive.c unchanged)")
                    print("*" * 78 + "\n", flush=True)

                best_dict = {
                    "optimal_speed": speed_opt.best_speed,
                    "speed": overall_best["speed"] if overall_best else current_speed,
                    "steer_kp": round(overall_best["pick"][0] if overall_best else pick[0], 3),
                    "steer_kd_s": overall_best["pick"][1] if overall_best else pick[1],
                    "steer_gain_d": overall_best["rs"][0]["kd_fw"] if overall_best else rs[0]["kd_fw"],
                    "lat_acc_max": overall_best["pick"][2] if overall_best else pick[2],
                    "lap_time": overall_best["lap_fast"] if overall_best else avg_lap,
                    "slip_pct": overall_best["slip"] if overall_best else avg_slip,
                    "rms_mm": overall_best["rms"] if overall_best else avg_rms,
                    "score": overall_best["score"] if overall_best else score,
                    "round": overall_best["round"] if overall_best else round_no,
                    "elapsed_str": overall_best["elapsed_str"] if overall_best else elapsed_str
                }
                if bs:
                    best_dict["most_consecutive_success"] = bs
                with open(os.path.join(out_dir, "best.json"), "w", encoding="utf-8") as fh:
                    json.dump(best_dict, fh, indent=2)

                consecutive_passes += 1
                consecutive_fails = 0
            else:
                speed_opt.record_run(current_speed, False, round_id=round_no)
                streak_tracker.record_result(None, False)
                why_txt = t["cell"][t["pick"]]["why"][0] if (t["pick"] and t["cell"][t["pick"]]["why"]) else "instability/stress fail"
                log_entry = f"{progress_hdr} Round #{round_no} MISS @ {current_speed:.2f} m/s ({why_txt})"
                print(log_entry, flush=True)
                endless_log.append(log_entry)

                with open(speed_curve_csv, "a", encoding="utf-8") as f_csv:
                    f_csv.write(f"{round_no},{elapsed:.1f},{current_speed:.3f},0,0,0,0,999,999,0,0,0\n")

                consecutive_fails += 1
                consecutive_passes = 0

            # Plan next speed using adaptive SpeedOptimizer
            current_speed = speed_opt.next_speed(round_no)

            with open(os.path.join(out_dir, "endless_history.log"), "w", encoding="utf-8") as fh:
                fh.write("\n".join(endless_log) + "\n")

    except KeyboardInterrupt:
        print("\n\n[USER STOPPED] KeyboardInterrupt received. Safely stopping auto-tune...", flush=True)

    total_elapsed = time.time() - t_start
    total_elapsed_str = time.strftime("%H:%M:%S", time.gmtime(total_elapsed))

    summary_lines = [
        "",
        "=" * 78,
        f"=== ENDLESS AUTO-TUNE FINAL REPORT (Elapsed: {total_elapsed_str}, Rounds: {round_no}) ===",
        "=" * 78,
    ]
    if overall_best:
        p = overall_best["pick"]
        summary_lines.append(f"OPTIMAL VERIFIED BASE SPEED: {overall_best['speed']:.2f} m/s")
        summary_lines.append(f"BEST GAINS (At Optimal Speed): {describe(p)}")
        summary_lines.append(f"  Robot Menu Parameter: steer_gain_d {overall_best['kd_fw']:.0f}")
        summary_lines.append(f"  Drive {fast_num} Lap Time: {overall_best['lap_fast']:.2f} s | Avg Slip: {overall_best['slip']:.1f}% | RMS: {overall_best['rms']:.1f} mm")
        summary_lines.append(f"  Composite Score: {overall_best['score']:.1f} (Raw: {overall_best['raw_score']:.1f})")
        summary_lines.append(f"  Discovered at round #{overall_best['round']} ({overall_best['elapsed_str']})")
        summary_lines.append("")
        summary_lines.append("SPEED VS PERFORMANCE PROFILE (Optimal Curve):")
        summary_lines.append(speed_opt.get_summary_table())
        summary_lines.append("")
        if apply_c:
            summary_lines.append(f"Permanent configuration saved to firmware/Main/Src/drive.c")
        summary_lines.append(f"Detailed logs saved to {out_dir}/best.json and {out_dir}/speed_curve.csv")
    else:
        summary_lines.append("No configuration passed all courses and stress tests.")

    bs = streak_tracker.get_best_streak()
    if bs:
        summary_lines.append("")
        summary_lines.append(f"MOST CONSECUTIVE PASSES GAIN (Longest Streak): {bs['consecutive_passes']} round(s)")
        summary_lines.append(f"  Gains: steer kp {bs['steer_kp']:.3f}, steer kd {bs['steer_kd_s']:.4f} s (steer_gain_d {bs['steer_gain_d']:.0f})"
                             + (f", lat_acc_max {bs['lat_acc_max']:.2f}" if bs.get('lat_acc_max', 0) > 0 else ""))
        if bs.get("speeds_passed"):
            summary_lines.append(f"  Speeds passed: {', '.join(f'{x:.2f} m/s' for x in bs['speeds_passed'])}")
        summary_lines.append(f"  Best Lap Time: {bs['best_lap_time']:.2f} s (Score: {bs['best_score']:.1f})")
        summary_lines.append(f"  Achieved at round #{bs['round_achieved']}")

    summary_lines.append("=" * 78 + "\n")

    summary_text = "\n".join(summary_lines)
    print(summary_text, flush=True)
    with open(os.path.join(out_dir, "final_summary.txt"), "w", encoding="utf-8") as fh:
        fh.write(summary_text + "\n")


def main():
    a = sys.argv[1:]

    def opt(name, default):
        if name in a:
            i = a.index(name)
            v = a[i + 1]
            del a[i:i + 2]
            return v
        return default

    def flag(name):
        if name in a:
            a.remove(name)
            return True
        return False

    extra, tracks = [], []
    while "--set" in a:
        extra.append(opt("--set", None))
    while "--track" in a:
        tracks.append(os.path.abspath(opt("--track", None)))
    reuse = flag("--reuse")
    max_speed = flag("--max-speed")
    apply_c = flag("--apply")
    global KEEP_RUNS
    KEEP_RUNS = flag("--keep-runs")
    hours_str = opt("--hours", None)
    hours = float(hours_str) if hours_str is not None else None
    endless = flag("--endless")
    robot = os.path.abspath(opt("--robot", os.path.join(ROOT, "robots", "full.json")))
    if not os.path.exists(robot):
        robot = os.path.join(ROOT, "robots", "default.json")
    drives = opt("--drives", "1,3")
    fast_num = drives.split(",")[-1].strip()
    jobs = int(opt("--jobs", max(1, (os.cpu_count() or 2) - 1)))
    n_courses = int(opt("--courses", 4))
    refine = int(opt("--refine", "3"))
    kps = [float(x) for x in opt("--kp", ",".join(map(str, KP_COARSE if refine > 0 else KP))).split(",")]
    kds = [float(x) for x in opt("--kd", ",".join(map(str, KD_COARSE if refine > 0 else KD))).split(",")]
    lats = [float(x) for x in opt("--lat", "0").split(",")]
    axes = (kps, kds, lats)
    if not a or "--help" in a or "-h" in a:
        sys.exit(__doc__)
    speed = float(a[0])
    use_private_exe()

    if hours is not None or endless:
        run_endless(speed, tracks, robot, axes, extra, jobs, reuse, refine, drives, hours=hours, n_courses=n_courses, apply_c=apply_c)
        return

    if not tracks:
        if os.path.exists(REAL_COURSE):
            tracks = [REAL_COURSE]
        else:
            cdir = os.path.join(ROOT, "runs", "tune", "courses")
            os.makedirs(cdir, exist_ok=True)
            for s in SEEDS[:n_courses]:
                c = os.path.join(cdir, f"course_{s}.json")
                subprocess.run([EXE, "gen", "--seed", str(s), "--out", c], stdout=subprocess.DEVNULL, check=True)
                tracks.append(c)
    print(f"courses: {', '.join(os.path.relpath(c, ROOT) for c in tracks)}  robot: {os.path.basename(robot)}  drives: {drives}")

    def evaluate(v, quiet):
        s = search_at(round(v, 3), tracks, robot, axes, extra, jobs, reuse, refine, quiet, drives=drives)
        text = report_search(s, tracks, robot, extra, drives=drives)
        save(s["final"], text, drives=drives)
        return s, text

    if not max_speed:
        s, text = evaluate(speed, False)
        print()
        print(text)
        print(f"\nwritten to {os.path.relpath(s['final']['out_root'], ROOT)}\\result.txt")
        return

    best, dv, v, log = None, 0.2, speed, []
    speed_streak = StreakTracker()
    round_cnt = 0
    while True:
        round_cnt += 1
        s, _ = evaluate(v, True)
        t = s["final"]
        ok = t["pick"] is not None and t["pick"] in t["safe"] and s["verified"]
        if ok:
            r = t["cell"][t["pick"]]["runs"]
            avg_lap = sum(x['lap_fast'] for x in r) / len(r)
            speed_streak.record_result(t["pick"], True, s["speed"], t["cell"][t["pick"]]["score"], avg_lap, r[0]["kd_fw"], round_cnt)
            line = (f"  {s['speed']:.3f} m/s  pass: {describe(t['pick'])}, "
                    f"drive {fast_num} {avg_lap:.2f} s [Streak: {speed_streak.current_streak}]")
            best = s
            v = s["speed"] + dv
        else:
            speed_streak.record_result(None, False)
            line = f"  {s['speed']:.3f} m/s  no reliable pass" + (f" ({s['log'][-1].strip()})" if s["log"] else "")
            if best is None:
                if v <= 0.5:
                    break
                v = max(0.5, v - dv)
            else:
                if dv <= 0.01 + 1e-9:
                    print(line, flush=True)
                    log.append(line)
                    break
                dv = max(0.01, dv / 2)
                v = best["speed"] + dv
        print(line, flush=True)
        log.append(line)

    lines = [f"max-speed search (drive 1 + {fast_num}), " + ", ".join(os.path.basename(c) for c in tracks), ""] + log + [""]
    if best:
        lines.append(f"highest base speed that passes: {best['speed']:.3f} m/s")
        lines.append(report_search(best, tracks, robot, extra, drives=drives))
    else:
        lines.append("no speed passed")

    bs = speed_streak.get_best_streak()
    if bs:
        lines.append("")
        lines.append(f"most consecutive speed passes (longest streak): {bs['consecutive_passes']} steps")
        lines.append(f"  gains: steer kp {bs['steer_kp']:.3f}, steer kd {bs['steer_kd_s']:.4f} s (steer_gain_d {bs['steer_gain_d']:.0f})"
                     + (f", lat_acc_max {bs['lat_acc_max']:.2f}" if bs.get('lat_acc_max', 0) > 0 else ""))
        if bs.get("speeds_passed"):
            lines.append(f"  speeds passed: {', '.join(f'{x:.2f} m/s' for x in bs['speeds_passed'])}")
        lines.append(f"  best lap: {bs['best_lap_time']:.2f} s (Score: {bs['best_score']:.1f})")

    text = "\n".join(lines)
    with open(os.path.join(ROOT, "runs", "tune", "max_speed.txt"), "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    if bs or best:
        with open(os.path.join(ROOT, "runs", "tune", "max_speed.json"), "w", encoding="utf-8") as fh:
            json.dump({
                "highest_speed": best["speed"] if best else None,
                "most_consecutive_success": bs
            }, fh, indent=2)
    print()
    print(text)
    print("\nwritten to runs\\tune\\max_speed.txt and runs\\tune\\max_speed.json")


if __name__ == "__main__":
    main()
