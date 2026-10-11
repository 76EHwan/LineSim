#include "core/scenario.h"

#include <windows.h>

#include <algorithm>
#include <cstdarg>
#include <chrono>
#include <ctime>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>

#include "core/json.h"
#include "core/simulation.h"
#include "sim_mcu.h"
#include "view/live_view.h"
#include "view/viewer.h"
#include "world/robot.h"
#include "world/sensors.h"
#include "world/track.h"
#include "world/track_gen.h"

namespace fs = std::filesystem;

namespace {
constexpr double kRadToDeg = 57.29577951308232;

// menu.c "Last Used" reads/writes STM32 Bank2 sector 7 (0x081E0000..0x08200000) through
// raw pointers. Back that address range with erased (0xFF) memory in this process.
bool map_firmware_flash() {
	static bool mapped = false;
	if (mapped)
		return true;
	void* p = VirtualAlloc(reinterpret_cast<void*>(0x081E0000), 0x20000, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
	if (!p)
		return false;
	std::memset(p, 0xFF, 0x20000);
	mapped = true;
	return true;
}

std::string lcd_text() {
	std::string s;
	for (int r = 0; r < SIM_LCD_ROWS; r++) {
		std::string row = sim_lcd_row(r);
		while (!row.empty() && row.back() == ' ')
			row.pop_back();
		s += row;
		s += '\n';
	}
	while (s.size() > 1 && s[s.size() - 1] == '\n' && s[s.size() - 2] == '\n')
		s.pop_back();
	return s;
}

double parse_lap_time(const std::string& lcd) {
	size_t p = lcd.find("T:");
	if (p == std::string::npos)
		return -1;
	return std::atof(lcd.c_str() + p + 2);
}

// rms of the yaw rate minus its 60 ms moving average (1 ms samples): the fast steering weave
double weave_dps(const std::vector<Sample>& smp) {
	std::vector<double> wz;
	for (const Sample& s : smp)
		if (s.v > 0.3f)
			wz.push_back(s.wz_dps);
	const size_t k = 60;
	if (wz.size() < 2 * k)
		return 0.0;
	double acc = 0, sq = 0;
	for (size_t i = 0; i < k; i++)
		acc += wz[i];
	for (size_t i = k; i < wz.size(); i++) {
		double hp = wz[i - k / 2] - acc / k;
		sq += hp * hp;
		acc += wz[i] - wz[i - k];
	}
	return std::sqrt(sq / (wz.size() - k));
}

// Auto-tune stress test: steering lag (sensor_delay) factors. With the first, the weave may grow at
// most to 1.5 x the nominal drive's + 10 dps (a weave that keeps going reads 3x and more).
constexpr double kStressLag[2] = {1.4, 1.85};

// mean wheel slip [%]: how much slower the body moved than the treads (odometry reads long by that)
double slip_pct(const std::vector<Sample>& smp) {
	double sum = 0;
	int n = 0;
	for (const Sample& s : smp) {
		const double w = 0.5 * (s.wheel_v[0] + s.wheel_v[1]);
		if (w > 0.3) {
			sum += std::fabs(1.0 - s.v / w);
			n++;
		}
	}
	return n ? 100.0 * sum / n : 0.0;
}

// Session auto-tune (A key): base speed and steer kp / kd (seconds) for the race on the course in
// the window. Endless, until A is pressed again. Same rules as tools/autotune.py, coarser grid.
//
// A trial is drive 1 (learning: markers and distances must come out clean) followed by drive 2
// (accelerates on the straights from that data; max speed, accel, decel as set in the panel). It
// passes when both finish, never more than 70 mm off the line, with every marker read right, and
// drive 1 still holds with more steering lag than the model has (kStressLag: the lag was fitted to
// one gyro log). Score (lower is better): 5 x drive 2 lap + drive 1 cleanliness (line error, weave,
// wheel slip).
// A round runs the whole kp x kd grid at the test speed. Of the pairs that pass and whose grid
// neighbours are stable too, the gentlest within 1 % of the best score wins (lower kp, then lower
// kd). Something passed: that speed is the new best, the next round tries dv faster. Nothing
// passed: back to the best speed + dv/2 (down to 0.01 m/s steps; below that it re-checks).
class GainTuner {
public:
	bool active = false;

	void start(double v, double /*kp*/, double /*kd_s*/) {
		active = true;
		v_ = std::max(0.3, v);
		dv_ = 0.2;
		best_v_ = 0;
		best_kp_ = best_kd_ = best_lap_ = 0;
		round_ = 0;
		last_.clear();
		log_.clear();
		plan();
	}
	double speed() const { return v_; }
	bool has_best() const { return best_v_ > 0; }
	double best_v() const { return best_v_; }
	double best_kp() const { return best_kp_; }
	double best_kd() const { return best_kd_; }
	std::pair<double, double> next() const {
		const Cell& c = cells_[done_];
		return {c.kp, c.kd};
	}
	bool round_complete() const { return done_ == cells_.size(); }

	// lost: lost the line or unstable with more lag. pass: both drives right and stable.
	void record(bool lost, bool pass, double score, double lap2, const std::string& text) {
		Cell& c = cells_[done_];
		c.state = pass ? 3 : lost ? 1 : 2;
		c.score = score;
		c.lap2 = lap2;
		log_.push_back(fmt("%.3f kp %4.1f kd %.3f  %s", v_, c.kp, c.kd, text.c_str()));
		if (log_.size() > 4)
			log_.erase(log_.begin());
		done_++;
	}

	void finish_round() {
		round_++;
		const int nkp = static_cast<int>(sizeof kKp / sizeof kKp[0]), nkd = static_cast<int>(sizeof kKd / sizeof kKd[0]);
		auto cell = [&](int i, int j) -> const Cell& { return cells_[i * nkd + j]; };
		auto stable = [&](int i, int j) {   // no grid neighbour lost the line / was unstable
			const int d[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
			for (const auto& o : d) {
				int a = i + o[0], b = j + o[1];
				if (a >= 0 && a < nkp && b >= 0 && b < nkd && cell(a, b).state == 1)
					return false;
			}
			return true;
		};
		double best = 1e9;
		for (int i = 0; i < nkp; i++)
			for (int j = 0; j < nkd; j++)
				if (cell(i, j).state == 3 && stable(i, j))
					best = std::min(best, cell(i, j).score);
		const Cell* pick = nullptr;
		for (int i = 0; i < nkp && !pick; i++)   // lowest kp first, then lowest kd
			for (int j = 0; j < nkd && !pick; j++)
				if (cell(i, j).state == 3 && stable(i, j) && cell(i, j).score <= best * 1.01)
					pick = &cell(i, j);
		if (pick) {
			if (v_ > best_v_ + 1e-9 || (std::fabs(v_ - best_v_) < 1e-9 && pick->lap2 < best_lap_)) {
				best_v_ = v_;
				best_kp_ = pick->kp;
				best_kd_ = pick->kd;
				best_lap_ = pick->lap2;
			}
			last_ = fmt("%.3f m/s passed (kp %.0f kd %.3f): try %.3f", v_, pick->kp, pick->kd, v_ + dv_);
			v_ += dv_;
		} else if (best_v_ > 0) {
			if (dv_ > 0.01 + 1e-9)
				dv_ = std::max(0.01, dv_ * 0.5);
			last_ = fmt("%.3f m/s: no stable pass -> %.3f", v_, best_v_ + dv_);
			v_ = best_v_ + dv_;
		} else {
			last_ = fmt("%.3f m/s: no stable pass -> slower", v_);
			v_ = std::max(0.3, v_ - dv_);
		}
		plan();
	}

	std::vector<std::string> status(double dt) const {
		std::vector<std::string> v;
		v.push_back(fmt("AUTO-TUNE  round %d  (A: stop, keep BEST)", round_ + 1));
		v.push_back(fmt("testing  %.3f m/s   pair %zu / %zu", v_, std::min(done_ + 1, cells_.size()), cells_.size()));
		v.push_back(fmt("lag stress x%.2g / x%.3g, speed step %.3f", kStressLag[0], kStressLag[1], dv_));
		if (best_v_ > 0)
			v.push_back(fmt("BEST  %.3f m/s  kp %.0f  kd %.3f s (fw %.0f)  2nd %.2f s", best_v_, best_kp_, best_kd_,
				best_kd_ / dt, best_lap_));
		else
			v.push_back("BEST  none yet");
		v.push_back(last_);
		// the grid of this round: . pending, x lost / unstable, m markers, number = score
		std::string h = "kp/kd ";
		for (double kd : kKd)
			h += fmt("%6.3g", kd);
		v.push_back(h);
		const int nkd = static_cast<int>(sizeof kKd / sizeof kKd[0]);
		for (size_t i = 0; i < sizeof kKp / sizeof kKp[0]; i++) {
			std::string r = fmt("%5.0f ", kKp[i]);
			for (int j = 0; j < nkd; j++) {
				const Cell& c = cells_[i * nkd + j];
				r += c.state == 0 ? "     ." : c.state == 1 ? "     x" : c.state == 2 ? "     m" : fmt("%6.1f", c.score);
			}
			v.push_back(r);
		}
		for (const std::string& l : log_)
			v.push_back(l);
		return v;
	}

private:
	static constexpr double kKp[5] = {12, 16, 20, 25, 32};
	static constexpr double kKd[6] = {0, 0.1, 0.15, 0.225, 0.3, 0.4};
	struct Cell {
		double kp = 0, kd = 0;
		int state = 0;   // 0 pending, 1 lost / unstable, 2 markers wrong, 3 pass
		double score = 0, lap2 = 0;
	};
	static std::string fmt(const char* f, ...) {
		char buf[200];
		va_list ap;
		va_start(ap, f);
		std::vsnprintf(buf, sizeof buf, f, ap);
		va_end(ap);
		return buf;
	}
	void plan() {   // row by row, gentle (low kp) first
		cells_.clear();
		for (double kp : kKp)
			for (double kd : kKd)
				cells_.push_back({kp, kd});
		done_ = 0;
	}

	double v_ = 1.5, dv_ = 0.2, best_v_ = 0, best_kp_ = 0, best_kd_ = 0, best_lap_ = 0;
	int round_ = 0;
	size_t done_ = 0;
	std::string last_;
	std::vector<Cell> cells_;
	std::vector<std::string> log_;
};
}  // namespace

bool apply_param(FwDriveParams* p, const std::string& key, double v) {
	if (key == "base_mps") p->base_mps = (float) v;
	else if (key == "max_mps") p->max_mps = (float) v;
	else if (key == "accel") p->accel = (float) v;
	else if (key == "decel") p->decel = (float) v;
	else if (key == "steer_kp") p->steer_kp = (float) v;
	else if (key == "steer_kd") p->steer_kd = (float) v;              // firmware value (CMSIS Kd per 2 kHz call)
	else if (key == "steer_kd_s") p->steer_kd = (float) (v / fw_steer_dt());   // derivative gain in seconds
	else if (key == "pos_atten_gain") p->pos_atten_gain = (float) v;
	else if (key == "pit_in_m") p->pit_in_m = (float) v;
	else if (key == "target_shift") p->target_shift = (float) v;
	else if (key == "fan_en") p->fan_en = (int) v;
	else return false;
	return true;
}

bool load_scenario_file(const std::string& path, ScenarioOptions* opt, std::string* err) {
	json::Value j;
	if (!json::parse_file(path, &j, err))
		return false;
	fs::path base = fs::path(path).parent_path();
	auto rel = [&](const std::string& p) { return p.empty() || fs::path(p).is_absolute() ? p : (base / p).string(); };
	opt->track_path = rel(j.str("track", opt->track_path));
	opt->robot_path = rel(j.str("robot", opt->robot_path));
	if (j.has("out"))
		opt->out_dir = rel(j.str("out", ""));
	if (const json::Value* d = j.get("drives"); d && d->is_arr()) {
		opt->drives.clear();
		for (const json::Value& e : d->a)
			opt->drives.push_back(static_cast<int>(e.n));
	}
	if (const json::Value* p = j.get("params"); p && p->is_obj())
		for (const auto& kv : p->o)
			opt->params[kv.first] = kv.second.n;
	opt->seed = static_cast<unsigned>(j.num("seed", opt->seed));
	opt->max_drive_time = j.num("max_drive_time", opt->max_drive_time);
	opt->off_track_mm = j.num("off_track_mm", opt->off_track_mm);
	return true;
}

ScenarioResult run_scenario(const ScenarioOptions& opt) {
	ScenarioResult res;
	auto wall0 = std::chrono::steady_clock::now();

	Track track;
	std::string err;
	unsigned map_seed = static_cast<unsigned>(std::time(nullptr));
	std::string generated;   // JSON of a generated course, saved into the output folder below
	if (opt.track_path == "random") {
		TrackGenOptions g;
		g.seed = map_seed;
		generated = generate_track(g, &err);
		if (generated.empty() || !track.load_text(generated, "random", &err)) {
			res.error = err;
			return res;
		}
	} else if (!track.load(opt.track_path, &err)) {
		res.error = err;
		return res;
	}
	res.track_path_used = opt.track_path;
	for (const std::string& w : track.warnings())
		std::fprintf(stderr, "track warning: %s\n", w.c_str());
	RobotParams rp;
	std::string robot_path = opt.robot_path;
	if (!robot_path.empty() && !fs::exists(robot_path)) {
		// robots/full.json is made locally from the CAD model and is not in the repository
		fs::path fallback = fs::path(robot_path).parent_path() / "default.json";
		if (fs::exists(fallback)) {
			std::fprintf(stderr,
				"robot file %s not found (make it with tools/step_import.py robots/Full.step); using %s\n",
				robot_path.c_str(), fallback.string().c_str());
			robot_path = fallback.string();
		}
	}
	if (!robot_path.empty()) {
		json::Value j;
		if (!json::parse_file(robot_path, &j, &err)) {
			res.error = err;
			return res;
		}
		rp.load(j, fs::absolute(robot_path).parent_path().generic_string());
	}
	for (const auto& [k, v] : opt.robot_params) {   // --robot-param (model-error stress tests)
		if (k == "sensor_delay") rp.sensor_delay = v;
		else if (k == "tire_mu") rp.tire.mu = v;
		else if (k == "drive_lag") rp.motor.drive_lag = v;
		else if (k == "fan_force") rp.fan_force = v;
		else std::fprintf(stderr, "warning: unknown --robot-param %s\n", k.c_str());
	}

	fs::path out = opt.out_dir;
	fs::path sd = out / "sdcard";
	std::error_code ec;
	// Reusing an output folder (e.g. runs/latest): drop the previous run's result files so
	// nothing stale is left next to the new ones. The virtual SD card is kept, like a real card.
	if (fs::is_directory(out, ec)) {
		for (const auto& e : fs::directory_iterator(out, ec)) {
			std::string name = e.path().filename().string();
			bool ours = name == "summary.json" || name == "index.html" || name == "trajectory.svg" || name == "robot.xml" ||
				(name.rfind("drive", 0) == 0 && (e.path().extension() == ".csv" || e.path().extension() == ".svg"));
			if (ours && e.is_regular_file())
				fs::remove(e.path(), ec);
		}
	}
	// A card that has been used on the robot already has these folders. On a blank card
	// Delete_All_Marker_Logs() gets FR_NO_PATH and waits 10 x 500 ms at boot (same on hardware).
	fs::create_directories(sd / "Drive_Data", ec);
	if (ec) {
		res.error = "cannot create " + sd.string();
		return res;
	}
	auto save_generated = [&](const std::string& text, unsigned seed) {
		fs::path f = out / ("random_" + std::to_string(seed) + ".json");
		FILE* fp = std::fopen(f.string().c_str(), "wb");
		if (fp) {
			std::fwrite(text.data(), 1, text.size(), fp);
			std::fclose(fp);
		}
		res.track_path_used = f.string();
	};
	if (!generated.empty())
		save_generated(generated, map_seed);
	if (!map_firmware_flash()) {
		res.error = "cannot map the firmware flash region at 0x081E0000";
		return res;
	}

	auto robot = std::make_unique<Robot>();
	if (!robot->init(rp, (out / "robot.xml").string(), &err)) {
		res.error = err;
		return res;
	}
	IrBar ir;
	ir.init(rp, opt.seed);
	Imu imu;
	imu.init(rp, opt.seed + 101);

	auto sim = std::make_unique<Simulation>(*robot, ir, imu, track, sd.generic_string());
	if (const char* w = fw_timing_warning())
		std::fprintf(stderr, "warning: %s\n", w);
	// Sensor bar 8 cm behind the start line unless the track fixes the start position.
	auto start_pose = [&]() {
		return track.pose_at(track.has_robot_start() ? track.robot_start_s() : track.start_s() - rp.sensor_x - 0.08);
	};
	Pose2 start = start_pose();

	// ---- optional 3D viewer ----
	std::unique_ptr<Viewer> viewer;
	std::unique_ptr<LiveView> live;
	SessionState session;
	session.map_name = track.name();
	session.expected = track.expected_markers();
	bool booted = false;
	if (opt.view || opt.interactive || opt.session) {
		viewer = std::make_unique<Viewer>();
		std::string title = "LineSim - " + track.name() +
			(opt.interactive ? " (interactive)" : opt.session ? " (session)" : "");
		if (!viewer->open(1400, 860, title.c_str(), &err)) {
			res.error = "viewer: " + err;
			return res;
		}
		viewer->attach(robot->model(), robot->data(), robot->body_id(), track);
		viewer->set_sensor_offset(rp.ir_sensors.empty() ? rp.sensor_x : rp.ir_sensors[7][0] - 0.01);
		{
			std::vector<std::array<double, 2>> pts;
			for (int i = 0; i < 16; i++)
				pts.push_back(i < (int) rp.ir_sensors.size() ? rp.ir_sensors[i]
				                                              : std::array<double, 2>{rp.sensor_x, -(-1.0 + 2.0 * i / 15.0) * rp.sensor_half_width});
			viewer->set_sensor_points(pts);
		}
		live = std::make_unique<LiveView>(*viewer, *robot, track, opt.interactive, opt.view_speed);
		live->on_reset = [&]() {
			robot->place(start);
			ir.set_lifted(false);
		};
		if (opt.session)
			live->session = &session;
	}

	// ---- per-drive bookkeeping, shared between the two fibers ----
	int current = 0;              // drive running on the firmware fiber (0 = none)
	bool ramp_seen = false, ramp_done = false;
	double t_finished = 0;
	bool lifted = false;
	bool lcd_captured = false;
	std::string lift_reason;
	DriveResult* cur = nullptr;
	double lat_sq = 0;

	sim->on_ramp_timer = [&](bool on) {
		if (!current)
			return;
		if (on) {
			ramp_seen = true;
		} else if (ramp_seen && !ramp_done) {
			ramp_done = true;
			t_finished = sim->t();
		}
	};

	sim->on_millisecond = [&]() {
		// operator: once the run is over, hold K to save / confirm, release, repeat
		bool press = false;
		if (!opt.interactive && current && ramp_done) {
			double since = sim->t() - t_finished;
			// the result screen is up once the robot has stopped; keep it before pressing K
			if (!lcd_captured && since > 0.25 && cur) {
				cur->lcd = lcd_text();
				cur->lap_time_fw = parse_lap_time(cur->lcd);
				lcd_captured = true;
			}
			press = since > 0.3 && std::fmod(since - 0.3, 1.6) < 0.8;
		}
		if (!opt.interactive)
			fwb_set_button(FWB_BTN_K, press);

		if (session.request_abort) {   // X in the session: the operator picks the robot up
			session.request_abort = false;
			if (current && !lifted) {
				lifted = true;
				lift_reason = "stopped";
				ir.set_lifted(true);
			}
		}
		if (live) {
			std::string phase;
			bool ff;
			if (opt.interactive) {
				ff = !booted;
				phase = booted ? "interactive (firmware menu)" : "booting firmware";
			} else if (opt.session) {
				ff = !booted || (current && !(ramp_seen && !ramp_done));
				phase = !booted ? "booting firmware"
				        : current ? "drive " + std::to_string(current) + (ramp_done ? " - finished" : ramp_seen ? "" : " - starting")
				                  : "session: press 1-4 to drive";
			} else {
				ff = !(current && ramp_seen && !ramp_done);
				phase = current ? "drive " + std::to_string(current) + (ramp_done ? " - finished" : ramp_seen ? "" : " - starting")
				                : "booting firmware";
			}
			live->tick(sim->t(), ff, phase);
		}

		if (!cur || !ramp_seen || ramp_done)
			return;
		Robot& r = sim->robot();
		Pose2 pz = r.pose();
		Vec2 bar = r.body_to_ground(rp.sensor_x, 0.0);
		double lat = 0, s = 0;
		track.nearest(bar, &lat, &s);
		FwTelemetry tm;
		fwb_telemetry(&tm);
		Sample smp{};
		smp.t = sim->t();
		smp.x = (float) pz.x;
		smp.y = (float) pz.y;
		smp.yaw_deg = (float) (pz.th * kRadToDeg);
		smp.v = (float) r.forward_speed();
		smp.wz_dps = (float) (r.yaw_rate() * kRadToDeg);
		smp.lat_mm = (float) (lat * 1000);
		smp.s = (float) s;
		smp.pitch_deg = (float) (r.pitch() * kRadToDeg);
		for (int k = 0; k < 2; k++) {
			smp.wheel_v[k] = (float) (r.wheel_speed(k) * rp.wheel_radius);
			smp.iq[k] = (float) r.iq(k);
			smp.omega_t[k] = tm.omega_target[k];
			smp.omega_m[k] = tm.omega_meas[k];
			smp.iq_cmd[k] = tm.iq_cmd[k];
		}
		smp.tgt_base = tm.target_base_mps;
		smp.cur_base = tm.current_base_mps;
		smp.tgt_pos = tm.target_pos;
		smp.odom = tm.odom_m;
		smp.yaw_fw = tm.yaw_deg;
		smp.lost = tm.lost;
		smp.markers = tm.marker_count;
		smp.contact = r.chassis_contact();
		if (!cur->samples.empty()) {
			const Sample& pv = cur->samples.back();
			cur->dist_m += std::hypot(smp.x - pv.x, smp.y - pv.y);
		}
		cur->samples.push_back(smp);
		cur->max_lat_mm = std::max(cur->max_lat_mm, (double) std::fabs(smp.lat_mm));
		cur->max_v = std::max(cur->max_v, (double) smp.v);
		lat_sq += smp.lat_mm * smp.lat_mm;

		if (!lifted) {
			double running = sim->t() - cur->t_start;
			if (std::fabs(lat * 1000) > opt.off_track_mm) {
				lifted = true;
				lift_reason = "off_track";
			} else if (running > opt.max_drive_time) {
				lifted = true;
				lift_reason = "timeout";
			}
			if (lifted)
				ir.set_lifted(true);
		}
	};

	res.drives.reserve(opt.session ? 4096 : opt.drives.size());

	// One drive, as the operator does it: put the robot on the start line, select the drive.
	auto run_one = [&](int n) {
		res.drives.emplace_back();
		cur = &res.drives.back();
		cur->n = n;
		robot->place(start);
		ir.set_lifted(false);
		lifted = false;
		lift_reason.clear();
		ramp_seen = ramp_done = false;
		lcd_captured = false;
		lat_sq = 0;
		current = n;
		cur->t_start = sim->t();
		if (live)
			live->drive_started();
		if (!opt.quiet)
			std::printf("[%8.3f s] drive %d start\n", sim->t(), n);
		auto w0 = std::chrono::steady_clock::now();
		uint64_t r0 = sim->resumes, s0 = sim->sleeps, f0[16];
		std::memcpy(f0, sim->fired, sizeof f0);

		fwb_drive(n);

		current = 0;
		cur->t_end = sim->t();
		if (getenv("LINESIM_PROFILE")) {
			FILETIME c, e, k, u;
			GetThreadTimes(GetCurrentThread(), &c, &e, &k, &u);
			double cpu = (((uint64_t) u.dwHighDateTime << 32) | u.dwLowDateTime) * 1e-7;
			std::printf("  profile: wall %.2f s, thread cpu total %.2f s, resumes %llu, sleeps %llu, events:",
				std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count(), cpu,
				(unsigned long long) (sim->resumes - r0), (unsigned long long) (sim->sleeps - s0));
			for (int k = 0; k < 9; k++)
				std::printf(" %llu", (unsigned long long) (sim->fired[k] - f0[k]));
			std::printf("\n");
		}
		if (!lcd_captured)
			cur->lcd = lcd_text();
		FwMarker m[256];
		int nm = fwb_marker_log(m, 256);
		cur->markers.assign(m, m + nm);
		if (!cur->samples.empty())
			cur->rms_lat_mm = std::sqrt(lat_sq / cur->samples.size());
		if (!ramp_seen)
			cur->outcome = "not_started";
		else if (lifted)
			cur->outcome = lift_reason;
		else if (cur->lcd.find("Line Lost") != std::string::npos)
			cur->outcome = "line_lost";
		else
			cur->outcome = "finished";

		// logged markers vs. the course
		std::string got, want = track.expected_markers();
		for (const FwMarker& x : cur->markers)
			got += "?LRCU"[x.type >= 1 && x.type <= 4 ? x.type : 0];
		size_t k = 0;
		while (k < got.size() && k < want.size() && got[k] == want[k])
			k++;
		char buf[96];
		if (k == got.size() && k == want.size())
			cur->marker_check = "OK";
		else if (k < got.size() && k < want.size())
			std::snprintf(buf, sizeof buf, "#%zu %c (expected %c)", k, got[k], want[k]), cur->marker_check = buf;
		else
			std::snprintf(buf, sizeof buf, "%zu of %zu markers", got.size(), want.size()), cur->marker_check = buf;

		if (!opt.quiet)
			std::printf("[%8.3f s] drive %d %s, lap %.2f s, markers %s\n", sim->t(), n, cur->outcome.c_str(),
				cur->lap_time_fw, cur->marker_check.c_str());
		cur = nullptr;
	};

	// Session: replace the course by a new random one (firmware idle, between events).
	auto new_map = [&]() {
		TrackGenOptions g;
		g.seed = ++map_seed;
		std::string e;
		std::string text = generate_track(g, &e);
		if (text.empty() || !track.load_text(text, "random", &e)) {
			session.message = "map generator: " + e;
			return;
		}
		save_generated(text, g.seed);
		if (viewer)
			viewer->set_track(track);
		start = start_pose();
		robot->place(start);
		ir.set_lifted(false);
		session.map_name = track.name();
		session.expected = track.expected_markers();
		session.message = "new course - run drive 1 first (2-4 replay its marker log)";
		if (!opt.quiet)
			std::printf("[%8.3f s] new course %s (%.2f m)\n", sim->t(), res.track_path_used.c_str(), track.length());
	};

	sim->start([&]() {
		robot->place(start);
		fwb_boot();

		uint16_t white[IrBar::kChannels], black[IrBar::kChannels];
		ir.calibration(white, black);
		fwb_install_sensor_calibration(white, black);
		fwb_save_sensor_calibration();

		FwDriveParams p;
		fwb_get_drive_params(&p);
		for (const auto& kv : opt.params)
			apply_param(&p, kv.first, kv.second);
		fwb_set_drive_params(&p);
		res.params = p;
		session.params = p;
		session.ready = true;
		booted = true;

		if (opt.interactive) {   // like main.c: the menu loop forever, buttons from the keyboard
			for (;;) {
				fwb_menu_step();
				sim_fw_yield();   // main.c's while(1) is firmware code and yields per pass; this loop is ours
			}
		}

		if (opt.session) {   // wait for keys from the viewer, forever
			for (const int n : opt.drives) {   // scenario drives first, if any were given
				fwb_set_drive_params(&session.params);
				session.busy = true;
				run_one(n);
				session.busy = false;
			}
			GainTuner tuner;
			const double dt = fw_steer_dt();
			for (;;) {
				if (session.auto_tune) {   // endless: drive 1 on this course, score, adjust speed and gains
					session.request_drive = 0;
					session.request_new_map = false;
					if (!tuner.active) {
						tuner.start(session.params.base_mps, session.params.steer_kp, session.params.steer_kd * dt);
						session.busy = true;
						session.message.clear();
					}
					auto [kp, kd] = tuner.next();
					session.params.base_mps = static_cast<float>(tuner.speed());
					session.params.steer_kp = static_cast<float>(kp);
					session.params.steer_kd = static_cast<float>(kd / dt);
					fwb_set_drive_params(&session.params);
					res.params = session.params;
					session.tune_lines = tuner.status(dt);
					// drive 1 learns the course; drive 2 accelerates on its straights from that data
					auto drive = [&](int n) {
						run_one(n);
						DriveResult d = std::move(res.drives.back());
						res.drives.pop_back();   // endless: keep the scores, not every drive's samples
						return d;
					};
					auto lost_line = [](const DriveResult& d) { return d.outcome != "finished" || d.max_lat_mm > 70.0; };
					const DriveResult d1 = drive(1);
					if (!session.auto_tune)   // stopped (A or X) during this drive: not a fair trial
						continue;
					const double weave1 = weave_dps(d1.samples), slip1 = slip_pct(d1.samples);
					const double clean1 = d1.rms_lat_mm + weave1 / 20.0 + slip1 / 2.0;   // drive 1 data quality
					char text[200];
					bool lost = lost_line(d1), pass = false;
					double score = 0, lap2 = 0;
					if (lost) {
						score = 1000;
						std::snprintf(text, sizeof text, "1: %s", d1.outcome.c_str());
					} else if (d1.marker_check != "OK") {
						score = 500 + clean1;
						std::snprintf(text, sizeof text, "1: markers %s", d1.marker_check.c_str());
					} else {
						const DriveResult d2 = drive(2);
						if (!session.auto_tune)
							continue;
						const bool ok2 = !lost_line(d2) && d2.marker_check == "OK" && d2.lap_time_fw > 0;
						lost = lost_line(d2);
						pass = ok2;
						lap2 = d2.lap_time_fw;
						score = ok2 ? 5.0 * lap2 + clean1 : (lost ? 300.0 : 200.0) + clean1;
						std::snprintf(text, sizeof text, "1: lat %.1f w %.0f slip %.0f%% | 2: %s", d1.rms_lat_mm, weave1, slip1,
							ok2 ? (std::to_string(lap2).substr(0, 5) + " s").c_str()
							    : (lost ? d2.outcome.c_str() : ("markers " + d2.marker_check).c_str()));
						if (pass) {
							// Model-error stress: the steering lag (sensor_delay, fitted to one gyro log) may be
							// larger on the real robot. Drive 1 again with it x1.4 (must keep the line, weave at
							// most 1.5 x nominal + 10 dps) and x1.85 (must keep the line). Failing counts as unstable.
							const double nominal = ir.delay();
							std::string why;
							ir.set_delay(nominal * kStressLag[0]);
							const DriveResult s1 = drive(1);
							if (lost_line(s1))
								why = "x1.4: " + s1.outcome;
							else if (weave_dps(s1.samples) > 1.5 * weave1 + 10.0)
								why = "x1.4: weave " + std::to_string(static_cast<int>(weave_dps(s1.samples))) + " dps";
							if (why.empty() && session.auto_tune) {
								ir.set_delay(nominal * kStressLag[1]);
								const DriveResult s2 = drive(1);
								if (lost_line(s2))
									why = "x1.85: " + s2.outcome;
							}
							ir.set_delay(nominal);
							if (!session.auto_tune)
								continue;
							if (!why.empty()) {
								lost = true;   // unstable with a bit more lag: the search backs away from it
								pass = false;
								score = 250.0 + clean1;
								std::snprintf(text, sizeof text, "2: %.2f s but lag %s", lap2, why.c_str());
							}
						}
					}
					tuner.record(lost, pass, score, lap2, text);
					if (tuner.round_complete()) {
						tuner.finish_round();
						if (!opt.quiet) {
							const std::vector<std::string> st = tuner.status(dt);
							std::printf("[%8.3f s] auto-tune %s | %s | %s\n", sim->t(), st[0].c_str() + 11, st[3].c_str(),
								st.size() > 4 ? st[4].c_str() : "");
							std::fflush(stdout);
						}
					}
					session.tune_lines = tuner.status(dt);
					continue;
				}
				if (tuner.active) {   // A pressed again: keep the best speed and gains found
					tuner.active = false;
					if (tuner.has_best())
						session.params.base_mps = static_cast<float>(tuner.best_v());
					session.params.steer_kp = static_cast<float>(tuner.best_kp());
					session.params.steer_kd = static_cast<float>(tuner.best_kd() / dt);
					fwb_set_drive_params(&session.params);
					session.busy = false;
					char line[200];
					if (tuner.has_best())
						std::snprintf(line, sizeof line, "auto-tune: best %.3f m/s, steer kp %.1f, kd %.3f s (fw steer_gain_d %.0f)",
							tuner.best_v(), tuner.best_kp(), tuner.best_kd(), tuner.best_kd() / dt);
					else
						std::snprintf(line, sizeof line, "auto-tune: no speed passed yet (kept kp %.1f, kd %.3f s)",
							tuner.best_kp(), tuner.best_kd());
					session.history.push_back(line);
					session.message = "kept the best found (drive 1-4 to try it)";
					session.tune_lines.clear();
				}
				if (session.request_new_map) {
					session.request_new_map = false;
					new_map();
				} else if (session.request_drive) {
					int n = session.request_drive;
					session.request_drive = 0;
					fwb_set_drive_params(&session.params);
					res.params = session.params;
					session.busy = true;
					session.message.clear();
					run_one(n);
					session.busy = false;
					const DriveResult& d = res.drives.back();
					char line[160];
					std::snprintf(line, sizeof line, "drive %d  %-9s lap %5.2f s  vmax %.2f  markers %s", n, d.outcome.c_str(),
						d.lap_time_fw, d.max_v, d.marker_check.c_str());
					session.history.push_back(line);
				} else {
					sim_fw_sleep_until_ns(sim_now_ns() + 1000000);
				}
			}
		}

		for (int n : opt.drives)
			run_one(n);
	});

	const double t_limit = (opt.interactive || opt.session) ? 1e7 : 30.0 + opt.drives.size() * (opt.max_drive_time + 20.0);
	Simulation::Stop st = sim->run(t_limit, [&]() { return live && live->stop_requested(); });
	if (live && st != Simulation::Stop::Predicate)
		live->hold_until_closed();
	res.sim_seconds = sim->t();
	res.wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
	if (st == Simulation::Stop::Halted) {
		res.halt_reason = sim->halt_reason();
		res.error = "firmware halted: " + res.halt_reason;
		return res;
	}
	if (st == Simulation::Stop::Predicate && !opt.interactive && !opt.session) {
		res.error = "stopped from the viewer before the run finished";
		return res;
	}
	if (st == Simulation::Stop::TimeLimit) {
		res.error = "simulation time limit reached (firmware stuck waiting?)\nLCD:\n" + lcd_text();
		return res;
	}
	res.ok = true;
	return res;
}
