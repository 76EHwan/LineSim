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
#include <fstream>
#include <future>
#include <memory>
#include <random>
#include <regex>

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
// rms line error [mm] at the sensor bar over the samples on arcs only: on the straights any gains
// keep the line, the curves are where the steering shows
double curve_rms_lat(const std::vector<Sample>& smp, const Track& track) {
	const std::vector<PathPiece>& ps = track.pieces();
	double sq = 0;
	int n = 0;
	for (const Sample& s : smp) {
		auto it = std::upper_bound(ps.begin(), ps.end(), static_cast<double>(s.s),
			[](double v, const PathPiece& p) { return v < p.s0; });
		if (it == ps.begin() || !std::prev(it)->arc)
			continue;
		sq += static_cast<double>(s.lat_mm) * s.lat_mm;
		n++;
	}
	return n ? std::sqrt(sq / n) : 0.0;
}

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

// Returns true if vehicle experienced significant oversteer / spin-out / drift (sideslip beta > max_beta_deg)
bool check_oversteer(const DriveResult& d, double max_beta_deg = 0.0, double min_v_mps = 0.6) {
	if (max_beta_deg <= 0.0) {
		static const double env_deg = std::getenv("LINESIM_MAX_BETA") ? std::atof(std::getenv("LINESIM_MAX_BETA")) : 25.0;
		max_beta_deg = env_deg;
	}
	if (d.max_beta_deg > 0.0 && d.max_beta_deg < max_beta_deg)
		return false;
	if (d.samples.size() < 10)
		return false;
	for (size_t i = 1; i < d.samples.size(); ++i) {
		const double dt = d.samples[i].t - d.samples[i - 1].t;
		if (dt < 1e-5)
			continue;
		const double dx = d.samples[i].x - d.samples[i - 1].x;
		const double dy = d.samples[i].y - d.samples[i - 1].y;
		const double v = std::hypot(dx, dy) / dt;
		if (v < min_v_mps)
			continue;
		const double th_traj = std::atan2(dy, dx) * 180.0 / 3.14159265358979323846;
		const double th_veh = d.samples[i].yaw_deg;
		const double diff = std::remainder(th_veh - th_traj, 360.0);
		if (std::abs(diff) > max_beta_deg)
			return true;
	}
	return false;
}

// Session auto-tune (A key): base speed, steer kp / kd (seconds), lat_acc_max and slalom_ratio for
// the race. Endless, until A is pressed again.
//
// Several courses at once, against over-fitting: every trial runs on the session's course and
// kTuneCourses - 1 random ones. On each: drive 1 (learning: markers and distances must come out
// clean), then drive 4, the race run (target_pos shift + acceleration on the straights from that
// data; the parameters not tuned here as set in the panel). A trial passes when every drive finishes,
// never more than 70 mm off the line, with every marker read right, drive 1 on the first course still
// holds with more steering lag than the model has (kStressLag: the lag was fitted to one gyro log),
// and its drive 4 pace beats the BEST so far: a pass that is slower is thrown out like a failure.
// Pace = mean over the courses of drive 4 lap / course length, in s per 10 m.
// Score (lower is better): drive 1 counts accuracy only, drive 4 accuracy and time:
// mean over the courses of acc(1) + acc(4), + 10 x pace; acc = rms line error in the curves [mm]
// + weave / 20 + wheel slip [%] / 2 (on the straights any gains keep the line).
//
// Rounds alternate between two pairs of parameters, each searched as a 3 x 3 block around its
// centre (the panel's values at the start; steps kp 10 %, kd 15 %, lat 1 m/s2, slalom 0.03): gains
// (kp x kd) and curve entry (lat_acc_max x slalom_ratio: how hard drive 4 slows for a curve after a
// straight). When 6 or more of the 9 have a score, a quadratic surface fitted to them gives the
// best point between the grid points: if it is a minimum away from the tried points, it is tried
// as a 10th. Of the passes whose block neighbours did not lose the line, the gentlest within 1 % of
// the best score wins (lower kp / lat first) and becomes BEST. Coarse to fine: a winner in the
// middle of the block (or the fitted point) halves the steps, a winner on the edge moves the block
// there; no winner halves the steps too.
// The speed changes after each gains + curve pair of rounds: something passed at this speed, so the
// next pair tries dv faster (0.1 m/s at first); nothing passed: back to the best speed + dv/2 (down
// to 0.01 m/s steps; below that it re-checks).
// stale_drives() drives in a row (as numbered in the lap list) without a new BEST: the session
// replaces one of the random courses and starts over from BEST's values (paces belong to courses).
constexpr int kTuneCourses = 3;

class GainTuner {
public:
	struct Set {
		double kp = 16.64, kd = 0.101;     // steering PD, kd in seconds (202 * 0.0005s)
		double lat = 14.77, sl = 0.10;     // lat_acc_max [m/s2], slalom_ratio
		double target_shift = 0.40;         // target_shift_val
		double target_curve_gain = 1.50;    // target_curve_gain
		double brake_margin = 0.10;         // brake_margin_m [m]
		double pos_atten = 0.0;             // pos_atten_gain (fixed 0.0)
		double yaw_kp = 0.33, yaw_ki = 0.9; // yaw cascade
		double accel = 6.0, decel = 6.0;    // accel/decel
		double max_mps = 10.0;              // max speed
	};

	bool active = false;

	static int target_window() {
		static const int w = std::getenv("LINESIM_ENDLESS_WINDOW")
			? std::max(10, std::atoi(std::getenv("LINESIM_ENDLESS_WINDOW"))) : 100;
		return w;
	}

	static double dv_down() {
		static const double d = std::getenv("LINESIM_ENDLESS_DV_DOWN")
			? std::max(0.005, std::atof(std::getenv("LINESIM_ENDLESS_DV_DOWN"))) : 0.01;
		return d;
	}

	static double dv_up() {
		static const double u = std::getenv("LINESIM_ENDLESS_DV_UP")
			? std::max(0.005, std::atof(std::getenv("LINESIM_ENDLESS_DV_UP"))) : 0.01;
		return u;
	}

	void start(double v, const Set& from) {
		active = true;
		v_ = std::max(0.50, v);
		verified_v_ = 0.0;
		best_ = from;
		best_.pos_atten = 0.0;
		streak_ = 0;
		total_maps_ = 0;
		total_passes_ = 0;
		total_fails_ = 0;
		speed_changed_ = false;
		just_verified_ = false;
		window_.clear();
		last_.clear();
		log_.clear();
	}

	double speed() const { return v_; }
	size_t current_map_no() const { return static_cast<size_t>(total_maps_ + 1); }
	bool has_verified() const { return verified_v_ > 0.0; }
	double verified_v() const { return verified_v_; }
	const Set& best() const { return best_; }

	bool consume_speed_changed(bool* out_verified = nullptr) {
		if (out_verified)
			*out_verified = just_verified_;
		bool res = speed_changed_;
		speed_changed_ = false;
		just_verified_ = false;
		return res;
	}

	int fails_in_window() const {
		int f = 0;
		for (bool ok : window_)
			if (!ok) f++;
		return f;
	}

	double fail_rate_pct() const {
		if (window_.empty())
			return 0.0;
		return 100.0 * static_cast<double>(fails_in_window()) / static_cast<double>(window_.size());
	}

	void record_map(bool pass, double pace, double lap4_s, double dist_diff, const std::string& detail) {
		const int win_max = target_window();
		total_maps_++;
		window_.push_back(pass);
		if (static_cast<int>(window_.size()) > win_max)
			window_.erase(window_.begin());

		char log_buf[180];
		if (!pass) {
			total_fails_++;
			const double old_v = v_;
			v_ = std::max(0.50, v_ - dv_down());
			streak_ = 0;
			speed_changed_ = true;
			just_verified_ = false;
			last_ = fmt("Map #%d FAIL at %.2f m/s (%s) -> base speed down to %.2f m/s",
				total_maps_, old_v, detail.c_str(), v_);
			std::snprintf(log_buf, sizeof log_buf, "#%03d [FAIL] %.2f->%.2fm/s | %s",
				total_maps_, old_v, v_, detail.c_str());
		} else {
			total_passes_++;
			streak_++;
			const int fails_100 = fails_in_window();
			const double fr_pct = fail_rate_pct();
			// 100 attempts at current speed with failure rate < 1% (0 fails in 100 runs)
			if (streak_ >= win_max && static_cast<int>(window_.size()) >= win_max && fr_pct < 1.0) {
				const double old_v = v_;
				verified_v_ = std::max(verified_v_, old_v);
				v_ = old_v + dv_up();
				streak_ = 0;
				speed_changed_ = true;
				just_verified_ = true;
				last_ = fmt("Map #%d: %d/%d PASS (Fail %.1f%% < 1%%) at %.2f m/s! -> speed up to %.2f m/s",
					total_maps_, win_max, win_max, fr_pct, old_v, v_);
				std::snprintf(log_buf, sizeof log_buf, "#%03d [100 PASS <1%%] %.2f->%.2fm/s | 4:%.2fs (%+.2fm)",
					total_maps_, old_v, v_, lap4_s, dist_diff);
			} else {
				last_ = fmt("Map #%d PASS (%d/%d) at %.2f m/s | 4th: %.2fs (%.2fs/10m, dist %+.2fm)",
					total_maps_, streak_, win_max, v_, lap4_s, pace, dist_diff);
				std::snprintf(log_buf, sizeof log_buf, "#%03d [PASS %d/%d] %.2fm/s | 4:%.2fs (%+.2fm)",
					total_maps_, streak_, win_max, v_, lap4_s, dist_diff);
			}
		}
		log_.push_back(log_buf);
		if (log_.size() > 6)
			log_.erase(log_.begin());
	}

	std::vector<std::string> status(double dt, const std::string& cur_map_info) const {
		const int win_max = target_window();
		std::vector<std::string> v;
		v.push_back(fmt("ENDLESS SPEED TUNER  Map #%d  (A: stop/save)", total_maps_ + 1));
		v.push_back("Mode: Drive 1 + Drive 4 -> New Map Every Trial");
		v.push_back("Map:  " + cur_map_info);
		if (verified_v_ > 0.0)
			v.push_back(fmt("Base Speed: %.2f m/s  (Verified <1%% fail: %.2f m/s)", v_, verified_v_));
		else
			v.push_back(fmt("Base Speed: %.2f m/s  (Verifying <1%% fail over %d maps)", v_, win_max));
		v.push_back(fmt("Streak at %.2f m/s: %d / %d  (Fail rate <1%% needs %d/%d)",
			v_, streak_, win_max, win_max, win_max));
		v.push_back(fmt("Last %zu Maps Fail Rate: %.1f%% (%d fail / %zu runs) | Total: %dP / %dF",
			window_.size(), fail_rate_pct(), fails_in_window(), window_.size(), total_passes_, total_fails_));
		v.push_back(fmt("Fixed Gains: kp=%.2f kd=%.0f lat=%.2f brk=%.2fm shf=%.2f crv=%.2f",
			best_.kp, best_.kd / dt, best_.lat, best_.brake_margin, best_.target_shift, best_.target_curve_gain));
		if (!last_.empty())
			v.push_back(last_);
		for (const std::string& l : log_)
			v.push_back(l);
		return v;
	}

private:
	static std::string fmt(const char* f, ...) {
		char buf[240];
		va_list ap;
		va_start(ap, f);
		std::vsnprintf(buf, sizeof buf, f, ap);
		va_end(ap);
		return buf;
	}

	double v_ = 2.04;
	double verified_v_ = 0.0;
	Set best_;
	int streak_ = 0;
	int total_maps_ = 0;
	int total_passes_ = 0;
	int total_fails_ = 0;
	bool speed_changed_ = false;
	bool just_verified_ = false;
	std::vector<bool> window_;
	std::string last_;
	std::vector<std::string> log_;
};
}  // namespace

bool save_drive_params_to_firmware_file(const FwDriveParams& p, std::string* out_path) {
	const std::vector<std::string> candidates = {
		"firmware/Main/Src/drive.c",
		"../firmware/Main/Src/drive.c",
		"../../firmware/Main/Src/drive.c",
		"../../../firmware/Main/Src/drive.c"
	};
	std::string found_path;
	for (const auto& path : candidates) {
		std::error_code ec;
		if (fs::exists(path, ec)) {
			found_path = fs::canonical(path, ec).string();
			if (found_path.empty())
				found_path = path;
			break;
		}
	}
	if (found_path.empty()) {
		std::fprintf(stderr, "[SAVE ERROR] Could not find firmware/Main/Src/drive.c\n");
		return false;
	}

	if (out_path)
		*out_path = found_path;

	std::ifstream in(found_path, std::ios::binary);
	if (!in.is_open()) {
		std::fprintf(stderr, "[SAVE ERROR] Cannot open %s for reading\n", found_path.c_str());
		return false;
	}
	std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	in.close();

	const std::string marker = "DriveParam_t driveData";
	size_t pos_marker = content.find(marker);
	if (pos_marker == std::string::npos) {
		std::fprintf(stderr, "[SAVE ERROR] 'DriveParam_t driveData' not found in %s\n", found_path.c_str());
		return false;
	}
	size_t open_brace = content.find('{', pos_marker);
	if (open_brace == std::string::npos) {
		std::fprintf(stderr, "[SAVE ERROR] '{' not found after marker in %s\n", found_path.c_str());
		return false;
	}
	size_t close_brace = content.find("};", open_brace);
	if (close_brace == std::string::npos) {
		std::fprintf(stderr, "[SAVE ERROR] '};' not found after '{' in %s\n", found_path.c_str());
		return false;
	}

	char new_body[1024];
	std::snprintf(new_body, sizeof new_body,
		"\n    .base_mps = %.2ff,"
		"\n    .max_mps = %.2ff,"
		"\n    .accel = %.1ff,"
		"\n    .decel = %.1ff,"
		"\n    .steer_gain_p = %.2ff,"
		"\n    .steer_gain_d = %.1ff,"
		"\n    .pos_atten_gain = %.2ff,"
		"\n    .pit_in_distance_m = %.2ff,"
		"\n    .fan_en = %d,"
		"\n\t.target_shift_val = %.2ff,"
		"\n\t.yaw_gain_p = %.2ff,"
		"\n\t.yaw_gain_i = %.1ff,"
		"\n\t.lat_acc_max = %.2ff,"
		"\n\t.target_curve_gain = %.2ff,"
		"\n\t.slalom_ratio = %.2ff,"
		"\n\t.brake_margin_m = %.2ff,\n",
		p.base_mps, p.max_mps > 0 ? p.max_mps : 10.0f,
		p.accel > 0 ? p.accel : 10.0f, p.decel > 0 ? p.decel : 10.0f,
		p.steer_kp, p.steer_kd, p.pos_atten_gain,
		p.pit_in_m > 0 ? p.pit_in_m : 0.15f, p.fan_en,
		p.target_shift,
		p.yaw_kp, p.yaw_ki,
		p.lat_acc_max,
		p.target_curve_gain,
		p.slalom_ratio,
		p.brake_margin_m > 0 ? p.brake_margin_m : 0.12f
	);

	content.replace(open_brace + 1, close_brace - (open_brace + 1), new_body);

	std::ofstream out(found_path, std::ios::binary | std::ios::trunc);
	if (!out.is_open()) {
		std::fprintf(stderr, "[SAVE ERROR] Cannot open %s for writing\n", found_path.c_str());
		return false;
	}
	out << content;
	out.close();
	std::printf("[SAVE] Parameters written directly to %s\n", found_path.c_str());
	return true;
}

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
	else if (key == "yaw_kp") p->yaw_kp = (float) v;              // cascade steering, inner gyro loop
	else if (key == "yaw_ki") p->yaw_ki = (float) v;
	else if (key == "lat_acc_max") p->lat_acc_max = (float) v;
	else if (key == "target_curve_gain") p->target_curve_gain = (float) v;   // drive 3/4: target_pos x gain in curves
	else if (key == "slalom_ratio") p->slalom_ratio = (float) v;             // drive 4: 45 deg speed ratio
	else if (key == "brake_margin_m" || key == "brake_margin") p->brake_margin_m = (float) v;
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
	// Saves a generated course into the output folder and returns its path. Only putting a course on
	// the floor makes it res.track_path_used (auto-tune generates courses it does not drive yet).
	auto save_generated = [&](const std::string& text, unsigned seed) {
		fs::path f = out / ("random_" + std::to_string(seed) + ".json");
		FILE* fp = std::fopen(f.string().c_str(), "wb");
		if (fp) {
			std::fwrite(text.data(), 1, text.size(), fp);
			std::fclose(fp);
		}
		return f.string();
	};
	if (!generated.empty())
		res.track_path_used = save_generated(generated, map_seed);
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
	bool drive1_ok = false;       // the last drive 1 read the course's marker sequence: drives 2-4 may run
	int lap_no = 0;               // drives so far (lap list in the viewer)
	std::string lap_key, lap_label;   // course of the next laps in the list ("" = the one on the floor)
	std::string lap_tag;          // explicit attempt tag for auto-tune (e.g. "#15-1")
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
		smp.cmd_mps = tm.cmd_mps;
		smp.tgt_pos = tm.target_pos;
		smp.odom = tm.odom_m;
		smp.yaw_fw = tm.yaw_deg;
		smp.lost = tm.lost;
		smp.markers = tm.marker_count;
		smp.contact = r.chassis_contact();
		smp.beta_deg = 0.0f;
		if (cur->samples.size() >= 5) {
			const Sample& pv = cur->samples[cur->samples.size() - 5];
			double dt = smp.t - pv.t;
			if (dt > 1e-5 && smp.v >= 0.5f) {
				double dx = smp.x - pv.x;
				double dy = smp.y - pv.y;
				double th_traj = std::atan2(dy, dx) * 180.0 / 3.14159265358979323846;
				double diff = std::remainder((double) smp.yaw_deg - th_traj, 360.0);
				smp.beta_deg = static_cast<float>(std::abs(diff));
			}
		} else if (!cur->samples.empty()) {
			const Sample& pv = cur->samples.back();
			double dt = smp.t - pv.t;
			if (dt > 1e-5 && smp.v >= 0.5f) {
				double dx = smp.x - pv.x;
				double dy = smp.y - pv.y;
				double th_traj = std::atan2(dy, dx) * 180.0 / 3.14159265358979323846;
				double diff = std::remainder((double) smp.yaw_deg - th_traj, 360.0);
				smp.beta_deg = static_cast<float>(std::abs(diff));
			}
		}
		if (smp.v >= 0.5f)
			cur->max_beta_deg = std::max(cur->max_beta_deg, (double) smp.beta_deg);
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
		const std::string attempt_tag = lap_tag.empty() ? ("#" + std::to_string(++lap_no)) : lap_tag;
		if (!opt.quiet)
			std::printf("[%8.3f s] drive %d (%s) start\n", sim->t(), n, attempt_tag.c_str());
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
		else if (cur->max_beta_deg >= 25.0 && cur->max_lat_mm <= 70.0)
			cur->outcome = "finished (oversteer)";
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

		if (!opt.quiet) {
			const double d_diff = cur->dist_m - track.length();
			if (cur->outcome == "finished (oversteer)") {
				std::printf("[%8.3f s] drive %d (%s) finished (oversteer occurred, completed within track limits), lap %.2f s, dist %.2fm (%+4.2fm), markers %s, max beta %.1f deg\n",
					sim->t(), n, attempt_tag.c_str(), cur->lap_time_fw, cur->dist_m, d_diff, cur->marker_check.c_str(), cur->max_beta_deg);
			} else if (cur->max_beta_deg >= 20.0) {
				const bool limit_exceeded = !cur->is_finished() || cur->max_lat_mm > 70.0;
				std::printf("[%8.3f s] drive %d (%s) %s, lap %.2f s, dist %.2fm (%+4.2fm), markers %s, max beta %.1f deg%s\n",
					sim->t(), n, attempt_tag.c_str(), cur->outcome.c_str(), cur->lap_time_fw, cur->dist_m, d_diff, cur->marker_check.c_str(),
					cur->max_beta_deg,
					cur->max_beta_deg >= 25.0 ? (limit_exceeded ? " (OVERSTEER: OFF LIMIT)" : " (DRIFT: LIMIT OK)") : "");
			} else {
				std::printf("[%8.3f s] drive %d (%s) %s, lap %.2f s, dist %.2fm (%+4.2fm), markers %s\n", sim->t(), n, attempt_tag.c_str(),
					cur->outcome.c_str(), cur->lap_time_fw, cur->dist_m, d_diff, cur->marker_check.c_str());
			}
		}
		if (n == 1)
			drive1_ok = cur->is_finished() && cur->marker_check == "OK";
		if (live) {   // lap list on the left: the firmware's lap time, else the time driven until it stopped
			const bool ok = cur->is_finished() && cur->marker_check == "OK" && cur->lap_time_fw > 0;
			char what[160];
			if (cur->lap_time_fw > 0) {
				const char* mk = cur->marker_check == "OK" ? "OK" : cur->marker_check.c_str();
				const double d_diff = cur->dist_m - track.length();
				if (cur->outcome == "finished (oversteer)")
					std::snprintf(what, sizeof what, "%6.2f s %s (%+4.2fm, ov)", cur->lap_time_fw, mk, d_diff);
				else
					std::snprintf(what, sizeof what, "%6.2f s %s (%+4.2fm)", cur->lap_time_fw, mk, d_diff);
			} else {
				double t0 = -1, t1 = 0;
				for (const Sample& smp : cur->samples)
					if (smp.tgt_base > 0.01f) {
						if (t0 < 0)
							t0 = smp.t;
						t1 = smp.t;
					}
				std::snprintf(what, sizeof what, "~%5.2f s %s %s", t0 >= 0 ? t1 - t0 : 0.0, cur->outcome.c_str(),
					cur->marker_check == "OK" ? "" : cur->marker_check.c_str());
			}
			live->record_lap(lap_key.empty() ? track.name() : lap_key, lap_label.empty() ? track.name() : lap_label, n,
				attempt_tag, ok, cur->lap_time_fw, what);
		}
		cur = nullptr;
	};

	// Drives 2-4 follow drive 1's marker log: they run only when drive 1 read the course right.
	auto skip_drive = [&](int n) {
		res.drives.emplace_back();
		DriveResult& d = res.drives.back();
		d.n = n;
		d.outcome = "skipped";
		d.marker_check = "drive 1 markers wrong";
		d.t_start = d.t_end = sim->t();
		const std::string attempt_tag = lap_tag.empty() ? ("#" + std::to_string(++lap_no)) : lap_tag;
		if (!opt.quiet)
			std::printf("[%8.3f s] drive %d (%s) skipped: drive 1 did not read the markers right\n", sim->t(), n, attempt_tag.c_str());
		if (live)
			live->record_lap(track.name(), track.name(), n, attempt_tag, false, 0.0, "skipped (drive 1 markers wrong)");
	};
	auto run_or_skip = [&](int n) {
		if (n >= 2 && !drive1_ok)
			skip_drive(n);
		else
			run_one(n);
	};

	// Session: replace the course by a new random one (firmware idle, between events).
	// Background prefetch queue so the next random map is already built while the robot is driving.
	std::future<std::pair<std::string, unsigned>> prefetched_map;
	auto start_map_prefetch = [&]() {
		if (!prefetched_map.valid()) {
			const unsigned next_seed = ++map_seed;
			prefetched_map = std::async(std::launch::async, [next_seed]() {
				TrackGenOptions g;
				g.seed = next_seed;
				std::string e;
				return std::make_pair(generate_track(g, &e), next_seed);
			});
		}
	};
	if (opt.session)
		start_map_prefetch();

	// Session: a new random course (saved into the output folder), "" on failure.
	auto make_map = [&](std::string* label) {
		std::string text;
		unsigned used_seed = 0;
		if (prefetched_map.valid()) {
			auto res_pair = prefetched_map.get();
			text = std::move(res_pair.first);
			used_seed = res_pair.second;
		}
		start_map_prefetch();   // immediately start generating the next map in the background
		if (text.empty()) {
			TrackGenOptions g;
			g.seed = ++map_seed;
			used_seed = g.seed;
			std::string e;
			text = generate_track(g, &e);
			if (text.empty()) {
				session.message = "map generator: " + e;
				return text;
			}
		}
		*label = save_generated(text, used_seed);   // load_course() makes it the course on the floor
		return text;
	};
	// Session: put a course on the floor (firmware idle, between events).
	auto load_course = [&](const std::string& text, const std::string& label) {
		std::string e;
		if (!track.load_text(text, label, &e)) {
			session.message = "course: " + e;
			return false;
		}
		res.track_path_used = label;
		if (viewer)
			viewer->set_track(track);
		start = start_pose();
		robot->place(start);
		ir.set_lifted(false);
		session.map_name = track.name();
		session.expected = track.expected_markers();
		drive1_ok = false;
		return true;
	};
	// Session: replace the course by a new random one.
	auto new_map = [&]() {
		std::string label;
		const std::string text = make_map(&label);
		if (text.empty() || !load_course(text, label))
			return;
		session.message = "new course - run drive 1 first (2-4 replay its marker log)";
		lap_no = 0;   // lap records belong to a course
		if (live)
			live->clear_laps();
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
				run_or_skip(n);
				session.busy = false;
			}
			GainTuner tuner;
			const double dt = fw_steer_dt();
			bool need_next_map = false;
			auto cur_map_info = [&]() {
				char buf[120];
				std::snprintf(buf, sizeof buf, "%s (%.1f m)", track.name().c_str(), track.length());
				return std::string(buf);
			};
			for (;;) {
				if (session.auto_tune) {   // true endless: drive 1 + drive 4 per map, new map every trial, adapt base_mps for <1% fail over 100 maps
					session.request_drive = 0;
					if (!tuner.active) {
						GainTuner::Set from;
						from.kp = session.params.steer_kp;
						from.kd = session.params.steer_kd * dt;
						from.lat = session.params.lat_acc_max;
						from.sl = session.params.slalom_ratio;
						from.target_shift = session.params.target_shift;
						from.target_curve_gain = session.params.target_curve_gain;
						from.brake_margin = session.params.brake_margin_m;
						from.pos_atten = 0.0;
						from.yaw_kp = session.params.yaw_kp;
						from.yaw_ki = session.params.yaw_ki;
						from.accel = session.params.accel;
						from.decel = session.params.decel;
						from.max_mps = session.params.max_mps;
						tuner.start(session.params.base_mps, from);
						need_next_map = false;
						session.busy = true;
						session.message.clear();
					}
					if (need_next_map || session.request_new_map) {
						session.request_new_map = false;
						need_next_map = false;
						std::string label;
						const std::string fresh = make_map(&label);
						if (!fresh.empty() && load_course(fresh, label)) {
							lap_no = 0;
							if (live)
								live->clear_laps();
						}
					}
					const GainTuner::Set& ts = tuner.best();
					session.params.base_mps = static_cast<float>(tuner.speed());
					session.params.steer_kp = static_cast<float>(ts.kp);
					session.params.steer_kd = static_cast<float>(ts.kd / dt);
					session.params.lat_acc_max = static_cast<float>(ts.lat);
					session.params.slalom_ratio = static_cast<float>(ts.sl);
					session.params.target_shift = static_cast<float>(ts.target_shift);
					session.params.target_curve_gain = static_cast<float>(ts.target_curve_gain);
					session.params.brake_margin_m = static_cast<float>(ts.brake_margin);
					session.params.pos_atten_gain = 0.0f;
					session.params.yaw_kp = static_cast<float>(ts.yaw_kp);
					session.params.yaw_ki = static_cast<float>(ts.yaw_ki);
					session.params.accel = static_cast<float>(ts.accel);
					session.params.decel = static_cast<float>(ts.decel);
					session.params.max_mps = static_cast<float>(ts.max_mps);
					fwb_set_drive_params(&session.params);
					res.params = session.params;
					session.tune_lines = tuner.status(dt, cur_map_info());

					const size_t map_no = tuner.current_map_no();
					auto drive = [&](int n) {
						lap_tag = "#" + std::to_string(map_no) + "-d" + std::to_string(n);
						run_one(n);
						lap_tag.clear();
						DriveResult d = std::move(res.drives.back());
						res.drives.pop_back();
						return d;
					};
					auto track_limit_mm = []() {
						static const double lim = std::getenv("LINESIM_TRACK_LIMIT_MM") ? std::atof(std::getenv("LINESIM_TRACK_LIMIT_MM")) : 70.0;
						return lim;
					};
					auto lost_line = [&](const DriveResult& d) {
						return !d.is_finished() || d.max_lat_mm > track_limit_mm();
					};
					auto oversteered = [&](const DriveResult& d) {
						const bool spinout = check_oversteer(d, 90.0);
						const bool limit_exceeded = lost_line(d) && check_oversteer(d, 25.0);
						return spinout || limit_exceeded;
					};

					bool pass = true;
					double pace = 0.0, lap4_s = 0.0, dist_diff = 0.0;
					char detail[200] = "";

					lap_key = track.name();
					lap_label = cur_map_info();

					// 1st drive: course learning
					const DriveResult d1 = drive(1);
					if (!session.auto_tune) {
						lap_key.clear();
						lap_label.clear();
						continue;
					}
					if (lost_line(d1) || d1.marker_check != "OK" || oversteered(d1)) {
						const bool ov = oversteered(d1);
						pass = false;
						const char* reason = ov ? "oversteer"
							: (!d1.is_finished() ? d1.outcome.c_str()
							: (d1.max_lat_mm > track_limit_mm() ? "track limit" : ("markers " + d1.marker_check).c_str()));
						std::snprintf(detail, sizeof detail, "1st: %s", reason);
					} else {
						// 4th drive: high-speed race run on the same map
						const DriveResult d4 = drive(4);
						if (!session.auto_tune) {
							lap_key.clear();
							lap_label.clear();
							continue;
						}
						if (lost_line(d4) || d4.marker_check != "OK" || d4.lap_time_fw <= 0 || oversteered(d4)) {
							const bool ov = oversteered(d4);
							pass = false;
							const char* reason = ov ? "oversteer"
								: (!d4.is_finished() ? d4.outcome.c_str()
								: (d4.max_lat_mm > track_limit_mm() ? "track limit" : ("markers " + d4.marker_check).c_str()));
							std::snprintf(detail, sizeof detail, "1st: OK | 4th: %s", reason);
						} else {
							lap4_s = d4.lap_time_fw;
							pace = (track.length() > 0) ? (10.0 * d4.lap_time_fw / track.length()) : 0.0;
							dist_diff = d4.dist_m - track.length();
							std::snprintf(detail, sizeof detail, "1st: OK | 4th: %.2fs (%.2fs/10m, %+.2fm)", lap4_s, pace, dist_diff);
						}
					}
					lap_key.clear();
					lap_label.clear();

					const double prev_v = tuner.speed();
					tuner.record_map(pass, pace, lap4_s, dist_diff, detail);
					bool verified = false;
					if (tuner.consume_speed_changed(&verified)) {
						session.params.base_mps = static_cast<float>(tuner.speed());
						fwb_set_drive_params(&session.params);
						std::string path_out;
						if (save_drive_params_to_firmware_file(session.params, &path_out)) {
							char save_msg[200];
							if (verified) {
								std::snprintf(save_msg, sizeof save_msg,
									"[SAVED <1%% VERIFIED] 100 maps passed at %.2f m/s -> base_mps up to %.2f m/s",
									prev_v, session.params.base_mps);
							} else {
								std::snprintf(save_msg, sizeof save_msg,
									"[SAVED] Map #%zu failed at %.2f m/s (%s) -> base_mps down to %.2f m/s",
									map_no, prev_v, detail, session.params.base_mps);
							}
							session.history.push_back(save_msg);
							if (!opt.quiet) {
								std::printf("[%8.3f s] %s\n", sim->t(), save_msg);
								std::fflush(stdout);
							}
						}
					} else if (!opt.quiet) {
						const std::vector<std::string> st = tuner.status(dt, cur_map_info());
						std::printf("[%8.3f s] %s | %s\n", sim->t(), st[0].c_str(),
							st.size() > 7 ? st[7].c_str() : "");
						std::fflush(stdout);
					}

					// Always switch to a new random map for the next trial
					need_next_map = true;
					session.tune_lines = tuner.status(dt, cur_map_info());
					continue;
				}
				if (tuner.active) {   // A pressed again: keep current adapted base_mps and save to drive.c
					tuner.active = false;
					const GainTuner::Set& b = tuner.best();
					session.params.base_mps = static_cast<float>(tuner.speed());
					session.params.steer_kp = static_cast<float>(b.kp);
					session.params.steer_kd = static_cast<float>(b.kd / dt);
					session.params.lat_acc_max = static_cast<float>(b.lat);
					session.params.slalom_ratio = static_cast<float>(b.sl);
					session.params.target_shift = static_cast<float>(b.target_shift);
					session.params.target_curve_gain = static_cast<float>(b.target_curve_gain);
					session.params.brake_margin_m = static_cast<float>(b.brake_margin);
					session.params.pos_atten_gain = 0.0f;
					session.params.yaw_kp = static_cast<float>(b.yaw_kp);
					session.params.yaw_ki = static_cast<float>(b.yaw_ki);
					session.params.accel = static_cast<float>(b.accel);
					session.params.decel = static_cast<float>(b.decel);
					session.params.max_mps = static_cast<float>(b.max_mps);
					fwb_set_drive_params(&session.params);
					session.busy = false;
					char line[260];
					std::snprintf(line, sizeof line,
						"endless stopped: base_mps %.2f m/s (verified <1%%: %.2f m/s), fail rate %.1f%%",
						tuner.speed(), tuner.has_verified() ? tuner.verified_v() : tuner.speed(), tuner.fail_rate_pct());
					std::string saved_path;
					if (save_drive_params_to_firmware_file(session.params, &saved_path)) {
						session.message = "endless ended: base_mps saved to drive.c!";
						std::printf("[AUTO-SAVED] Endless base_mps saved to %s\n", saved_path.c_str());
					} else {
						session.message = "endless ended (drive 1-4 to try it)";
					}
					session.history.push_back(line);
					session.tune_lines.clear();
				}
				if (session.request_new_map) {
					session.request_new_map = false;
					new_map();
				} else if (session.request_drive) {
					int n = session.request_drive;
					session.request_drive = 0;
					if (n >= 2 && !drive1_ok) {
						skip_drive(n);
						session.message = "drive 1 did not read the markers right - run drive 1 again";
						continue;
					}
					fwb_set_drive_params(&session.params);
					res.params = session.params;
					session.busy = true;
					session.message.clear();
					run_one(n);
					session.busy = false;
					const DriveResult& d = res.drives.back();
					char line[160];
					const double d_diff = d.dist_m - track.length();
					std::snprintf(line, sizeof line, "drive %d  %-20s lap %5.2f s (%+4.2fm)  vmax %.2f", n, d.outcome.c_str(),
						d.lap_time_fw, d_diff, d.max_v);
					session.history.push_back(line);
					if (d.outcome == "finished (oversteer)")
						session.message = "drive " + std::to_string(n) + " finished (oversteer occurred, completed within track limits)";
				} else {
					sim_fw_sleep_until_ns(sim_now_ns() + 1000000);
				}
			}
		}

		for (int n : opt.drives)
			run_or_skip(n);
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
