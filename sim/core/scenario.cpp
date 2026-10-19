#include "core/scenario.h"

#include <windows.h>

#include <algorithm>
#include <cstdarg>
#include <chrono>
#include <csignal>
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

static volatile std::sig_atomic_t g_stop_requested = 0;

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

struct TuneSet {
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

// [A] 3-Course AI Bayesian Gain Auto-Tuner (c1, c2, c3)
class BayesGainTuner {
public:
	using Set = TuneSet;

	struct TrialRecord {
		Set set;
		double v = 0;
		double score = 1e6;
		double pace = 0;
		bool safe = true;     // true if not oversteered or crashed
		bool pass = false;
		int state = 0;        // 1: lost/oversteer, 2: markers, 3: pass, 4: slower
		std::string text;
	};

	struct Cell {
		Set set;
		int state = 0;        // 0 pending, 1 lost/oversteer, 2 markers, 3 pass, 4 slower
		double score = 0, pace = 0;
		double dist_diff = 0; // mean diff from nominal track length [m] (negative = cut inside)
		double pred_mu = 0, pred_sigma = 0, p_safe = 1.0;
		std::string role;     // "Anchor", "Exploit", "AntiOv", "Explore"
		std::string laps;
		std::vector<double> lap_times; // 4th drive lap times [c1, c2, c3]
	};

	bool active = false;

	void start(double v, const Set& from) {
		active = true;
		min_v_ = std::max(0.3, v);
		v_ = min_v_;
		dv_ = 0.08;
		best_v_ = best_pace_ = 0;
		best_dist_diff_ = 0;
		best_laps_.clear();
		best_lap_times_.clear();
		best_ = from;
		best_.pos_atten = 0.0;
		if (best_.lat <= 0 || best_.lat > 13.0) best_.lat = 11.5;
		if (best_.target_shift < 0.20) best_.target_shift = 0.40;
		if (best_.target_curve_gain < 1.10) best_.target_curve_gain = 1.50;
		if (best_.brake_margin <= 0) best_.brake_margin = 0.14;
		if (best_.accel <= 0) best_.accel = 10.0;
		if (best_.decel <= 0) best_.decel = 10.0;
		if (best_.max_mps <= 0) best_.max_mps = 10.0;
		round_ = 0;
		since_best_ = 0;
		passed_at_v_ = false;
		just_got_best_ = false;
		last_.clear();
		log_.clear();
		plan();
	}

	double speed() const { return v_; }
	size_t current_candidate_no() const { return db_.size() + 1; }
	bool has_best() const { return best_v_ > 0; }
	double best_v() const { return best_v_; }
	double best_pace() const { return best_pace_; }
	double best_dist_diff() const { return best_dist_diff_; }
	const std::string& best_laps() const { return best_laps_; }
	const std::vector<double>& best_lap_times() const { return best_lap_times_; }
	bool has_best_laps() const { return !best_lap_times_.empty(); }

	bool all_courses_improved(const std::vector<double>& cur_laps) const {
		if (best_lap_times_.empty())
			return true;
		if (cur_laps.size() != best_lap_times_.size())
			return false;
		for (size_t i = 0; i < cur_laps.size(); i++) {
			if (cur_laps[i] >= best_lap_times_[i] - 1e-4)
				return false;
		}
		return true;
	}

	const Set& best() const { return best_; }
	const Set& next() const { return cells_[done_].set; }
	bool round_complete() const { return done_ == cells_.size(); }
	bool just_found_best() {
		bool res = just_got_best_;
		just_got_best_ = false;
		return res;
	}

	size_t trial_count() const { return db_.size(); }
	bool find_best_overall(Set* out_set, double* out_v, double* out_score, std::string* out_text) const {
		if (db_.empty()) return false;
		size_t best_idx = 0;
		double min_score = 1e9;
		for (size_t i = 0; i < db_.size(); i++) {
			if (db_[i].score < min_score) {
				min_score = db_[i].score;
				best_idx = i;
			}
		}
		if (out_set) *out_set = db_[best_idx].set;
		if (out_v) *out_v = db_[best_idx].v;
		if (out_score) *out_score = db_[best_idx].score;
		if (out_text) *out_text = db_[best_idx].text;
		return true;
	}

	static int stale_drives() {
		static const int n = std::getenv("LINESIM_TUNE_STALE") ? std::max(1, std::atoi(std::getenv("LINESIM_TUNE_STALE"))) : 18;
		return n;
	}

	bool stale() const { return since_best_ >= stale_drives(); }

	double kernel(const Set& a, double va, const Set& b, double vb) const {
		const double d_kp = (a.kp - b.kp) / 3.0;
		const double d_kd = (a.kd - b.kd) / 0.030;
		const double d_lat = (a.lat - b.lat) / 1.5;
		const double d_sl = (a.sl - b.sl) / 0.04;
		const double d_tshift = (a.target_shift - b.target_shift) / 0.06;
		const double d_tcgain = (a.target_curve_gain - b.target_curve_gain) / 0.12;
		const double d_brake = (a.brake_margin - b.brake_margin) / 0.04;
		const double d_yaw_p = (a.yaw_kp - b.yaw_kp) / 0.12;
		const double d_v = (va - vb) / 0.15;
		const double d2 = d_kp * d_kp + d_kd * d_kd + d_lat * d_lat + d_sl * d_sl +
		                  d_tshift * d_tshift + d_tcgain * d_tcgain + d_brake * d_brake +
		                  d_yaw_p * d_yaw_p + d_v * d_v;
		return std::exp(-0.5 * d2);
	}

	double stability_kernel(const Set& a, double va, const Set& b, double vb) const {
		const double d_kp = (a.kp - b.kp) / 2.5;
		const double d_kd = (a.kd - b.kd) / 0.025;
		const double d_lat = (a.lat - b.lat) / 1.2;
		const double d_v = (va - vb) / 0.15;
		const double d2 = d_kp * d_kp + d_kd * d_kd + d_lat * d_lat + d_v * d_v;
		return std::exp(-0.5 * d2);
	}

	struct AIPrediction {
		double mu = 40.0;
		double sigma = 1.0;
		double p_safe = 0.95;
		double acq = 40.0;
	};

	AIPrediction predict(const Set& s, double v, double kappa = 1.2) const {
		AIPrediction p;
		if (db_.empty()) {
			p.mu = 35.0;
			p.sigma = 1.0;
			p.p_safe = 0.95;
			p.acq = p.mu - kappa * p.sigma * 4.0;
			return p;
		}
		double w_sum = 0.0, weighted_score = 0.0, density = 0.0;
		double safe_w = 0.0, total_w = 0.0;
		for (const auto& tr : db_) {
			const double k_full = kernel(s, v, tr.set, tr.v);
			const double k_stab = stability_kernel(s, v, tr.set, tr.v);
			const double k_eff = 0.5 * k_full + 0.5 * k_stab;
			density += k_eff;
			total_w += k_stab;
			if (tr.safe) {
				safe_w += k_stab;
				if (tr.score < 400.0) {
					w_sum += k_eff;
					weighted_score += k_eff * tr.score;
				}
			}
		}
		p.p_safe = (safe_w + 0.6) / (total_w + 1.0);
		p.sigma = 1.0 / (1.0 + 0.65 * density);
		if (w_sum > 1e-4)
			p.mu = weighted_score / w_sum;
		else
			p.mu = (best_pace_ > 0 ? best_pace_ * 10.0 + 10.0 : 35.0);

		const double kp_drift = (s.kp - best_.kp) / 4.0;
		const double trust_penalty = 1.5 * kp_drift * kp_drift;
		const double risk_penalty = (1.0 - p.p_safe) * 60.0;
		p.acq = p.mu - kappa * p.sigma * 4.0 + risk_penalty + trust_penalty;
		return p;
	}

	void record(bool lost, bool pass, double score, double pace, double dist_diff, const std::string& laps,
		const std::vector<double>& lap_times, int drives, const std::string& text) {
		Cell& c = cells_[done_];
		const bool improved_all = pass && all_courses_improved(lap_times);
		const bool slower = pass && !improved_all;
		c.state = slower ? 4 : pass ? 3 : lost ? 1 : 2;
		c.score = score;
		c.pace = pace;
		c.dist_diff = dist_diff;
		c.laps = laps;
		c.lap_times = lap_times;
		since_best_ += drives;

		TrialRecord tr;
		tr.set = c.set;
		tr.v = v_;
		tr.score = score;
		tr.pace = pace;
		tr.safe = !lost && (text.find("oversteer") == std::string::npos);
		tr.pass = pass && improved_all;
		tr.state = c.state;
		tr.text = text;
		db_.push_back(tr);

		char log_buf[160];
		std::snprintf(log_buf, sizeof log_buf, "#%02zu [%.7s] kp%.1f/d%.0f: %s%s",
			db_.size(), c.role.c_str(), c.set.kp, c.set.kd / 0.0005, text.c_str(),
			slower ? " (slow)" : "");
		log_.push_back(log_buf);
		if (log_.size() > 5)
			log_.erase(log_.begin());
		done_++;
	}

	void finish_round() {
		round_++;
		const Cell* winner = nullptr;
		for (const auto& c : cells_) {
			if (c.state == 3) {
				if (!winner || c.score < winner->score)
					winner = &c;
			}
		}

		if (winner) {
			best_v_ = v_;
			best_ = winner->set;
			best_pace_ = winner->pace;
			best_laps_ = winner->laps;
			best_lap_times_ = winner->lap_times;
			best_dist_diff_ = winner->dist_diff;
			passed_at_v_ = true;
			since_best_ = 0;
			just_got_best_ = true;
			last_ = fmt("%.3f m/s [%s]: ALL %zu COURSES RENEWED! (laps: %s, dist %+.2fm)", v_, winner->role.c_str(),
				winner->lap_times.size(), winner->laps.c_str(), best_dist_diff_);
		} else {
			last_ = fmt("%.3f m/s: explored (no candidate renewed all courses)", v_);
		}

		if (passed_at_v_) {
			v_ += dv_;
			passed_at_v_ = false;
			last_ += fmt(" -> try %.2f m/s", v_);
		} else if (best_v_ > 0) {
			dv_ = std::max(0.02, dv_ * 0.7);
			v_ = std::max(min_v_, best_v_ + dv_);
			last_ += fmt(" -> aim %.2f m/s", v_);
		} else {
			v_ = min_v_;
			last_ += fmt(" -> re-tuning at %.2f m/s", v_);
		}

		plan();
	}

	void plan() {
		cells_.clear();
		done_ = 0;
		std::mt19937 rng(1337 + static_cast<unsigned>(round_) * 31);
		std::normal_distribution<double> norm(0.0, 1.0);
		std::uniform_real_distribution<double> unif(0.0, 1.0);
		std::uniform_real_distribution<double> sym(-1.0, 1.0);

		Cell c0;
		c0.set = best_;
		c0.role = "Anchor";
		AIPrediction p0 = predict(c0.set, v_);
		c0.pred_mu = p0.mu; c0.pred_sigma = p0.sigma; c0.p_safe = p0.p_safe;
		cells_.push_back(c0);

		struct Candidate {
			Set set;
			AIPrediction pred;
			std::string role;
		};
		std::vector<Candidate> exploit_pool, anti_pool, explore_pool;

		for (int i = 0; i < 60; i++) {
			Set s = best_;
			s.kp = std::clamp(best_.kp + norm(rng) * 1.2, 8.0, 32.0);
			s.kd = std::clamp(best_.kd + norm(rng) * 0.012, 0.04, 0.35);
			s.lat = std::clamp(best_.lat + norm(rng) * 0.4, 7.5, 12.8);
			s.sl = std::clamp(best_.sl + norm(rng) * 0.015, 0.01, 0.25);
			s.target_shift = std::clamp(best_.target_shift + norm(rng) * 0.03, 0.20, 0.65);
			s.target_curve_gain = std::clamp(best_.target_curve_gain + norm(rng) * 0.06, 1.15, 2.10);
			s.brake_margin = std::clamp(best_.brake_margin + norm(rng) * 0.015, 0.05, 0.28);
			s.yaw_kp = std::clamp(best_.yaw_kp + norm(rng) * 0.03, 0.10, 0.70);
			s.yaw_ki = std::clamp(best_.yaw_ki + norm(rng) * 0.12, 0.1, 3.0);
			s.pos_atten = 0.0;
			AIPrediction pr = predict(s, v_, 0.8);
			exploit_pool.push_back({s, pr, "Exploit"});
		}

		for (int i = 0; i < 45; i++) {
			Set s = best_;
			s.kp = std::clamp(best_.kp + (-2.0 + unif(rng) * 3.0), 8.0, 28.0);
			s.kd = std::clamp(best_.kd + (-0.005 + unif(rng) * 0.030), 0.05, 0.35);
			s.lat = std::clamp(best_.lat + (-1.0 + unif(rng) * 1.5), 7.5, 12.8);
			s.brake_margin = std::clamp(best_.brake_margin + unif(rng) * 0.06, 0.08, 0.28);
			s.target_shift = std::clamp(best_.target_shift + sym(rng) * 0.06, 0.25, 0.60);
			s.target_curve_gain = std::clamp(best_.target_curve_gain + sym(rng) * 0.12, 1.20, 2.00);
			s.sl = std::clamp(best_.sl + sym(rng) * 0.03, 0.02, 0.20);
			s.yaw_kp = std::clamp(best_.yaw_kp + sym(rng) * 0.06, 0.15, 0.60);
			s.yaw_ki = std::clamp(best_.yaw_ki + sym(rng) * 0.20, 0.3, 2.5);
			s.pos_atten = 0.0;
			AIPrediction pr = predict(s, v_, 1.1);
			anti_pool.push_back({s, pr, "AntiOv"});
		}

		for (int i = 0; i < 45; i++) {
			Set s = best_;
			s.kp = std::clamp(best_.kp + sym(rng) * 4.5, 8.0, 28.0);
			s.kd = std::clamp(best_.kd + sym(rng) * 0.035, 0.04, 0.32);
			s.lat = std::clamp(best_.lat + sym(rng) * 1.5, 7.5, 12.8);
			s.sl = std::clamp(best_.sl + sym(rng) * 0.05, 0.02, 0.22);
			s.target_shift = std::clamp(best_.target_shift + sym(rng) * 0.10, 0.22, 0.62);
			s.target_curve_gain = std::clamp(best_.target_curve_gain + sym(rng) * 0.20, 1.15, 2.05);
			s.brake_margin = std::clamp(best_.brake_margin + sym(rng) * 0.05, 0.06, 0.26);
			s.yaw_kp = std::clamp(best_.yaw_kp + sym(rng) * 0.10, 0.12, 0.65);
			s.yaw_ki = std::clamp(best_.yaw_ki + sym(rng) * 0.35, 0.2, 2.5);
			s.pos_atten = 0.0;
			AIPrediction pr = predict(s, v_, 1.5);
			explore_pool.push_back({s, pr, "Explore"});
		}

		auto by_acq = [](const Candidate& a, const Candidate& b) {
			return a.pred.acq < b.pred.acq;
		};
		std::sort(exploit_pool.begin(), exploit_pool.end(), by_acq);
		std::sort(anti_pool.begin(), anti_pool.end(), by_acq);
		std::sort(explore_pool.begin(), explore_pool.end(), by_acq);

		auto pick_from = [&](const std::vector<Candidate>& pool, int count) {
			int added = 0;
			for (const auto& cd : pool) {
				if (added >= count || cells_.size() >= 6)
					break;
				bool too_close = false;
				for (const auto& exist : cells_) {
					if (std::fabs(exist.set.kp - cd.set.kp) < 0.35 &&
						std::fabs(exist.set.kd - cd.set.kd) < 0.006 &&
						std::fabs(exist.set.lat - cd.set.lat) < 0.25 &&
						std::fabs(exist.set.brake_margin - cd.set.brake_margin) < 0.012 &&
						std::fabs(exist.set.target_shift - cd.set.target_shift) < 0.015) {
						too_close = true;
						break;
					}
				}
				if (too_close)
					continue;

				Cell c;
				c.set = cd.set;
				c.pred_mu = cd.pred.mu;
				c.pred_sigma = cd.pred.sigma;
				c.p_safe = cd.pred.p_safe;
				c.role = cd.role;
				cells_.push_back(c);
				added++;
			}
		};

		pick_from(exploit_pool, 2);
		pick_from(anti_pool, 2);
		pick_from(explore_pool, 1);
		if (cells_.size() < 6)
			pick_from(exploit_pool, 6 - static_cast<int>(cells_.size()));
	}

	std::vector<std::string> status(double dt, const std::string& courses) const {
		std::vector<std::string> v;
		v.push_back(fmt("AI GAIN AUTO-TUNER [A]  R#%d  (A: stop | E: endless)", round_ + 1));
		v.push_back("Surrogate: RBF Gaussian Process + Trust-Region");
		int safe_cnt = 0, over_cnt = 0;
		for (const auto& tr : db_) {
			if (tr.safe) safe_cnt++; else over_cnt++;
		}
		v.push_back(fmt("Dataset: %zu runs (%d safe, %d oversteer)", db_.size(), safe_cnt, over_cnt));
		v.push_back("courses: " + courses);
		if (best_v_ > 0)
			v.push_back(fmt("BEST: %.2fm/s (laps: %s, dist %+.2fm) kp%.1f kd%.0f lat%.1f shf%.2f",
				best_v_, best_laps_.c_str(), best_dist_diff_, best_.kp, best_.kd / dt, best_.lat, best_.target_shift));
		else
			v.push_back("BEST: searching initial frontier...");

		if (done_ < cells_.size()) {
			const Cell& cur = cells_[done_];
			v.push_back(fmt("Target #%zu [%.7s] (cell %zu/%zu) v=%.2f:", db_.size() + 1, cur.role.c_str(), done_ + 1, cells_.size(), v_));
			v.push_back(fmt("  kp%.1f kd%.0f lat%.1f brk%.2fm shf%.2f crv%.2f",
				cur.set.kp, cur.set.kd / dt, cur.set.lat, cur.set.brake_margin, cur.set.target_shift, cur.set.target_curve_gain));
			v.push_back(fmt("  AI: Score=%.1f  Uncert=%.2f  Safe=%.0f%%",
				cur.pred_mu, cur.pred_sigma, cur.p_safe * 100.0));
		}
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

	double v_ = 1.9, min_v_ = 1.9, dv_ = 0.08, best_v_ = 0, best_pace_ = 0, best_dist_diff_ = 0;
	std::string best_laps_;
	std::vector<double> best_lap_times_;
	Set best_;
	int round_ = 0;
	int since_best_ = 0;
	bool passed_at_v_ = false;
	bool just_got_best_ = false;
	size_t done_ = 0;
	std::string last_;
	std::vector<Cell> cells_;
	std::vector<TrialRecord> db_;
	std::vector<std::string> log_;
};

// [E] Endless 100-Map Reliability + AI Gain Auto-Heal Tuner
// Runs Drive 1 + Drive 4 per random map -> switches to a brand-new map every trial.
// When a map fails at current speed, generates 3 AI Gain Auto-Heal candidates around best_
// and tests them on that same map BEFORE lowering base_mps! If healed, updates best_ gains
// and keeps base_mps intact. Also recovers base_mps every 20 maps whenever below start_v_.
class EndlessTuner {
public:
	using Set = TuneSet;

	struct HealCandidate {
		Set set;
		std::string role;
	};

	struct FailMetrics {
		bool d1_failed = false;
		bool was_oversteer = false;
		double max_beta_deg = 0.0;   // Peak slip angle (deg)
		double max_lat_mm = 0.0;     // Peak lateral track deviation (mm)
		double weave_dps = 0.0;      // Yaw rate oscillation energy (deg/s)
		double fail_v = 0.0;         // Robot speed right before failure / peak error (m/s)
	};

	bool active = false;

	int target_window() const { return target_win_; }
	int recover_window() const { return recover_win_; }
	double max_fail_rate_pct() const { return max_fail_pct_; }
	double min_speed() const { return min_v_; }

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

	void start(double v, const Set& from, double max_fail_pct = 10.0, int window_size = 50, double min_speed = 0.0) {
		active = true;
		v_ = std::max(0.50, v);
		start_v_ = v_;
		min_v_ = (min_speed > 0.0) ? min_speed : std::max(1.40, start_v_ - 0.20);
		verified_v_ = 0.0;
		best_ = from;
		best_.pos_atten = 0.0;
		// Clamp initial gains to healthy non-saturated bounds before storing anchor_
		best_.kp = std::clamp(std::round(best_.kp * 1000.0) / 1000.0, 11.5, 24.0);
		best_.kd = std::clamp(best_.kd, 0.07, 0.24);   // 140 .. 480 in steer_kd
		best_.lat = std::clamp(best_.lat, 11.0, 15.0);
		best_.brake_margin = std::clamp(best_.brake_margin, 0.06, 0.24);
		anchor_ = best_;
		max_fail_pct_ = max_fail_pct > 0.0 ? max_fail_pct : 10.0;
		target_win_ = window_size > 0 ? window_size : 50;
		recover_win_ = std::max(5, target_win_ / 5);
		streak_ = 0;
		consecutive_fails_ = 0;
		total_maps_ = 0;
		total_passes_ = 0;
		total_fails_ = 0;
		total_healed_ = 0;
		speed_changed_ = false;
		just_verified_ = false;
		just_recovered_ = false;
		just_healed_ = false;
		window_.clear();
		last_.clear();
		log_.clear();
	}

	double speed() const { return v_; }
	double start_v() const { return start_v_; }
	size_t current_map_no() const { return static_cast<size_t>(total_maps_ + 1); }
	bool has_verified() const { return verified_v_ > 0.0; }
	double verified_v() const { return verified_v_; }
	const Set& best() const { return best_; }

	// Generate 5 targeted AI Gain Auto-Heal candidates using proportional failure telemetry & anti-ratchet centering
	std::vector<HealCandidate> make_heal_candidates(const FailMetrics& fm) const {
		std::mt19937 rng(9001 + static_cast<unsigned>(total_maps_) * 97 + static_cast<unsigned>(total_healed_) * 31);
		std::normal_distribution<double> norm(0.0, 1.0);
		std::uniform_real_distribution<double> sym(-1.0, 1.0);

		// 1. Anti-Ratchet Centering: pull 15% toward anchor_ and desaturate if kd/kp/lat drifted near limits
		Set base = best_;
		base.kp = std::round((0.85 * best_.kp + 0.15 * anchor_.kp) * 1000.0) / 1000.0;
		base.kd = 0.85 * best_.kd + 0.15 * anchor_.kd;
		base.lat = 0.85 * best_.lat + 0.15 * anchor_.lat;
		base.brake_margin = 0.85 * best_.brake_margin + 0.15 * anchor_.brake_margin;
		base.target_shift = 0.85 * best_.target_shift + 0.15 * anchor_.target_shift;
		base.target_curve_gain = 0.85 * best_.target_curve_gain + 0.15 * anchor_.target_curve_gain;

		// Detect if D-gain is already saturated high (> 390, i.e., kd_s > 0.195s), which causes D-kick chatter
		const bool kd_oversaturated = (best_.kd > 0.195);
		if (kd_oversaturated)
			base.kd = std::clamp(0.65 * best_.kd + 0.35 * std::min(anchor_.kd, 0.165), 0.12, 0.21);
		if (best_.kp < 12.2)
			base.kp = std::round(std::max(12.8, 0.65 * best_.kp + 0.35 * std::max(anchor_.kp, 14.2)) * 1000.0) / 1000.0;

		// 2. Normalized Failure Severity Factors (0.0 .. 1.5) from actual telemetry
		// - slip_sev: how much beta exceeded the 22 deg drift onset (e.g., 35 deg -> 0.34, 60 deg -> 1.0, 90 deg -> 1.5)
		const double slip_sev = std::clamp((fm.max_beta_deg - 22.0) / 38.0, 0.0, 1.5);
		// - under_sev: line loss with low beta (< 26 deg) or large lateral excursion -> insufficient turn-in!
		const double under_sev = fm.was_oversteer ? 0.0
			: std::clamp((26.0 - fm.max_beta_deg) / 16.0 + (fm.max_lat_mm - 75.0) / 75.0, 0.25, 1.5);
		// - osc_sev: high yaw-rate oscillation energy (weave_dps > 165 deg/s)
		const double osc_sev = std::clamp((fm.weave_dps - 165.0) / 135.0, 0.0, 1.5);
		// - speed_sev: how much faster than base_mps the robot was moving right before failing (4th drive braking entry)
		const double speed_sev = fm.d1_failed ? 0.0 : std::clamp((fm.fail_v - v_) / 0.75, 0.0, 1.5);

		// Ideal damping ratio target: steer_kd ~= 22 * steer_kp  =>  kd_s ~= 0.011 * kp
		const auto ideal_kd = [](double kp) { return std::clamp(kp * 0.0112, 0.11, 0.22); };

		std::vector<HealCandidate> cands;

		// Candidate 1 [Heal-PropPD]: Error-proportional P/D correction based on slip, understeer, and weave
		{
			Set s = base;
			if (fm.was_oversteer) {
				// Proportional kp reduction based on slip & oscillation severity
				const double dkp = -(0.35 * slip_sev + 0.45 * osc_sev);
				s.kp = std::clamp(std::round((base.kp + dkp) * 1000.0) / 1000.0, 11.5, 24.0);
				// If kd was already oversaturated, high weave is caused by D-noise kick -> lower kd!
				// Otherwise, increase kd proportionally to slip/oscillation toward healthy ceiling (<= 0.23s = 460)
				if (kd_oversaturated) {
					s.kd = std::clamp(best_.kd - (0.012 + 0.018 * osc_sev), 0.12, 0.21);
				} else {
					const double dkd = 0.007 * slip_sev + 0.010 * osc_sev;
					s.kd = std::clamp(base.kd + dkd, 0.08, 0.23);
				}
				s.yaw_kp = std::clamp(base.yaw_kp + 0.03 * slip_sev, 0.15, 0.60);
			} else {
				// Understeer / line loss: increase kp proportionally to understeer severity!
				const double dkp = 0.65 * under_sev - 0.25 * osc_sev;
				s.kp = std::clamp(std::round((base.kp + dkp) * 1000.0) / 1000.0, 12.0, 24.0);
				// Pull kd toward the ideal damping ratio for the new kp instead of blindly adding +kd
				s.kd = std::clamp(0.65 * base.kd + 0.35 * ideal_kd(s.kp) + 0.005 * osc_sev, 0.08, 0.22);
			}
			if (!fm.d1_failed) {
				s.lat = std::clamp(base.lat - (0.25 * slip_sev + 0.35 * speed_sev), 11.2, 15.0);
				s.brake_margin = std::clamp(base.brake_margin + (0.012 * slip_sev + 0.022 * speed_sev), 0.06, 0.24);
			}
			s.pos_atten = 0.0;
			cands.push_back({s, "Heal-PropPD"});
		}

		// Candidate 2 [Heal-Entry]: Speed-excess (fail_v) proportional braking & cornering grip adjustment
		{
			Set s = base;
			const double entry_sev = std::max({speed_sev, slip_sev, 0.35 * under_sev});
			// Keep kp/kd well-balanced near ideal damping ratio while fixing corner entry speed
			s.kp = std::clamp(std::round((base.kp + (fm.was_oversteer ? -0.25 * slip_sev : +0.40 * under_sev)) * 1000.0) / 1000.0, 11.8, 23.5);
			s.kd = std::clamp(0.60 * base.kd + 0.40 * ideal_kd(s.kp), 0.10, 0.22);
			if (!fm.d1_failed) {
				s.brake_margin = std::clamp(base.brake_margin + (0.015 + 0.030 * entry_sev), 0.07, 0.24);
				s.lat = std::clamp(base.lat - (0.30 + 0.55 * entry_sev), 11.0, 14.8);
				s.sl = std::clamp(base.sl - 0.015 * slip_sev, 0.03, 0.20);
			} else {
				s.kd = std::clamp(s.kd + 0.008 * (osc_sev - 0.5), 0.09, 0.22);
			}
			s.pos_atten = 0.0;
			cands.push_back({s, "Heal-Entry"});
		}

		// Candidate 3 [Heal-DeSat]: Anti-saturation / D-kick relief & Anchor restoration
		// Specifically tests lowering excessive kd and restoring brisk kp/lat response
		{
			Set s = best_;
			s.kp = std::clamp(std::round((0.55 * best_.kp + 0.45 * std::max(anchor_.kp, 14.5) + 0.35 * under_sev) * 1000.0) / 1000.0, 12.2, 23.5);
			// Explicitly reduce kd toward nominal (280..350) to eliminate high-D derivative kick!
			const double nominal_kd = std::clamp(anchor_.kd, 0.135, 0.175);
			s.kd = std::clamp(0.50 * best_.kd + 0.50 * nominal_kd - 0.008 * (kd_oversaturated ? 1.0 : 0.0), 0.10, 0.20);
			if (!fm.d1_failed) {
				s.lat = std::clamp(0.60 * best_.lat + 0.40 * anchor_.lat - 0.20 * slip_sev, 11.4, 14.8);
				s.brake_margin = std::clamp(0.60 * best_.brake_margin + 0.40 * anchor_.brake_margin + 0.015 * speed_sev, 0.07, 0.22);
			}
			s.pos_atten = 0.0;
			cands.push_back({s, "Heal-DeSat"});
		}

		// Candidate 4 [Heal-Apex]: Proportional Turn-in & In-course Apex Shift for tight S-curves / hairpins
		{
			Set s = base;
			const double turn_need = fm.was_oversteer ? -0.30 * slip_sev : +0.55 * under_sev;
			s.kp = std::clamp(std::round((base.kp + turn_need + sym(rng) * 0.35) * 1000.0) / 1000.0, 11.8, 24.0);
			s.kd = std::clamp(0.70 * base.kd + 0.30 * ideal_kd(s.kp) + sym(rng) * 0.006, 0.09, 0.22);
			if (!fm.d1_failed) {
				const double shift_dir = fm.was_oversteer ? (-0.02 * slip_sev) : (+0.035 * under_sev);
				s.target_shift = std::clamp(base.target_shift + shift_dir + sym(rng) * 0.025, 0.26, 0.56);
				s.target_curve_gain = std::clamp(base.target_curve_gain + 0.05 * (under_sev - 0.5 * slip_sev) + sym(rng) * 0.05, 1.25, 2.00);
				s.brake_margin = std::clamp(base.brake_margin + 0.015 * speed_sev, 0.06, 0.23);
			}
			s.pos_atten = 0.0;
			cands.push_back({s, "Heal-Apex"});
		}

		// Candidate 5 [Heal-Trust]: Zero-mean Gaussian exploration around anchor-regularized base
		{
			Set s = base;
			const double scale = std::clamp(0.5 + 0.35 * std::max({slip_sev, under_sev, osc_sev}), 0.5, 1.2);
			s.kp = std::clamp(std::round((base.kp + norm(rng) * (0.85 * scale)) * 1000.0) / 1000.0, 11.5, 24.0);
			s.kd = std::clamp(base.kd + norm(rng) * (0.009 * scale), 0.09, 0.23);
			if (!fm.d1_failed) {
				s.lat = std::clamp(base.lat + norm(rng) * (0.35 * scale), 11.2, 15.0);
				s.brake_margin = std::clamp(base.brake_margin + norm(rng) * (0.015 * scale), 0.06, 0.24);
				s.target_shift = std::clamp(base.target_shift + norm(rng) * 0.025, 0.26, 0.56);
				s.target_curve_gain = std::clamp(base.target_curve_gain + norm(rng) * 0.05, 1.25, 2.00);
			}
			s.pos_atten = 0.0;
			cands.push_back({s, "Heal-Trust"});
		}
		return cands;
	}

	void adopt_healed_gains(const Set& healed) {
		// Soft-blend healed gains (15% healed + 85% previous best_) to prevent a single outlier map
		// from ratcheting gains to extreme boundaries, ensuring generalized robust convergence.
		constexpr double a = 0.15;
		best_.kp = std::clamp(std::round((a * healed.kp + (1.0 - a) * best_.kp) * 1000.0) / 1000.0, 11.5, 24.0);
		best_.kd = std::clamp(a * healed.kd + (1.0 - a) * best_.kd, 0.08, 0.23);   // max 460 in steer_kd (never 600!)
		best_.lat = std::clamp(a * healed.lat + (1.0 - a) * best_.lat, 11.0, 15.0);
		best_.brake_margin = std::clamp(a * healed.brake_margin + (1.0 - a) * best_.brake_margin, 0.06, 0.24);
		best_.target_shift = std::clamp(a * healed.target_shift + (1.0 - a) * best_.target_shift, 0.24, 0.58);
		best_.target_curve_gain = std::clamp(a * healed.target_curve_gain + (1.0 - a) * best_.target_curve_gain, 1.20, 2.05);
		best_.sl = std::clamp(a * healed.sl + (1.0 - a) * best_.sl, 0.02, 0.22);
		best_.yaw_kp = std::clamp(a * healed.yaw_kp + (1.0 - a) * best_.yaw_kp, 0.15, 0.65);
		best_.yaw_ki = std::clamp(a * healed.yaw_ki + (1.0 - a) * best_.yaw_ki, 0.0, 0.20);
		best_.pos_atten = 0.0;
		total_healed_++;
		just_healed_ = true;
	}

	bool consume_state_changed(bool* out_verified = nullptr, bool* out_recovered = nullptr, bool* out_healed = nullptr) {
		if (out_verified) *out_verified = just_verified_;
		if (out_recovered) *out_recovered = just_recovered_;
		if (out_healed) *out_healed = just_healed_;
		bool res = speed_changed_ || just_healed_;
		speed_changed_ = false;
		just_verified_ = false;
		just_recovered_ = false;
		just_healed_ = false;
		return res;
	}

	int passes_in_window() const {
		int p = 0;
		for (int st : window_)
			if (st == 2) p++;
		return p;
	}

	int heals_in_window() const {
		int h = 0;
		for (int st : window_)
			if (st == 1) h++;
		return h;
	}

	int fails_in_window() const {
		int f = 0;
		for (int st : window_)
			if (st == 0) f++;
		return f;
	}

	double pass_rate_pct() const {
		if (window_.empty())
			return 0.0;
		return 100.0 * static_cast<double>(passes_in_window()) / static_cast<double>(window_.size());
	}

	double heal_rate_pct() const {
		if (window_.empty())
			return 0.0;
		return 100.0 * static_cast<double>(heals_in_window()) / static_cast<double>(window_.size());
	}

	double fail_rate_pct() const {
		if (window_.empty())
			return 0.0;
		return 100.0 * static_cast<double>(fails_in_window()) / static_cast<double>(window_.size());
	}

	double min_pass_rate_pct() const {
		return std::clamp(100.0 - max_fail_pct_, 50.0, 99.0);
	}

	int current_step_up_target() const {
		const double ref_v = std::max(start_v_, verified_v_);
		return (v_ + 1e-4 < ref_v) ? recover_window() : target_window();
	}

	void record_map(bool pass, bool was_healed, const std::string& heal_role,
		double pace, double lap4_s, double dist_diff, const std::string& detail) {
		const int win_max = target_window();
		total_maps_++;
		// 3-state window: 0 = FAIL, 1 = HEALED (neither pure pass nor fail), 2 = PURE PASS (1st try success)
		const int outcome_state = !pass ? 0 : (was_healed ? 1 : 2);
		window_.push_back(outcome_state);

		const size_t cur_win_cnt = window_.size();
		const double sr_pct = pass_rate_pct();
		const double fr_pct = fail_rate_pct();
		const double need_sr_pct = min_pass_rate_pct();
		const double ref_v = std::max(start_v_, verified_v_);
		const bool recovering = (v_ + 1e-4 < ref_v);
		const int step_need = recovering ? recover_win_ : win_max;

		if (!pass) {
			total_fails_++;
			consecutive_fails_++;
			streak_ = 0;
		} else {
			if (!was_healed) {
				total_passes_++;
				streak_++;
			}
			consecutive_fails_ = 0;
		}

		char log_buf[200];

		// 1. Check if full 50-map window (win_max) has been reached -> evaluate & ALWAYS reset window!
		if (!recovering && static_cast<int>(cur_win_cnt) >= win_max) {
			const int p_cnt = passes_in_window();
			const int h_cnt = heals_in_window();
			const int f_cnt = fails_in_window();
			window_.clear();   // Always reset 50-map window back to 0/50!
			streak_ = 0;
			consecutive_fails_ = 0;

			if (sr_pct >= need_sr_pct) {
				// Success Rate >= target (e.g. >= 80% pure Pass) -> Verified & step UP speed!
				const double old_v = v_;
				verified_v_ = std::max(verified_v_, old_v);
				v_ = old_v + dv_up();
				speed_changed_ = true;
				just_verified_ = true;
				last_ = fmt("Map #%d: %d/%d Window Done (Pass %.1f%% >= %.0f%% [%dP/%dH/%dF]) -> speed UP %.2f -> %.2f m/s!",
					total_maps_, win_max, win_max, sr_pct, need_sr_pct, p_cnt, h_cnt, f_cnt, old_v, v_);
				std::snprintf(log_buf, sizeof log_buf, "#%03d [WIN %d: P%.0f%%>=%.0f%%] %.2f->%.2fm/s",
					total_maps_, win_max, sr_pct, need_sr_pct, old_v, v_);
			} else if (fr_pct > max_fail_pct_) {
				// Fail Rate > allowed -> step DOWN speed!
				const double old_v = v_;
				if (old_v > min_v_ + 1e-4) {
					v_ = std::max(min_v_, old_v - dv_down());
					speed_changed_ = true;
					last_ = fmt("Map #%d: %d/%d Window Done (Pass %.1f%% < %.0f%%, Fail %.1f%%) -> speed DOWN %.2f -> %.2f m/s",
						total_maps_, win_max, win_max, sr_pct, need_sr_pct, fr_pct, old_v, v_);
					std::snprintf(log_buf, sizeof log_buf, "#%03d [WIN %d: F%.0f%%>%.0f%%] %.2f->%.2fm/s",
						total_maps_, win_max, fr_pct, max_fail_pct_, old_v, v_);
				} else {
					last_ = fmt("Map #%d: %d/%d Window Done at floor %.2f m/s (Pass %.1f%%) -> window reset",
						total_maps_, win_max, win_max, min_v_, sr_pct);
					std::snprintf(log_buf, sizeof log_buf, "#%03d [WIN %d RESET] %.2fm/s (P:%.0f%%)",
						total_maps_, win_max, min_v_, sr_pct);
				}
			} else {
				// Pass Rate < target due to Healed maps, but Fail Rate <= max_fail_pct_ -> keep speed & reset window!
				last_ = fmt("Map #%d: %d/%d Window Done (Pass %.1f%% < %.0f%% [%dP/%dH/%dF]) -> speed kept at %.2f m/s, window reset!",
					total_maps_, win_max, win_max, sr_pct, need_sr_pct, p_cnt, h_cnt, f_cnt, v_);
				std::snprintf(log_buf, sizeof log_buf, "#%03d [WIN %d KEEP] %.2fm/s (P:%.0f%% H:%d F:%d) Reset",
					total_maps_, win_max, v_, sr_pct, h_cnt, f_cnt);
			}
		} else if (!pass) {
			// 2. Mid-window Failure check: lower speed early only if >= 10 maps & Fail Rate > max_fail_pct_, or 2 consecutive fails
			const bool should_lower = (static_cast<int>(cur_win_cnt) >= 10 && fr_pct > max_fail_pct_) || consecutive_fails_ >= 2;
			if (should_lower) {
				const double old_v = v_;
				window_.clear();   // Reset window for the new lower speed tier!
				streak_ = 0;
				consecutive_fails_ = 0;
				if (old_v > min_v_ + 1e-4) {
					v_ = std::max(min_v_, old_v - dv_down());
					speed_changed_ = true;
					last_ = fmt("Map #%d FAIL (Pass %.0f%% | Fail %.1f%%) at %.2f m/s -> speed down to %.2f m/s (window reset)",
						total_maps_, sr_pct, fr_pct, old_v, v_);
					std::snprintf(log_buf, sizeof log_buf, "#%03d [FAIL:SLOW] %.2f->%.2fm/s (P:%.0f%% F:%.0f%%) | %s",
						total_maps_, old_v, v_, sr_pct, fr_pct, detail.c_str());
				} else {
					last_ = fmt("Map #%d FAIL at min speed floor %.2f m/s (Speed locked at floor, window reset)",
						total_maps_, min_v_);
					std::snprintf(log_buf, sizeof log_buf, "#%03d [FLOOR:%.2f] P:%.0f%% F:%.0f%% (Locked) | %s",
						total_maps_, min_v_, sr_pct, fr_pct, detail.c_str());
				}
			} else {
				last_ = fmt("Map #%d FAIL (Window %zu/%d: Pass %.0f%% | Fail %.1f%% <= %.0f%%, speed kept) at %.2f m/s",
					total_maps_, cur_win_cnt, step_need, sr_pct, fr_pct, max_fail_pct_, v_);
				std::snprintf(log_buf, sizeof log_buf, "#%03d [FAIL:KEEP %zu/%d] %.2fm/s (P:%.0f%% F:%.0f%%) | %s",
					total_maps_, cur_win_cnt, step_need, v_, sr_pct, fr_pct, detail.c_str());
			}
		} else if (recovering && !was_healed && streak_ >= step_need) {
			// 3. Fast recovery back toward start_v_ / verified_v_ after recover_win_ pure passes!
			const double old_v = v_;
			v_ = std::min(ref_v, old_v + dv_up());
			streak_ = 0;
			window_.clear();   // Reset window for the recovered speed tier!
			speed_changed_ = true;
			just_recovered_ = true;
			last_ = fmt("Map #%d: %d/%d RECOVER PASS at %.2f m/s -> speed recovered to %.2f m/s! (window reset)",
				total_maps_, step_need, step_need, old_v, v_);
			std::snprintf(log_buf, sizeof log_buf, "#%03d [RECOVER %d/%d] %.2f->%.2fm/s | 4:%.2fs",
				total_maps_, step_need, step_need, old_v, v_, lap4_s);
		} else if (was_healed) {
			last_ = fmt("Map #%d HEALED [%s] (Window %zu/%d: Pass %.0f%%, Heal %d, Fail %.0f%%) at %.2f m/s | kp=%.3f kd=%.0f lat=%.1f | 4th: %.2fs",
				total_maps_, heal_role.c_str(), cur_win_cnt, step_need, sr_pct, heals_in_window(), fr_pct, v_, best_.kp, best_.kd / 0.0005, best_.lat, lap4_s);
			std::snprintf(log_buf, sizeof log_buf, "#%03d [HEALED:%s %zu/%d] %.2fm/s (P:%.0f%% F:%.0f%%) | 4:%.2fs",
				total_maps_, heal_role.c_str(), cur_win_cnt, step_need, v_, sr_pct, fr_pct, lap4_s);
		} else {
			last_ = fmt("Map #%d PASS (Window %zu/%d: Pass %.0f%% [>=%.0f%%], Fail %.0f%%) at %.2f m/s | 4th: %.2fs (%+.2fm)",
				total_maps_, cur_win_cnt, step_need, sr_pct, need_sr_pct, fr_pct, v_, lap4_s, dist_diff);
			std::snprintf(log_buf, sizeof log_buf, "#%03d [PASS %zu/%d (P:%.0f%%)] %.2fm/s | 4:%.2fs",
				total_maps_, cur_win_cnt, step_need, sr_pct, v_, lap4_s);
		}
		log_.push_back(log_buf);
		if (log_.size() > 6)
			log_.erase(log_.begin());
	}

	std::vector<std::string> status(double dt, const std::string& cur_map_info, const std::string& heal_note = "") const {
		const int step_need = current_step_up_target();
		const double ref_v = std::max(start_v_, verified_v_);
		const bool recovering = (v_ + 1e-4 < ref_v);
		const double need_sr_pct = min_pass_rate_pct();
		std::vector<std::string> v;
		v.push_back(fmt("ENDLESS %d-MAP + AI HEAL [E]  Map #%d  (E: stop | A: 3-course)", target_win_, total_maps_ + 1));
		v.push_back("Mode: Drive 1 + Drive 4/Map + AI Gain Auto-Heal on Fail");
		v.push_back("Map:  " + cur_map_info);
		if (verified_v_ > 0.0)
			v.push_back(fmt("Base Speed: %.2f m/s  (Start: %.2f | Floor: %.2f | Verified >=%.0f%% Pass: %.2f m/s)", v_, start_v_, min_v_, need_sr_pct, verified_v_));
		else
			v.push_back(fmt("Base Speed: %.2f m/s  (Start: %.2f | Floor: %.2f | Up: >=%.0f%% Pass in %d maps)", v_, start_v_, min_v_, need_sr_pct, target_win_));
		v.push_back(fmt("Progress at %.2f m/s: %zu / %d maps (%s)",
			v_, window_.size(), step_need, recovering ? fmt("Fast-Recovery streak %d/%d", streak_, step_need).c_str() : fmt("Need >=%.0f%% pure Pass", need_sr_pct).c_str()));
		v.push_back(fmt("Window (%zu/%d): Pass %.1f%% (%dP, >=%.0f%%) | Heal %.1f%% (%dH) | Fail %.1f%% (%dF, <=%.0f%%) | Total: %dP/%dH/%dF",
			window_.size(), target_win_, pass_rate_pct(), passes_in_window(), need_sr_pct,
			heal_rate_pct(), heals_in_window(), fail_rate_pct(), fails_in_window(), max_fail_pct_,
			total_passes_, total_healed_, total_fails_));
		v.push_back(fmt("Active Gains: kp=%.3f kd=%.0f lat=%.2f brk=%.2fm shf=%.2f crv=%.2f",
			best_.kp, best_.kd / dt, best_.lat, best_.brake_margin, best_.target_shift, best_.target_curve_gain));
		if (!heal_note.empty())
			v.push_back(heal_note);
		else if (!last_.empty())
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
	double start_v_ = 2.04;
	double min_v_ = 1.84;
	double verified_v_ = 0.0;
	Set best_;
	Set anchor_;
	double max_fail_pct_ = 10.0;
	int target_win_ = 50;
	int recover_win_ = 10;
	int streak_ = 0;
	int consecutive_fails_ = 0;
	int total_maps_ = 0;
	int total_passes_ = 0;
	int total_fails_ = 0;
	int total_healed_ = 0;
	bool speed_changed_ = false;
	bool just_verified_ = false;
	bool just_recovered_ = false;
	bool just_healed_ = false;
	std::vector<int> window_;
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
		"\n    .steer_gain_p = %.3ff,"
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

namespace col {
constexpr const char* Rst  = "\033[0m";
constexpr const char* Bold = "\033[1m";
constexpr const char* Dim  = "\033[90m";
constexpr const char* Red  = "\033[91m";
constexpr const char* Grn  = "\033[92m";
constexpr const char* Yel  = "\033[93m";
constexpr const char* Blu  = "\033[94m";
constexpr const char* Mag  = "\033[95m";
constexpr const char* Cyn  = "\033[96m";
constexpr const char* Wht  = "\033[97m";
}  // namespace col

static void enable_ansi_console() {
	static bool done = false;
	if (done) return;
	done = true;
	HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hOut != INVALID_HANDLE_VALUE) {
		DWORD mode = 0;
		if (GetConsoleMode(hOut, &mode))
			SetConsoleMode(hOut, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
	}
}

ScenarioResult run_scenario(const ScenarioOptions& opt) {
	enable_ansi_console();
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
	if (robot_path.empty()) {
		if (fs::exists("robots/full.json"))
			robot_path = "robots/full.json";
		else
			robot_path = "robots/default.json";
	}
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
	if (opt.view || opt.interactive) {
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

		if (session.request_abort || g_stop_requested) {   // X or Ctrl+C: stop current drive
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
			std::printf("%s[%8.3f s]%s %sdrive %d%s %s(%s)%s %sstart%s\n",
				col::Dim, sim->t(), col::Rst,
				col::Blu, n, col::Rst,
				col::Wht, attempt_tag.c_str(), col::Rst,
				col::Dim, col::Rst);
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
		else if (cur->max_beta_deg >= 25.0 && cur->max_lat_mm <= opt.off_track_mm)
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
			const char* out_col = (cur->outcome == "finished") ? col::Grn
				: (cur->outcome == "finished (oversteer)") ? col::Yel : col::Red;
			const char* lap_col = (cur->lap_time_fw > 0) ? col::Cyn : col::Dim;
			const char* diff_col = (d_diff <= 0.0) ? col::Grn : col::Yel;
			const char* mk_col = (cur->marker_check == "OK") ? col::Grn : col::Red;
			const char* beta_col = (cur->max_beta_deg >= 90.0) ? col::Red
				: (cur->max_beta_deg >= 25.0) ? col::Yel : col::Grn;

			if (cur->outcome == "finished (oversteer)") {
				std::printf("%s[%8.3f s]%s %sdrive %d%s %s(%s)%s %s%sfinished (oversteer occurred, completed within track limits)%s, "
					"lap %s%s%.2f s%s, dist %s%.2fm%s (%s%+4.2fm%s), markers %s%s%s, max beta %s%.1f deg%s\n",
					col::Dim, sim->t(), col::Rst,
					col::Blu, n, col::Rst,
					col::Wht, attempt_tag.c_str(), col::Rst,
					col::Bold, out_col, col::Rst,
					col::Bold, lap_col, cur->lap_time_fw, col::Rst,
					col::Wht, cur->dist_m, col::Rst, diff_col, d_diff, col::Rst,
					mk_col, cur->marker_check.c_str(), col::Rst,
					beta_col, cur->max_beta_deg, col::Rst);
			} else if (cur->max_beta_deg >= 20.0) {
				const bool limit_exceeded = !cur->is_finished() || cur->max_lat_mm > opt.off_track_mm;
				std::printf("%s[%8.3f s]%s %sdrive %d%s %s(%s)%s %s%s%s%s, "
					"lap %s%s%.2f s%s, dist %s%.2fm%s (%s%+4.2fm%s), markers %s%s%s, max beta %s%.1f deg%s%s%s\n",
					col::Dim, sim->t(), col::Rst,
					col::Blu, n, col::Rst,
					col::Wht, attempt_tag.c_str(), col::Rst,
					col::Bold, out_col, cur->outcome.c_str(), col::Rst,
					col::Bold, lap_col, cur->lap_time_fw, col::Rst,
					col::Wht, cur->dist_m, col::Rst, diff_col, d_diff, col::Rst,
					mk_col, cur->marker_check.c_str(), col::Rst,
					beta_col, cur->max_beta_deg, col::Rst,
					cur->max_beta_deg >= 25.0 ? (limit_exceeded ? "\033[91m (OVERSTEER: OFF LIMIT)" : "\033[93m (DRIFT: LIMIT OK)") : "",
					col::Rst);
			} else {
				std::printf("%s[%8.3f s]%s %sdrive %d%s %s(%s)%s %s%s%s%s, "
					"lap %s%s%.2f s%s, dist %s%.2fm%s (%s%+4.2fm%s), markers %s%s%s\n",
					col::Dim, sim->t(), col::Rst,
					col::Blu, n, col::Rst,
					col::Wht, attempt_tag.c_str(), col::Rst,
					col::Bold, out_col, cur->outcome.c_str(), col::Rst,
					col::Bold, lap_col, cur->lap_time_fw, col::Rst,
					col::Wht, cur->dist_m, col::Rst, diff_col, d_diff, col::Rst,
					mk_col, cur->marker_check.c_str(), col::Rst);
			}
			std::fflush(stdout);
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
			std::printf("%s[%8.3f s]%s %sdrive %d%s %s(%s)%s %sskipped: drive 1 did not read the markers right%s\n",
				col::Dim, sim->t(), col::Rst, col::Blu, n, col::Rst, col::Wht, attempt_tag.c_str(), col::Rst, col::Yel, col::Rst);
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

	g_stop_requested = 0;
	auto prev_sig = std::signal(SIGINT, [](int) {
		g_stop_requested = 1;
	});

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
			BayesGainTuner bayes_tuner;
			EndlessTuner endless_tuner;
			const double dt = fw_steer_dt();
			bool need_next_map = false;
			if (opt.auto_tune) {
				session.auto_tune = true;
				session.tune_mode = opt.tune_mode ? opt.tune_mode : 1;
			}

			struct TuneCourse {
				std::string text, label, name;
				double length = 0;
			};
			std::vector<TuneCourse> tune_courses;
			size_t cur_course = 0, next_replace = 1;
			auto read_text = [](const std::string& path) {
				std::string t;
				if (FILE* fp = std::fopen(path.c_str(), "rb")) {
					char buf[4096];
					size_t k;
					while ((k = std::fread(buf, 1, sizeof buf, fp)) > 0)
						t.append(buf, k);
					std::fclose(fp);
				}
				return t;
			};
			auto course_entry = [](const std::string& text, const std::string& label) {
				TuneCourse c{text, label, label, 0};
				Track t;
				std::string e;
				if (t.load_text(text, label, &e)) {
					c.name = t.name();
					c.length = t.length();
				}
				return c;
			};
			auto add_course = [&](const std::string& text, const std::string& label) {
				tune_courses.push_back(course_entry(text, label));
			};
			auto course_list = [&]() {
				std::string l;
				char buf[96];
				for (size_t m = 0; m < tune_courses.size(); m++) {
					const std::string& nm = tune_courses[m].name;
					std::snprintf(buf, sizeof buf, "%sc%zu %s %.1f m", m ? "   " : "", m + 1,
						nm.rfind("random_", 0) == 0 ? "random" : nm.c_str(), tune_courses[m].length);
					l += buf;
				}
				return l;
			};
			auto cur_map_info = [&]() {
				char buf[120];
				std::snprintf(buf, sizeof buf, "%s (%.1f m)", track.name().c_str(), track.length());
				return std::string(buf);
			};
			auto params_to_tuneset = [&]() {
				TuneSet from;
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
				return from;
			};
			auto apply_tuneset = [&](double v_mps, const TuneSet& ts) {
				session.params.base_mps = static_cast<float>(v_mps);
				session.params.steer_kp = static_cast<float>(std::round(ts.kp * 1000.0) / 1000.0);
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
			};
			auto track_limit_mm = [&]() {
				static const double lim = std::getenv("LINESIM_TRACK_LIMIT_MM") ? std::atof(std::getenv("LINESIM_TRACK_LIMIT_MM")) : opt.off_track_mm;
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

			for (;;) {
				if (g_stop_requested) {
					session.auto_tune = false;
					session.tune_mode = 0;
				}
				if (session.auto_tune && session.tune_mode == 0)
					session.tune_mode = 2;

				// Clean up BayesGainTuner [A] if stopped or switched to [E]
				if (bayes_tuner.active && (!session.auto_tune || session.tune_mode != 1)) {
					bayes_tuner.active = false;
					TuneSet final_set = bayes_tuner.best();
					double final_v = bayes_tuner.has_best() ? bayes_tuner.best_v() : session.params.base_mps;
					double best_score = 0;
					std::string best_desc;
					bool has_best_cand = bayes_tuner.has_best();
					if (!has_best_cand) {
						has_best_cand = bayes_tuner.find_best_overall(&final_set, &final_v, &best_score, &best_desc);
					}
					apply_tuneset(final_v, final_set);
					if (!tune_courses.empty() && cur_course != 0 && load_course(tune_courses[0].text, tune_courses[0].label))
						cur_course = 0;
					std::string saved_path;
					save_drive_params_to_firmware_file(session.params, &saved_path);

					if (!opt.quiet) {
						std::printf("\n%s%s================================================================================%s\n", col::Bold, col::Cyn, col::Rst);
						std::printf("%s%s=== [AUTO-TUNING RESULT] BEST PARAMETERS FOUND (%zu TRIALS EVALUATED) ===%s\n", col::Bold, col::Grn, bayes_tuner.trial_count(), col::Rst);
						std::printf("%s%s================================================================================%s\n", col::Bold, col::Cyn, col::Rst);
						std::printf("  Base Speed      : %s%s%.2f m/s%s\n", col::Bold, col::Yel, session.params.base_mps, col::Rst);
						if (bayes_tuner.has_best()) {
							std::printf("  Lap Time (4th)  : %s%s%s s%s\n", col::Bold, col::Cyn, bayes_tuner.best_laps().c_str(), col::Rst);
							std::printf("  Pace / Dist Diff: %s%.3f s/10m%s (%s%+.2fm%s)\n",
								col::Grn, bayes_tuner.best_pace(), col::Rst,
								bayes_tuner.best_dist_diff() <= 0.0 ? col::Grn : col::Yel, bayes_tuner.best_dist_diff(), col::Rst);
						} else {
							std::printf("  Best Run Status : %s%s%s (score: %.1f)\n", col::Yel, best_desc.c_str(), col::Rst, best_score);
						}
						std::printf("\n  %s[Best Steering & Control Parameters]%s\n", col::Bold, col::Rst);
						std::printf("    %s.base_mps%s         = %s%.2ff%s,\n", col::Cyn, col::Rst, col::Yel, session.params.base_mps, col::Rst);
						std::printf("    %s.steer_gain_p%s     = %s%.3ff%s,\n", col::Cyn, col::Rst, col::Grn, session.params.steer_kp, col::Rst);
						std::printf("    %s.steer_gain_d%s     = %s%.1ff%s,   %s(derivative kd_s = %.4fs)%s\n", col::Cyn, col::Rst, col::Grn, session.params.steer_kd, col::Rst, col::Dim, final_set.kd, col::Rst);
						std::printf("    %s.lat_acc_max%s      = %s%.2ff%s,   %s(lateral grip limit)%s\n", col::Cyn, col::Rst, col::Mag, session.params.lat_acc_max, col::Rst, col::Dim, col::Rst);
						std::printf("    %s.brake_margin_m%s   = %s%.2ff%s,\n", col::Cyn, col::Rst, col::Yel, session.params.brake_margin_m, col::Rst);
						std::printf("    %s.target_shift_val%s = %s%.2ff%s,\n", col::Cyn, col::Rst, col::Wht, session.params.target_shift, col::Rst);
						std::printf("    %s.target_curve_gain%s= %s%.2ff%s,\n", col::Cyn, col::Rst, col::Wht, session.params.target_curve_gain, col::Rst);
						std::printf("    %s.slalom_ratio%s     = %s%.2ff%s,\n", col::Cyn, col::Rst, col::Wht, session.params.slalom_ratio, col::Rst);
						std::printf("    %s.yaw_gain_p%s       = %s%.2ff%s,\n", col::Cyn, col::Rst, col::Wht, session.params.yaw_kp, col::Rst);
						std::printf("    %s.yaw_gain_i%s       = %s%.2ff%s,\n", col::Cyn, col::Rst, col::Wht, session.params.yaw_ki, col::Rst);
						if (!saved_path.empty()) {
							std::printf("\n  %s%s[Permanently Saved to Drive File]%s\n", col::Bold, col::Grn, col::Rst);
							std::printf("    -> %s%s%s\n", col::Wht, saved_path.c_str(), col::Rst);
						}
						std::printf("%s%s================================================================================%s\n\n", col::Bold, col::Cyn, col::Rst);
						std::fflush(stdout);
					}
					char line[260];
					std::snprintf(line, sizeof line, "auto-tune [A]: best %.3f m/s, kp %.3f, kd %.0f, lat %.1f, sl %.2f, brk %.2fm, shf %.2f",
						final_v, final_set.kp, final_set.kd / dt, final_set.lat, final_set.sl, final_set.brake_margin, final_set.target_shift);
					session.history.push_back(line);
					if (!session.auto_tune) {
						session.busy = false;
						session.tune_lines.clear();
						if (!opt.view || g_stop_requested || opt.tune_trials > 0)
							break;
					}
				}

				// Clean up EndlessTuner [E] if stopped or switched to [A]
				if (endless_tuner.active && (!session.auto_tune || session.tune_mode != 2)) {
					endless_tuner.active = false;
					const TuneSet& b = endless_tuner.best();
					apply_tuneset(endless_tuner.speed(), b);
					char line[260];
					std::snprintf(line, sizeof line,
						"endless [E] stopped: base_mps %.2f m/s (verified <=%.0f%%: %.2f m/s), kp %.3f, kd %.0f, fail rate %.1f%%",
						endless_tuner.speed(), endless_tuner.max_fail_rate_pct(),
						endless_tuner.has_verified() ? endless_tuner.verified_v() : endless_tuner.speed(),
						b.kp, b.kd / dt, endless_tuner.fail_rate_pct());
					std::string saved_path;
					if (save_drive_params_to_firmware_file(session.params, &saved_path)) {
						session.message = "endless [E] ended: speed & healed gains saved to drive.c!";
						std::printf("%s%s[AUTO-SAVED]%s Endless speed & gains saved to %s%s%s\n",
							col::Bold, col::Grn, col::Rst, col::Cyn, saved_path.c_str(), col::Rst);
					} else {
						session.message = "endless [E] ended (drive 1-4 to try it)";
					}
					session.history.push_back(line);
					if (!session.auto_tune) {
						session.busy = false;
						session.tune_lines.clear();
						if (!opt.view || g_stop_requested)
							break;
					}
				}

				if (g_stop_requested)
					break;

				// ====================================================================
				// MODE 1 [A]: 3-Course AI Bayesian Gain Auto-Tuner (c1, c2, c3)
				// ====================================================================
				if (session.auto_tune && session.tune_mode == 1) {
					session.request_drive = 0;
					if (!bayes_tuner.active) {
						TuneSet from = params_to_tuneset();
						tune_courses.clear();
						if (std::string here = read_text(res.track_path_used); !here.empty())
							add_course(here, res.track_path_used);
						session.message = "auto-tune [A]: preparing 3 courses";
						while (static_cast<int>(tune_courses.size()) < kTuneCourses) {
							std::string label;
							const std::string text = make_map(&label);
							if (text.empty())
								break;
							add_course(text, label);
						}
						if (tune_courses.empty() || !load_course(tune_courses[0].text, tune_courses[0].label)) {
							session.auto_tune = false;
							session.tune_mode = 0;
							session.message = "auto-tune [A]: no course to run on";
							continue;
						}
						cur_course = 0;
						next_replace = 1;
						bayes_tuner.start(session.params.base_mps, from);
						session.busy = true;
						session.message.clear();
					}

					const TuneSet& ts = bayes_tuner.next();
					apply_tuneset(bayes_tuner.speed(), ts);
					session.tune_lines = bayes_tuner.status(dt, course_list());

					int trial_drives = 0;
					const size_t cand_no = bayes_tuner.current_candidate_no();
					if (opt.tune_trials > 0 && cand_no > static_cast<size_t>(opt.tune_trials)) {
						if (!opt.quiet) {
							std::printf("\n%s%s[COMPLETED]%s Reached %d trials limit! Finalizing best parameters...\n",
								col::Bold, col::Grn, col::Rst, opt.tune_trials);
							std::fflush(stdout);
						}
						session.auto_tune = false;
						session.tune_mode = 0;
						continue;
					}
					if (!opt.quiet) {
						std::printf("\n%s[%8.3f s]%s %s--- [A] Candidate #%zu%s:%s "
							"v=%s%.2f%s kp=%s%.1f%s kd=%s%.0f%s %s(%.4fs)%s lat=%s%.1f%s brk=%s%.2fm%s shf=%s%.2f%s %s---%s\n",
							col::Dim, sim->t(), col::Rst,
							col::Bold, cand_no,
							opt.tune_trials > 0 ? ("/" + std::to_string(opt.tune_trials)).c_str() : "", col::Rst,
							col::Yel, bayes_tuner.speed(), col::Rst,
							col::Cyn, ts.kp, col::Rst,
							col::Cyn, ts.kd / dt, col::Rst, col::Dim, ts.kd, col::Rst,
							col::Mag, ts.lat, col::Rst,
							col::Grn, ts.brake_margin, col::Rst,
							col::Wht, ts.target_shift, col::Rst,
							col::Bold, col::Rst);
						std::fflush(stdout);
					}
					auto drive = [&](int n) {
						trial_drives++;
						lap_tag = "#" + std::to_string(cand_no) + "-" + std::to_string(trial_drives);
						run_one(n);
						lap_tag.clear();
						DriveResult d = std::move(res.drives.back());
						res.drives.pop_back();
						return d;
					};
					auto accuracy = [&](double lat_c, double weave, double slip) { return lat_c + weave / 20.0 + slip / 2.0; };
					auto use_course = [&](size_t m) {
						if (cur_course != m && load_course(tune_courses[m].text, tune_courses[m].label))
							cur_course = m;
					};

					bool lost = false, pass = true, stopped = false;
					double score = 0, acc_sum = 0, pace_sum = 0, weave_first = 0;
					double dist_ratio_sum = 0, dist_diff_sum = 0;
					std::string laps;
					char text[240] = "";
					std::vector<double> cur_lap_times;
					for (size_t m = 0; m < tune_courses.size(); m++) {
						use_course(m);
						char tag[96];
						std::snprintf(tag, sizeof tag, "c%zu %s %.1f m", m + 1, tune_courses[m].name.c_str(), tune_courses[m].length);
						lap_key = tune_courses[m].name;
						lap_label = tag;
						const DriveResult d1 = drive(1);
						if (!session.auto_tune || session.tune_mode != 1 || g_stop_requested) {
							stopped = true;
							break;
						}
						const double weave1 = weave_dps(d1.samples), slip1 = slip_pct(d1.samples);
						const double latc1 = curve_rms_lat(d1.samples, track);
						const double clean1 = accuracy(latc1, weave1, slip1);
						if (m == 0)
							weave_first = weave1;
						if (lost_line(d1) || d1.marker_check != "OK" || oversteered(d1)) {
							const bool ov = oversteered(d1);
							lost = lost_line(d1) || ov;
							pass = false;
							score = lost ? 1000 : 500 + clean1;
							const char* reason = ov ? "oversteer"
								: (!d1.is_finished() ? d1.outcome.c_str()
								: (d1.max_lat_mm > track_limit_mm() ? "track limit" : ("markers " + d1.marker_check).c_str()));
							std::snprintf(text, sizeof text, "c%zu 1: %s", m + 1, reason);
							break;
						}
						const DriveResult d4 = drive(4);
						if (!session.auto_tune || session.tune_mode != 1 || g_stop_requested) {
							stopped = true;
							break;
						}
						if (lost_line(d4) || d4.marker_check != "OK" || d4.lap_time_fw <= 0 || oversteered(d4)) {
							const bool ov = oversteered(d4);
							lost = lost_line(d4) || ov;
							pass = false;
							score = (lost ? 300.0 : 200.0) + clean1;
							const char* reason = ov ? "oversteer"
								: (!d4.is_finished() ? d4.outcome.c_str()
								: (d4.max_lat_mm > track_limit_mm() ? "track limit" : ("markers " + d4.marker_check).c_str()));
							std::snprintf(text, sizeof text, "c%zu 1: lat %.1f | 4: %s", m + 1, latc1, reason);
							break;
						}
						const double weave4 = weave_dps(d4.samples), slip4 = slip_pct(d4.samples);
						const double stab4 = (weave4 / 50.0) + (slip4 / 20.0);
						acc_sum += stab4;
						pace_sum += 10.0 * d4.lap_time_fw / track.length();
						cur_lap_times.push_back(d4.lap_time_fw);

						const double trk_len = track.length();
						const double d_ratio = (trk_len > 0) ? (d4.dist_m / trk_len) : 1.0;
						const double d_diff = d4.dist_m - trk_len;
						dist_ratio_sum += d_ratio;
						dist_diff_sum += d_diff;

						char lap[16];
						std::snprintf(lap, sizeof lap, "%s%.2f", laps.empty() ? "" : "/", d4.lap_time_fw);
						laps += lap;
					}
					lap_key.clear();
					lap_label.clear();
					lap_tag.clear();
					if (stopped)
						continue;

					double pace = 0;
					double mean_dist_diff = 0;
					if (pass) {
						const double n = static_cast<double>(tune_courses.size());
						pace = pace_sum / n;
						mean_dist_diff = dist_diff_sum / n;
						const double mean_dist_ratio = dist_ratio_sum / n;
						static const double kDistWeight = std::getenv("LINESIM_DIST_WEIGHT")
							? std::atof(std::getenv("LINESIM_DIST_WEIGHT")) : 25.0;
						const double dist_penalty = kDistWeight * (mean_dist_ratio - 1.0);
						score = 10.0 * pace + acc_sum / n + dist_penalty;

						const bool all_renewed = (cur_lap_times.size() == tune_courses.size()) && bayes_tuner.all_courses_improved(cur_lap_times);
						if (all_renewed) {
							std::snprintf(text, sizeof text, "4: %.3f (%s) dist %+.2fm [ALL RENEWED]", pace, laps.c_str(), mean_dist_diff);
							use_course(0);
							lap_key = tune_courses[0].name + " lag";
							lap_label = "c1 " + tune_courses[0].name + ", lag stress x1.4 / x1.85";
							const double nominal = ir.delay();
							std::string why;
							ir.set_delay(nominal * kStressLag[0]);
							const DriveResult s1 = drive(1);
							if (lost_line(s1) || oversteered(s1))
								why = oversteered(s1) ? "x1.4: oversteer" : ("x1.4: " + (!s1.is_finished() ? s1.outcome : "track limit"));
							else if (weave_dps(s1.samples) > 1.5 * weave_first + 10.0)
								why = "x1.4: weave " + std::to_string(static_cast<int>(weave_dps(s1.samples))) + " dps";
							if (why.empty() && session.auto_tune && session.tune_mode == 1) {
								ir.set_delay(nominal * kStressLag[1]);
								const DriveResult s2 = drive(1);
								if (lost_line(s2) || oversteered(s2))
									why = oversteered(s2) ? "x1.85: oversteer" : ("x1.85: " + (!s2.is_finished() ? s2.outcome : "track limit"));
							}
							ir.set_delay(nominal);
							lap_key.clear();
							lap_label.clear();
							lap_tag.clear();
							if (!session.auto_tune || session.tune_mode != 1)
								continue;
							if (!why.empty()) {
								lost = true;
								pass = false;
								score = 250.0 + acc_sum / n;
								std::snprintf(text, sizeof text, "4: %.3f s/10m but lag %s", pace, why.c_str());
							}
						} else {
							std::string slower_list;
							const auto& b_laps = bayes_tuner.best_lap_times();
							for (size_t k = 0; k < cur_lap_times.size() && k < b_laps.size(); k++) {
								if (cur_lap_times[k] >= b_laps[k] - 1e-4) {
									if (!slower_list.empty()) slower_list += ",";
									slower_list += "c" + std::to_string(k + 1);
								}
							}
							std::snprintf(text, sizeof text, "4: (%s) dist %+.2fm [slower in %s]", laps.c_str(), mean_dist_diff, slower_list.c_str());
						}
					}
					bayes_tuner.record(lost, pass, score, pace, mean_dist_diff, laps, cur_lap_times, trial_drives, text);
					if (!opt.quiet) {
						const char* eval_col = pass ? (std::string(text).find("ALL RENEWED") != std::string::npos ? col::Grn : col::Cyn) : col::Red;
						std::printf("%s[%8.3f s]%s %sCandidate #%zu evaluated:%s %s%s%s %s(score=%.1f)%s\n",
							col::Dim, sim->t(), col::Rst,
							col::Wht, cand_no, col::Rst,
							eval_col, text, col::Rst,
							col::Dim, score, col::Rst);
						std::fflush(stdout);
					}
					if (bayes_tuner.round_complete()) {
						bayes_tuner.finish_round();
						if (bayes_tuner.just_found_best()) {
							apply_tuneset(bayes_tuner.best_v(), bayes_tuner.best());
							std::string path_out;
							if (save_drive_params_to_firmware_file(session.params, &path_out)) {
								char save_msg[180];
								std::snprintf(save_msg, sizeof save_msg,
									"[SAVED] BEST (all renewed: %s, dist %+.2fm) v=%.2f kp=%.1f kd=%.0f lat=%.1f brk=%.2f shf=%.2f",
									bayes_tuner.best_laps().c_str(), bayes_tuner.best_dist_diff(),
									session.params.base_mps, session.params.steer_kp, session.params.steer_kd,
									session.params.lat_acc_max, session.params.brake_margin_m, session.params.target_shift);
								session.history.push_back(save_msg);
								if (!opt.quiet) {
									std::printf("%s[%8.3f s]%s %s%s[SAVED] BEST%s (all renewed: %s%s%s, dist %s%+.2fm%s) "
										"v=%s%.2f%s kp=%s%.1f%s kd=%s%.0f%s lat=%s%.1f%s brk=%s%.2f%s shf=%s%.2f%s\n",
										col::Dim, sim->t(), col::Rst,
										col::Bold, col::Grn, col::Rst,
										col::Cyn, bayes_tuner.best_laps().c_str(), col::Rst,
										bayes_tuner.best_dist_diff() <= 0.0 ? col::Grn : col::Yel, bayes_tuner.best_dist_diff(), col::Rst,
										col::Yel, session.params.base_mps, col::Rst,
										col::Cyn, session.params.steer_kp, col::Rst,
										col::Cyn, session.params.steer_kd, col::Rst,
										col::Mag, session.params.lat_acc_max, col::Rst,
										col::Grn, session.params.brake_margin_m, col::Rst,
										col::Wht, session.params.target_shift, col::Rst);
									std::fflush(stdout);
								}
							}
						}
						if (!opt.quiet) {
							const std::vector<std::string> st = bayes_tuner.status(dt, course_list());
							std::printf("%s[%8.3f s]%s %s%s%s | %s%s%s\n",
								col::Dim, sim->t(), col::Rst,
								col::Blu, st[0].c_str(), col::Rst,
								col::Grn, st.size() > 4 ? st[4].c_str() : "", col::Rst);
							std::fflush(stdout);
						}
					}
					bool user_swap = session.request_new_map;
					if (user_swap)
						session.request_new_map = false;
					if ((bayes_tuner.stale() || user_swap) && !tune_courses.empty()) {
						const TuneSet b = bayes_tuner.best();
						const double v = bayes_tuner.has_best() ? bayes_tuner.best_v() : bayes_tuner.speed();
						std::string label;
						const std::string fresh = make_map(&label);
						char line[300];
						if (!fresh.empty()) {
							const bool keep_first = (!opt.track_path.empty() && opt.track_path != "random") && tune_courses.size() > 1;
							const size_t replace_idx = keep_first ? (1 + (next_replace % (tune_courses.size() - 1)))
							                                      : (next_replace % tune_courses.size());
							const std::string old = tune_courses[replace_idx].name;
							if (live)
								live->drop_course(old);
							tune_courses[replace_idx] = course_entry(fresh, label);
							if (cur_course == replace_idx)
								cur_course = tune_courses.size();
							if (replace_idx == 0)
								load_course(tune_courses[0].text, tune_courses[0].label);
							std::snprintf(line, sizeof line, "auto-tune [A]: %s -> new c%zu %s (%.1f m)",
								user_swap ? "user requested map" : ("stale (" + std::to_string(BayesGainTuner::stale_drives()) + " drives)").c_str(),
								replace_idx + 1, tune_courses[replace_idx].name.c_str(), tune_courses[replace_idx].length);
							if (!opt.quiet) {
								std::printf("%s[%8.3f s]%s %sauto-tune [A]:%s c%zu %s%s%s replaced by %s%s%s (%.1f m)\n",
									col::Dim, sim->t(), col::Rst,
									col::Mag, col::Rst,
									replace_idx + 1, col::Dim, old.c_str(), col::Rst,
									col::Cyn, tune_courses[replace_idx].name.c_str(), col::Rst, tune_courses[replace_idx].length);
								std::fflush(stdout);
							}
							next_replace = (replace_idx + 1) % tune_courses.size();
							session.history.push_back(line);
						}
						bayes_tuner.start(v, b);
					}
					session.tune_lines = bayes_tuner.status(dt, course_list());
					continue;
				}

				// ====================================================================
				// MODE 2 [E]: Endless 100-Map Reliability + AI Gain Auto-Heal
				// ====================================================================
				if (session.auto_tune && session.tune_mode == 2) {
					session.request_drive = 0;
					if (!endless_tuner.active) {
						TuneSet from = params_to_tuneset();
						endless_tuner.start(session.params.base_mps, from, opt.endless_max_fail_pct, opt.endless_window, opt.endless_min_speed);
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

					const size_t map_no = endless_tuner.current_map_no();
					auto extract_fail_v = [](const DriveResult& d) -> double {
						double fv = 0.0;
						for (size_t i = d.samples.size(); i > 0; i--) {
							const Sample& s = d.samples[i - 1];
							if (!s.lost && s.tgt_base > 0.1f) {
								for (size_t j = i; j > 0 && (s.t - d.samples[j - 1].t) <= 0.30; j--)
									fv = std::max(fv, static_cast<double>(d.samples[j - 1].v));
								break;
							}
						}
						return fv > 0.0 ? fv : d.max_v;
					};

					auto run_map_trial = [&](const TuneSet& cand_set, const std::string& tag_prefix,
						bool* out_pass, EndlessTuner::FailMetrics* out_fm,
						double* out_pace, double* out_lap4_s, double* out_dist_diff, std::string* out_detail) -> bool {
						apply_tuneset(endless_tuner.speed(), cand_set);
						auto drive = [&](int n) {
							lap_tag = tag_prefix + "-d" + std::to_string(n);
							run_one(n);
							lap_tag.clear();
							DriveResult d = std::move(res.drives.back());
							res.drives.pop_back();
							return d;
						};

						*out_pass = true;
						if (out_fm) *out_fm = EndlessTuner::FailMetrics{};
						*out_pace = 0.0;
						*out_lap4_s = 0.0;
						*out_dist_diff = 0.0;
						char buf[200] = "";

						lap_key = track.name();
						lap_label = cur_map_info();

						const DriveResult d1 = drive(1);
						if (!session.auto_tune || session.tune_mode != 2) {
							lap_key.clear();
							lap_label.clear();
							return false;   // user interrupted
						}
						if (lost_line(d1) || d1.marker_check != "OK" || oversteered(d1)) {
							const bool ov = oversteered(d1);
							*out_pass = false;
							if (out_fm) {
								out_fm->d1_failed = true;
								out_fm->was_oversteer = ov;
								out_fm->max_beta_deg = d1.max_beta_deg;
								out_fm->max_lat_mm = d1.max_lat_mm;
								out_fm->weave_dps = weave_dps(d1.samples);
								out_fm->fail_v = extract_fail_v(d1);
							}
							const char* reason = ov ? "oversteer"
								: (!d1.is_finished() ? d1.outcome.c_str()
								: (d1.max_lat_mm > track_limit_mm() ? "track limit" : ("markers " + d1.marker_check).c_str()));
							std::snprintf(buf, sizeof buf, "1st: %s", reason);
						} else {
							const DriveResult d4 = drive(4);
							if (!session.auto_tune || session.tune_mode != 2) {
								lap_key.clear();
								lap_label.clear();
								return false;   // user interrupted
							}
							if (lost_line(d4) || d4.marker_check != "OK" || d4.lap_time_fw <= 0 || oversteered(d4)) {
								const bool ov = oversteered(d4);
								*out_pass = false;
								if (out_fm) {
									out_fm->d1_failed = false;
									out_fm->was_oversteer = ov;
									out_fm->max_beta_deg = d4.max_beta_deg;
									out_fm->max_lat_mm = d4.max_lat_mm;
									out_fm->weave_dps = weave_dps(d4.samples);
									out_fm->fail_v = extract_fail_v(d4);
								}
								const char* reason = ov ? "oversteer"
									: (!d4.is_finished() ? d4.outcome.c_str()
									: (d4.max_lat_mm > track_limit_mm() ? "track limit" : ("markers " + d4.marker_check).c_str()));
								std::snprintf(buf, sizeof buf, "1st: OK | 4th: %s", reason);
							} else {
								*out_lap4_s = d4.lap_time_fw;
								*out_pace = (track.length() > 0) ? (10.0 * d4.lap_time_fw / track.length()) : 0.0;
								*out_dist_diff = d4.dist_m - track.length();
								std::snprintf(buf, sizeof buf, "1st: OK | 4th: %.2fs (%.2fs/10m, %+.2fm)",
									*out_lap4_s, *out_pace, *out_dist_diff);
							}
						}
						lap_key.clear();
						lap_label.clear();
						*out_detail = buf;
						return true;
					};

					session.tune_lines = endless_tuner.status(dt, cur_map_info());

					bool pass = false;
					EndlessTuner::FailMetrics fm;
					double pace = 0.0, lap4_s = 0.0, dist_diff = 0.0;
					std::string detail;
					if (!run_map_trial(endless_tuner.best(), "#" + std::to_string(map_no),
						&pass, &fm, &pace, &lap4_s, &dist_diff, &detail)) {
						continue;
					}

					// If initial trial on this map failed, run AI Gain Auto-Heal (up to 5 candidates) BEFORE lowering speed!
					bool was_healed = false;
					std::string healed_role;
					if (!pass) {
						const auto heal_cands = endless_tuner.make_heal_candidates(fm);
						bool aborted = false;
						for (size_t h = 0; h < heal_cands.size(); h++) {
							char heal_msg[220];
							std::snprintf(heal_msg, sizeof heal_msg,
								"AI AUTO-HEAL (%zu/%zu) [%s]: retrying Map #%zu at %.2f m/s (kp=%.3f kd=%.0f lat=%.1f brk=%.2f)",
								h + 1, heal_cands.size(), heal_cands[h].role.c_str(), map_no, endless_tuner.speed(),
								heal_cands[h].set.kp, heal_cands[h].set.kd / dt, heal_cands[h].set.lat, heal_cands[h].set.brake_margin);
							session.tune_lines = endless_tuner.status(dt, cur_map_info(), heal_msg);
							if (!opt.quiet) {
								if (h == 0) {
									std::printf("%s[%8.3f s]%s %s[DIAGNOSIS]%s %s fail (%s): beta=%s%.1f deg%s, lat=%s%.0f mm%s, weave=%s%.0f dps%s, v_fail=%s%.2f m/s%s\n",
										col::Dim, sim->t(), col::Rst,
										col::Yel, col::Rst,
										fm.d1_failed ? "1st drive" : "4th drive",
										fm.was_oversteer ? "OVERSTEER" : "UNDERSTEER/LINE-LOSS",
										fm.max_beta_deg >= 25.0 ? col::Red : col::Grn, fm.max_beta_deg, col::Rst,
										fm.max_lat_mm > 90.0 ? col::Yel : col::Wht, fm.max_lat_mm, col::Rst,
										fm.weave_dps > 180.0 ? col::Yel : col::Wht, fm.weave_dps, col::Rst,
										col::Cyn, fm.fail_v, col::Rst);
								}
								std::printf("%s[%8.3f s]%s %s%sAI AUTO-HEAL (%zu/%zu)%s %s[%s]%s: retrying %sMap #%zu%s at %s%s%.2f m/s%s "
									"(kp=%s%.3f%s kd=%s%.0f%s lat=%s%.1f%s brk=%s%.2f%s)\n",
									col::Dim, sim->t(), col::Rst,
									col::Bold, col::Mag, h + 1, heal_cands.size(), col::Rst,
									col::Yel, heal_cands[h].role.c_str(), col::Rst,
									col::Wht, map_no, col::Rst,
									col::Bold, col::Yel, endless_tuner.speed(), col::Rst,
									col::Cyn, heal_cands[h].set.kp, col::Rst,
									col::Cyn, heal_cands[h].set.kd / dt, col::Rst,
									col::Mag, heal_cands[h].set.lat, col::Rst,
									col::Grn, heal_cands[h].set.brake_margin, col::Rst);
								std::fflush(stdout);
							}

							bool h_pass = false;
							EndlessTuner::FailMetrics h_fm;
							double h_pace = 0.0, h_lap4 = 0.0, h_dd = 0.0;
							std::string h_det;
							const std::string h_tag = "#" + std::to_string(map_no) + "h" + std::to_string(h + 1);
							if (!run_map_trial(heal_cands[h].set, h_tag, &h_pass, &h_fm, &h_pace, &h_lap4, &h_dd, &h_det)) {
								aborted = true;
								break;
							}
							if (h_pass) {
								pass = true;
								was_healed = true;
								healed_role = heal_cands[h].role;
								pace = h_pace;
								lap4_s = h_lap4;
								dist_diff = h_dd;
								detail = h_det;
								endless_tuner.adopt_healed_gains(heal_cands[h].set);
								apply_tuneset(endless_tuner.speed(), endless_tuner.best());
								break;
							}
						}
						if (aborted)
							continue;
						// Restore best_ if none of the heal candidates passed
						if (!was_healed)
							apply_tuneset(endless_tuner.speed(), endless_tuner.best());
					}

					const double prev_v = endless_tuner.speed();
					endless_tuner.record_map(pass, was_healed, healed_role, pace, lap4_s, dist_diff, detail);

					bool verified = false, recovered = false, healed = false;
					if (endless_tuner.consume_state_changed(&verified, &recovered, &healed)) {
						apply_tuneset(endless_tuner.speed(), endless_tuner.best());
						std::string path_out;
						const bool should_save_to_disk = verified || healed || (session.params.base_mps >= endless_tuner.start_v());
						if (should_save_to_disk && save_drive_params_to_firmware_file(session.params, &path_out)) {
							char save_msg[220];
							if (verified) {
								std::snprintf(save_msg, sizeof save_msg,
									"[SAVED >=%.0f%% PASS VERIFIED] %d maps evaluated at %.2f m/s -> base_mps up to %.2f m/s",
									endless_tuner.min_pass_rate_pct(), endless_tuner.target_window(), prev_v, session.params.base_mps);
							} else if (recovered) {
								std::snprintf(save_msg, sizeof save_msg,
									"[SAVED RECOVERED] %d maps passed at %.2f m/s -> base_mps recovered to %.2f m/s",
									endless_tuner.recover_window(), prev_v, session.params.base_mps);
							} else if (healed) {
								std::snprintf(save_msg, sizeof save_msg,
									"[SAVED AI-HEALED] Map #%zu healed [%s] at %.2f m/s! New gains: kp=%.3f kd=%.0f lat=%.2f brk=%.2f",
									map_no, healed_role.c_str(), session.params.base_mps,
									session.params.steer_kp, session.params.steer_kd, session.params.lat_acc_max, session.params.brake_margin_m);
							} else {
								std::snprintf(save_msg, sizeof save_msg,
									"[SAVED] Map #%zu at %.2f m/s (>= start_v) -> base_mps adjusted to %.2f m/s",
									map_no, prev_v, session.params.base_mps);
							}
							session.history.push_back(save_msg);
							if (!opt.quiet) {
								if (healed) {
									std::printf("%s[%8.3f s]%s %s%s[SAVED AI-HEALED]%s Map #%zu healed %s[%s]%s at %s%s%.2f m/s%s! "
										"New gains: kp=%s%.3f%s kd=%s%.0f%s lat=%s%.2f%s brk=%s%.2f%s\n",
										col::Dim, sim->t(), col::Rst,
										col::Bold, col::Mag, col::Rst, map_no,
										col::Yel, healed_role.c_str(), col::Rst,
										col::Bold, col::Yel, session.params.base_mps, col::Rst,
										col::Cyn, session.params.steer_kp, col::Rst,
										col::Cyn, session.params.steer_kd, col::Rst,
										col::Mag, session.params.lat_acc_max, col::Rst,
										col::Grn, session.params.brake_margin_m, col::Rst);
								} else {
									std::printf("%s[%8.3f s]%s %s%s%s\n",
										col::Dim, sim->t(), col::Rst,
										col::Bold, verified ? col::Grn : col::Cyn, save_msg);
									std::printf("%s", col::Rst);
								}
								std::fflush(stdout);
							}
						} else if (!should_save_to_disk) {
							char fail_msg[220];
							std::snprintf(fail_msg, sizeof fail_msg,
								"[ADJUST] Map #%zu failed (after 5 AI heal tries) at %.2f m/s -> base_mps down to %.2f m/s (drive.c preserved)",
								map_no, prev_v, session.params.base_mps);
							session.history.push_back(fail_msg);
							if (!opt.quiet) {
								std::printf("%s[%8.3f s]%s %s%s[ADJUST]%s Map #%zu %sfailed%s (after 5 AI heal tries) at %s%.2f m/s%s -> "
									"base_mps down to %s%s%.2f m/s%s %s(drive.c preserved)%s\n",
									col::Dim, sim->t(), col::Rst,
									col::Bold, col::Yel, col::Rst, map_no,
									col::Red, col::Rst,
									col::Yel, prev_v, col::Rst,
									col::Bold, col::Yel, session.params.base_mps, col::Rst,
									col::Dim, col::Rst);
								std::fflush(stdout);
							}
						}
					} else if (!opt.quiet) {
						const std::vector<std::string> st = endless_tuner.status(dt, cur_map_info());
						const char* res_col = was_healed ? col::Mag : (pass ? col::Grn : col::Red);
						std::printf("%s[%8.3f s]%s %s%s%s | %s%s%s\n",
							col::Dim, sim->t(), col::Rst,
							col::Blu, st[0].c_str(), col::Rst,
							res_col, st.size() > 7 ? st[7].c_str() : "", col::Rst);
						std::fflush(stdout);
					}

					// Always switch to a new random map for the next trial
					need_next_map = true;
					session.tune_lines = endless_tuner.status(dt, cur_map_info());
					continue;
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
	Simulation::Stop st = sim->run(t_limit, [&]() { return (live && live->stop_requested()) || g_stop_requested; });
	std::signal(SIGINT, prev_sig);
	if (live && st != Simulation::Stop::Predicate && !g_stop_requested)
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
