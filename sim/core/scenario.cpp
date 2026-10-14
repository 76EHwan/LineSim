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
		double kp = 0, kd = 0;     // steering PD, kd in seconds
		double lat = 0, sl = 0;    // lat_acc_max [m/s2], slalom_ratio
	};
	bool active = false;

	void start(double v, const Set& from) {
		active = true;
		v_ = std::max(0.3, v);
		dv_ = 0.1;
		best_v_ = best_pace_ = 0;
		best_laps_.clear();
		best_ = from;
		if (best_.lat <= 0)
			best_.lat = 14.0;
		gains_ = {best_.kp, best_.kd, std::max(0.5, best_.kp * 0.1), std::max(0.01, best_.kd * 0.15)};
		curve_ = {best_.lat, best_.sl, 1.0, 0.03};
		kind_ = 0;
		round_ = 0;
		since_best_ = 0;
		passed_at_v_ = false;
		last_.clear();
		log_.clear();
		plan();
	}
	double speed() const { return v_; }
	bool has_best() const { return best_v_ > 0; }
	double best_v() const { return best_v_; }
	double best_pace() const { return best_pace_; }   // drive 4 pace of BEST [s / 10 m], 0 = none yet
	const std::string& best_laps() const { return best_laps_; }
	const Set& best() const { return best_; }          // the starting values until something passes
	const Set& next() const { return cells_[done_].set; }
	bool round_complete() const { return done_ == cells_.size(); }
	// drives without a new BEST before a course is replaced, counted like the lap list's #numbers
	// (every drive 1, drive 4 and stress drive; LINESIM_TUNE_STALE=n overrides, for tests)
	static int stale_drives() {
		static const int n = std::getenv("LINESIM_TUNE_STALE") ? std::max(1, std::atoi(std::getenv("LINESIM_TUNE_STALE"))) : 100;
		return n;
	}
	// Time for another course: no new BEST for stale_drives() drives, and no pass in this round that
	// would still become one when the round ends.
	bool stale() const {
		if (since_best_ < stale_drives())
			return false;
		for (size_t k = 0; k < done_; k++)
			if (cells_[k].state == 3)
				return false;
		return true;
	}

	// lost: lost the line or unstable with more lag. pass: every drive right and stable. A pass with
	// a pace no better than BEST is dropped (state 4). drives: how many the trial took.
	void record(bool lost, bool pass, double score, double pace, const std::string& laps, int drives,
		const std::string& text) {
		Cell& c = cells_[done_];
		const bool slower = pass && best_pace_ > 0 && pace >= best_pace_;
		c.state = slower ? 4 : pass ? 3 : lost ? 1 : 2;
		c.score = score;
		c.pace = pace;
		c.laps = laps;
		since_best_ += drives;
		log_.push_back(fmt("%.2f %s%s  %s%s", v_, c.extra ? "fit " : "", label(c.set).c_str(), text.c_str(),
			slower ? fmt(" >= %.3f: dropped", best_pace_).c_str() : ""));
		if (log_.size() > 4)
			log_.erase(log_.begin());
		done_++;
		if (done_ == 9)
			add_fitted_point();
	}

	void finish_round() {
		round_++;
		auto cell = [&](int i, int j) -> const Cell& { return cells_[i * 3 + j]; };
		auto stable = [&](int i, int j) {   // no block neighbour lost the line / was unstable
			if (cell(i, j).state == 1)
				return false;
			const int d[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
			for (const auto& o : d) {
				int a = i + o[0], b = j + o[1];
				if (a >= 0 && a < 3 && b >= 0 && b < 3 && cell(a, b).state == 1)
					return false;
			}
			return true;
		};
		// candidates: grid passes with stable neighbours, and the fitted point when its nearest grid
		// point is stable
		std::vector<size_t> cand;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				if (cell(i, j).state == 3 && stable(i, j))
					cand.push_back(static_cast<size_t>(i * 3 + j));
		if (cells_.size() > 9 && cells_[9].state == 3 &&
			stable(static_cast<int>(std::lround(fit_u_)) + 1, static_cast<int>(std::lround(fit_v_)) + 1))
			cand.push_back(9);
		double best = 1e9;
		for (size_t k : cand)
			best = std::min(best, cells_[k].score);
		auto a_of = [&](const Set& s) { return kind_ == 0 ? s.kp : s.lat; };
		auto b_of = [&](const Set& s) { return kind_ == 0 ? s.kd : s.sl; };
		const Cell* p = nullptr;   // gentlest within 1 %: lower kp / lat, then lower kd / slalom
		size_t pk = 0;
		for (size_t k : cand)
			if (cells_[k].score <= best * 1.01 &&
				(!p || a_of(cells_[k].set) < a_of(p->set) - 1e-12 ||
					(std::fabs(a_of(cells_[k].set) - a_of(p->set)) < 1e-12 && b_of(cells_[k].set) < b_of(p->set)))) {
				p = &cells_[k];
				pk = k;
			}
		Axes& ax = kind_ == 0 ? gains_ : curve_;
		const char* what = kind_ == 0 ? "gains" : "curve entry";
		if (p) {   // every pass beat BEST's pace (slower ones were dropped)
			best_v_ = v_;
			best_ = p->set;
			best_pace_ = p->pace;
			best_laps_ = p->laps;
			passed_at_v_ = true;
			since_best_ = 0;
			if (pk == 4 || pk == 9) {   // the middle or the fitted point: look closer
				ax.da *= 0.5;
				ax.db *= 0.5;
			}
			ax.a = a_of(best_);
			ax.b = b_of(best_);
			last_ = fmt("%.3f m/s %s: new BEST %.3f s/10m (%s%s)", v_, what, p->pace, pk == 9 ? "fit " : "",
				label(p->set).c_str());
		} else {
			ax.da *= 0.5;
			ax.db *= 0.5;
			last_ = fmt("%.3f m/s %s: no faster stable pass, finer steps", v_, what);
		}
		// smallest steps: kp 0.25, kd 0.005 s, lat 0.25 m/s2, slalom 0.01
		ax.da = std::max(ax.da, 0.25);
		ax.db = std::max(ax.db, kind_ == 0 ? 0.005 : 0.01);
		if (kind_ == 0) {   // curve entry next, same speed
			kind_ = 1;
		} else {            // both pairs done at this speed
			kind_ = 0;
			if (passed_at_v_) {
				last_ += fmt(" -> try %.3f", v_ + dv_);
				v_ += dv_;
			} else if (best_v_ > 0) {
				if (dv_ > 0.01 + 1e-9)
					dv_ = std::max(0.01, dv_ * 0.5);
				last_ += fmt(" -> %.3f", best_v_ + dv_);
				v_ = best_v_ + dv_;
			} else {
				last_ += " -> slower";
				v_ = std::max(0.3, v_ - dv_);
			}
			passed_at_v_ = false;
		}
		plan();
	}

	std::vector<std::string> status(double dt, const std::string& courses) const {
		std::vector<std::string> v;
		v.push_back(fmt("AUTO-TUNE  round %d: %s  (A: stop, keep BEST)", round_ + 1,
			kind_ == 0 ? "gains kp x kd" : "curve entry lat acc max x slalom"));
		v.push_back(fmt("testing  %.3f m/s   pair %zu / %zu   no new BEST for %d / %d drives", v_,
			std::min(done_ + 1, cells_.size()), cells_.size(), since_best_, stale_drives()));
		v.push_back("courses  " + courses);
		v.push_back(fmt("lag stress x%.2g / x%.3g, speed step %.3f", kStressLag[0], kStressLag[1], dv_));
		v.push_back("lat = curve rms [mm], pace = drive 4 s / 10 m");
		if (best_v_ > 0)
			v.push_back(fmt("BEST  %.3f m/s  kp %.2f  kd %.4f s (fw %.0f)  lat max %.2f  slalom %.2f  4th %.3f s/10m (%s)",
				best_v_, best_.kp, best_.kd, best_.kd / dt, best_.lat, best_.sl, best_pace_, best_laps_.c_str()));
		else
			v.push_back("BEST  none yet");
		v.push_back(last_);
		// this round's block: . pending, x lost / unstable, m markers, s passed but slower than BEST,
		// number = score
		const Axes& ax = kind_ == 0 ? gains_ : curve_;
		std::string h = kind_ == 0 ? "kp \\ kd  " : "lat \\ sl  ";
		for (int j = 0; j < 3; j++)
			h += kind_ == 0 ? fmt("%8.4f", cells_[j].set.kd) : fmt("%8.2f", cells_[j].set.sl);
		h += kind_ == 0 ? fmt("   (steps %.2f / %.4f)", ax.da, ax.db) : fmt("   (steps %.2f / %.2f)", ax.da, ax.db);
		v.push_back(h);
		auto mark = [&](const Cell& c, int w) {
			return c.state == 0 ? fmt("%*s", w, ".") : c.state == 1 ? fmt("%*s", w, "x") : c.state == 2 ? fmt("%*s", w, "m")
				: c.state == 4 ? fmt("%*s", w, "s") : fmt("%*.1f", w, c.score);
		};
		for (int i = 0; i < 3; i++) {
			const Set& rs = cells_[i * 3].set;
			std::string r = fmt("%7.2f  ", kind_ == 0 ? rs.kp : rs.lat);
			for (int j = 0; j < 3; j++)
				r += mark(cells_[i * 3 + j], 8);
			v.push_back(r);
		}
		if (cells_.size() > 9)
			v.push_back("fit  " + label(cells_[9].set) + "  " + mark(cells_[9], 1));
		for (const std::string& l : log_)
			v.push_back(l);
		return v;
	}

private:
	struct Axes {
		double a = 0, b = 0;     // centre: kp, kd [s]  or  lat_acc_max, slalom_ratio
		double da = 0, db = 0;   // steps
	};
	struct Cell {
		Set set;
		bool extra = false;   // the point from the quadratic fit
		int state = 0;        // 0 pending, 1 lost / unstable, 2 markers wrong, 3 pass, 4 pass but slower than BEST
		double score = 0, pace = 0;
		std::string laps;
	};
	static std::string fmt(const char* f, ...) {
		char buf[240];
		va_list ap;
		va_start(ap, f);
		std::vsnprintf(buf, sizeof buf, f, ap);
		va_end(ap);
		return buf;
	}
	std::string label(const Set& s) const {
		return kind_ == 0 ? fmt("kp%.2f kd%.4f", s.kp, s.kd) : fmt("lat%.2f sl%.2f", s.lat, s.sl);
	}
	void plan() {   // the 3 x 3 block around the centre, row by row, gentle (low kp / lat) first
		cells_.clear();
		const Axes& ax = kind_ == 0 ? gains_ : curve_;
		// keep the block inside the valid range: kp >= 1, kd >= 0, lat >= 2, 0 <= slalom <= 1
		const double lo_a = kind_ == 0 ? 1.0 : 2.0, lo_b = 0.0, hi_b = kind_ == 0 ? 1e9 : 1.0;
		ca_ = std::max(ax.a, lo_a + ax.da);
		cb_ = std::clamp(ax.b, lo_b + ax.db, hi_b - ax.db);
		for (int i = -1; i <= 1; i++)
			for (int j = -1; j <= 1; j++)
				cells_.push_back(make_cell(i, j));
		done_ = 0;
	}
	Cell make_cell(double u, double v) const {
		const Axes& ax = kind_ == 0 ? gains_ : curve_;
		Cell c;
		c.set = best_;
		if (kind_ == 0) {
			c.set.kp = ca_ + u * ax.da;
			c.set.kd = cb_ + v * ax.db;
		} else {
			c.set.lat = ca_ + u * ax.da;
			c.set.sl = cb_ + v * ax.db;
		}
		return c;
	}
	// Least-squares quadratic z = c0 + c1 u + c2 v + c3 u^2 + c4 v^2 + c5 u v over the scored grid
	// points (u, v in -1, 0, 1). A minimum inside the block, away from the tried points, becomes a
	// 10th trial.
	void add_fitted_point() {
		double A[6][7] = {};
		int n = 0;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) {
				const Cell& c = cells_[i * 3 + j];
				if (c.state != 3 && c.state != 4)
					continue;
				const double u = i - 1, v = j - 1, f[6] = {1, u, v, u * u, v * v, u * v};
				for (int r = 0; r < 6; r++) {
					for (int k = 0; k < 6; k++)
						A[r][k] += f[r] * f[k];
					A[r][6] += f[r] * c.score;
				}
				n++;
			}
		if (n < 6)
			return;
		for (int col = 0; col < 6; col++) {   // Gauss-Jordan with partial pivoting
			int piv = col;
			for (int r = col + 1; r < 6; r++)
				if (std::fabs(A[r][col]) > std::fabs(A[piv][col]))
					piv = r;
			if (std::fabs(A[piv][col]) < 1e-9)
				return;   // the scored points do not pin down a quadratic
			for (int k = 0; k < 7; k++)
				std::swap(A[col][k], A[piv][k]);
			for (int r = 0; r < 6; r++)
				if (r != col) {
					const double m = A[r][col] / A[col][col];
					for (int k = col; k < 7; k++)
						A[r][k] -= m * A[col][k];
				}
		}
		double cf[6];
		for (int r = 0; r < 6; r++)
			cf[r] = A[r][6] / A[r][r];
		const double h11 = 2 * cf[3], h22 = 2 * cf[4], h12 = cf[5], det = h11 * h22 - h12 * h12;
		if (h11 <= 0 || det <= 0)
			return;   // not a bowl: no minimum to aim at
		double u = (-cf[1] * h22 + cf[2] * h12) / det, v = (-cf[2] * h11 + cf[1] * h12) / det;
		u = std::clamp(u, -1.0, 1.0);
		v = std::clamp(v, -1.0, 1.0);
		if (std::fabs(u - std::round(u)) < 0.2 && std::fabs(v - std::round(v)) < 0.2)
			return;   // (about) a point already tried
		fit_u_ = u;
		fit_v_ = v;
		Cell c = make_cell(u, v);
		c.extra = true;
		cells_.push_back(c);
	}

	double v_ = 1.5, dv_ = 0.1, best_v_ = 0, best_pace_ = 0;
	std::string best_laps_;
	Set best_;
	Axes gains_, curve_;
	double ca_ = 0, cb_ = 0;    // this block's centre (after keeping it in range)
	double fit_u_ = 0, fit_v_ = 0;
	int kind_ = 0;              // 0 gains round, 1 curve entry round
	int round_ = 0;
	int since_best_ = 0;        // drives since the last new BEST (or the start)
	bool passed_at_v_ = false;  // something passed at this speed (gains or curve round)
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
	else if (key == "yaw_kp") p->yaw_kp = (float) v;              // cascade steering, inner gyro loop
	else if (key == "yaw_ki") p->yaw_ki = (float) v;
	else if (key == "lat_acc_max") p->lat_acc_max = (float) v;
	else if (key == "target_curve_gain") p->target_curve_gain = (float) v;   // drive 3/4: target_pos x gain in curves
	else if (key == "slalom_ratio") p->slalom_ratio = (float) v;             // drive 4: 45 deg speed ratio
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
		if (n == 1)
			drive1_ok = cur->outcome == "finished" && cur->marker_check == "OK";
		if (live) {   // lap list on the left: the firmware's lap time, else the time driven until it stopped
			const bool ok = cur->outcome == "finished" && cur->marker_check == "OK" && cur->lap_time_fw > 0;
			char what[160];
			if (cur->lap_time_fw > 0) {
				std::snprintf(what, sizeof what, "%6.2f s %s", cur->lap_time_fw,
					cur->marker_check == "OK" ? "OK" : cur->marker_check.c_str());
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
				++lap_no, ok, cur->lap_time_fw, what);
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
		if (!opt.quiet)
			std::printf("[%8.3f s] drive %d skipped: drive 1 did not read the markers right\n", sim->t(), n);
		if (live)
			live->record_lap(track.name(), track.name(), n, ++lap_no, false, 0.0, "skipped (drive 1 markers wrong)");
	};
	auto run_or_skip = [&](int n) {
		if (n >= 2 && !drive1_ok)
			skip_drive(n);
		else
			run_one(n);
	};

	// Session: replace the course by a new random one (firmware idle, between events).
	// Session: a new random course (saved into the output folder), "" on failure.
	auto make_map = [&](std::string* label) {
		TrackGenOptions g;
		g.seed = ++map_seed;
		std::string e;
		std::string text = generate_track(g, &e);
		if (text.empty()) {
			session.message = "map generator: " + e;
			return text;
		}
		*label = save_generated(text, g.seed);   // load_course() makes it the course on the floor
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
			// auto-tune courses: the one on the floor when A was pressed, then random ones
			struct TuneCourse {
				std::string text, label, name;
				double length = 0;
			};
			std::vector<TuneCourse> tune_courses;
			size_t cur_course = 0, next_replace = 1;   // course on the floor; random course swapped next
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
			for (;;) {
				if (session.auto_tune) {   // endless: score speed and gains on several courses, adjust
					session.request_drive = 0;
					session.request_new_map = false;
					if (!tuner.active) {
						GainTuner::Set from;
						from.kp = session.params.steer_kp;
						from.kd = session.params.steer_kd * dt;
						from.lat = session.params.lat_acc_max;
						from.sl = session.params.slalom_ratio;
						// the courses: the one on the floor and kTuneCourses - 1 random ones
						tune_courses.clear();
						if (std::string here = read_text(res.track_path_used); !here.empty())
							add_course(here, res.track_path_used);
						session.message = "auto-tune: making the other courses";
						while (static_cast<int>(tune_courses.size()) < kTuneCourses) {
							std::string label;
							const std::string text = make_map(&label);
							if (text.empty())
								break;
							add_course(text, label);
						}
						if (tune_courses.empty() || !load_course(tune_courses[0].text, tune_courses[0].label)) {
							session.auto_tune = false;
							session.message = "auto-tune: no course to run on";
							continue;
						}
						cur_course = 0;
						next_replace = 1;
						tuner.start(session.params.base_mps, from);
						session.busy = true;
						session.message.clear();
					}
					const GainTuner::Set& ts = tuner.next();
					session.params.base_mps = static_cast<float>(tuner.speed());
					session.params.steer_kp = static_cast<float>(ts.kp);
					session.params.steer_kd = static_cast<float>(ts.kd / dt);
					session.params.lat_acc_max = static_cast<float>(ts.lat);
					session.params.slalom_ratio = static_cast<float>(ts.sl);
					fwb_set_drive_params(&session.params);
					res.params = session.params;
					session.tune_lines = tuner.status(dt, course_list());
					int trial_drives = 0;
					auto drive = [&](int n) {
						trial_drives++;
						run_one(n);
						DriveResult d = std::move(res.drives.back());
						res.drives.pop_back();   // endless: keep the scores, not every drive's samples
						return d;
					};
					auto lost_line = [](const DriveResult& d) { return d.outcome != "finished" || d.max_lat_mm > 70.0; };
					// accuracy (lower = better): line error in the curves, steering weave, wheel slip (odometry error)
					auto accuracy = [&](double lat_c, double weave, double slip) { return lat_c + weave / 20.0 + slip / 2.0; };
					auto use_course = [&](size_t m) {
						if (cur_course != m && load_course(tune_courses[m].text, tune_courses[m].label))
							cur_course = m;
					};
					// on every course: drive 1 learns it, drive 4 (the race run) drives it fast from that data
					bool lost = false, pass = true, stopped = false;
					double score = 0, acc_sum = 0, pace_sum = 0, weave_first = 0;
					double lat1_sum = 0, lat4_sum = 0;
					std::string laps;
					char text[240] = "";
					for (size_t m = 0; m < tune_courses.size(); m++) {
						use_course(m);
						char tag[96];
						std::snprintf(tag, sizeof tag, "c%zu %s %.1f m", m + 1, tune_courses[m].name.c_str(), tune_courses[m].length);
						lap_key = tune_courses[m].name;
						lap_label = tag;
						const DriveResult d1 = drive(1);
						if (!session.auto_tune) {   // stopped (A or X) during this drive: not a fair trial
							stopped = true;
							break;
						}
						const double weave1 = weave_dps(d1.samples), slip1 = slip_pct(d1.samples);
						const double latc1 = curve_rms_lat(d1.samples, track);
						const double clean1 = accuracy(latc1, weave1, slip1);   // drive 1: accuracy only
						if (m == 0)
							weave_first = weave1;
						if (lost_line(d1) || d1.marker_check != "OK") {
							lost = lost_line(d1);
							pass = false;
							score = lost ? 1000 : 500 + clean1;
							std::snprintf(text, sizeof text, "c%zu 1: %s", m + 1,
								lost ? d1.outcome.c_str() : ("markers " + d1.marker_check).c_str());
							break;
						}
						const DriveResult d4 = drive(4);
						if (!session.auto_tune) {
							stopped = true;
							break;
						}
						if (lost_line(d4) || d4.marker_check != "OK" || d4.lap_time_fw <= 0) {
							lost = lost_line(d4);
							pass = false;
							score = (lost ? 300.0 : 200.0) + clean1;
							std::snprintf(text, sizeof text, "c%zu 1: lat %.1f | 4: %s", m + 1, latc1,
								lost ? d4.outcome.c_str() : ("markers " + d4.marker_check).c_str());
							break;
						}
						const double weave4 = weave_dps(d4.samples), slip4 = slip_pct(d4.samples);
						const double latc4 = curve_rms_lat(d4.samples, track);
						acc_sum += clean1 + accuracy(latc4, weave4, slip4);   // drive 4: accuracy + time (below)
						pace_sum += 10.0 * d4.lap_time_fw / track.length();
						lat1_sum += latc1;
						lat4_sum += latc4;
						char lap[16];
						std::snprintf(lap, sizeof lap, "%s%.2f", laps.empty() ? "" : "/", d4.lap_time_fw);
						laps += lap;
					}
					lap_key.clear();
					lap_label.clear();
					if (stopped)
						continue;
					double pace = 0;
					if (pass) {
						const double n = static_cast<double>(tune_courses.size());
						pace = pace_sum / n;
						score = acc_sum / n + 10.0 * pace;
						std::snprintf(text, sizeof text, "4: %.3f (%s) lat %.1f/%.1f", pace, laps.c_str(), lat1_sum / n,
							lat4_sum / n);
						// slower than BEST: record() drops it, so the stress drives would be wasted
						if (tuner.best_pace() <= 0 || pace < tuner.best_pace()) {
							// Model-error stress on the first course: the steering lag (sensor_delay, fitted to one
							// gyro log) may be larger on the real robot. Drive 1 again with it x1.4 (must keep the
							// line, weave at most 1.5 x nominal + 10 dps) and x1.85 (must keep the line). Failing
							// counts as unstable.
							use_course(0);
							lap_key = tune_courses[0].name + " lag";   // lap times with more lag: a list of their own
							lap_label = "c1 " + tune_courses[0].name + ", lag stress x1.4 / x1.85";
							const double nominal = ir.delay();
							std::string why;
							ir.set_delay(nominal * kStressLag[0]);
							const DriveResult s1 = drive(1);
							if (lost_line(s1))
								why = "x1.4: " + s1.outcome;
							else if (weave_dps(s1.samples) > 1.5 * weave_first + 10.0)
								why = "x1.4: weave " + std::to_string(static_cast<int>(weave_dps(s1.samples))) + " dps";
							if (why.empty() && session.auto_tune) {
								ir.set_delay(nominal * kStressLag[1]);
								const DriveResult s2 = drive(1);
								if (lost_line(s2))
									why = "x1.85: " + s2.outcome;
							}
							ir.set_delay(nominal);
							lap_key.clear();
							lap_label.clear();
							if (!session.auto_tune)
								continue;
							if (!why.empty()) {
								lost = true;   // unstable with a bit more lag: the search backs away from it
								pass = false;
								score = 250.0 + acc_sum / n;
								std::snprintf(text, sizeof text, "4: %.3f s/10m but lag %s", pace, why.c_str());
							}
						}
					}
					tuner.record(lost, pass, score, pace, laps, trial_drives, text);
					if (tuner.round_complete()) {
						tuner.finish_round();
						if (!opt.quiet) {
							const std::vector<std::string> st = tuner.status(dt, course_list());
							std::printf("[%8.3f s] auto-tune %s | %s | %s\n", sim->t(), st[0].c_str() + 11, st[5].c_str(),
								st.size() > 6 ? st[6].c_str() : "");
							std::fflush(stdout);
						}
					}
					if (tuner.stale() && tune_courses.size() > 1) {   // no new BEST for a while: swap a random course
						const GainTuner::Set b = tuner.best();
						const double v = tuner.has_best() ? tuner.best_v() : tuner.speed();
						std::string label;
						const std::string fresh = make_map(&label);
						char line[300];
						if (!fresh.empty()) {
							const std::string old = tune_courses[next_replace].name;
							if (live)
								live->drop_course(old);   // its laps do not compare with anything any more
							tune_courses[next_replace] = course_entry(fresh, label);
							if (cur_course == next_replace)
								cur_course = tune_courses.size();   // reload it before the next drive
							std::snprintf(line, sizeof line, "auto-tune: no new BEST in %d drives -> new c%zu (%.1f m)",
								GainTuner::stale_drives(), next_replace + 1, tune_courses[next_replace].length);
							if (!opt.quiet) {   // the full story on the console
								std::printf("[%8.3f s] auto-tune: no new BEST in %d drives -> c%zu %s replaced by %s (BEST %.3f m/s kp %.2f kd %.4f lat %.2f sl %.2f)\n",
									sim->t(), GainTuner::stale_drives(), next_replace + 1, old.c_str(),
									tune_courses[next_replace].name.c_str(), v, b.kp, b.kd, b.lat, b.sl);
								std::fflush(stdout);
							}
							next_replace = next_replace + 1 < tune_courses.size() ? next_replace + 1 : 1;
							session.history.push_back(line);
						}
						tuner.start(v, b);   // paces belong to the courses: start over from BEST's values
					}
					session.tune_lines = tuner.status(dt, course_list());
					continue;
				}
				if (tuner.active) {   // A pressed again: keep the best found (the starting values if nothing passed)
					tuner.active = false;
					const GainTuner::Set& b = tuner.best();
					if (tuner.has_best())
						session.params.base_mps = static_cast<float>(tuner.best_v());
					session.params.steer_kp = static_cast<float>(b.kp);
					session.params.steer_kd = static_cast<float>(b.kd / dt);
					session.params.lat_acc_max = static_cast<float>(b.lat);
					session.params.slalom_ratio = static_cast<float>(b.sl);
					fwb_set_drive_params(&session.params);
					if (!tune_courses.empty() && cur_course != 0 && load_course(tune_courses[0].text, tune_courses[0].label))
						cur_course = 0;   // back to the course the tuning started on
					session.busy = false;
					char line[240];
					if (tuner.has_best())
						std::snprintf(line, sizeof line,
							"auto-tune: best %.3f m/s, kp %.2f, kd %.4f s (fw %.0f), lat max %.2f, slalom %.2f, 4th %.3f s/10m (%s s)",
							tuner.best_v(), b.kp, b.kd, b.kd / dt, b.lat, b.sl, tuner.best_pace(), tuner.best_laps().c_str());
					else
						std::snprintf(line, sizeof line, "auto-tune: nothing passed yet (kept kp %.2f, kd %.4f s, lat max %.2f, slalom %.2f)",
							b.kp, b.kd, b.lat, b.sl);
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
					std::snprintf(line, sizeof line, "drive %d  %-9s lap %5.2f s  vmax %.2f  markers %s", n, d.outcome.c_str(),
						d.lap_time_fw, d.max_v, d.marker_check.c_str());
					session.history.push_back(line);
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
