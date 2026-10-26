#include "world/track.h"

#include <algorithm>
#include <cmath>

#include "core/json.h"

#include <fstream>
#include <sstream>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRuleRadius = 0.25;   // ZETIN rules: every arc has R = 25 cm

double wrap_pi(double a) {
	while (a > kPi)
		a -= 2 * kPi;
	while (a < -kPi)
		a += 2 * kPi;
	return a;
}

Vec2 left_normal(double th) { return {-std::sin(th), std::cos(th)}; }

// 5x5 sample pattern over +-2 sigma, Gaussian weights normalized to 1.
struct SpotPattern {
	double ox[25], oy[25], w[25];
	SpotPattern() {
		double sum = 0;
		int k = 0;
		for (int i = -2; i <= 2; i++)
			for (int j = -2; j <= 2; j++, k++) {
				ox[k] = i;
				oy[k] = j;
				w[k] = std::exp(-0.5 * (i * i + j * j));
				sum += w[k];
			}
		for (double& x : w)
			x /= sum;
	}
};
const SpotPattern kSpot;
}  // namespace

Pose2 PathPiece::at(double ds) const {
	ds = std::clamp(ds, 0.0, len);
	if (!arc)
		return {p0.x + ds * std::cos(th0), p0.y + ds * std::sin(th0), th0};
	double phi = turn * ds / radius;
	double rx = p0.x - center.x, ry = p0.y - center.y;
	double c = std::cos(phi), s = std::sin(phi);
	return {center.x + rx * c - ry * s, center.y + rx * s + ry * c, th0 + phi};
}

void Shape::compute_bounds() {
	switch (kind) {
	case Band: {
		Vec2 b{a.x + u.x * len, a.y + u.y * len};
		minx = std::min(a.x, b.x) - hw;
		maxx = std::max(a.x, b.x) + hw;
		miny = std::min(a.y, b.y) - hw;
		maxy = std::max(a.y, b.y) + hw;
		break;
	}
	case ArcBand: {
		minx = a.x - R - hw;
		maxx = a.x + R + hw;
		miny = a.y - R - hw;
		maxy = a.y + R + hw;
		double rin = std::max(0.0, R - hw);
		double rout = R + hw;
		r_in_sq = rin * rin;
		r_out_sq = rout * rout;
		break;
	}
	case Rect: {
		double ex = std::fabs(u.x) * hl + std::fabs(u.y) * hw;
		double ey = std::fabs(u.y) * hl + std::fabs(u.x) * hw;
		minx = a.x - ex;
		maxx = a.x + ex;
		miny = a.y - ey;
		maxy = a.y + ey;
		break;
	}
	}
}

bool Shape::contains(Vec2 p) const {
	if (p.x < minx || p.x > maxx || p.y < miny || p.y > maxy)
		return false;
	double dx = p.x - a.x, dy = p.y - a.y;
	switch (kind) {
	case Band: {
		double t = dx * u.x + dy * u.y;
		double n = -dx * u.y + dy * u.x;
		return t >= 0 && t <= len && std::fabs(n) <= hw;
	}
	case ArcBand: {
		double d2 = dx * dx + dy * dy;
		if (d2 < r_in_sq || d2 > r_out_sq)
			return false;
		double ang = wrap_pi(std::atan2(dy, dx) - a0);
		if (sweep >= 0) {
			if (ang < 0)
				ang += 2 * kPi;
			return ang <= sweep;
		}
		if (ang > 0)
			ang -= 2 * kPi;
		return ang >= sweep;
	}
	case Rect: {
		double t = dx * u.x + dy * u.y;
		double n = -dx * u.y + dy * u.x;
		return std::fabs(t) <= hl && std::fabs(n) <= hw;
	}
	}
	return false;
}

bool Track::load(const std::string& path, std::string* err) {
	std::ifstream f(path, std::ios::binary);
	if (!f) {
		*err = "cannot open " + path;
		return false;
	}
	std::stringstream ss;
	ss << f.rdbuf();
	return load_text(ss.str(), path, err);
}

bool Track::load_text(const std::string& text, const std::string& path, std::string* err) {
	*this = Track();
	std::string perr;
	json::Value j = json::parse(text, &perr);
	if (!perr.empty()) {
		*err = path + ": " + perr;
		return false;
	}

	name_ = j.str("name", path);
	closed_ = j.boolean("closed", true);
	line_width_ = j.num("line_width", 0.02);
	if (const json::Value* m = j.get("mark")) {
		mark_gap_ = m->num("gap", mark_gap_);
		mark_along_ = m->num("along", mark_along_);
		mark_across_ = m->num("across", mark_across_);
	}

	Pose2 cur;
	if (const json::Value* o = j.get("origin"); o && o->is_arr() && o->size() >= 3) {
		cur.x = (*o)[0].n;
		cur.y = (*o)[1].n;
		cur.th = (*o)[2].n * kPi / 180.0;
	}
	const Pose2 origin = cur;

	const json::Value* segs = j.get("segments");
	if (!segs || !segs->is_arr() || segs->size() == 0) {
		*err = path + ": \"segments\" missing";
		return false;
	}
	double s = 0;
	for (const json::Value& sg : segs->a) {
		PathPiece p;
		p.s0 = s;
		p.p0 = {cur.x, cur.y};
		p.th0 = cur.th;
		if (sg.has("straight")) {
			p.arc = false;
			p.len = sg.num("straight", 0);
			p.ux = std::cos(p.th0);
			p.uy = std::sin(p.th0);
		} else if (sg.has("arc")) {
			double deg = sg.num("arc", 0);
			p.arc = true;
			p.radius = sg.num("radius", kRuleRadius);
			p.turn = deg >= 0 ? 1 : -1;
			p.len = std::fabs(deg) * kPi / 180.0 * p.radius;
			Vec2 n = left_normal(cur.th);
			p.center = {cur.x + p.turn * p.radius * n.x, cur.y + p.turn * p.radius * n.y};
			p.ux = std::cos(p.th0);
			p.uy = std::sin(p.th0);
			p.a0 = std::atan2(p.p0.y - p.center.y, p.p0.x - p.center.x);
			p.sweep = p.len / p.radius;
			double a = std::fabs(deg);
			if (std::fabs(p.radius - kRuleRadius) > 1e-9 || std::fabs(a / 45.0 - std::round(a / 45.0)) > 1e-6 || a < 45 ||
				a > 270)
				warnings_.push_back("arc " + std::to_string(deg) + " deg R " + std::to_string(p.radius) +
					" m is outside the rules (R 0.25 m, 45..270 deg in 45 deg steps)");
		} else {
			*err = path + ": segment needs \"straight\" or \"arc\"";
			return false;
		}
		if (p.len <= 0) {
			*err = path + ": zero-length segment";
			return false;
		}
		if (!p.arc) {
			Vec2 p1 = {p.p0.x + p.len * p.ux, p.p0.y + p.len * p.uy};
			p.minx = std::min(p.p0.x, p1.x);
			p.maxx = std::max(p.p0.x, p1.x);
			p.miny = std::min(p.p0.y, p1.y);
			p.maxy = std::max(p.p0.y, p1.y);
		} else {
			p.minx = p.center.x - p.radius;
			p.maxx = p.center.x + p.radius;
			p.miny = p.center.y - p.radius;
			p.maxy = p.center.y + p.radius;
		}
		pieces_.push_back(p);
		cur = p.at(p.len);
		s += p.len;
	}
	length_ = s;
	if (closed_) {
		double gap = std::hypot(cur.x - origin.x, cur.y - origin.y);
		double dth = std::fabs(wrap_pi(cur.th - origin.th));
		if (gap > 0.001 || dth > 0.001)
			warnings_.push_back("closed course does not close: end is " + std::to_string(gap * 1000) + " mm / " +
				std::to_string(dth * 180 / kPi) + " deg away from the start");
	}

	// the line itself (self-crossings come out of the geometry: two perpendicular lines)
	const double hw = line_width_ / 2;
	for (const PathPiece& p : pieces_) {
		Shape sh;
		sh.hw = hw;
		if (!p.arc) {
			sh.kind = Shape::Band;
			sh.a = p.p0;
			sh.u = {std::cos(p.th0), std::sin(p.th0)};
			sh.len = p.len;
		} else {
			sh.kind = Shape::ArcBand;
			sh.a = p.center;
			sh.R = p.radius;
			sh.a0 = std::atan2(p.p0.y - p.center.y, p.p0.x - p.center.x);
			sh.sweep = p.turn * p.len / p.radius;
		}
		sh.compute_bounds();
		shapes_.push_back(sh);
	}

	// End marks: left + right mark at the start point and at the goal point
	// (the firmware reads a pair as CROSS_STOP; the 2nd one ends the run).
	if (!j.has("start_s") || !j.has("goal_s")) {
		*err = path + ": \"start_s\" and \"goal_s\" (end mark positions) are required";
		return false;
	}
	start_s_ = j.num("start_s", 0.0);
	goal_s_ = j.num("goal_s", 0.0);
	for (double e : {start_s_, goal_s_}) {
		add_mark(e, +1);
		add_mark(e, -1);
		markers_.push_back({e, "end"});
	}

	// Turn marks: at the start of every curve section, on the side it turns to; where the
	// curve section ends and a straight starts, on the side of the last turn. A section is
	// a run of arcs in the same direction, so S-bends get a mark at each change.
	if (j.boolean("auto_turn_marks", true)) {
		int prev = closed_ ? pieces_.back().turn : 0;
		for (const PathPiece& p : pieces_) {
			if (p.turn != prev) {
				int side = p.turn != 0 ? p.turn : prev;
				add_mark(p.s0, side);
				markers_.push_back({p.s0, side > 0 ? "left" : "right"});
			}
			prev = p.turn;
		}
		if (!closed_ && prev != 0) {
			add_mark(length_, prev);
			markers_.push_back({length_, prev > 0 ? "left" : "right"});
		}
	}

	if (const json::Value* ms = j.get("marks"); ms && ms->is_arr()) {
		for (const json::Value& m : ms->a) {
			double ms_s = m.num("s", 0);
			std::string side = m.str("side", "");
			if (side == "left") {
				add_mark(ms_s, +1);
			} else if (side == "right") {
				add_mark(ms_s, -1);
			} else if (side == "both") {
				add_mark(ms_s, +1);
				add_mark(ms_s, -1);
			} else {
				*err = path + ": mark side must be left / right / both";
				return false;
			}
			markers_.push_back({ms_s, side});
		}
	}

	has_robot_start_ = j.has("robot_start_s");
	robot_start_s_ = j.num("robot_start_s", start_s_);
	build_index();
	find_crossings();
	return true;
}

void Track::add_mark(double s, int side) {
	// A bar perpendicular to the line: `along` wide in the driving direction, `across` long,
	// starting `gap` away from the edge of the line.
	Pose2 p = pose_at(s);
	Vec2 n = left_normal(p.th);
	double off = line_width_ / 2 + mark_gap_ + mark_across_ / 2;
	Shape sh;
	sh.kind = Shape::Rect;
	sh.a = {p.x + side * off * n.x, p.y + side * off * n.y};
	sh.u = {std::cos(p.th), std::sin(p.th)};
	sh.hl = mark_along_ / 2;
	sh.hw = mark_across_ / 2;
	sh.compute_bounds();
	shapes_.push_back(sh);
}

Pose2 Track::pose_at(double s) const {
	if (closed_) {
		s = std::fmod(s, length_);
		if (s < 0)
			s += length_;
	} else if (s < 0) {
		// extend the first straight backwards (robot placed before the start)
		const PathPiece& p = pieces_.front();
		return {p.p0.x + s * std::cos(p.th0), p.p0.y + s * std::sin(p.th0), p.th0};
	}
	for (const PathPiece& p : pieces_)
		if (s <= p.s0 + p.len)
			return p.at(s - p.s0);
	return pieces_.back().at(pieces_.back().len);
}

bool Track::nearest(Vec2 q, double* lateral, double* s_out) const {
	if (pieces_.empty())
		return false;

	double best_d2 = 1e18, best_lat = 0, best_s = 0;
	size_t best_idx = last_piece_idx_ < pieces_.size() ? last_piece_idx_ : 0;

	auto eval_piece = [&](size_t idx, const PathPiece& p) {
		if (!p.arc) {
			double dx = q.x - p.p0.x, dy = q.y - p.p0.y;
			double t = std::clamp(dx * p.ux + dy * p.uy, 0.0, p.len);
			double px = p.p0.x + t * p.ux, py = p.p0.y + t * p.uy;
			double ex = q.x - px, ey = q.y - py;
			double d2 = ex * ex + ey * ey;
			if (d2 < best_d2) {
				best_d2 = d2;
				best_lat = -ex * p.uy + ey * p.ux;
				best_s = p.s0 + t;
				best_idx = idx;
			}
		} else {
			double dx = q.x - p.center.x, dy = q.y - p.center.y;
			double ang = wrap_pi(std::atan2(dy, dx) - p.a0) * p.turn;
			if (ang < -1e-9)
				ang += 2 * kPi;
			double t = (ang <= p.sweep) ? ang * p.radius
			         : ((ang - p.sweep < 2 * kPi - ang) ? p.len : 0.0);
			Pose2 pp = p.at(t);
			double ex = q.x - pp.x, ey = q.y - pp.y;
			double d2 = ex * ex + ey * ey;
			if (d2 < best_d2) {
				best_d2 = d2;
				best_lat = -ex * std::sin(pp.th) + ey * std::cos(pp.th);
				best_s = p.s0 + t;
				best_idx = idx;
			}
		}
	};

	// 1. Evaluate cached nearest piece first for temporal continuity
	eval_piece(best_idx, pieces_[best_idx]);

	// 2. Evaluate other pieces with bounding-box distance rejection
	for (size_t i = 0; i < pieces_.size(); i++) {
		if (i == best_idx)
			continue;
		const PathPiece& p = pieces_[i];
		double d_box_x = 0;
		if (q.x < p.minx) d_box_x = p.minx - q.x;
		else if (q.x > p.maxx) d_box_x = q.x - p.maxx;

		double d_box_y = 0;
		if (q.y < p.miny) d_box_y = p.miny - q.y;
		else if (q.y > p.maxy) d_box_y = q.y - p.maxy;

		if (d_box_x * d_box_x + d_box_y * d_box_y >= best_d2)
			continue;

		eval_piece(i, p);
	}

	last_piece_idx_ = best_idx;
	if (lateral)
		*lateral = best_lat;
	if (s_out)
		*s_out = best_s;
	return best_d2 < 1e16;
}

void Track::build_index() {
	double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
	for (const Shape& s : shapes_) {
		x0 = std::min(x0, s.minx);
		y0 = std::min(y0, s.miny);
		x1 = std::max(x1, s.maxx);
		y1 = std::max(y1, s.maxy);
	}
	gx0_ = x0 - cell_;
	gy0_ = y0 - cell_;
	gnx_ = static_cast<int>((x1 - gx0_) / cell_) + 2;
	gny_ = static_cast<int>((y1 - gy0_) / cell_) + 2;
	grid_.assign(static_cast<size_t>(gnx_) * gny_, {});
	const double margin = 0.020; // 20 mm sensor margin covers max 2.5 * sigma
	for (int k = 0; k < static_cast<int>(shapes_.size()); k++) {
		const Shape& s = shapes_[k];
		int ix0 = std::max(0, static_cast<int>((s.minx - margin - gx0_) / cell_));
		int ix1 = std::min(gnx_ - 1, static_cast<int>((s.maxx + margin - gx0_) / cell_));
		int iy0 = std::max(0, static_cast<int>((s.miny - margin - gy0_) / cell_));
		int iy1 = std::min(gny_ - 1, static_cast<int>((s.maxy + margin - gy0_) / cell_));
		for (int ix = ix0; ix <= ix1; ix++)
			for (int iy = iy0; iy <= iy1; iy++)
				grid_[static_cast<size_t>(iy) * gnx_ + ix].push_back(k);
	}
}

bool Track::white(Vec2 p) const {
	int ix = static_cast<int>((p.x - gx0_) / cell_);
	int iy = static_cast<int>((p.y - gy0_) / cell_);
	if (ix < 0 || iy < 0 || ix >= gnx_ || iy >= gny_)
		return false;
	for (int k : grid_[static_cast<size_t>(iy) * gnx_ + ix])
		if (shapes_[k].contains(p))
			return true;
	return false;
}

double Track::coverage(Vec2 p, double sigma) const {
	int ix = static_cast<int>((p.x - gx0_) / cell_);
	int iy = static_cast<int>((p.y - gy0_) / cell_);
	if (ix < 0 || iy < 0 || ix >= gnx_ || iy >= gny_)
		return 0.0;

	const auto& cg = grid_[static_cast<size_t>(iy) * gnx_ + ix];
	if (cg.empty())
		return 0.0;

	double c = 0;
	for (int k = 0; k < 25; k++) {
		Vec2 sp = {p.x + kSpot.ox[k] * sigma, p.y + kSpot.oy[k] * sigma};
		for (int sk : cg) {
			if (shapes_[sk].contains(sp)) {
				c += kSpot.w[k];
				break;
			}
		}
	}
	return c;
}

void Track::bounds(double* x0, double* y0, double* x1, double* y1) const {
	*x0 = 1e9;
	*y0 = 1e9;
	*x1 = -1e9;
	*y1 = -1e9;
	for (const Shape& s : shapes_) {
		*x0 = std::min(*x0, s.minx);
		*y0 = std::min(*y0, s.miny);
		*x1 = std::max(*x1, s.maxx);
		*y1 = std::max(*y1, s.maxy);
	}
}

// ---------------------------------------------------------------------------
// crossings, expected marker sequence, rule check
// ---------------------------------------------------------------------------

double Track::wrap_s(double s) const {
	if (!closed_ || length_ <= 0)
		return s;
	s = std::fmod(s, length_);
	return s < 0 ? s + length_ : s;
}

namespace {
struct Sample {
	double s, x, y, th;
};

std::vector<Sample> sample_track(const Track& t, double step) {
	std::vector<Sample> out;
	for (const PathPiece& p : t.pieces()) {
		int n = std::max(1, static_cast<int>(std::ceil(p.len / step)));
		for (int k = 0; k < n; k++) {
			double d = p.len * k / n;
			Pose2 q = p.at(d);
			out.push_back({p.s0 + d, q.x, q.y, q.th});
		}
	}
	return out;
}

bool seg_intersect(const Sample& a, const Sample& b, const Sample& c, const Sample& d, double* ta) {
	double rx = b.x - a.x, ry = b.y - a.y, sx = d.x - c.x, sy = d.y - c.y;
	double den = rx * sy - ry * sx;
	if (std::fabs(den) < 1e-12)
		return false;
	double t = ((c.x - a.x) * sy - (c.y - a.y) * sx) / den;
	double u = ((c.x - a.x) * ry - (c.y - a.y) * rx) / den;
	if (t < 0 || t > 1 || u < 0 || u > 1)
		return false;
	*ta = t;
	return true;
}
}  // namespace

void Track::find_crossings() {
	crossings_.clear();
	std::vector<Sample> sm = sample_track(*this, 0.005);
	const size_t n = sm.size();
	if (n < 4)
		return;
	const double cell = 0.05;
	std::vector<std::pair<long long, size_t>> keyed;
	auto key = [&](double x, double y) {
		return (static_cast<long long>(std::floor(x / cell)) << 32) ^ (static_cast<long long>(std::floor(y / cell)) & 0xffffffffLL);
	};
	for (size_t i = 0; i < n; i++)
		keyed.push_back({key(sm[i].x, sm[i].y), i});
	std::sort(keyed.begin(), keyed.end());
	auto circ = [&](double a, double b) {
		double d = std::fabs(a - b);
		return closed_ ? std::min(d, length_ - d) : d;
	};
	for (size_t i = 0; i + 1 < n + (closed_ ? 1 : 0); i++) {
		const Sample& a = sm[i % n];
		const Sample& b = sm[(i + 1) % n];
		for (int dx = -1; dx <= 1; dx++)
			for (int dy = -1; dy <= 1; dy++) {
				long long k = key(a.x + dx * cell, a.y + dy * cell);
				auto it = std::lower_bound(keyed.begin(), keyed.end(), std::make_pair(k, size_t(0)));
				for (; it != keyed.end() && it->first == k; ++it) {
					size_t j = it->second;
					if (j <= i || j + 1 >= n + (closed_ ? 1 : 0))
						continue;
					const Sample& c = sm[j % n];
					const Sample& d = sm[(j + 1) % n];
					if (circ(a.s, c.s) < 0.3)
						continue;
					double t;
					if (!seg_intersect(a, b, c, d, &t))
						continue;
					double s1 = a.s + t * 0.005;
					bool dup = false;
					for (const TrackCrossing& x : crossings_)
						if (circ(x.s1, s1) < 0.1 || circ(x.s2, s1) < 0.1)
							dup = true;
					if (dup)
						continue;
					TrackCrossing x;
					x.s1 = s1;
					x.s2 = c.s;
					x.p = {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
					double ang = std::fabs(wrap_pi(a.th - c.th)) * 180.0 / kPi;
					x.angle_deg = ang > 90 ? 180 - ang : ang;
					crossings_.push_back(x);
				}
			}
	}
}

void Track::straight_run(double s, double* before, double* after) const {
	*before = *after = 0;
	s = wrap_s(s);
	const size_t n = pieces_.size();
	for (size_t i = 0; i < n; i++) {
		const PathPiece& p = pieces_[i];
		if (s < p.s0 - 1e-9 || s > p.s0 + p.len + 1e-9)
			continue;
		if (p.arc)
			return;
		*before = s - p.s0;
		*after = p.s0 + p.len - s;
		for (size_t k = 1; k < n; k++) {   // extend over neighbouring straights
			size_t j = closed_ ? (i + n - k) % n : i - k;
			if ((!closed_ && k > i) || pieces_[j].arc)
				break;
			*before += pieces_[j].len;
		}
		for (size_t k = 1; k < n; k++) {
			size_t j = closed_ ? (i + k) % n : i + k;
			if ((!closed_ && j >= n) || pieces_[j].arc)
				break;
			*after += pieces_[j].len;
		}
		return;
	}
}

std::vector<ExpectedMarkerEvent> Track::expected_marker_events() const {
	std::vector<std::pair<double, char>> ev;
	const double span = closed_ ? wrap_s(goal_s_ - start_s_) : goal_s_ - start_s_;
	auto rel = [&](double s) { return closed_ ? wrap_s(s - start_s_) : s - start_s_; };
	for (const TrackMarker& m : markers_) {
		char c = m.kind == "left" ? 'L' : m.kind == "right" ? 'R' : 'U';
		if (m.kind == "end")
			continue;
		ev.push_back({rel(m.s), c});
	}
	for (const TrackCrossing& x : crossings_) {
		ev.push_back({rel(x.s1), 'C'});
		ev.push_back({rel(x.s2), 'C'});
	}
	std::sort(ev.begin(), ev.end());

	std::vector<ExpectedMarkerEvent> res;
	size_t idx = 0;
	// Start mark 'U'
	{
		Pose2 p = pose_at(start_s_);
		ExpectedMarkerEvent em;
		em.index = idx++;
		em.type = 'U';
		em.s = start_s_;
		em.side = 0;
		em.p_centerline = Vec2{p.x, p.y};
		em.center = Vec2{p.x, p.y};
		em.th = p.th;
		res.push_back(em);
	}

	const double mark_dist = line_width_ / 2.0 + mark_gap_ + mark_across_ / 2.0;
	for (const auto& e : ev) {
		if (e.first > 1e-6 && e.first < span - 1e-6) {
			double s_abs = closed_ ? wrap_s(start_s_ + e.first) : start_s_ + e.first;
			Pose2 p = pose_at(s_abs);
			Vec2 n = left_normal(p.th);
			ExpectedMarkerEvent em;
			em.index = idx++;
			em.type = e.second;
			em.s = s_abs;
			em.p_centerline = Vec2{p.x, p.y};
			em.th = p.th;
			if (em.type == 'L') {
				em.side = +1;
				em.center = {p.x + mark_dist * n.x, p.y + mark_dist * n.y};
			} else if (em.type == 'R') {
				em.side = -1;
				em.center = {p.x - mark_dist * n.x, p.y - mark_dist * n.y};
			} else {
				em.side = 0;
				em.center = Vec2{p.x, p.y};
			}
			res.push_back(em);
		}
	}
	// Goal mark 'U'
	{
		Pose2 p = pose_at(goal_s_);
		ExpectedMarkerEvent em;
		em.index = idx++;
		em.type = 'U';
		em.s = goal_s_;
		em.side = 0;
		em.p_centerline = Vec2{p.x, p.y};
		em.center = Vec2{p.x, p.y};
		em.th = p.th;
		res.push_back(em);
	}
	return res;
}

std::string Track::expected_markers() const {
	std::string out;
	for (const auto& ev : expected_marker_events())
		out += ev.type;
	return out;
}

std::vector<std::string> Track::rule_problems(double board_w, double board_h, double min_clear) const {
	std::vector<std::string> out = warnings_;
	char buf[256];
	double x0, y0, x1, y1;
	bounds(&x0, &y0, &x1, &y1);
	if ((x1 - x0) > board_w || (y1 - y0) > board_h) {
		std::snprintf(buf, sizeof buf, "does not fit the board: %.2f x %.2f m", x1 - x0, y1 - y0);
		out.push_back(buf);
	}
	double b, a;
	straight_run(goal_s_, &b, &a);
	if (b < 0.2)
		out.push_back("less than 20 cm of straight before the goal");
	straight_run(start_s_, &b, &a);
	if (a < 0.2)
		out.push_back("less than 20 cm of straight after the start");
	if (closed_ && wrap_s(start_s_ - goal_s_) < 0.3)
		out.push_back("start and goal are less than 30 cm apart");
	for (const TrackCrossing& x : crossings_) {
		for (double s : {x.s1, x.s2})   // keep crossings off the goal..start straight
			if (wrap_s(s - (goal_s_ - 0.25)) < wrap_s(start_s_ + 0.25 - (goal_s_ - 0.25))) {
				std::snprintf(buf, sizeof buf, "crossing at (%.2f, %.2f) is in the end-mark straight", x.p.x, x.p.y);
				out.push_back(buf);
			}
		if (std::fabs(x.angle_deg - 90) > 2) {
			std::snprintf(buf, sizeof buf, "crossing at (%.2f, %.2f) is %.0f deg, not 90", x.p.x, x.p.y, x.angle_deg);
			out.push_back(buf);
			continue;
		}
		for (double s : {x.s1, x.s2}) {
			straight_run(s, &b, &a);
			if (b < 0.2 || a < 0.2) {
				std::snprintf(buf, sizeof buf, "crossing at (%.2f, %.2f) has less than 20 cm of straight", x.p.x, x.p.y);
				out.push_back(buf);
				break;
			}
		}
	}
	// different parts of the course must not run close to each other (except at crossings)
	std::vector<Sample> sm = sample_track(*this, 0.01);
	auto circ = [&](double p, double q) {
		double d = std::fabs(p - q);
		return closed_ ? std::min(d, length_ - d) : d;
	};
	auto near_cross = [&](double s) {
		for (const TrackCrossing& x : crossings_)
			if (circ(s, x.s1) < 0.3 || circ(s, x.s2) < 0.3)
				return true;
		return false;
	};
	// spatial grid with cells of min_clear: only samples in neighbouring cells can be too close
	const size_t n = sm.size();
	std::vector<char> skip(n);
	double gx0 = 1e18, gy0 = 1e18;
	for (size_t i = 0; i < n; i++) {
		skip[i] = near_cross(sm[i].s);
		gx0 = std::min(gx0, sm[i].x);
		gy0 = std::min(gy0, sm[i].y);
	}
	const double cell = std::max(min_clear, 0.05);
	auto cx = [&](double x) { return static_cast<int>((x - gx0) / cell); };
	auto cy = [&](double y) { return static_cast<int>((y - gy0) / cell); };
	int nx = 0, ny = 0;
	for (size_t i = 0; i < n; i++) {
		nx = std::max(nx, cx(sm[i].x) + 1);
		ny = std::max(ny, cy(sm[i].y) + 1);
	}
	std::vector<std::vector<int>> grid(static_cast<size_t>(nx) * ny);
	for (size_t i = 0; i < n; i++)
		if (!skip[i])
			grid[static_cast<size_t>(cy(sm[i].y)) * nx + cx(sm[i].x)].push_back(static_cast<int>(i));
	for (size_t i = 0; i < n; i++) {
		if (skip[i])
			continue;
		const int gx = cx(sm[i].x), gy = cy(sm[i].y);
		for (int yy = std::max(0, gy - 1); yy <= std::min(ny - 1, gy + 1); yy++)
			for (int xx = std::max(0, gx - 1); xx <= std::min(nx - 1, gx + 1); xx++)
				for (int j : grid[static_cast<size_t>(yy) * nx + xx]) {
					if (static_cast<size_t>(j) <= i || circ(sm[i].s, sm[j].s) < 0.45)
						continue;
					double dx = sm[i].x - sm[j].x, dy = sm[i].y - sm[j].y;
					if (dx * dx + dy * dy < min_clear * min_clear) {
						std::snprintf(buf, sizeof buf, "lines closer than %.0f cm near (%.2f, %.2f)", min_clear * 100,
							sm[i].x, sm[i].y);
						out.push_back(buf);
						return out;
					}
				}
	}
	return out;
}
