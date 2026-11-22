// linesim maprec: real-robot (or sim) marker log -> ZETIN-rule map -> track JSON.
//
// Input (any of these, auto-detected per line):
//   - map log from the firmware SD card  /Map_Log/map_NNN_dK.csv   ("# key=value" header + idx,type,dist,yaw)
//   - sim run output                     driveN_markers.csv        (idx,type,dist_from_prev_m,yaw_deg)
//   - old save slot                      /Drive_Data/save_slot_N.txt ("i<TAB>L<TAB>0.123 m<TAB>45.0 deg")
//   - SysID log                          /SysID_Data/drive_sysid.csv ("# marker_i=L,0.123,45.0")
// Output: a closed track JSON the simulator can drive (start mark at the origin, heading +x).
#include "app/maprec.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "view/map_calib.h"
#include "world/track.h"

namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kR = 0.25;

int type_code(const std::string& t) {
	if (t == "L") return 1;
	if (t == "R") return 2;
	if (t == "C") return 3;
	if (t == "STOP" || t == "U" || t == "S") return 4;
	return 0;
}

std::string trim(const std::string& s) {
	size_t a = s.find_first_not_of(" \t\r\n");
	size_t b = s.find_last_not_of(" \t\r\n");
	return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}

bool read_log(const std::string& path, std::vector<MapLogMark>* marks, std::map<std::string, std::string>* meta,
	std::string* err) {
	std::ifstream f(path);
	if (!f) {
		*err = "cannot open " + path;
		return false;
	}
	std::string line;
	while (std::getline(f, line)) {
		line = trim(line);
		if (line.empty())
			continue;
		if (line[0] == '#') {
			std::string kv = trim(line.substr(1));
			size_t eq = kv.find('=');
			if (eq == std::string::npos)
				continue;
			std::string k = trim(kv.substr(0, eq)), v = trim(kv.substr(eq + 1));
			if (k.rfind("marker_", 0) == 0) {   // SysID log: "# marker_i=T,dist,yaw"
				char t[8] = {0};
				float d = 0, y = 0;
				if (std::sscanf(v.c_str(), "%7[^,],%f,%f", t, &d, &y) == 3 && type_code(t)) {
					marks->push_back({type_code(t), d, y});
				}
			} else {
				(*meta)[k] = v;
			}
			continue;
		}
		// save slot: "i<TAB>L<TAB>0.123 m<TAB>45.0 deg"
		{
			int i = 0;
			char t[8] = {0};
			float d = 0, y = 0;
			if (line.find('\t') != std::string::npos &&
				std::sscanf(line.c_str(), "%d %7s %f m %f", &i, t, &d, &y) == 4 && type_code(t)) {
				marks->push_back({type_code(t), d, y});
				continue;
			}
		}
		// csv: idx,type,dist,yaw
		{
			std::vector<std::string> c;
			std::stringstream ss(line);
			std::string tok;
			while (std::getline(ss, tok, ','))
				c.push_back(trim(tok));
			if (c.size() >= 4 && !c[0].empty() && std::isdigit(static_cast<unsigned char>(c[0][0])) && type_code(c[1])) {
				marks->push_back({type_code(c[1]), std::strtof(c[2].c_str(), nullptr), std::strtof(c[3].c_str(), nullptr)});
			}
		}
	}
	if (marks->size() < 2) {
		*err = path + ": fewer than 2 marker rows";
		return false;
	}
	return true;
}

struct Piece {
	int k45;     // 0 straight
	double len;  // straight length [m]
};

// end pose of a piece list from (x, y, th)
void walk(const std::vector<Piece>& ps, double* x, double* y, double* th) {
	for (const Piece& p : ps) {
		if (p.k45 == 0) {
			*x += p.len * std::cos(*th);
			*y += p.len * std::sin(*th);
		} else {
			const double phi = p.k45 * kPi / 4.0;
			const double turn = phi > 0 ? 1.0 : -1.0;
			const double cx = *x - turn * kR * std::sin(*th), cy = *y + turn * kR * std::cos(*th);
			const double rx = *x - cx, ry = *y - cy, c = std::cos(phi), s = std::sin(phi);
			*x = cx + rx * c - ry * s;
			*y = cy + rx * s + ry * c;
			*th += phi;
		}
	}
}

void push_piece(std::vector<Piece>* ps, Piece p) {
	if (p.k45 == 0 && !ps->empty() && ps->back().k45 == 0)
		ps->back().len += p.len;   // C marks split straights; the track has no need for that
	else
		ps->push_back(p);
}

std::string to_json(const std::string& name, const std::vector<Piece>& ps, bool closed, double start_s, double goal_s,
	const std::string& source) {
	std::ostringstream o;
	o.setf(std::ios::fixed);
	o.precision(4);
	o << "{\n  \"name\": \"" << name << "\",\n  \"closed\": " << (closed ? "true" : "false")
	  << ",\n  \"line_width\": 0.02,\n  \"start_s\": " << start_s << ",\n  \"goal_s\": " << goal_s
	  << ",\n  \"auto_turn_marks\": true,\n  \"segments\": [\n";
	for (size_t i = 0; i < ps.size(); ++i) {
		if (ps[i].k45 == 0)
			o << "    { \"straight\": " << ps[i].len << " }";
		else
			o << "    { \"arc\": " << ps[i].k45 * 45 << " }";
		o << (i + 1 < ps.size() ? ",\n" : "\n");
	}
	o << "  ],\n  \"_source\": \"linesim maprec " << source << "\"\n}\n";
	return o.str();
}

}  // namespace

int run_maprec(int argc, char** argv) {
	std::vector<std::string> ins;
	std::string out, truth;
	for (int i = 2; i < argc; i++) {
		std::string a = argv[i];
		auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
		if (a == "--out")
			out = next();
		else if (a == "--track")
			truth = next();
		else if (a.rfind("--", 0) != 0)
			ins.push_back(a);
		else {
			std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
			return 2;
		}
	}
	if (ins.empty()) {
		std::fprintf(stderr,
			"usage: linesim maprec <map log> [more logs of the same course] [--out map.json] [--track true.json]\n");
		return 2;
	}
	// read every log; the first finished log with the highest weight is the reference
	struct Log {
		std::string path;
		std::vector<MapLogMark> marks;
		std::map<std::string, std::string> meta;
		int drive = 0;
		bool lost = false;
	};
	std::vector<Log> logs;
	std::string err;
	for (const std::string& path : ins) {
		Log L;
		L.path = path;
		if (!read_log(path, &L.marks, &L.meta, &err)) {
			std::fprintf(stderr, "%s\n", err.c_str());
			return 1;
		}
		L.drive = L.meta.count("drive") ? std::atoi(L.meta["drive"].c_str()) : 0;
		L.lost = L.meta.count("line_lost") && std::atoi(L.meta["line_lost"].c_str()) != 0;
		logs.push_back(std::move(L));
	}
	// 1-2회차(라인 위 주행) 가중치 1, 3회차 이후(레이싱 라인, 코너 안쪽 주행) 0.25
	auto weight = [](const Log& L) { return (L.drive >= 3) ? 0.25 : 1.0; };
	size_t ref = 0;
	for (size_t i = 1; i < logs.size(); ++i)
		if (!logs[i].lost && (logs[ref].lost || weight(logs[i]) > weight(logs[ref])))
			ref = i;
	const std::string in = logs[ref].path;
	std::vector<MapLogMark> marks = logs[ref].marks;
	std::map<std::string, std::string> meta = logs[ref].meta;
	const int n = static_cast<int>(marks.size());
	int n_fused = 0;
	if (logs.size() > 1) {
		std::vector<double> wsum(n, 0.0), dsum(n, 0.0), ysum(n, 0.0);
		for (const Log& L : logs) {
			bool same = !L.lost && static_cast<int>(L.marks.size()) == n;
			for (int j = 0; same && j < n; ++j)
				same = L.marks[j].type == marks[j].type;
			if (!same) {
				std::printf("  skip %s (%s)\n", L.path.c_str(), L.lost ? "line lost" : "different marker sequence");
				continue;
			}
			const double w = weight(L);
			for (int j = 0; j < n; ++j) {
				wsum[j] += w;
				dsum[j] += w * L.marks[j].dist_m;
				ysum[j] += w * L.marks[j].seg_yaw_deg;
			}
			++n_fused;
		}
		for (int j = 0; j < n; ++j) {
			if (wsum[j] <= 0) continue;
			marks[j].dist_m = static_cast<float>(dsum[j] / wsum[j]);
			marks[j].seg_yaw_deg = static_cast<float>(ysum[j] / wsum[j]);
		}
		std::printf("  fused %d of %zu logs (reference %s)\n", n_fused, logs.size(), in.c_str());
	}

	// start mark = first STOP, goal mark = last STOP after it
	int i_start = -1, i_goal = -1;
	for (int j = 0; j < n; ++j)
		if (marks[j].type == 4) {
			if (i_start < 0) i_start = j;
			else i_goal = j;
		}
	const bool lost = logs[ref].lost;
	const bool closed_run = i_start >= 0 && i_goal == n - 1 && !lost;

	std::vector<int32_t> k45(n);
	std::vector<float> len(n);
	MapCalibStats st{};
	if (!MapCalib_RuleFromLog(marks.data(), n, closed_run ? 1 : 0, k45.data(), len.data(), &st)) {
		std::fprintf(stderr, "%s: reconstruction failed (%d marks)\n", in.c_str(), n);
		return 1;
	}
	int k_sum = 0;
	for (int j = 0; j < n; ++j) k_sum += k45[j];

	// lap pieces: start mark -> goal mark (segment j ends at mark j)
	const int j0 = (i_start >= 0) ? i_start + 1 : 0;
	const int j1 = closed_run ? i_goal : n - 1;
	std::vector<Piece> ps;
	for (int j = j0; j <= j1; ++j)
		push_piece(&ps, {k45[j], k45[j] ? std::fabs(k45[j]) * kPi / 4.0 * kR : static_cast<double>(len[j])});
	double lap = 0;
	for (const Piece& p : ps) lap += p.k45 ? std::fabs(p.k45) * kPi / 4.0 * kR : p.len;

	bool closed = false;
	double start_s = 0, goal_s = lap, gap = 0, lat_res = 0;
	if (closed_run && (k_sum % 8) == 0) {
		// goal mark -> start mark: a straight on the start line (lateral offset is the closure residual)
		double x = 0, y = 0, th = 0;
		walk(ps, &x, &y, &th);
		gap = -x;      // start mark at the origin, heading +x
		lat_res = y;
		if (gap > 0.01) {
			push_piece(&ps, {0, gap});
			closed = true;
			goal_s = lap;
		} else if (gap > -0.01) {
			closed = true;   // goal and start marks on the same spot
			goal_s = std::max(0.0, lap - 0.01);
		}
	}
	if (!closed) {
		// open course: robot start straight in front, 0.3 m run-out after the last mark
		std::vector<Piece> o;
		double pre = 0;
		for (int j = 0; j < j0; ++j) pre += k45[j] ? std::fabs(k45[j]) * kPi / 4.0 * kR : len[j];
		o.push_back({0, std::max(0.3, pre + 0.1)});
		start_s = o[0].len;
		for (const Piece& p : ps) push_piece(&o, p);
		push_piece(&o, {0, 0.3});
		goal_s = start_s + lap;
		ps.swap(o);
	}

	if (out.empty()) {
		fs::path p(in);
		out = (p.parent_path() / (p.stem().string() + (n_fused > 1 ? "_fused_map.json" : "_map.json"))).string();
	}
	const std::string name = fs::path(out).stem().string();
	const std::string text = to_json(name, ps, closed, start_s, goal_s, fs::path(in).filename().string());
	{
		std::ofstream f(out, std::ios::binary);
		if (!f) {
			std::fprintf(stderr, "cannot write %s\n", out.c_str());
			return 1;
		}
		f << text;
	}

	int nL = 0, nR = 0, nC = 0;
	for (const auto& m : marks) nL += m.type == 1, nR += m.type == 2, nC += m.type == 3;
	int n_arc = 0;
	for (const Piece& p : ps) n_arc += p.k45 != 0;
	std::printf("%s: %d marks (L%d R%d C%d), drive %s, %s\n", in.c_str(), n, nL, nR, nC,
		meta.count("drive") ? meta["drive"].c_str() : "?", lost ? "line lost" : (closed_run ? "finished" : "open"));
	std::printf("  map: %zu pieces (%d arcs), lap %.3f m, total turn %+d deg, %s", ps.size(), n_arc, lap, k_sum * 45,
		closed ? "closed" : "open");
	if (closed)
		std::printf(", goal->start %.3f m, closure residual %.1f mm, crossing pairs %d", gap, lat_res * 1000.0,
			st.crossing_pairs);
	std::printf("\n  wrote %s\n", out.c_str());

	Track rec;
	if (!rec.load(out, &err)) {
		std::fprintf(stderr, "  warning: the written map does not load: %s\n", err.c_str());
		return 1;
	}
	if (!truth.empty()) {
		Track tt;
		if (!tt.load(truth, &err)) {
			std::fprintf(stderr, "%s\n", err.c_str());
			return 1;
		}
		// start marks on top of each other, same heading
		const Pose2 a = tt.pose_at(tt.start_s());
		const Pose2 b = rec.pose_at(rec.start_s());
		const double dth = a.th - b.th, c = std::cos(dth), s = std::sin(dth);
		double sum2 = 0, mx = 0;
		int cnt = 0;
		const double span = lap;
		for (double u = 0; u <= span; u += 0.02) {
			Pose2 p = rec.pose_at(rec.start_s() + u);
			const double dx = p.x - b.x, dy = p.y - b.y;
			Vec2 q{a.x + c * dx - s * dy, a.y + s * dx + c * dy};
			double lat = 0, ss = 0;
			tt.nearest(q, &lat, &ss);
			const double e = std::fabs(lat) * 1000.0;
			sum2 += e * e;
			mx = std::max(mx, e);
			++cnt;
		}
		std::printf("  vs %s: map_err rms %.1f mm, max %.1f mm (true lap %.3f m)\n", truth.c_str(),
			cnt ? std::sqrt(sum2 / cnt) : 0.0, mx,
			tt.closed() ? std::fmod(tt.goal_s() - tt.start_s() + 2 * tt.length(), tt.length()) : tt.goal_s() - tt.start_s());
	}
	return 0;
}
