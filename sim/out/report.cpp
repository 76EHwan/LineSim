#include "out/report.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace {

const char* marker_name(int t) {
	switch (t) {
	case 1: return "L";
	case 2: return "R";
	case 3: return "C";
	case 4: return "STOP";
	default: return "?";
	}
}

std::string esc(const std::string& s) {
	std::string o;
	for (char c : s) {
		if (c == '"' || c == '\\')
			o += '\\';
		if (c == '\n') {
			o += "\\n";
			continue;
		}
		o += c;
	}
	return o;
}

// speed -> color (dark blue .. cyan .. yellow .. red)
std::string speed_color(double v, double vmax) {
	double t = vmax > 0 ? std::clamp(v / vmax, 0.0, 1.0) : 0.0;
	struct C { double r, g, b; } stops[] = {{40, 60, 200}, {0, 190, 220}, {250, 220, 40}, {230, 40, 30}};
	double x = t * 3;
	int i = std::min(2, static_cast<int>(x));
	double f = x - i;
	char buf[16];
	std::snprintf(buf, sizeof buf, "#%02x%02x%02x", (int) (stops[i].r + (stops[i + 1].r - stops[i].r) * f),
		(int) (stops[i].g + (stops[i + 1].g - stops[i].g) * f), (int) (stops[i].b + (stops[i + 1].b - stops[i].b) * f));
	return buf;
}

void write_csv(const fs::path& p, const DriveResult& d) {
	std::ofstream f(p);
	f << "t,x,y,yaw_deg,v,wz_dps,lat_mm,s,pitch_deg,wheel_v_l,wheel_v_r,iq_l,iq_r,"
	     "fw_target_base,fw_current_base,fw_target_pos,fw_odom,fw_yaw,fw_omega_t_l,fw_omega_t_r,"
	     "fw_omega_m_l,fw_omega_m_r,fw_iq_cmd_l,fw_iq_cmd_r,fw_lost,fw_markers,body_contact,fw_cmd_mps\n";
	char line[512];
	for (const Sample& s : d.samples) {
		std::snprintf(line, sizeof line,
			"%.4f,%.5f,%.5f,%.3f,%.4f,%.2f,%.2f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,"
			"%.4f,%.4f,%.4f,%.4f,%.3f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%d,%d,%d,%.4f\n",
			s.t, s.x, s.y, s.yaw_deg, s.v, s.wz_dps, s.lat_mm, s.s, s.pitch_deg, s.wheel_v[0], s.wheel_v[1], s.iq[0],
			s.iq[1], s.tgt_base, s.cur_base, s.tgt_pos, s.odom, s.yaw_fw, s.omega_t[0], s.omega_t[1], s.omega_m[0],
			s.omega_m[1], s.iq_cmd[0], s.iq_cmd[1], s.lost, s.markers, s.contact, s.cmd_mps);
		f << line;
	}
}

void write_markers(const fs::path& p, const DriveResult& d) {
	std::ofstream f(p);
	f << "idx,type,dist_from_prev_m,yaw_deg\n";
	for (size_t i = 0; i < d.markers.size(); i++)
		f << i << "," << marker_name(d.markers[i].type) << "," << d.markers[i].dist_m << "," << d.markers[i].yaw_deg
		  << "\n";
}

void track_svg_body(std::ostream& o, const Track& track) {
	// white shapes, drawn in meters inside a flipped group
	o << "<g fill=\"#f4f4f4\" stroke=\"none\">\n";
	for (const Shape& s : track.shapes()) {
		if (s.kind == Shape::ArcBand) {
			o << "<path d=\"";
			int n = std::max(8, static_cast<int>(std::fabs(s.sweep) * s.R / 0.005));
			for (int k = 0; k <= n; k++) {
				double a = s.a0 + s.sweep * k / n;
				o << (k ? " L" : "M") << s.a.x + (s.R + s.hw) * std::cos(a) << "," << s.a.y + (s.R + s.hw) * std::sin(a);
			}
			for (int k = n; k >= 0; k--) {
				double a = s.a0 + s.sweep * k / n;
				o << " L" << s.a.x + (s.R - s.hw) * std::cos(a) << "," << s.a.y + (s.R - s.hw) * std::sin(a);
			}
			o << " Z\"/>\n";
		} else {
			double hl = s.kind == Shape::Band ? s.len / 2 : s.hl;
			double cx = s.kind == Shape::Band ? s.a.x + s.u.x * hl : s.a.x;
			double cy = s.kind == Shape::Band ? s.a.y + s.u.y * hl : s.a.y;
			double nx = -s.u.y, ny = s.u.x;
			o << "<polygon points=\"";
			const int sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, -1, 1};
			for (int k = 0; k < 4; k++)
				o << cx + sx[k] * hl * s.u.x + sy[k] * s.hw * nx << "," << cy + sx[k] * hl * s.u.y + sy[k] * s.hw * ny
				  << " ";
			o << "\"/>\n";
		}
	}
	o << "</g>\n";
}

void write_track_svg(const fs::path& p, const Track& track, const ScenarioResult& res) {
	double x0, y0, x1, y1;
	track.bounds(&x0, &y0, &x1, &y1);
	const double pad = 0.15;
	x0 -= pad;
	y0 -= pad;
	x1 += pad;
	y1 += pad;
	const double scale = std::min(1400.0 / (x1 - x0), 1000.0 / (y1 - y0));
	const double W = (x1 - x0) * scale, H = (y1 - y0) * scale;
	double vmax = 0;
	for (const DriveResult& d : res.drives)
		vmax = std::max(vmax, d.max_v);

	std::ofstream o(p);
	o.precision(6);
	o << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"100%\" viewBox=\"0 0 "
	  << W << " " << H + 60 << "\" font-family=\"sans-serif\" font-size=\"13\">\n";
	o << "<rect width=\"100%\" height=\"100%\" fill=\"#15171a\"/>\n";
	o << "<g transform=\"translate(" << -x0 * scale << "," << y1 * scale << ") scale(" << scale << "," << -scale
	  << ")\">\n";
	track_svg_body(o, track);
	for (size_t di = 0; di < res.drives.size(); di++) {
		const DriveResult& d = res.drives[di];
		const size_t step = 10;
		for (size_t i = 0; i + step < d.samples.size(); i += step) {
			const Sample& a = d.samples[i];
			const Sample& b = d.samples[i + step];
			o << "<line x1=\"" << a.x << "\" y1=\"" << a.y << "\" x2=\"" << b.x << "\" y2=\"" << b.y << "\" stroke=\""
			  << speed_color(0.5 * (a.v + b.v), vmax) << "\" stroke-width=\"" << 3.0 / scale
			  << "\" stroke-linecap=\"round\" opacity=\"0.9\"/>\n";
		}
	}
	if (!res.drives.empty() && !res.drives.front().samples.empty()) {
		const Sample& st = res.drives.front().samples.front();
		o << "<circle cx=\"" << st.x << "\" cy=\"" << st.y << "\" r=\"" << 6.0 / scale << "\" fill=\"#4caf50\"/>\n";
	}
	o << "</g>\n";
	// legend
	o << "<g transform=\"translate(16," << H + 18 << ")\">\n";
	for (int k = 0; k < 100; k++)
		o << "<rect x=\"" << k * 2 << "\" y=\"0\" width=\"2\" height=\"12\" fill=\"" << speed_color(k / 99.0 * vmax, vmax)
		  << "\"/>\n";
	char buf[160];
	std::snprintf(buf, sizeof buf, "speed 0 .. %.2f m/s", vmax);
	o << "<text x=\"210\" y=\"11\" fill=\"#ddd\">" << buf << "</text>\n";
	std::string desc = track.name() + " (" + std::to_string(static_cast<int>(track.length() * 100) / 100.0).substr(0, 5) + " m)";
	o << "<text x=\"420\" y=\"11\" fill=\"#aaa\">" << desc << "</text>\n";
	o << "</g>\n</svg>\n";
}

struct Series {
	std::string name, color;
	std::vector<double> y;
};

void plot_panel(std::ostream& o, double px, double py, double pw, double ph, const std::vector<double>& t,
	const std::vector<Series>& ser, const std::string& title) {
	double lo = 1e9, hi = -1e9;
	for (const Series& s : ser)
		for (double v : s.y) {
			lo = std::min(lo, v);
			hi = std::max(hi, v);
		}
	if (!(hi > lo)) {
		hi = lo + 1;
	}
	double margin = (hi - lo) * 0.05;
	lo -= margin;
	hi += margin;
	double t0 = t.empty() ? 0 : t.front(), t1 = t.empty() ? 1 : t.back();
	if (t1 <= t0)
		t1 = t0 + 1;
	auto X = [&](double v) { return px + (v - t0) / (t1 - t0) * pw; };
	auto Y = [&](double v) { return py + ph - (v - lo) / (hi - lo) * ph; };
	o << "<rect x=\"" << px << "\" y=\"" << py << "\" width=\"" << pw << "\" height=\"" << ph
	  << "\" fill=\"#1d2025\" stroke=\"#444\"/>\n";
	if (lo < 0 && hi > 0)
		o << "<line x1=\"" << px << "\" x2=\"" << px + pw << "\" y1=\"" << Y(0) << "\" y2=\"" << Y(0)
		  << "\" stroke=\"#555\" stroke-dasharray=\"4 4\"/>\n";
	char buf[128];
	std::snprintf(buf, sizeof buf, "%.3g", hi);
	o << "<text x=\"" << px - 6 << "\" y=\"" << py + 10 << "\" text-anchor=\"end\" fill=\"#999\">" << buf << "</text>\n";
	std::snprintf(buf, sizeof buf, "%.3g", lo);
	o << "<text x=\"" << px - 6 << "\" y=\"" << py + ph << "\" text-anchor=\"end\" fill=\"#999\">" << buf << "</text>\n";
	o << "<text x=\"" << px + 6 << "\" y=\"" << py + 16 << "\" fill=\"#ddd\">" << title << "</text>\n";
	double lx = px + pw - 10;
	for (int k = static_cast<int>(ser.size()) - 1; k >= 0; k--) {
		const Series& s = ser[k];
		o << "<text x=\"" << lx << "\" y=\"" << py + 16 << "\" text-anchor=\"end\" fill=\"" << s.color << "\">" << s.name
		  << "</text>\n";
		lx -= 12 + 7.0 * s.name.size();
		o << "<polyline fill=\"none\" stroke=\"" << s.color << "\" stroke-width=\"1.3\" points=\"";
		size_t stride = std::max<size_t>(1, s.y.size() / 2000);
		for (size_t i = 0; i < s.y.size(); i += stride)
			o << X(t[i]) << "," << Y(s.y[i]) << " ";
		o << "\"/>\n";
	}
}

void write_drive_plot(const fs::path& p, const DriveResult& d) {
	std::vector<double> t;
	Series v{"v", "#4fc3f7"}, cb{"fw base", "#ffb74d"}, tb{"fw target", "#777"};
	Series cm{"fw command (after grip limit / pos atten)", "#fff176"};
	Series lat{"lateral error [mm]", "#e57373"}, tp{"target_pos x10", "#81c784"};
	Series il{"iq L", "#ba68c8"}, ir{"iq R", "#4db6ac"};
	for (const Sample& s : d.samples) {
		t.push_back(s.t - d.t_start);
		v.y.push_back(s.v);
		cb.y.push_back(s.cur_base);
		tb.y.push_back(s.tgt_base);
		cm.y.push_back(s.cmd_mps);
		lat.y.push_back(s.lat_mm);
		tp.y.push_back(s.tgt_pos * 10);
		il.y.push_back(s.iq[0]);
		ir.y.push_back(s.iq[1]);
	}
	const double W = 1200, ph = 220, px = 70, pw = W - 90;
	std::ofstream o(p);
	o.precision(6);
	o << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"100%\" viewBox=\"0 0 " << W << " " << 3 * (ph + 30) + 40
	  << "\" font-family=\"sans-serif\" font-size=\"12\">\n<rect width=\"100%\" height=\"100%\" fill=\"#15171a\"/>\n";
	char title[160];
	std::snprintf(title, sizeof title, "drive %d: speed [m/s] (%s, lap %.2f s)", d.n, d.outcome.c_str(), d.lap_time_fw);
	plot_panel(o, px, 20, pw, ph, t, {tb, cb, cm, v}, title);
	plot_panel(o, px, 20 + (ph + 30), pw, ph, t, {lat, tp}, "line tracking (sensor-bar center vs. line)");
	plot_panel(o, px, 20 + 2 * (ph + 30), pw, ph, t, {il, ir}, "motor q-axis current [A]");
	o << "<text x=\"" << px + pw / 2 << "\" y=\"" << 3 * (ph + 30) + 30
	  << "\" text-anchor=\"middle\" fill=\"#999\">time since drive start [s]</text>\n</svg>\n";
}

}  // namespace

void write_reports(const ScenarioOptions& opt, const ScenarioResult& res, const Track& track) {
	fs::path out = opt.out_dir;
	for (const DriveResult& d : res.drives) {
		std::string n = std::to_string(d.n);
		write_csv(out / ("drive" + n + ".csv"), d);
		write_markers(out / ("drive" + n + "_markers.csv"), d);
		write_drive_plot(out / ("drive" + n + ".svg"), d);
	}
	write_track_svg(out / "trajectory.svg", track, res);

	// index.html: summary table + all plots on one page
	{
		std::ofstream h(out / "index.html");
		h << "<!doctype html><html><head><meta charset=\"utf-8\"><title>LineSim - " << track.name()
		  << "</title><style>body{background:#15171a;color:#ddd;font-family:sans-serif;margin:16px}"
		     "table{border-collapse:collapse;margin:12px 0}td,th{border:1px solid #444;padding:4px 10px;text-align:right}"
		     "th{background:#22252a}td.l{text-align:left}img{width:100%;max-width:1200px;display:block;margin:12px 0}"
		     "pre{background:#1d2025;padding:8px;display:inline-block;margin:4px}</style></head><body>\n";
		h << "<h2>" << track.name() << " (" << track.length() << " m)</h2>\n";
		if (!res.error.empty())
			h << "<p style=\"color:#e57373\">" << res.error << "</p>\n";
		const FwDriveParams& p = res.params;
		h << "<p>base " << p.base_mps << " m/s, max " << p.max_mps << " m/s, accel " << p.accel << ", decel " << p.decel
		  << ", steer kp " << p.steer_kp << ", kd " << p.steer_kd << ", target_shift " << p.target_shift << "</p>\n";
		h << "<table><tr><th>drive</th><th>outcome</th><th>lap [s]</th><th>max v</th><th>max lat [mm]</th>"
		     "<th>rms lat [mm]</th><th>markers</th></tr>\n";
		for (const DriveResult& d : res.drives) {
			char row[256];
			std::snprintf(row, sizeof row,
				"<tr><td>%d</td><td class=\"l\">%s</td><td>%.2f</td><td>%.2f</td><td>%.1f</td><td>%.1f</td><td class=\"l\">",
				d.n, d.outcome.c_str(), d.lap_time_fw, d.max_v, d.max_lat_mm, d.rms_lat_mm);
			h << row;
			for (const FwMarker& m : d.markers)
				h << marker_name(m.type) << " ";
			h << "</td></tr>\n";
		}
		h << "</table>\n<img src=\"trajectory.svg\">\n";
		for (const DriveResult& d : res.drives)
			h << "<h3>drive " << d.n << "</h3><pre>" << d.lcd << "</pre><img src=\"drive" << d.n << ".svg\">\n";
		h << "</body></html>\n";
	}

	std::ofstream j(out / "summary.json");
	j.precision(6);
	j << "{\n  \"ok\": " << (res.ok ? "true" : "false") << ",\n";
	if (!res.error.empty())
		j << "  \"error\": \"" << esc(res.error) << "\",\n";
	j << "  \"track\": \"" << esc(track.name()) << "\",\n  \"track_length_m\": " << track.length() << ",\n";
	j << "  \"sim_seconds\": " << res.sim_seconds << ",\n  \"wall_seconds\": " << res.wall_seconds << ",\n";
	const FwDriveParams& p = res.params;
	j << "  \"params\": {\"base_mps\": " << p.base_mps << ", \"max_mps\": " << p.max_mps << ", \"accel\": " << p.accel
	  << ", \"decel\": " << p.decel << ", \"steer_kp\": " << p.steer_kp << ", \"steer_kd\": " << p.steer_kd
	  << ", \"pos_atten_gain\": " << p.pos_atten_gain << ", \"pit_in_m\": " << p.pit_in_m
	  << ", \"target_shift\": " << p.target_shift << ", \"fan_en\": " << p.fan_en << ", \"yaw_kp\": " << p.yaw_kp
	  << ", \"yaw_ki\": " << p.yaw_ki << ", \"lat_acc_max\": " << p.lat_acc_max
	  << ", \"target_curve_gain\": " << p.target_curve_gain << ", \"slalom_ratio\": " << p.slalom_ratio << "},\n";
	j << "  \"drives\": [\n";
	for (size_t i = 0; i < res.drives.size(); i++) {
		const DriveResult& d = res.drives[i];
		j << "    {\"drive\": " << d.n << ", \"outcome\": \"" << d.outcome << "\", \"lap_time_fw\": " << d.lap_time_fw
		  << ", \"duration\": " << d.t_end - d.t_start << ", \"distance_m\": " << d.dist_m
		  << ", \"max_speed\": " << d.max_v << ", \"max_lat_mm\": " << d.max_lat_mm << ", \"rms_lat_mm\": "
		  << d.rms_lat_mm << ", \"markers\": \"";
		for (const FwMarker& m : d.markers)
			j << marker_name(m.type) << " ";
		j << "\", \"marker_check\": \"" << esc(d.marker_check) << "\", \"lcd\": \"" << esc(d.lcd) << "\"}" << (i + 1 < res.drives.size() ? "," : "") << "\n";
	}
	j << "  ]\n}\n";
}

void print_summary(const ScenarioResult& res) {
	std::printf("\n drive | outcome     | lap [s] | max v | max lat [mm] | rms lat [mm] | marker check | markers\n");
	std::printf(" ------+-------------+---------+-------+--------------+--------------+--------------+--------\n");
	for (const DriveResult& d : res.drives) {
		std::string mk;
		for (const FwMarker& m : d.markers) {
			mk += marker_name(m.type);
			mk += ' ';
		}
		std::printf("   %d   | %-11s | %7.2f | %5.2f | %12.1f | %12.1f | %-12s | %s\n", d.n, d.outcome.c_str(), d.lap_time_fw,
			d.max_v, d.max_lat_mm, d.rms_lat_mm, d.marker_check.c_str(), mk.c_str());
	}
	std::printf("\n simulated %.1f s in %.1f s wall clock (%.1fx real time)\n", res.sim_seconds, res.wall_seconds,
		res.wall_seconds > 0 ? res.sim_seconds / res.wall_seconds : 0.0);
}
