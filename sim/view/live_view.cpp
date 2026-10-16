#include "view/live_view.h"

#include <windows.h>
#include <timeapi.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "core/scenario.h"
#include "core/simulation.h"
#include "fw_bridge.h"
#include "sim_mcu.h"

namespace {
double wall() {
	using namespace std::chrono;
	return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// session: editable firmware drive parameters (driveData)
struct ParamRow {
	const char* name;
	float FwDriveParams::*field;
	float step, lo, hi;   // in the shown unit
	const char* fmt;
	bool per_dt = false;  // shown in seconds, stored as the firmware's CMSIS Kd (= seconds / control period)
};
const ParamRow kRows[] = {
	{"base speed", &FwDriveParams::base_mps, 0.01f, 0.1f, 6.0f, "%.3f m/s"},
	{"max speed", &FwDriveParams::max_mps, 0.01f, 0.5f, 12.0f, "%.3f m/s"},
	{"accel", &FwDriveParams::accel, 0.5f, 0.5f, 30.0f, "%.1f m/s2"},
	{"decel", &FwDriveParams::decel, 0.5f, 0.5f, 30.0f, "%.1f m/s2"},
	{"steer kp", &FwDriveParams::steer_kp, 0.2f, 0.0f, 200.0f, "%.2f"},
	{"steer kd", &FwDriveParams::steer_kd, 0.01f, 0.0f, 2.0f, "%.3f s", true},
	{"pos atten gain", &FwDriveParams::pos_atten_gain, 0.05f, 0.0f, 2.0f, "%.2f"},
	{"target shift", &FwDriveParams::target_shift, 0.05f, 0.0f, 1.0f, "%.2f"},
	{"tgt curve gain", &FwDriveParams::target_curve_gain, 0.1f, 0.0f, 3.0f, "x%.2f"},   // drive 3/4, in curves
	{"slalom ratio", &FwDriveParams::slalom_ratio, 0.05f, 0.0f, 1.0f, "%.2f"},          // drive 4, 45 deg segments
	{"brake margin", &FwDriveParams::brake_margin_m, 0.01f, 0.0f, 1.0f, "%.2f m"},       // straight braking margin
	{"pit-in dist", &FwDriveParams::pit_in_m, 0.01f, 0.0f, 1.0f, "%.2f m"},
	{"yaw kp", &FwDriveParams::yaw_kp, 0.1f, 0.0f, 20.0f, "%.2f"},          // cascade (0 / 0 = plain PD)
	{"yaw ki", &FwDriveParams::yaw_ki, 1.0f, 0.0f, 500.0f, "%.1f /s"},
	{"lat acc max", &FwDriveParams::lat_acc_max, 0.5f, 0.0f, 40.0f, "%.1f m/s2"},
};
constexpr int kRowCount = sizeof kRows / sizeof kRows[0];

float shown(const FwDriveParams& p, const ParamRow& r) {
	const float v = p.*(r.field);
	return r.per_dt ? static_cast<float>(v * fw_steer_dt()) : v;
}
void store(FwDriveParams& p, const ParamRow& r, float v) {
	p.*(r.field) = r.per_dt ? static_cast<float>(v / fw_steer_dt()) : v;
}
}  // namespace

LiveView::LiveView(Viewer& viewer, Robot& robot, const Track& track, bool interactive, double speed)
	: v_(viewer), robot_(robot), track_(track), interactive_(interactive), speed_(speed) {
	timeBeginPeriod(1);   // 1 ms Sleep granularity for smooth pacing
	if (const char* a = std::getenv("LINESIM_AUTOSHOT"))
		auto_shot_ = std::atof(a);
	anchor_wall_ = wall();
}

void LiveView::reanchor(double t_sim) {
	anchor_wall_ = wall();
	anchor_sim_ = t_sim;
}

void LiveView::handle_keys() {
	for (int k; (k = v_.pop_key()) != 0;) {
		if (session && session->editing) {   // typing a value: every key belongs to the entry
			edit_key(k);
			continue;
		}
		switch (k) {
		case VK_SPACE: paused_ = !paused_; break;
		case VK_OEM_PLUS:
		case VK_ADD:
			speed_ = std::min(256.0, speed_ * 2);
			reanchor(last_t_);   // pace from now on; the time already played stays as it was
			break;
		case VK_OEM_MINUS:
		case VK_SUBTRACT:
			speed_ = std::max(1.0 / 16, speed_ / 2);
			reanchor(last_t_);
			break;
		case 'C': v_.set_cam_mode((v_.cam_mode() + 1) % Viewer::kCamModes); break;
		case 'T':   // trail on/off; off also erases it
			show_trail_ = !show_trail_;
			v_.clear_trail();
			break;
		case 'S':
			if (session)
				session_key(k);
			else
				v_.toggle_shadows();
			break;
		case 'M':
		case 'V': v_.cycle_inset(); break;
		case 'P': {
			char name[64];
			std::snprintf(name, sizeof name, "linesim_shot_%03d.bmp", ++shot_no_);
			v_.request_screenshot(name);
			break;
		}
		case 'R':
			if ((interactive_ || (session && !session->busy)) && on_reset) {
				on_reset();
				v_.clear_trail();
				has_trail_pose_ = false;
				live_beta_ = 0;
				max_beta_seen_ = 0;
			}
			break;
		case VK_ESCAPE: stop_ = true; break;
		default:
			if (session)
				session_key(k);
			break;
		}
	}
	if (interactive_) {
		fwb_set_button(FWB_BTN_U, v_.key_held(VK_UP));
		fwb_set_button(FWB_BTN_D, v_.key_held(VK_DOWN));
		fwb_set_button(FWB_BTN_L, v_.key_held(VK_LEFT));
		fwb_set_button(FWB_BTN_R, v_.key_held(VK_RIGHT));
		fwb_set_button(FWB_BTN_K, v_.key_held(VK_RETURN));
	}
}

void LiveView::session_key(int k) {
	SessionState& s = *session;
	if (!s.ready)
		return;   // parameters arrive from the firmware once it has booted
	const bool shift = v_.key_held(VK_SHIFT);
	switch (k) {
	case VK_UP: s.selected = (s.selected + kRowCount - 1) % kRowCount; break;
	case VK_DOWN: s.selected = (s.selected + 1) % kRowCount; break;
	case VK_LEFT:
	case VK_RIGHT: {
		const ParamRow& r = kRows[s.selected];
		float v = shown(s.params, r);
		v += (k == VK_RIGHT ? 1 : -1) * r.step * (shift ? 10 : 1);
		v = std::clamp(v, r.lo, r.hi);
		v = std::round(v / (r.step / 10)) * (r.step / 10);   // no float drift
		store(s.params, r, v);
		fwb_set_drive_params(&s.params);
		break;
	}
	case VK_RETURN: {   // type the value of the selected row
		s.editing = true;
		s.edit_buf.clear();
		s.message.clear();
		break;
	}
	case '1':
	case '2':
	case '3':
	case '4':
		if (s.busy)
			s.message = "a drive is running (X stops it)";
		else
			s.request_drive = k - '0';
		break;
	case 'N':
		if (s.busy && !s.auto_tune)
			s.message = "wait for the drive to finish (X stops it)";
		else {
			s.request_new_map = true;
			if (s.auto_tune)
				s.message = "auto-tune: new map requested (swapping next)...";
		}
		break;
	case 'X':
		if (s.busy)
			s.request_abort = true;
		s.auto_tune = false;
		s.tune_mode = 0;
		break;
	case 'A':   // 3-course AI Bayesian Gain Auto-Tune (mode 1)
		if (s.auto_tune && s.tune_mode == 1) {
			s.auto_tune = false;
			s.tune_mode = 0;
		} else if (s.busy && !s.auto_tune) {
			s.message = "wait for the drive to finish (X stops it)";
		} else {
			s.auto_tune = true;
			s.tune_mode = 1;
			s.message = "switched to [A] 3-Course AI Gain Auto-Tune";
			if (speed_ < 16.0)
				speed_ = 32.0;
		}
		break;
	case 'E':   // Endless 100-Map Reliability + AI Gain Auto-Heal (mode 2)
		if (s.auto_tune && s.tune_mode == 2) {
			s.auto_tune = false;
			s.tune_mode = 0;
		} else if (s.busy && !s.auto_tune) {
			s.message = "wait for the drive to finish (X stops it)";
		} else {
			s.auto_tune = true;
			s.tune_mode = 2;
			s.message = "switched to [E] Endless 100-Map + AI Gain Auto-Heal";
			if (speed_ < 16.0)
				speed_ = 32.0;
		}
		break;
	case 's':
	case 'S': {  // save current session parameters directly to firmware/Main/Src/drive.c
		std::string out_path;
		fwb_set_drive_params(&s.params);
		if (save_drive_params_to_firmware_file(s.params, &out_path)) {
			char msg[160];
			std::snprintf(msg, sizeof msg, "[SAVED to drive.c] v=%.2f kp=%.1f kd=%.0f shf=%.2f",
				s.params.base_mps, s.params.steer_kp, s.params.steer_kd, s.params.target_shift);
			s.message = msg;
			std::printf("[SAVE] Current parameters saved to %s\n", out_path.c_str());
		} else {
			s.message = "[SAVE FAILED] Could not write to drive.c (check console)";
		}
		break;
	}
	default: break;
	}
}

void LiveView::edit_key(int k) {
	SessionState& s = *session;
	if (k == 's' || k == 'S') {  // user pressed 's' while editing: commit typed value and save immediately
		s.editing = false;
		if (!s.edit_buf.empty() && s.edit_buf != ".") {
			const ParamRow& r = kRows[s.selected];
			const float v = static_cast<float>(std::atof(s.edit_buf.c_str()));
			store(s.params, r, std::clamp(v, r.lo, r.hi));
			fwb_set_drive_params(&s.params);
		}
		session_key(k);
		return;
	}
	char c = 0;
	if (k >= '0' && k <= '9')
		c = static_cast<char>(k);
	else if (k >= VK_NUMPAD0 && k <= VK_NUMPAD9)
		c = static_cast<char>('0' + (k - VK_NUMPAD0));
	else if (k == VK_OEM_PERIOD || k == VK_DECIMAL)
		c = '.';
	if (c) {
		if (s.edit_buf.size() < 12 && !(c == '.' && s.edit_buf.find('.') != std::string::npos))
			s.edit_buf += c;
		return;
	}
	switch (k) {
	case VK_BACK:
		if (!s.edit_buf.empty())
			s.edit_buf.pop_back();
		break;
	case VK_ESCAPE:
		s.editing = false;
		break;
	case VK_RETURN: {
		s.editing = false;
		if (s.edit_buf.empty() || s.edit_buf == ".")
			break;
		const ParamRow& r = kRows[s.selected];
		const float v = static_cast<float>(std::atof(s.edit_buf.c_str()));
		store(s.params, r, std::clamp(v, r.lo, r.hi));
		fwb_set_drive_params(&s.params);
		if (v < r.lo || v > r.hi) {
			char msg[96];
			std::snprintf(msg, sizeof msg, "%s: kept within %g .. %g", r.name, r.lo, r.hi);
			s.message = msg;
		}
		break;
	}
	default: break;
	}
}

// LINESIM_SESSION_SCRIPT="1,RIGHT,RIGHT,1,N,1": session keys for unattended tests. Each key
// waits until the previous drive / new map is done; the window closes after the last one.
void LiveView::run_session_script() {
	if (!session)
		return;
	if (!script_loaded_) {
		script_loaded_ = true;
		if (const char* sc = std::getenv("LINESIM_SESSION_SCRIPT")) {
			std::string s = sc;
			for (size_t p = 0; p < s.size();) {
				size_t q = s.find(',', p);
				if (q == std::string::npos)
					q = s.size();
				script_.push_back(s.substr(p, q - p));
				p = q + 1;
			}
			script_active_ = true;
		}
	}
	if (!script_active_ || !session->ready)
		return;
	SessionState& s = *session;
	if (script_waiting_drive_) {
		if (s.busy)
			script_saw_busy_ = true;
		// a drive that never starts (e.g. 2 after a wrong drive 1) does not hold the script: 2 s
		if (!script_saw_busy_ && ++script_wait_ticks_ < 2000)
			return;
		if (s.busy)
			return;
		script_waiting_drive_ = false;
	}
	if (s.busy || s.request_drive || s.request_new_map)
		return;
	if (script_.empty()) {
		if (script_end_wall_ <= 0)   // 1 s (wall clock) more, so a last P screenshot gets drawn
			script_end_wall_ = wall();
		if (wall() - script_end_wall_ < 1.0)
			return;
		stop_ = true;
		return;
	}
	std::string k = script_.front();
	script_.erase(script_.begin());
	const std::pair<const char*, int> names[] = {{"UP", VK_UP}, {"DOWN", VK_DOWN}, {"LEFT", VK_LEFT}, {"RIGHT", VK_RIGHT},
		{"PLUS", VK_OEM_PLUS}, {"MINUS", VK_OEM_MINUS}, {"ENTER", VK_RETURN}, {"ESC", VK_ESCAPE}, {"BACK", VK_BACK},
		{"DOT", VK_OEM_PERIOD}};
	int vk = k.size() == 1 ? static_cast<unsigned char>(k[0]) : 0;
	for (const auto& n : names)
		if (k == n.first)
			vk = n.second;
	if (vk == VK_OEM_PLUS || vk == VK_OEM_MINUS || vk == 'V' || vk == 'C' || vk == 'T' || vk == 'P') {
		v_.inject_key(vk);   // playback / view keys go through the normal key handling
		return;
	}
	if (s.editing) {
		edit_key(vk);
		return;
	}
	session_key(vk);
	if (vk >= '1' && vk <= '4') {
		script_waiting_drive_ = true;
		script_saw_busy_ = false;
		script_wait_ticks_ = 0;
	}
}

std::string LiveView::session_panel() const {
	const SessionState& s = *session;
	std::string map_display = s.map_name;
	if (map_display.size() > 24) {
		size_t slash = map_display.find_last_of("/\\");
		if (slash != std::string::npos)
			map_display = map_display.substr(slash + 1);
	}
	std::string t = "course   " + map_display + "\n";
	if (!s.expected.empty()) {
		if (s.expected.size() > 36) {
			t += "markers: ";
			size_t col = 9;
			for (size_t k = 0; k < s.expected.size(); ++k) {
				t += s.expected[k];
				col++;
				if (col >= 35 && s.expected[k] == ' ') {
					t += "\n         ";
					col = 9;
				}
			}
			t += "\n\n";
		} else {
			t += "markers: " + s.expected + "\n\n";
		}
	}
	char buf[160];
	for (int i = 0; i < kRowCount && s.tune_lines.empty(); i++) {   // auto-tune shows its own table instead
		char val[64];
		if (s.editing && i == s.selected) {
			std::snprintf(val, sizeof val, "[%s_]", s.edit_buf.c_str());
		} else {
			std::snprintf(val, sizeof val, kRows[i].fmt, shown(s.params, kRows[i]));
			if (kRows[i].per_dt) {   // what goes into the robot's menu (driveData.steer_gain_d)
				char fw[24];
				std::snprintf(fw, sizeof fw, " (fw %.0f)", s.params.*(kRows[i].field));
				std::strncat(val, fw, sizeof val - std::strlen(val) - 1);
			}
		}
		std::snprintf(buf, sizeof buf, "%s %-16s %s\n", i == s.selected ? ">" : " ", kRows[i].name, val);
		t += buf;
	}
	t += "\n";
	if (!s.tune_lines.empty()) {
		for (const std::string& l : s.tune_lines)
			t += l + "\n";
		// the rest of the parameters (the tuner sets base speed, kp, kd, lat acc max, slalom ratio), two per line
		t += "\nother parameters\n";
		int col = 0;
		for (int i = 0; i < kRowCount; i++) {
			const float* f = &(s.params.*(kRows[i].field));
			if (f == &s.params.base_mps || f == &s.params.steer_kp || f == &s.params.steer_kd ||
				f == &s.params.lat_acc_max || f == &s.params.slalom_ratio)
				continue;
			char val[48];
			std::snprintf(val, sizeof val, kRows[i].fmt, shown(s.params, kRows[i]));
			std::snprintf(buf, sizeof buf, "  %-14s %-9s", kRows[i].name, val);
			t += buf;
			col = (col + 1) % 2;
			t += col ? "" : "\n";
		}
		t += col ? "\n\n" : "\n";
	}
	size_t from = s.history.size() > (s.tune_lines.empty() ? 6u : 3u) ? s.history.size() - (s.tune_lines.empty() ? 6 : 3) : 0;
	for (size_t i = from; i < s.history.size(); i++) {
		const std::string& h = s.history[i];
		if (h.size() > 38) {
			size_t p = 0;
			while (p < h.size()) {
				if (h.size() - p <= 38) {
					t += h.substr(p) + "\n";
					break;
				}
				size_t cut = h.rfind(' ', p + 36);
				if (cut == std::string::npos || cut <= p)
					cut = p + 36;
				t += h.substr(p, cut - p) + "\n  ";
				p = (h[cut] == ' ') ? cut + 1 : cut;
			}
		} else {
			t += h + "\n";
		}
	}
	if (!s.message.empty())
		t += s.message + "\n";
	return t;
}

void LiveView::render(double t_sim, const std::string& phase) {
	FwTelemetry tm;
	fwb_telemetry(&tm);
	ViewerHud hud;
	double lat = 0, s = 0;
	const RobotParams& rp = robot_.params();
	track_.nearest(robot_.body_to_ground(rp.sensor_x, 0.0), &lat, &s);
	// wheels: mean tread speed; slip: how much slower the body moves than the treads
	const double v_body = robot_.forward_speed();
	const double v_wheels = 0.5 * (robot_.wheel_speed(0) + robot_.wheel_speed(1)) * rp.wheel_radius;
	const double slip = v_wheels > 0.1 ? 100.0 * (1.0 - v_body / v_wheels) : 0.0;
	char actual[48] = "";
	if (achieved_ > 0 && (speed_ >= 16.0 || achieved_ < 0.9 * speed_))
		std::snprintf(actual, sizeof actual, " (running x%.1f)", achieved_);
	// command: the speed the firmware sends the wheels to (base after pos_atten and the grip limit)
	char cut[48] = "";
	if (tm.current_base_mps > 0.2f && tm.cmd_mps < 0.97f * tm.current_base_mps)
		std::snprintf(cut, sizeof cut, " (-%.0f %%)", 100.0 * (1.0 - tm.cmd_mps / tm.current_base_mps));
	char buf[600];
	std::snprintf(buf, sizeof buf,
		"%s\n"
		"sim speed x%.3g%s%s%s\n"
		"speed     %.2f m/s   wheels %.2f m/s   slip %.0f %%\n"
		"fw base   %.2f / target %.2f m/s   command %.2f m/s%s\n"
		"line      %+.1f mm (sensor bar)   target_pos %+.2f\n"
		"yaw (fw)  %+.1f deg   beta %4.1f deg (max %.1f)%s\n"
		"markers   %d%s\n"
		"iq        L %+.2f A   R %+.2f A",
		phase.c_str(), speed_, actual, paused_ ? ", PAUSED" : "", show_trail_ ? "" : "   (trail off)", v_body, v_wheels,
		slip, tm.current_base_mps, tm.target_base_mps, tm.cmd_mps, cut, lat * 1000, tm.target_pos,
		tm.yaw_deg, live_beta_, max_beta_seen_,
		live_beta_ >= 25.0 ? (std::fabs(lat * 1000.0) > 70.0 ? "  [OVERSTEER - OFF LIMIT!]" : "  [DRIFT: LIMIT OK]")
		                   : (std::fabs(lat * 1000.0) > 70.0 ? "  [LINE DEPARTED! (>70mm)]" : ""),
		tm.marker_count, tm.lost ? "   LINE LOST" : "", robot_.iq(0), robot_.iq(1));
	hud.status = buf;
	hud.live_beta = live_beta_;
	hud.live_lat_mm = lat * 1000.0;
	// lap list: one course as before (drive 1-4); several (auto-tune) under a heading each, with the
	// drives that have a record
	auto lap_line = [](int d, const LapSlot& l) {
		// best first: fixed width, so the columns line up in the proportional font
		char best[48];
		if (!l.best_tag.empty())
			std::snprintf(best, sizeof best, "best %-8s %6.2f s", l.best_tag.c_str(), l.best_s);
		else
			std::snprintf(best, sizeof best, "%-22s", "best   -");
		char line[240];
		std::snprintf(line, sizeof line, "drive %d   %s    %s", d + 1, best, l.last.empty() ? "" : l.last.c_str());
		return std::string(line);
	};
	if (courses_.size() == 1) {
		hud.status += "\n\nlaps";
		for (int d = 0; d < 4; d++)
			hud.status += "\n" + lap_line(d, courses_[0].d[d]);
	} else if (courses_.size() > 1) {
		hud.status += "\n\nlaps (per course)";
		for (const CourseLaps& c : courses_) {
			hud.status += "\n" + c.label;
			for (int d = 0; d < 4; d++)
				if (!c.d[d].last.empty())
					hud.status += "\n   " + lap_line(d, c.d[d]);
		}
	}
	hud.help = "C: camera (Follow/Sensor/Top Map)   M: minimap (Track Map/Sensor/Off)   T: trail (orange=drift, magenta=line-out, red=oversteer off)\n"
	           "Space: pause   +/-: speed   S: save to drive.c   P: screenshot   Esc: quit   (mouse: drag, wheel)";
	if (interactive_)
		hud.help = "Arrows: U/D/L/R buttons   Enter: K button   R: robot back to start\n" + hud.help;
	if (session) {
		hud.help = "1-4: run drive 1-4   N: new random course   X: stop   R: robot to start   S: save to drive.c\n"
		           "Up/Down: pick   Left/Right: change   Enter: type   A: gain auto-tune (3-course)   E: endless (1+4/map + AI heal)\n" + hud.help;
		hud.panel = session_panel();
	}

	int last = -1;
	for (int r = 0; r < SIM_LCD_ROWS; r++) {
		std::string row = sim_lcd_row(r);
		while (!row.empty() && row.back() == ' ')
			row.pop_back();
		hud.lcd.push_back(row.empty() ? " " : row);
		if (!row.empty())
			last = r;
	}
	hud.lcd.resize(static_cast<size_t>(std::max(last + 1, 1)));
	hud.lcd_inverted = sim_lcd_inverted() != 0;
	for (int i = 0; i < 18; i++) {
		hud.sensors[i] = tm.normalized[i] / 255.0f;
		hud.sensor_on[i] = (tm.sensor_state >> i) & 1u;
	}
	static const bool prof = std::getenv("LINESIM_PROFILE") != nullptr;
	static double acc = 0;
	static int frames = 0;
	double t0 = wall();
	v_.render(hud);
	last_render_ = wall() - t0;
	if (prof) {
		acc += wall() - t0;
		if (++frames % 30 == 0) {
			std::fprintf(stderr, "viewer: %.1f ms per frame (last 30)\n", acc / 30 * 1000);
			acc = 0;
		}
	}
}

void LiveView::tick(double t_sim, bool fast_forward, const std::string& phase) {
	if (stop_)
		return;

	double now = wall();
	if (now - last_pump_ >= 0.015) {   // ~60 Hz: rate-limit Win32 message pumping
		last_pump_ = now;
		if (!v_.pump()) {
			stop_ = true;
			return;
		}
		handle_keys();
		run_session_script();
	}

	const bool fast = (speed_ >= 2.0 || (session && session->auto_tune));
	v_.set_fast_mode(fast);

	if (++trail_div_ >= 10) {
		trail_div_ = 0;
		Pose2 p = robot_.pose();
		double beta = 0.0;
		if (has_trail_pose_) {
			double dx = p.x - last_trail_pose_.x;
			double dy = p.y - last_trail_pose_.y;
			double dist = std::hypot(dx, dy);
			if (dist > 1e-4 && robot_.forward_speed() > 0.4) {
				double th_traj = std::atan2(dy, dx) * 180.0 / 3.14159265358979323846;
				double th_veh = p.th * 180.0 / 3.14159265358979323846;
				beta = std::abs(std::remainder(th_veh - th_traj, 360.0));
			}
		}
		last_trail_pose_ = p;
		has_trail_pose_ = true;
		live_beta_ = beta;
		if (robot_.forward_speed() > 0.4)
			max_beta_seen_ = std::max(max_beta_seen_, beta);
		if (show_trail_ && robot_.forward_speed() > 0.02) {
			double lat = 0, s = 0;
			const RobotParams& rp = robot_.params();
			track_.nearest(robot_.body_to_ground(rp.sensor_x, 0.0), &lat, &s);
			v_.add_trail(p.x, p.y, robot_.forward_speed(), beta, lat * 1000.0);
		}
	}
	last_t_ = t_sim;
	if (auto_shot_ > 0 && t_sim >= next_auto_shot_ && !fast_forward) {
		next_auto_shot_ = t_sim + auto_shot_;
		char name[64];
		std::snprintf(name, sizeof name, "linesim_auto_%03d.bmp", ++shot_no_);
		v_.request_screenshot(name);
		last_frame_ = -1;   // render now
	}

	if (fast_forward) {
		if (now - last_frame_ > 0.05) {
			render(t_sim, phase + "   [fast forward]");
			last_frame_ = wall();
		}
		reanchor(t_sim);
		rate_wall_ = 0;
		return;
	}
	if (paused_) {
		while (paused_ && !stop_) {
			if (!v_.pump()) {
				stop_ = true;
				break;
			}
			handle_keys();
			render(t_sim, phase);
			Sleep(15);
		}
		reanchor(t_sim);
		rate_wall_ = 0;
		return;
	}

	// Turbo mode: decoupled unconstrained physics at max CPU throughput
	if (speed_ >= 4.0 || (session && session->auto_tune)) {
		// Render at ~5-8 FPS in turbo/auto-tune mode (0.15s - 0.20s) so GPU rendering
		// takes only ~10-15% of wall time, leaving 85-90% CPU for 6x-8x simulation physics.
		const double frame_dt = (speed_ >= 16.0 || (session && session->auto_tune)) ? 0.20 : 0.12;
		if (now - last_frame_ >= frame_dt) {
			if (rate_wall_ > 0 && now - rate_wall_ >= 0.5) {
				achieved_ = (t_sim - rate_sim_) / (now - rate_wall_);
				rate_wall_ = 0;
				if (std::getenv("LINESIM_PROFILE"))
					std::fprintf(stderr, "[SPEED] achieved = %.1fx real time\n", achieved_);
			}
			if (rate_wall_ <= 0) {
				rate_wall_ = now;
				rate_sim_ = t_sim;
			}
			render(t_sim, phase);
			last_frame_ = wall();
		}
		reanchor(t_sim);
		return;
	}

	// Normal paced mode (< 4x)
	double target = anchor_wall_ + (t_sim - anchor_sim_) / speed_;
	if (now - target > 0.25 || target - now > 0.5 + 0.002 / speed_) {
		reanchor(t_sim);
		target = now;
	}
	const double frame_dt = 1.0 / 60.0;
	if (now - last_frame_ >= frame_dt) {
		if (rate_wall_ > 0 && now - rate_wall_ >= 0.5) {   // simulated seconds per wall second
			achieved_ = (t_sim - rate_sim_) / (now - rate_wall_);
			rate_wall_ = 0;
		}
		if (rate_wall_ <= 0) {
			rate_wall_ = now;
			rate_sim_ = t_sim;
		}
		render(t_sim, phase);
		last_frame_ = wall();   // from the end of the frame: slow frames must not starve the simulation
	}
	if (now < target) {
		double wait = target - now;
		if (wait > 0.002)
			Sleep(static_cast<DWORD>(wait * 1000));
	}
}

void LiveView::hold_until_closed() {
	if (std::getenv("LINESIM_VIEW_AUTOCLOSE"))
		return;
	while (!stop_) {
		if (!v_.pump())
			break;
		handle_keys();
		render(last_t_, "run finished - close the window or press Esc");
		Sleep(15);
	}
}
