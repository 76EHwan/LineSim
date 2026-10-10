#include "view/live_view.h"

#include <windows.h>
#include <timeapi.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

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
	float step, lo, hi;
	const char* fmt;
};
const ParamRow kRows[] = {
	{"base speed", &FwDriveParams::base_mps, 0.1f, 0.1f, 6.0f, "%.2f m/s"},
	{"max speed", &FwDriveParams::max_mps, 0.25f, 0.5f, 12.0f, "%.2f m/s"},
	{"accel", &FwDriveParams::accel, 0.5f, 0.5f, 30.0f, "%.1f m/s2"},
	{"decel", &FwDriveParams::decel, 0.5f, 0.5f, 30.0f, "%.1f m/s2"},
	{"steer kp", &FwDriveParams::steer_kp, 0.2f, 0.0f, 200.0f, "%.2f"},
	{"steer kd", &FwDriveParams::steer_kd, 0.1f, 0.0f, 2000.0f, "%.2f"},
	{"pos atten gain", &FwDriveParams::pos_atten_gain, 0.05f, 0.0f, 2.0f, "%.2f"},
	{"target shift", &FwDriveParams::target_shift, 0.05f, 0.0f, 1.0f, "%.2f"},
	{"pit-in dist", &FwDriveParams::pit_in_m, 0.01f, 0.0f, 1.0f, "%.2f m"},
};
constexpr int kRowCount = sizeof kRows / sizeof kRows[0];
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
		switch (k) {
		case VK_SPACE: paused_ = !paused_; break;
		case VK_OEM_PLUS:
		case VK_ADD: speed_ = std::min(16.0, speed_ * 2); break;
		case VK_OEM_MINUS:
		case VK_SUBTRACT: speed_ = std::max(1.0 / 16, speed_ / 2); break;
		case 'C': v_.set_cam_mode((v_.cam_mode() + 1) % Viewer::kCamModes); break;
		case 'T': v_.clear_trail(); break;
		case 'S': v_.toggle_shadows(); break;
		case 'V': v_.toggle_inset(); break;
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
		float& v = s.params.*(r.field);
		v += (k == VK_RIGHT ? 1 : -1) * r.step * (shift ? 10 : 1);
		v = std::clamp(v, r.lo, r.hi);
		v = std::round(v / (r.step / 10)) * (r.step / 10);   // no float drift
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
		if (s.busy)
			s.message = "wait for the drive to finish (X stops it)";
		else
			s.request_new_map = true;
		break;
	case 'X':
		if (s.busy)
			s.request_abort = true;
		break;
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
		if (!script_saw_busy_ || s.busy)
			return;
		script_waiting_drive_ = false;
	}
	if (s.busy || s.request_drive || s.request_new_map)
		return;
	if (script_.empty()) {
		stop_ = true;
		return;
	}
	std::string k = script_.front();
	script_.erase(script_.begin());
	const std::pair<const char*, int> names[] = {{"UP", VK_UP}, {"DOWN", VK_DOWN}, {"LEFT", VK_LEFT}, {"RIGHT", VK_RIGHT}};
	int vk = k.size() == 1 ? static_cast<unsigned char>(k[0]) : 0;
	for (const auto& n : names)
		if (k == n.first)
			vk = n.second;
	session_key(vk);
	if (vk >= '1' && vk <= '4') {
		script_waiting_drive_ = true;
		script_saw_busy_ = false;
	}
}

std::string LiveView::session_panel() const {
	const SessionState& s = *session;
	std::string t = "course  " + s.map_name + "\n";
	t += "expected " + s.expected + "\n\n";
	char buf[160];
	for (int i = 0; i < kRowCount; i++) {
		char val[48];
		std::snprintf(val, sizeof val, kRows[i].fmt, s.params.*(kRows[i].field));
		std::snprintf(buf, sizeof buf, "%s %-15s %s\n", i == s.selected ? ">" : "  ", kRows[i].name, val);
		t += buf;
	}
	t += "\n";
	size_t from = s.history.size() > 8 ? s.history.size() - 8 : 0;
	for (size_t i = from; i < s.history.size(); i++)
		t += s.history[i] + "\n";
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
	char buf[512];
	std::snprintf(buf, sizeof buf,
		"%s\n"
		"sim time  %.2f s   (x%.3g%s)\n"
		"speed     %.2f m/s   fw base %.2f / target %.2f\n"
		"line      %+.1f mm (sensor bar)   target_pos %+.2f\n"
		"yaw (fw)  %+.1f deg   markers %d%s\n"
		"iq        L %+.2f A   R %+.2f A",
		phase.c_str(), t_sim, speed_, paused_ ? ", PAUSED" : "", robot_.forward_speed(), tm.current_base_mps,
		tm.target_base_mps, lat * 1000, tm.target_pos, tm.yaw_deg, tm.marker_count, tm.lost ? "   LINE LOST" : "",
		robot_.iq(0), robot_.iq(1));
	hud.status = buf;
	hud.help = "Space pause   +/- sim speed   C camera   V sensor view   S shadows   T clear trail   P screenshot   Esc quit\n"
	           "mouse: left drag rotate, right drag pan, wheel zoom";
	if (interactive_)
		hud.help = "Arrows: U/D/L/R buttons   Enter: K button   R: robot back to start\n" + hud.help;
	if (session) {
		hud.help = "1-4: run drive 1-4   N: new random course   X: stop the robot   R: robot to start\n"
		           "Up/Down: pick parameter   Left/Right: change it (Shift: x10)\n" + hud.help;
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
	if (!v_.pump()) {
		stop_ = true;
		return;
	}
	handle_keys();
	run_session_script();
	if (++trail_div_ >= 10) {
		trail_div_ = 0;
		Pose2 p = robot_.pose();
		if (robot_.forward_speed() > 0.02)
			v_.add_trail(p.x, p.y, robot_.forward_speed());
	}
	last_t_ = t_sim;
	if (auto_shot_ > 0 && t_sim >= next_auto_shot_ && !fast_forward) {
		next_auto_shot_ = t_sim + auto_shot_;
		char name[64];
		std::snprintf(name, sizeof name, "linesim_auto_%03d.bmp", ++shot_no_);
		v_.request_screenshot(name);
		last_frame_ = -1;   // render now
	}

	double now = wall();
	if (fast_forward) {
		if (now - last_frame_ > 0.05) {
			render(t_sim, phase + "   [fast forward]");
			last_frame_ = wall();
		}
		reanchor(t_sim);
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
		return;
	}
	double target = anchor_wall_ + (t_sim - anchor_sim_) / speed_;
	if (now - target > 0.25) {   // can't keep up (or speed changed): don't try to catch up
		reanchor(t_sim);
		target = now;
	}
	for (;;) {
		now = wall();
		if (now - last_frame_ >= 1.0 / fps_) {
			render(t_sim, phase);
			last_frame_ = wall();   // from the end of the frame: slow frames must not starve the simulation
		}
		if (now >= target)
			break;
		double wait = std::min(target - now, 1.0 / fps_);
		if (wait > 0.002)
			Sleep(static_cast<DWORD>(wait * 1000));
		else
			break;
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
