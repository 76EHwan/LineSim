// Connects the viewer to a running simulation: paces simulated time to the wall clock,
// maps keys to the robot's buttons (interactive mode) and builds the on-screen info.
#pragma once
#include <functional>
#include <string>

#include <vector>

#include "fw_bridge.h"
#include "view/viewer.h"
#include "world/robot.h"
#include "world/track.h"

// Session mode: the viewer edits these, the firmware fiber acts on the requests.
struct SessionState {
	FwDriveParams params{};
	int selected = 0;               // parameter row under the cursor
	bool editing = false;           // typing a value for the selected row
	std::string edit_buf;
	int request_drive = 0;          // 1..4
	bool request_new_map = false;
	bool request_abort = false;
	bool busy = false;              // a drive is running
	bool ready = false;             // firmware booted, params loaded
	bool auto_tune = false;         // A: drive 1 over and over, adjusting steer kp / kd
	std::vector<std::string> tune_lines;   // auto-tune status for the panel
	std::string map_name, expected; // expected marker sequence (U L R C)
	std::vector<std::string> history;
	std::string message;
};

class LiveView {
public:
	SessionState* session = nullptr;

	LiveView(Viewer& viewer, Robot& robot, const Track& track, bool interactive, double speed);

	// Called every simulated millisecond on the simulator fiber. `fast_forward` skips the
	// real-time pacing (boot, waiting at the start line ...).
	void tick(double t_sim, bool fast_forward, const std::string& phase);
	// Keeps the window alive after the run has ended, until the user closes it.
	void hold_until_closed();
	bool stop_requested() const { return stop_; }
	// A drive is starting: the trail shows this drive only.
	void drive_started() { v_.clear_trail(); }
	// One line per drive for the lap list on the left (finished or not); keeps the last 8.
	void add_lap(const std::string& line) {
		laps_.push_back(line);
		if (laps_.size() > 8)
			laps_.erase(laps_.begin());
	}

	std::function<void()> on_reset;   // R key (interactive): put the robot back on the start line

private:
	void render(double t_sim, const std::string& phase);
	void handle_keys();
	void session_key(int vk);
	void edit_key(int vk);   // session: typing a parameter value
	std::string session_panel() const;
	void run_session_script();
	void reanchor(double t_sim);

	Viewer& v_;
	Robot& robot_;
	const Track& track_;
	bool interactive_;
	double speed_;
	bool paused_ = false;
	bool show_trail_ = true;
	std::vector<std::string> laps_;
	bool stop_ = false;
	double anchor_wall_ = 0, anchor_sim_ = 0;
	double last_frame_ = -1;
	double last_t_ = 0;
	int trail_div_ = 0;
	double fps_ = 60;
	double achieved_ = 0, rate_wall_ = 0, rate_sim_ = 0;   // measured playback speed
	double last_render_ = 0;                               // wall time of the last frame [s]
	int shot_no_ = 0;
	bool script_loaded_ = false, script_active_ = false, script_waiting_drive_ = false, script_saw_busy_ = false;
	int script_wait_ticks_ = 0;
	std::vector<std::string> script_;
	double auto_shot_ = 0, next_auto_shot_ = 0;   // LINESIM_AUTOSHOT=<sim seconds>: periodic screenshots
};
