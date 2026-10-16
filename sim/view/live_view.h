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
	bool auto_tune = false;         // true when either A (mode 1) or E (mode 2) is active
	int tune_mode = 0;              // 0: off, 1: A (3-course gain auto-tune), 2: E (endless 1+4 per map + AI heal)
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
	void drive_started() {
		v_.clear_trail();
		has_trail_pose_ = false;
		live_beta_ = 0;
		max_beta_seen_ = 0;
	}
	// New course: the lap records start over.
	void clear_laps() { courses_.clear(); }
	// A course left the auto-tune set: its records go.
	void drop_course(const std::string& course) {
		for (size_t i = 0; i < courses_.size(); i++)
			if (courses_[i].key == course) {
				courses_.erase(courses_.begin() + static_cast<long>(i));
				return;
			}
	}
	// Lap list on the left, per course (lap times only compare on the same course) and drive 1-4:
	// the latest attempt (finished or not) and the best finished one with every marker right.
	// course: key of the course, label: its heading; tag = attempt tag over all drives (e.g. #15-1 or #1);
	// ok = counts as a lap.
	void record_lap(const std::string& course, const std::string& label, int drive, const std::string& tag, bool ok, double lap_s,
		const std::string& what) {
		if (drive < 1 || drive > 4)
			return;
		CourseLaps* c = nullptr;
		for (CourseLaps& e : courses_)
			if (e.key == course)
				c = &e;
		if (!c) {
			courses_.push_back({course, label, {}});
			c = &courses_.back();
		}
		c->label = label;
		LapSlot& l = c->d[drive - 1];
		std::string full_tag = (tag.empty() || tag[0] == '#') ? tag : ("#" + tag);
		char buf[160];
		std::snprintf(buf, sizeof buf, "last %-8s %s", full_tag.c_str(), what.c_str());
		l.last = buf;
		if (ok && (l.best_tag.empty() || lap_s < l.best_s)) {
			l.best_tag = full_tag;
			l.best_s = lap_s;
		}
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
	struct LapSlot {
		std::string last;      // latest attempt, empty = none yet
		std::string best_tag;  // e.g. "#15-2" or "#1"
		double best_s = 0;
	};
	struct CourseLaps {
		std::string key, label;
		LapSlot d[4];
	};
	std::vector<CourseLaps> courses_;   // in the order they were first driven
	bool stop_ = false;
	double anchor_wall_ = 0, anchor_sim_ = 0;
	double last_frame_ = -1;
	double last_t_ = 0;
	int trail_div_ = 0;
	double fps_ = 60;
	double achieved_ = 0, rate_wall_ = 0, rate_sim_ = 0;   // measured playback speed
	double last_render_ = 0;                               // wall time of the last frame [s]
	double last_pump_ = 0;                                 // wall time of the last window message pump
	int shot_no_ = 0;
	bool script_loaded_ = false, script_active_ = false, script_waiting_drive_ = false, script_saw_busy_ = false;
	int script_wait_ticks_ = 0;
	double script_end_wall_ = 0;
	std::vector<std::string> script_;
	double auto_shot_ = 0, next_auto_shot_ = 0;   // LINESIM_AUTOSHOT=<sim seconds>: periodic screenshots
	Pose2 last_trail_pose_{};
	bool has_trail_pose_ = false;
	double live_beta_ = 0;
	double max_beta_seen_ = 0;
};
