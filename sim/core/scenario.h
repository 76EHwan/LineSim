// A run: boot the firmware, install sensor calibration, apply drive parameters,
// then execute the requested drives (1st..4th) one after another, like an operator
// standing at the start line with the robot.
#pragma once
#include <map>
#include <string>
#include <vector>

#include "fw_bridge.h"

struct Sample {
	double t;
	float x, y, yaw_deg, v, wz_dps, lat_mm, s, pitch_deg;
	float wheel_v[2];       // wheel surface speed [m/s]
	float iq[2];            // actual q-axis current [A]
	float tgt_base, cur_base, tgt_pos, odom, yaw_fw;
	float cmd_mps;          // firmware's commanded wheel speed (after pos_atten / lat_acc_max)
	float omega_t[2], omega_m[2], iq_cmd[2];
	int lost, markers, contact;
	float beta_deg;         // sideslip angle [deg]
};

struct DriveResult {
	int n = 0;
	std::string outcome;    // finished / finished (oversteer) / line_lost / timeout / off_track / not_started
	bool is_finished() const { return outcome.rfind("finished", 0) == 0; }
	double t_start = 0, t_end = 0;
	double lap_time_fw = -1;            // from the firmware's LCD ("T:x.xxs")
	double max_lat_mm = 0, rms_lat_mm = 0, max_v = 0, dist_m = 0;
	double max_beta_deg = 0;            // peak sideslip angle [deg]
	std::vector<FwMarker> markers;
	std::vector<Sample> samples;
	std::string lcd;
	std::string marker_check;   // "OK" or where the logged markers first differ from the course
};

struct ScenarioOptions {
	std::string track_path;
	std::string robot_path;          // optional
	std::string out_dir;
	std::vector<int> drives{1};
	std::map<std::string, double> params;   // driveData overrides (base_mps, max_mps, ...)
	std::map<std::string, double> robot_params;   // robot model overrides (sensor_delay, tire_mu, ...)
	unsigned seed = 1;
	double max_drive_time = 60.0;     // [s] per drive before the operator lifts the robot
	double off_track_mm = 120.0;      // sensor bar this far from the line -> lift
	bool quiet = false;
	bool view = false;                // 3D viewer, real-time playback of the drives
	bool interactive = false;         // 3D viewer + firmware menu driven from the keyboard
	bool session = false;             // 3D viewer, drives started from the keyboard, parameters edited live
	double view_speed = 1.0;          // playback speed factor
	bool auto_tune = false;           // start auto-tune immediately (headless or GUI)
	int tune_mode = 0;                // 1 = A (Bayes AI), 2 = E (Endless)
	int tune_trials = 0;              // stop auto-tune after N candidate trials (0 = unlimited)
	double endless_max_fail_pct = 10.0; // max allowed fail rate in Endless mode (default 10.0%)
	int endless_window = 50;          // window size for Endless verification (default 50 maps)
	double endless_min_speed = 0.0;   // minimum speed floor for Endless mode (default start_v - 0.20 m/s)
};

struct ScenarioResult {
	bool ok = false;
	std::string error;
	std::string halt_reason;
	FwDriveParams params{};
	std::vector<DriveResult> drives;
	double wall_seconds = 0, sim_seconds = 0;
	std::string track_path_used;   // last course driven (session: may be a generated one)
};

ScenarioResult run_scenario(const ScenarioOptions& opt);
bool load_scenario_file(const std::string& path, ScenarioOptions* opt, std::string* err);
bool apply_param(FwDriveParams* p, const std::string& key, double v);
bool save_drive_params_to_firmware_file(const FwDriveParams& p, std::string* out_path = nullptr);
