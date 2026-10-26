// 3D viewer: a Win32 OpenGL window rendered with MuJoCo's own renderer.
#pragma once
#include <array>
#include <string>
#include <vector>

#include "world/track.h"

struct mjModel_;
struct mjData_;

struct ViewerHud {
	std::string status;                 // top-left block (multi-line)
	std::string help;                   // bottom-left block
	std::string panel;                  // bottom-right block (session: parameters, results)
	std::vector<std::string> lcd;       // firmware LCD rows (top-right)
	bool lcd_inverted = false;
	float sensors[18] = {};             // normalized IR values 0..1 (bottom bar)
	bool sensor_on[18] = {};
	double live_beta = 0.0;
	double live_lat_mm = 0.0;
	std::string marker_hud;             // 3D HUD 마커 분석 및 복원 정보 카드
};

class Viewer {
public:
	enum Key { kNone = 0 };
	enum CamMode { kFollow, kSensor, kTop, kFree, kCamModes };

	~Viewer();
	bool open(int width, int height, const char* title, std::string* err);
	void attach(mjModel_* m, mjData_* d, int follow_body, const Track& track);
	void set_track(const Track& track);   // replace the drawn course (new random map)

	bool pump();                        // handle window messages; false once the window is closed
	void render(const ViewerHud& hud);
	void add_trail(double x, double y, double speed, double beta = 0.0, double lat_mm = 0.0);
	void clear_trail() {
		trail_.clear();
		trail_vmax_ = 1.0;
	}
	// Saves the next rendered frame as a 24-bit BMP.
	void request_screenshot(const std::string& path) { shot_path_ = path; }

	bool key_held(int vk) const;        // VK_* code currently down
	int pop_key();                      // next key press (VK_* code), 0 if none
	void inject_key(int vk) { keys_.push_back(vk); }   // as if pressed (test scripts)
	bool closed() const { return closed_; }
	int cam_mode() const { return cam_mode_; }
	void set_cam_mode(int m);
	void toggle_shadows();
	enum InsetMode { kInsetOff = 0, kInsetMap = 1, kInsetSensor = 2 };
	void cycle_inset() { inset_mode_ = (inset_mode_ + 1) % 3; }
	void toggle_inset() { inset_mode_ = (inset_mode_ == kInsetOff) ? kInsetMap : kInsetOff; }
	int inset_mode() const { return inset_mode_; }
	void set_inset_mode(int m) { inset_mode_ = m; }
	void set_sensor_offset(double x) { sensor_x_ = x; }   // axle -> sensor bar [m], for the sensor view
	// IR sensor spots (robot frame x, y), drawn on the floor; colored by ViewerHud::sensor_on
	void set_sensor_points(const std::vector<std::array<double, 2>>& pts) { sensor_pts_ = pts; }
	void set_fast_mode(bool fast) { fast_mode_ = fast; }
	bool fast_mode() const { return fast_mode_; }

	void set_marker_diags(const std::vector<MarkerVisualInfo>& diags) {
		marker_diags_ = diags;
		focused_miss_idx_ = -1;
		for (size_t i = 0; i < marker_diags_.size(); ++i) {
			if (marker_diags_[i].status >= 2) {
				focused_miss_idx_ = (int)i;
				break;
			}
		}
	}
	void clear_marker_diags() {
		marker_diags_.clear();
		focused_miss_idx_ = -1;
	}
	const std::vector<MarkerVisualInfo>& marker_diags() const { return marker_diags_; }
	int focused_miss_idx() const { return focused_miss_idx_; }
	void jump_to_next_marker_miss();

private:
	static long long __stdcall wndproc(void* hwnd, unsigned msg, unsigned long long wp, long long lp);
	long long handle(unsigned msg, unsigned long long wp, long long lp);
	void mouse_move(int x, int y);
	// mjr_overlay for any length: it keeps at most mjMAXOVERLAY - 1 (499) characters per call
	void overlay(int grid, int left, int bottom, int width, int height, const std::string& text);

	void* hwnd_ = nullptr;
	void* hdc_ = nullptr;
	void* glrc_ = nullptr;
	bool closed_ = false;
	int width_ = 0, height_ = 0;

	mjModel_* m_ = nullptr;
	mjData_* d_ = nullptr;
	int follow_body_ = 0;
	struct Impl;
	Impl* im_ = nullptr;

	std::string shot_path_;
	struct TrailPt { float x, y, v, beta, lat_mm; };
	std::vector<TrailPt> trail_;
	double trail_vmax_ = 1.0;

	bool held_[256] = {};
	std::vector<int> keys_;
	bool btn_l_ = false, btn_r_ = false, btn_m_ = false;
	int mx_ = 0, my_ = 0;
	int cam_mode_ = kFollow;
	double follow_az_ = 0;
	std::vector<std::array<double, 2>> sensor_pts_;
	int inset_mode_ = kInsetMap;   // default to Track Map in corner
	double sensor_x_ = 0.13;
	void sensor_cam(void* cam) const;   // follow camera azimuth relative to the robot heading [deg]
	void map_cam(void* cam) const;      // top-down track map view
	double cx_ = 0, cy_ = 0, extent_ = 2;   // track center / size for the top view
	bool fast_mode_ = false;
	std::vector<MarkerVisualInfo> marker_diags_;
	int focused_miss_idx_ = -1;
};
