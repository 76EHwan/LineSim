#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "world/robot.h"
#include "world/track.h"

struct GpuDriveParams {
	float base_mps = 1.80f;
	float accel = 8.0f;
	float decel = 8.0f;
	float steer_kp = 16.4f;
	float steer_kd = 800.0f;
	float pos_atten_gain = 0.0f;
	float yaw_gain_p = 0.25f;
	float yaw_gain_i = 0.0f;
	float pit_in_m = 0.15f;
	float conv_gain = 0.05f;
};

struct GpuSimResult {
	int track_idx = 0;
	int cand_idx = 0;
	bool finished = false;
	bool line_lost = false;
	bool marker_ok = false;
	float lap_time_s = 0.0f;
	float odom_m = 0.0f;
	float max_lat_mm = 0.0f;
	float max_beta_deg = 0.0f;
	float weave_dps = 0.0f;
	float str_wz_rms = 0.0f;
	float str_lat_rms = 0.0f;
	float fail_v = 0.0f;
	float final_x = 0.0f;
	float final_y = 0.0f;
	int count_L = 0;
	int count_R = 0;
	int count_C = 0;
	int count_STOP = 0;
	int marker_mismatch_cnt = 0;
};

struct GpuBatchStats {
	std::string adapter_name;
	size_t vram_mb = 0;
	double init_ms = 0.0;
	double upload_ms = 0.0;
	double dispatch_ms = 0.0;
	double total_sim_time_s = 0.0;
	int total_runs = 0;
};

class GpuFirstRunSim {
public:
	GpuFirstRunSim();
	~GpuFirstRunSim();

	bool init(std::string* err);
	const std::string& adapter_name() const { return adapter_name_; }
	size_t adapter_vram_mb() const { return vram_mb_; }

	// Runs (tracks.size() * candidates.size()) 1st-run simulations in parallel on the GPU.
	// Tracks can be static or freshly generated random tracks.
	bool run_batch(
		const std::vector<Track>& tracks,
		const RobotParams& rp,
		const std::vector<GpuDriveParams>& candidates,
		double max_time_s,
		uint32_t seed,
		std::vector<GpuSimResult>* out_results,
		GpuBatchStats* out_stats,
		std::string* err);

private:
	struct Impl;
	Impl* im_ = nullptr;
	std::string adapter_name_;
	size_t vram_mb_ = 0;
};

int run_gpu_cli(int argc, char** argv);
