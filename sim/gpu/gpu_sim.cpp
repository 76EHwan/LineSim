#include "gpu/gpu_sim.h"

#include <windows.h>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <numeric>
#include <random>

#include "core/json.h"
#include "world/track_gen.h"

namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct GpuTrackHeader {
	uint32_t shape_offset;
	uint32_t shape_count;
	uint32_t piece_offset;
	uint32_t piece_count;
	uint32_t grid_cell_offset;
	int32_t gnx;
	int32_t gny;
	float gx0;
	float gy0;
	float cell;
	float start_x;
	float start_y;
	float start_th;
	float length_m;
	uint32_t marker_seq_offset;
	uint32_t marker_seq_count;
	int32_t expected_L;
	int32_t expected_R;
	int32_t expected_C;
	uint32_t pad0;
};

struct GpuShape {
	int32_t kind; // 0 = Band, 1 = ArcBand, 2 = Rect
	float ax, ay;
	float ux, uy;
	float len;
	float hl;
	float hw;
	float R;
	float a0;
	float sweep;
	float minx, miny, maxx, maxy;
	float r_in_sq, r_out_sq;
	float pad0, pad1, pad2;
};

struct GpuPiece {
	int32_t arc;
	float s0;
	float len;
	float p0x, p0y;
	float th0;
	float radius;
	int32_t turn;
	float cx, cy;
	float ux, uy;
	float a0;
	float sweep;
	float minx, miny, maxx, maxy;
	float pad0, pad1;
};

struct alignas(16) GpuSimConstants {
	uint32_t num_tracks;
	uint32_t num_candidates;
	uint32_t total_runs;
	uint32_t steps_per_dispatch; // number of 0.5ms control steps per dispatch
	uint32_t delay_slots;        // sensor_delay / 0.0005s (clamped 0..15)
	uint32_t base_seed;
	float dt_ctrl;               // 0.0005s
	float dt_phys;               // 0.0001s

	// Robot physical parameters
	float mass_total;
	float izz_total;
	float com_x;
	float wheel_radius;
	float track_width;
	float wheel_j_side;
	float m_eff_tire;
	float fz_per_wheel_base;

	// Motor parameters
	float kt;
	float r_phase;
	float gear;
	float efficiency;
	float motor_friction;
	float motor_viscous;
	float current_alpha;
	float vmax;
	float spd_kp;
	float spd_ki;
	float iq_limit;
	float mps_to_omega;

	// Tire brush parameters
	float tire_mu;
	float tire_mu_viscous;
	float slip_stiffness;
	float relaxation_length;
	float damping_ratio;
	float sensor_sigma;
	float ir_white;
	float ir_black;

	float ir_noise;
	float gyro_noise_dps;
	float gyro_bias_dps;
	float sensor_x;

	float sensor_xy[18][4];  // .xy = (x, y) in axle frame for channels 0..17
	float wheel_xy[4][4];    // .xy = (wx, wy) in axle frame, .z = side (0 or 1)
};

struct GpuSimState {
	// Pose & velocities
	float x, y, th;
	float vx_com, vy_com, wz;
	float wheel_w_L, wheel_w_R;
	float iq_L, iq_R;
	float spd_integ_L, spd_integ_R;
	float target_omega_L, target_omega_R;
	float spd_hist_L[4];
	float spd_hist_R[4];

	// 4 wheels brush deflection (ux, uy)
	float wheel_ux[4];
	float wheel_uy[4];

	// Delayed pose ring buffer (16 slots at 0.5ms = up to 8ms delay)
	uint32_t hist_head;
	uint32_t step_count;

	// Sensor & convolution state
	float conv_h1[4][4]; // 16 sensors history t-1
	float conv_h2[4][4]; // 16 sensors history t
	uint32_t conv_primed;
	int32_t prev_peak_idx;
	float prev_position;
	uint32_t lost_counter;
	uint32_t force_full_scan;
	uint32_t cross_state;
	uint32_t cross_accum_mask;
	uint32_t cross_event_pending;

	// Steering & ramp state
	float current_base_mps;
	float target_base_mps;
	float filtered_atten;
	float yaw_integ;
	float prev_steer_error;
	float filtered_deriv;
	uint32_t prev_was_cross;
	uint32_t is_braking;
	float decel_active;
	float odom_m;
	float total_odom_m;
	float post_cross_lockout_m;

	// Marker FSM
	uint32_t marker_state;
	uint32_t left_on_count;
	uint32_t right_on_count;
	uint32_t left_src_mask;
	uint32_t right_src_mask;
	uint32_t marker_on_count;
	uint32_t marker_off_count;
	int32_t marker_min_peak;
	int32_t marker_max_peak;
	int32_t last_marker_type;
	uint32_t count_L;
	uint32_t count_R;
	uint32_t count_C;
	uint32_t count_STOP;
	uint32_t marker_seq_idx;
	uint32_t marker_mismatch_cnt;

	// Diagnostics & status
	uint32_t status; // 0 = running, 1 = finished, 2 = line_lost
	float finish_time_s;
	float max_lat_mm;
	float beta_rad;
	float max_beta_deg;
	float gyro_lpf_dps;
	float weave_sq_sum;
	uint32_t weave_cnt;
	float str_wz_sq_sum;
	float str_lat_sq_sum;
	uint32_t str_cnt;
	float recent_v;
	float gyro_bias;
	uint32_t rng_state;
};

static const char* kComputeShaderParts[] = {
R"HLSL(
cbuffer SimConstants : register(b0) {
	uint g_num_tracks;
	uint g_num_candidates;
	uint g_total_runs;
	uint g_steps_per_dispatch;
	uint g_delay_slots;
	uint g_base_seed;
	float g_dt_ctrl;
	float g_dt_phys;

	float g_mass_total;
	float g_izz_total;
	float g_com_x;
	float g_wheel_radius;
	float g_track_width;
	float g_wheel_j_side;
	float g_m_eff_tire;
	float g_fz_per_wheel_base;

	float g_kt;
	float g_r_phase;
	float g_gear;
	float g_efficiency;
	float g_motor_friction;
	float g_motor_viscous;
	float g_current_alpha;
	float g_vmax;
	float g_spd_kp;
	float g_spd_ki;
	float g_iq_limit;
	float g_mps_to_omega;

	float g_tire_mu;
	float g_tire_mu_viscous;
	float g_slip_stiffness;
	float g_relaxation_length;
	float g_damping_ratio;
	float g_sensor_sigma;
	float g_ir_white;
	float g_ir_black;

	float g_ir_noise;
	float g_gyro_noise_dps;
	float g_gyro_bias_dps;
	float g_sensor_x;

	float4 g_sensor_xy[18];
	float4 g_wheel_xy[4];
};
)HLSL",
R"HLSL(
struct GpuTrackHeader {
	uint shape_offset;
	uint shape_count;
	uint piece_offset;
	uint piece_count;
	uint grid_cell_offset;
	int gnx;
	int gny;
	float gx0;
	float gy0;
	float cell;
	float start_x;
	float start_y;
	float start_th;
	float length_m;
	uint marker_seq_offset;
	uint marker_seq_count;
	int expected_L;
	int expected_R;
	int expected_C;
	uint pad0;
};

struct GpuShape {
	int kind;
	float ax, ay;
	float ux, uy;
	float len;
	float hl;
	float hw;
	float R;
	float a0;
	float sweep;
	float minx, miny, maxx, maxy;
	float r_in_sq, r_out_sq;
	float pad0, pad1, pad2;
};

struct GpuPiece {
	int arc;
	float s0;
	float len;
	float p0x, p0y;
	float th0;
	float radius;
	int turn;
	float cx, cy;
	float ux, uy;
	float a0;
	float sweep;
	float minx, miny, maxx, maxy;
	float pad0, pad1;
};

struct GpuDriveParams {
	float base_mps;
	float accel;
	float decel;
	float steer_kp;
	float steer_kd;
	float pos_atten_gain;
	float yaw_gain_p;
	float yaw_gain_i;
	float pit_in_m;
	float conv_gain;
};

struct GpuSimState {
	float x, y, th;
	float vx_com, vy_com, wz;
	float wheel_w_L, wheel_w_R;
	float iq_L, iq_R;
	float spd_integ_L, spd_integ_R;
	float target_omega_L, target_omega_R;
	float4 spd_hist_L;
	float4 spd_hist_R;

	float4 wheel_ux;
	float4 wheel_uy;

	uint hist_head;
	uint step_count;

	float4 conv_h1[4];
	float4 conv_h2[4];
	uint conv_primed;
	int prev_peak_idx;
	float prev_position;
	uint lost_counter;
	uint force_full_scan;
	uint cross_state;
	uint cross_accum_mask;
	uint cross_event_pending;

	float current_base_mps;
	float target_base_mps;
	float filtered_atten;
	float yaw_integ;
	float prev_steer_error;
	float filtered_deriv;
	uint prev_was_cross;
	uint is_braking;
	float decel_active;
	float odom_m;
	float total_odom_m;
	float post_cross_lockout_m;

	uint marker_state;
	uint left_on_count;
	uint right_on_count;
	uint left_src_mask;
	uint right_src_mask;
	uint marker_on_count;
	uint marker_off_count;
	int marker_min_peak;
	int marker_max_peak;
	int last_marker_type;
	uint count_L;
	uint count_R;
	uint count_C;
	uint count_STOP;
	uint marker_seq_idx;
	uint marker_mismatch_cnt;

	uint status;
	float finish_time_s;
	float max_lat_mm;
	float beta_rad;
	float max_beta_deg;
	float gyro_lpf_dps;
	float weave_sq_sum;
	uint weave_cnt;
	float str_wz_sq_sum;
	float str_lat_sq_sum;
	uint str_cnt;
	float recent_v;
	float gyro_bias;
	uint rng_state;
};

StructuredBuffer<GpuTrackHeader> g_tracks : register(t0);
StructuredBuffer<GpuShape> g_shapes : register(t1);
StructuredBuffer<uint2> g_grid_cells : register(t2);
StructuredBuffer<uint> g_grid_indices : register(t3);
StructuredBuffer<GpuPiece> g_pieces : register(t4);
StructuredBuffer<uint> g_expected_markers : register(t5);
StructuredBuffer<GpuDriveParams> g_candidates : register(t6);
RWStructuredBuffer<GpuSimState> g_states : register(u0);
RWStructuredBuffer<float4> g_hist_poses : register(u1);

static const float kPi = 3.14159265f;
static const float kTwoPi = 6.28318530f;
)HLSL",
R"HLSL(
uint pcg_hash(inout uint state) {
	uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
	state = state * 747796405u + 2891336453u;
	return (word >> 22u) ^ word;
}

float rand_uniform(inout uint state) {
	return float(pcg_hash(state) & 0x00FFFFFFu) * (1.0f / 16777216.0f);
}

float rand_normal(inout uint state) {
	float s = 0.0f;
	[unroll]
	for (int i = 0; i < 6; i++)
		s += rand_uniform(state);
	return (s - 3.0f) * 1.41421356f;
}

float wrap_pi(float a) {
	return a - kTwoPi * floor((a + kPi) * (1.0f / kTwoPi));
}

bool shape_contains(GpuShape s, float px, float py) {
	if (px < s.minx || px > s.maxx || py < s.miny || py > s.maxy)
		return false;
	float dx = px - s.ax;
	float dy = py - s.ay;
	if (s.kind == 0) {
		float t = dx * s.ux + dy * s.uy;
		float n = -dx * s.uy + dy * s.ux;
		return (t >= 0.0f && t <= s.len && abs(n) <= s.hw);
	} else if (s.kind == 1) {
		float d2 = dx * dx + dy * dy;
		if (d2 < s.r_in_sq || d2 > s.r_out_sq)
			return false;
		float ang = wrap_pi(atan2(dy, dx) - s.a0);
		if (s.sweep >= 0.0f) {
			if (ang < 0.0f) ang += kTwoPi;
			return ang <= s.sweep;
		} else {
			if (ang > 0.0f) ang -= kTwoPi;
			return ang >= s.sweep;
		}
	} else {
		float t = dx * s.ux + dy * s.uy;
		float n = -dx * s.uy + dy * s.ux;
		return (abs(t) <= s.hl && abs(n) <= s.hw);
	}
}

float sample_coverage(GpuTrackHeader trk, float px, float py, float sigma) {
	int ix = int(floor((px - trk.gx0) / trk.cell));
	int iy = int(floor((py - trk.gy0) / trk.cell));
	if (ix < 0 || iy < 0 || ix >= trk.gnx || iy >= trk.gny)
		return 0.0f;
	uint2 cell = g_grid_cells[trk.grid_cell_offset + uint(iy * trk.gnx + ix)];
	if (cell.y == 0u)
		return 0.0f;

	static const float ox[5] = { 0.0f, -1.1f, 1.1f,  0.0f, 0.0f };
	static const float oy[5] = { 0.0f,  0.0f, 0.0f, -1.1f, 1.1f };
	static const float w[5]  = { 0.44f, 0.14f, 0.14f, 0.14f, 0.14f };

	float cov = 0.0f;
	for (int k = 0; k < 5; k++) {
		float spx = px + ox[k] * sigma;
		float spy = py + oy[k] * sigma;
		for (uint s = 0u; s < cell.y; s++) {
			uint sh_idx = g_grid_indices[cell.x + s];
			if (shape_contains(g_shapes[trk.shape_offset + sh_idx], spx, spy)) {
				cov += w[k];
				break;
			}
		}
	}
	return cov;
}

float get_f4(float4 v, int c) {
	if (c == 0) return v.x;
	if (c == 1) return v.y;
	if (c == 2) return v.z;
	return v.w;
}

void set_f4(inout float4 v, int c, float val) {
	if (c == 0) v.x = val;
	else if (c == 1) v.y = val;
	else if (c == 2) v.z = val;
	else v.w = val;
}

float get_vec16(float4 v[4], int idx) {
	int r = idx >> 2;
	int c = idx & 3;
	if (r == 0) return get_f4(v[0], c);
	if (r == 1) return get_f4(v[1], c);
	if (r == 2) return get_f4(v[2], c);
	return get_f4(v[3], c);
}

void set_vec16(inout float4 v[4], int idx, float val) {
	int r = idx >> 2;
	int c = idx & 3;
	if (r == 0) set_f4(v[0], c, val);
	else if (r == 1) set_f4(v[1], c, val);
	else if (r == 2) set_f4(v[2], c, val);
	else set_f4(v[3], c, val);
}
)HLSL",
R"HLSL(
[numthreads(64, 1, 1)]
void CSMain(uint3 DTid : SV_DispatchThreadID) {
	uint tid = DTid.x;
	if (tid >= g_total_runs)
		return;

	GpuSimState st = g_states[tid];
	if (st.status != 0u)
		return;

	uint track_idx = tid / g_num_candidates;
	uint cand_idx  = tid % g_num_candidates;

	GpuTrackHeader trk = g_tracks[track_idx];
	GpuDriveParams dp  = g_candidates[cand_idx];

	[loop]
	for (uint step = 0u; step < g_steps_per_dispatch; step++) {
		if (st.status != 0u)
			break;

		uint delayed_slot = (st.hist_head + 16u - g_delay_slots) & 15u;
		float4 dpose = g_hist_poses[tid * 16u + delayed_slot];
		float c_dth = cos(dpose.z);
		float s_dth = sin(dpose.z);

		float4 norm_v[4];
		norm_v[0] = float4(0, 0, 0, 0);
		norm_v[1] = float4(0, 0, 0, 0);
		norm_v[2] = float4(0, 0, 0, 0);
		norm_v[3] = float4(0, 0, 0, 0);
		float norm_16 = 0.0f;
		float norm_17 = 0.0f;
		uint state_mask = 0u;
		[loop]
		for (int ch = 0; ch < 18; ch++) {
			float bx = g_sensor_xy[ch].x;
			float by = g_sensor_xy[ch].y;
			float gx = dpose.x + c_dth * bx - s_dth * by;
			float gy = dpose.y + s_dth * bx + c_dth * by;
			float cov = sample_coverage(trk, gx, gy, g_sensor_sigma);
			float raw = g_ir_black + (g_ir_white - g_ir_black) * cov + g_ir_noise * rand_normal(st.rng_state);
			float wmax = g_ir_white + 3.0f * g_ir_noise;
			float bmax = g_ir_black + 3.0f * g_ir_noise;
			float n_f = floor(clamp((raw - bmax) * 255.0f / max(1.0f, wmax - bmax), 0.0f, 255.0f));
			if (ch < 16) set_vec16(norm_v, ch, n_f);
			else if (ch == 16) norm_16 = n_f;
			else norm_17 = n_f;
			if (n_f > 95.0f)
				state_mask |= (1u << uint(ch));
		}

		float4 conv_v[4];
		conv_v[0] = float4(0, 0, 0, 0);
		conv_v[1] = float4(0, 0, 0, 0);
		conv_v[2] = float4(0, 0, 0, 0);
		conv_v[3] = float4(0, 0, 0, 0);
		[loop]
		for (int i = 0; i < 16; i++) {
			float left_n  = (i == 0)  ? norm_16 : get_vec16(norm_v, i - 1);
			float mid_n   = get_vec16(norm_v, i);
			float right_n = (i == 15) ? norm_17 : get_vec16(norm_v, i + 1);
			float cur = -left_n + 2.0f * mid_n - right_n;
			float h0, h1, h2;
			if (st.conv_primed == 0u) {
				h0 = cur; h1 = cur; h2 = cur;
			} else {
				h0 = get_vec16(st.conv_h1, i);
				h1 = get_vec16(st.conv_h2, i);
				h2 = cur;
			}
			set_vec16(st.conv_h1, i, h1);
			set_vec16(st.conv_h2, i, h2);
			set_vec16(conv_v, i, float(int((1.0f * h0 + 2.0f * h1 + 7.0f * h2) * 0.1f)));
		}
		st.conv_primed = 1u;

		uint state16 = state_mask & 0xFFFFu;
		int current_peak_idx = 8;
		bool is_cross_line = false;
		float best_metric = -1e9f;
		float cgain = dp.conv_gain;

		if (st.force_full_scan != 0u) {
			[loop]
			for (int f_i = 0; f_i < 16; f_i++) {
				float metric = get_vec16(norm_v, f_i) + (cgain > 0.0f ? cgain * get_vec16(conv_v, f_i) : 0.0f);
				if (metric > best_metric) {
					best_metric = metric;
					current_peak_idx = f_i;
				}
			}
			st.force_full_scan = 0u;
			st.cross_state = 0u;
			is_cross_line = false;
		} else {
			int search_start = max(0, st.prev_peak_idx - 3);
			int search_end   = min(15, st.prev_peak_idx + 3);
			uint window_len  = uint(search_end - search_start + 1);
			uint window_mask = ((1u << window_len) - 1u) << uint(search_start);

			int anchor = clamp(st.prev_peak_idx, 0, 15);
			if ((state16 & (1u << uint(anchor))) == 0u) {
				int best_dist = 16;
				for (int a_i = search_start; a_i <= search_end; a_i++) {
					if ((state16 & (1u << uint(a_i))) != 0u) {
						int d = abs(a_i - anchor);
						if (d < best_dist) {
							best_dist = d;
							anchor = a_i;
						}
					}
				}
			}

			uint left_marker_bits = 0u;
			if (anchor >= 2) {
				bool has_gap = false;
				for (int l_i = anchor - 1; l_i >= 0; l_i--) {
					if ((state16 & (1u << uint(l_i))) == 0u) has_gap = true;
					else if (has_gap && (l_i < search_start || l_i <= 2)) left_marker_bits |= (1u << uint(l_i));
				}
			}
			uint right_marker_bits = 0u;
			if (anchor <= 13) {
				bool has_gap = false;
				for (int r_i = anchor + 1; r_i < 16; r_i++) {
					if ((state16 & (1u << uint(r_i))) == 0u) has_gap = true;
					else if (has_gap && (r_i > search_end || r_i >= 13)) right_marker_bits |= (1u << uint(r_i));
				}
			}

			uint line_check_state = state16 & ~(left_marker_bits | right_marker_bits);
			uint active_in_window  = countbits(line_check_state & window_mask);
			uint active_out_window = countbits(line_check_state & ~window_mask);
			uint total_active = active_in_window + active_out_window;

			bool left_adj = (search_start >= 1 && (line_check_state & (1u << uint(search_start - 1))) != 0u) ||
			                (search_start >= 2 && (line_check_state & (1u << uint(search_start - 2))) != 0u);
			bool right_adj = (search_end + 1 < 16 && (line_check_state & (1u << uint(search_end + 1))) != 0u) ||
			                 (search_end + 2 < 16 && (line_check_state & (1u << uint(search_end + 2))) != 0u);
			bool cross_cond = (total_active >= 5u) || (active_out_window >= 3u) ||
			                  ((left_adj || right_adj) && (total_active >= 4u || active_in_window >= 4u));

			if (st.cross_state == 0u) {
				if (cross_cond) {
					st.cross_state = 1u;
					is_cross_line = true;
					current_peak_idx = st.prev_peak_idx;
					st.cross_accum_mask = state16;
				} else {
					for (int p_i = search_start; p_i <= search_end; p_i++) {
						float metric = get_vec16(norm_v, p_i) + (cgain > 0.0f ? cgain * get_vec16(conv_v, p_i) : 0.0f);
						if (metric > best_metric) {
							best_metric = metric;
							current_peak_idx = p_i;
						}
					}
					is_cross_line = false;
				}
			} else {
				st.cross_accum_mask |= state16;
				bool markers_active = (state_mask & ((1u << 16u) | (1u << 17u))) != 0u;
				uint raw_out_window = countbits(state16 & ~window_mask);
				if (!markers_active && raw_out_window == 0u && active_in_window <= 3u) {
					st.cross_state = 0u;
					is_cross_line = false;
					if (countbits(st.cross_accum_mask) >= 12u)
						st.cross_event_pending = 1u;
					st.cross_accum_mask = 0u;
					for (int q_i = search_start; q_i <= search_end; q_i++) {
						float metric = get_vec16(norm_v, q_i) + (cgain > 0.0f ? cgain * get_vec16(conv_v, q_i) : 0.0f);
						if (metric > best_metric) {
							best_metric = metric;
							current_peak_idx = q_i;
						}
					}
				} else {
					is_cross_line = true;
					current_peak_idx = st.prev_peak_idx;
				}
			}
		}

		st.prev_peak_idx = current_peak_idx;
		int calc_start = max(0, current_peak_idx - 3);
		int calc_end   = min(15, current_peak_idx + 3);

		uint mark_left = 0u, mark_right = 0u;
		if ((state_mask & (1u << 16u)) != 0u) mark_left  |= 1u;
		if ((state_mask & (1u << 17u)) != 0u) mark_right |= 1u;
		if (!is_cross_line && st.cross_state == 0u) {
			if (current_peak_idx >= 5 && (state16 & 0x0007u) != 0u) mark_left  |= 2u;
			if (current_peak_idx <= 10 && (state16 & 0xE000u) != 0u) mark_right |= 2u;
		}
		if (!is_cross_line) {
			if ((state16 & 0xFFE0u) == 0u) mark_left = 0u;
			if ((state16 & 0x07FFu) == 0u) mark_right = 0u;
		}

		uint min_noise = 255u;
		uint raw_sum   = 0u;
		for (int c_i = calc_start; c_i <= calc_end; c_i++) {
			uint v = uint(get_vec16(norm_v, c_i));
			raw_sum += v;
			if (v < min_noise) min_noise = v;
		}

		float weighted_sum = 0.0f;
		float total_weight = 0.0f;
		float peak_pos_fallback = -1.5f + 0.2f * float(current_peak_idx);
		for (int w_i = calc_start; w_i <= calc_end; w_i++) {
			float pos_i = -1.5f + 0.2f * float(w_i);
			int net_val = int(get_vec16(norm_v, w_i)) - int(min_noise);
			if (net_val > 0) {
				float net_f = float(net_val);
				float cv = get_vec16(conv_v, w_i);
				if (cgain > 0.0f && cv > 0.0f)
					net_f += cgain * cv;
				float w = net_f * net_f;
				weighted_sum += w * pos_i;
				total_weight += w;
			}
		}

		float line_pos = st.prev_position;
		bool is_cross_flag = false;
		if (raw_sum > 20u) {
			st.lost_counter = 0u;
			if (is_cross_line) {
				is_cross_flag = true;
				line_pos = st.prev_position;
			} else {
				float cur_pos = (total_weight > 0.0f) ? (weighted_sum / total_weight) : peak_pos_fallback;
				float diff = clamp(cur_pos - st.prev_position, -0.015f, 0.015f);
				line_pos = st.prev_position + diff;
				st.prev_position = line_pos;
			}
		} else {
			st.force_full_scan = 1u;
			st.lost_counter++;
			if (st.lost_counter > 50u && st.is_braking == 0u) {
				st.status = 2u;
				st.finish_time_s = float(st.step_count) * g_dt_ctrl;
				break;
			}
		}
)HLSL",
R"HLSL(
		uint cross_event = 0u;
		if (st.cross_event_pending != 0u) {
			st.cross_event_pending = 0u;
			st.marker_state = 0u;
			st.left_on_count = 0u;
			st.right_on_count = 0u;
			st.left_src_mask = 0u;
			st.right_src_mask = 0u;
			st.marker_on_count = 0u;
			st.marker_off_count = 0u;
			st.marker_min_peak = 15;
			st.marker_max_peak = 0;
			st.post_cross_lockout_m = 0.15f;
			cross_event = 3u;
		} else if (st.cross_state != 0u) {
			st.marker_state = 0u;
			st.left_on_count = 0u;
			st.right_on_count = 0u;
		} else if (st.post_cross_lockout_m > 0.0f) {
			st.marker_state = 0u;
			if (st.odom_m >= st.post_cross_lockout_m)
				st.post_cross_lockout_m = 0.0f;
		} else {
			if (st.marker_state == 0u) {
				if (mark_left != 0u || mark_right != 0u) {
					st.marker_state = 1u;
					st.left_on_count = mark_left != 0u ? 10u : 0u;
					st.right_on_count = mark_right != 0u ? 10u : 0u;
					st.left_src_mask = mark_left;
					st.right_src_mask = mark_right;
					st.marker_on_count = 10u;
					st.marker_off_count = 0u;
					st.marker_min_peak = st.prev_peak_idx;
					st.marker_max_peak = st.prev_peak_idx;
				}
			} else if (st.marker_state == 1u) {
				if (mark_left != 0u) {
					st.left_on_count += 10u;
					st.left_src_mask |= mark_left;
				}
				if (mark_right != 0u) {
					st.right_on_count += 10u;
					st.right_src_mask |= mark_right;
				}
				st.marker_min_peak = min(st.marker_min_peak, st.prev_peak_idx);
				st.marker_max_peak = max(st.marker_max_peak, st.prev_peak_idx);

				if (mark_left != 0u || mark_right != 0u) {
					st.marker_on_count += 10u;
					st.marker_off_count = 0u;
				} else {
					st.marker_off_count += 10u;
					if (st.marker_off_count >= 60u) {
						bool left_ok  = ((st.left_src_mask & 2u) != 0u) || (st.marker_min_peak <= 6);
						bool right_ok = ((st.right_src_mask & 2u) != 0u) || (st.marker_max_peak >= 9);
						uint min_l = ((st.left_src_mask & 1u) != 0u) ? 25u : 50u;
						uint min_r = ((st.right_src_mask & 1u) != 0u) ? 25u : 50u;
						bool valid_l = (st.left_on_count >= min_l) && left_ok;
						bool valid_r = (st.right_on_count >= min_r) && right_ok;

						if (valid_l && valid_r) {
							cross_event = 4u;
						} else if (valid_l) {
							if (!(st.last_marker_type == 1 && st.odom_m < 0.070f))
								cross_event = 1u;
						} else if (valid_r) {
							if (!(st.last_marker_type == 2 && st.odom_m < 0.070f))
								cross_event = 2u;
						}
						st.marker_state = 2u;
						st.marker_off_count = 0u;
					}
				}
			} else {
				if (mark_left != 0u || mark_right != 0u) {
					st.marker_off_count = 0u;
				} else {
					st.marker_off_count += 10u;
					if (st.marker_off_count >= 20u) {
						st.marker_state = 0u;
						st.left_on_count = 0u;
						st.right_on_count = 0u;
						st.left_src_mask = 0u;
						st.right_src_mask = 0u;
					}
				}
			}
		}

		if (cross_event != 0u && st.is_braking == 0u) {
			st.last_marker_type = int(cross_event);
			st.odom_m = 0.0f;
			if (st.marker_seq_idx < trk.marker_seq_count) {
				uint expected_ev = g_expected_markers[trk.marker_seq_offset + st.marker_seq_idx];
				if (expected_ev != cross_event)
					st.marker_mismatch_cnt++;
			} else {
				st.marker_mismatch_cnt++;
			}
			st.marker_seq_idx++;

			if (cross_event == 1u) st.count_L++;
			else if (cross_event == 2u) st.count_R++;
			else if (cross_event == 3u) st.count_C++;
			else if (cross_event == 4u) {
				st.count_STOP++;
				if (st.count_STOP >= 2u) {
					st.finish_time_s = float(st.step_count) * g_dt_ctrl;
					st.is_braking = 1u;
					st.target_base_mps = 0.0f;
					float pit_d = max(0.02f, dp.pit_in_m);
					st.decel_active = max(dp.decel, (st.current_base_mps * st.current_base_mps) / (2.0f * pit_d));
				}
			}
		}

		float meas_mps_L = abs(st.spd_hist_L.w) / g_mps_to_omega;
		float meas_mps_R = abs(st.spd_hist_R.w) / g_mps_to_omega;
		float ds_odom = 0.5f * (meas_mps_L + meas_mps_R) * g_dt_ctrl;
		st.odom_m += ds_odom;
		st.total_odom_m += ds_odom;

		float d_mps = st.target_base_mps - st.current_base_mps;
		float max_acc_step = dp.accel * g_dt_ctrl;
		float max_dec_step = st.decel_active * g_dt_ctrl;
		if (d_mps > max_acc_step)
			st.current_base_mps += max_acc_step;
		else if (d_mps < -max_dec_step)
			st.current_base_mps -= max_dec_step;
		else
			st.current_base_mps = st.target_base_mps;

		if (st.is_braking != 0u && st.current_base_mps <= 0.005f) {
			st.status = 1u;
			break;
		}

		float error = line_pos;
		float raw_deriv = 0.0f;
		if (is_cross_flag) {
			st.prev_was_cross = 4u;
			raw_deriv = 0.0f;
			st.prev_steer_error = line_pos;
			st.filtered_deriv = 0.0f;
		} else if (st.prev_was_cross > 0u) {
			st.prev_was_cross--;
			raw_deriv = 0.0f;
			st.prev_steer_error = line_pos;
			st.filtered_deriv = 0.0f;
		} else {
			raw_deriv = clamp(line_pos - st.prev_steer_error, -0.010f, 0.010f);
			st.prev_steer_error = line_pos;
		}
		st.filtered_deriv = 0.80f * st.filtered_deriv + 0.20f * raw_deriv;

		float raw_atten = max(0.4f, 1.0f - abs(error) * dp.pos_atten_gain);
		if (raw_atten < st.filtered_atten)
			st.filtered_atten = raw_atten;
		else
			st.filtered_atten = 0.998f * st.filtered_atten + 0.002f * raw_atten;

		float active_mps = st.current_base_mps * st.filtered_atten;

		float gyro_z_dps = st.wz * (180.0f / kPi) + g_gyro_noise_dps * rand_normal(st.rng_state);
		float yaw_meas = -gyro_z_dps * (kPi / 180.0f);

		float alpha = clamp(0.41450f * error, -1.2f, 1.2f);
		float kappa_pp = (2.0f * sin(alpha)) / 0.13213f;
		float err_sq = error * error;
		float corner_weight = err_sq / (err_sq + 0.0144f);
		float rejoin_gain = min(1.30f, 1.0f + 1.85f * err_sq);
		float sign_err = (error >= 0.0f) ? 1.0f : -1.0f;
		float kappa_corner = sign_err * 4.0f * corner_weight * rejoin_gain;

		float v_base = max(0.5f, dp.base_mps);
		float speed_ratio = 1.0f;
		float pp_speed_ratio = 1.0f;
		if (active_mps > v_base) {
			speed_ratio = v_base / active_mps;
			pp_speed_ratio = sqrt(speed_ratio);
		}

		float kp_scale = dp.steer_kp * (1.0f / 20.4f);
		float p_term = (kappa_pp * pp_speed_ratio + kappa_corner * speed_ratio) * kp_scale;
		float high_speed_weight = 1.0f - speed_ratio;
		float kd_scale = 0.65f + 0.35f * corner_weight + 0.60f * high_speed_weight;
		float d_term = (dp.steer_kd * kd_scale) * st.filtered_deriv * speed_ratio;
		float current_steer = p_term + d_term;

		float yaw_ref = active_mps * current_steer;
		float yaw_cmd = yaw_ref;
		if (dp.yaw_gain_p > 0.0f || dp.yaw_gain_i > 0.0f) {
			float yaw_err = yaw_ref - yaw_meas;
			if (active_mps > 0.05f) {
				st.yaw_integ = clamp(st.yaw_integ + dp.yaw_gain_i * yaw_err * g_dt_ctrl, -5.0f, 5.0f);
			} else {
				st.yaw_integ = 0.0f;
			}
			yaw_cmd = yaw_ref + dp.yaw_gain_p * yaw_err / (1.0f + dp.yaw_gain_p) + st.yaw_integ;
		} else {
			st.yaw_integ = 0.0f;
		}

		float thread_div2 = 0.093f;
		float mps_L = active_mps + yaw_cmd * thread_div2;
		float mps_R = active_mps - yaw_cmd * thread_div2;
		st.target_omega_L =  mps_L * g_mps_to_omega;
		st.target_omega_R = -mps_R * g_mps_to_omega;

		float omega_e_raw_L =  st.wheel_w_L * g_gear;
		float omega_e_raw_R = -st.wheel_w_R * g_gear;
		st.spd_hist_L = float4(st.spd_hist_L.yzw, omega_e_raw_L);
		st.spd_hist_R = float4(st.spd_hist_R.yzw, omega_e_raw_R);
		float omega_meas_L = dot(st.spd_hist_L, float4(0.25f, 0.25f, 0.25f, 0.25f));
		float omega_meas_R = dot(st.spd_hist_R, float4(0.25f, 0.25f, 0.25f, 0.25f));

		float err_L = st.target_omega_L - omega_meas_L;
		float err_R = st.target_omega_R - omega_meas_R;
		st.spd_integ_L = clamp(st.spd_integ_L + g_spd_ki * err_L * g_dt_ctrl, -g_iq_limit, g_iq_limit);
		st.spd_integ_R = clamp(st.spd_integ_R + g_spd_ki * err_R * g_dt_ctrl, -g_iq_limit, g_iq_limit);
		float iq_cmd_L = clamp(g_spd_kp * err_L + st.spd_integ_L, -g_iq_limit, g_iq_limit);
		float iq_cmd_R = clamp(g_spd_kp * err_R + st.spd_integ_R, -g_iq_limit, g_iq_limit);

		if (st.is_braking == 0u && st.target_base_mps > 0.05f) {
			float lat_mm = abs(line_pos) * 80.0f;
			st.max_lat_mm = max(st.max_lat_mm, lat_mm);

			if (st.current_base_mps > st.recent_v)
				st.recent_v = st.current_base_mps;
			else
				st.recent_v = 0.9985f * st.recent_v + 0.0015f * st.current_base_mps;

			if (active_mps >= 0.35f) {
				float yaw_diff = yaw_meas - yaw_ref;
				st.beta_rad = 0.9972f * st.beta_rad + yaw_diff * g_dt_ctrl;
				float beta_deg = abs(st.beta_rad) * (180.0f / kPi);
				st.max_beta_deg = max(st.max_beta_deg, beta_deg);

				st.gyro_lpf_dps = 0.983f * st.gyro_lpf_dps + 0.017f * gyro_z_dps;
				float hp_dps = gyro_z_dps - st.gyro_lpf_dps;
				st.weave_sq_sum += hp_dps * hp_dps;
				st.weave_cnt++;

				if (abs(error) < 0.25f && abs(yaw_ref) < 1.2f) {
					float wz_dps = abs(st.wz * (180.0f / kPi));
					st.str_wz_sq_sum += wz_dps * wz_dps;
					st.str_lat_sq_sum += lat_mm * lat_mm;
					st.str_cnt++;
				}
			}
		}

		const float lambda = g_kt / 1.5f;
		const float sigma = max(1e-4f, g_relaxation_length);
		const float k_tire = g_slip_stiffness * g_fz_per_wheel_base / sigma;
		const float c_tire = 2.0f * g_damping_ratio * sqrt(k_tire * g_m_eff_tire);
		const float fmax_coulomb = g_tire_mu * g_fz_per_wheel_base;

		[loop]
		for (int sub = 0; sub < 5; sub++) {
			float w_fw_L =  st.wheel_w_L * g_gear;
			float w_fw_R = -st.wheel_w_R * g_gear;

			float e_L = w_fw_L * lambda;
			float hi_L = (g_vmax - e_L) / g_r_phase;
			float lo_L = (-g_vmax - e_L) / g_r_phase;
			float tgt_iq_L = clamp(iq_cmd_L, min(lo_L, hi_L), max(lo_L, hi_L));
			st.iq_L += (tgt_iq_L - st.iq_L) * g_current_alpha;

			float e_R = w_fw_R * lambda;
			float hi_R = (g_vmax - e_R) / g_r_phase;
			float lo_R = (-g_vmax - e_R) / g_r_phase;
			float tgt_iq_R = clamp(iq_cmd_R, min(lo_R, hi_R), max(lo_R, hi_R));
			st.iq_R += (tgt_iq_R - st.iq_R) * g_current_alpha;

			float t_drv_L =  g_kt * st.iq_L * g_gear * g_efficiency;
			float t_drv_R = -g_kt * st.iq_R * g_gear * g_efficiency;

			float vx_axle = st.vx_com;
			float vy_axle = st.vy_com - st.wz * g_com_x;

			float Fx_tot = 0.0f;
			float Fy_tot = 0.0f;
			float Mz_tot = 0.0f;
			float fx_side_L = 0.0f;
			float fx_side_R = 0.0f;

			[unroll]
			for (int w = 0; w < 4; w++) {
				float wx = g_wheel_xy[w].x;
				float wy = g_wheel_xy[w].y;
				bool is_right = (g_wheel_xy[w].z > 0.5f);
				float ww = is_right ? st.wheel_w_R : st.wheel_w_L;

				float v_cx = vx_axle - st.wz * wy;
				float v_cy = vy_axle + st.wz * wx;
				float vroll = v_cx;
				float vsx = v_cx - g_wheel_radius * ww;
				float vsy = v_cy;

				float ux = get_f4(st.wheel_ux, w);
				float uy = get_f4(st.wheel_uy, w);
				float relax = abs(vroll) / sigma;
				float dux = vsx - relax * ux;
				float duy = vsy - relax * uy;
				ux += dux * g_dt_phys;
				uy += duy * g_dt_phys;

				float fx = -k_tire * ux - c_tire * dux;
				float fy = -k_tire * uy - c_tire * duy;
				float fm = sqrt(fx * fx + fy * fy);
				if (fm > fmax_coulomb) {
					float v_slide = ((fm - fmax_coulomb) / (k_tire + 1e-9f)) * relax;
					float fmax_total = fmax_coulomb + g_tire_mu_viscous * g_fz_per_wheel_base * v_slide;
					float scale = fmax_total / fm;
					fx *= scale;
					fy *= scale;
					ux = -fx * (fmax_coulomb / fmax_total) / k_tire;
					uy = -fy * (fmax_coulomb / fmax_total) / k_tire;
				}
				set_f4(st.wheel_ux, w, ux);
				set_f4(st.wheel_uy, w, uy);

				Fx_tot += fx;
				Fy_tot += fy;
				Mz_tot += (wx - g_com_x) * fy - wy * fx;
				if (is_right) fx_side_R += fx;
				else          fx_side_L += fx;
			}

			float damp_w = g_motor_viscous * g_gear * g_gear;
			float fric_w = g_motor_friction * g_gear;
			float sign_L = clamp(st.wheel_w_L * 10.0f, -1.0f, 1.0f);
			float sign_R = clamp(st.wheel_w_R * 10.0f, -1.0f, 1.0f);
			float net_t_L = t_drv_L - fx_side_L * g_wheel_radius - damp_w * st.wheel_w_L - fric_w * sign_L;
			float net_t_R = t_drv_R - fx_side_R * g_wheel_radius - damp_w * st.wheel_w_R - fric_w * sign_R;
			st.wheel_w_L += (net_t_L / g_wheel_j_side) * g_dt_phys;
			st.wheel_w_R += (net_t_R / g_wheel_j_side) * g_dt_phys;

			float ax_com = (Fx_tot / g_mass_total) + st.wz * st.vy_com;
			float ay_com = (Fy_tot / g_mass_total) - st.wz * st.vx_com;
			float awz    = Mz_tot / g_izz_total;

			st.vx_com += ax_com * g_dt_phys;
			st.vy_com += ay_com * g_dt_phys;
			st.wz     += awz    * g_dt_phys;

			float vx_ax_new = st.vx_com;
			float vy_ax_new = st.vy_com - st.wz * g_com_x;
			float c_th = cos(st.th), s_th = sin(st.th);
			st.x  += (vx_ax_new * c_th - vy_ax_new * s_th) * g_dt_phys;
			st.y  += (vx_ax_new * s_th + vy_ax_new * c_th) * g_dt_phys;
			st.th  = wrap_pi(st.th + st.wz * g_dt_phys);
		}

		st.hist_head = (st.hist_head + 1u) & 15u;
		g_hist_poses[tid * 16u + st.hist_head] = float4(st.x, st.y, st.th, 0.0f);
		st.step_count++;
	}

	g_states[tid] = st;
}
)HLSL"
};

bool create_structured_srv(ID3D11Device* dev, const void* data, size_t elem_size, size_t count,
	ID3D11Buffer** out_buf, ID3D11ShaderResourceView** out_srv, std::string* err) {
	size_t safe_count = std::max<size_t>(1, count);
	std::vector<uint8_t> dummy;
	if (!data || count == 0) {
		dummy.assign(elem_size, 0);
		data = dummy.data();
	}
	D3D11_BUFFER_DESC bd{};
	bd.ByteWidth = static_cast<UINT>(elem_size * safe_count);
	bd.Usage = D3D11_USAGE_DEFAULT;
	bd.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	bd.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
	bd.StructureByteStride = static_cast<UINT>(elem_size);

	D3D11_SUBRESOURCE_DATA init{};
	init.pSysMem = data;
	HRESULT hr = dev->CreateBuffer(&bd, &init, out_buf);
	if (FAILED(hr)) {
		*err = "CreateBuffer (SRV) failed";
		return false;
	}
	D3D11_SHADER_RESOURCE_VIEW_DESC sd{};
	sd.Format = DXGI_FORMAT_UNKNOWN;
	sd.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
	sd.Buffer.FirstElement = 0;
	sd.Buffer.NumElements = static_cast<UINT>(safe_count);
	hr = dev->CreateShaderResourceView(*out_buf, &sd, out_srv);
	if (FAILED(hr)) {
		*err = "CreateShaderResourceView failed";
		return false;
	}
	return true;
}

} // namespace

struct GpuFirstRunSim::Impl {
	ID3D11Device* dev = nullptr;
	ID3D11DeviceContext* ctx = nullptr;
	ID3D11ComputeShader* cs = nullptr;
	ID3D11Buffer* cbuf = nullptr;

	~Impl() {
		if (cbuf) cbuf->Release();
		if (cs) cs->Release();
		if (ctx) ctx->Release();
		if (dev) dev->Release();
	}
};

GpuFirstRunSim::GpuFirstRunSim() : im_(new Impl) {}
GpuFirstRunSim::~GpuFirstRunSim() { delete im_; }

bool GpuFirstRunSim::init(std::string* err) {
	if (im_->dev)
		return true;

	HMODULE hD3D11 = LoadLibraryA("d3d11.dll");
	if (!hD3D11) {
		*err = "LoadLibraryA(d3d11.dll) failed";
		return false;
	}
	auto pfnD3D11CreateDevice = reinterpret_cast<PFN_D3D11_CREATE_DEVICE>(
		GetProcAddress(hD3D11, "D3D11CreateDevice"));
	if (!pfnD3D11CreateDevice) {
		*err = "GetProcAddress(D3D11CreateDevice) failed";
		return false;
	}

	HMODULE hCompiler = LoadLibraryA("d3dcompiler_47.dll");
	if (!hCompiler)
		hCompiler = LoadLibraryA("d3dcompiler_43.dll");
	if (!hCompiler) {
		*err = "LoadLibraryA(d3dcompiler_47.dll) failed";
		return false;
	}
	auto pfnD3DCompile = reinterpret_cast<pD3DCompile>(
		GetProcAddress(hCompiler, "D3DCompile"));
	if (!pfnD3DCompile) {
		*err = "GetProcAddress(D3DCompile) failed";
		return false;
	}

	D3D_FEATURE_LEVEL fl_in = D3D_FEATURE_LEVEL_11_0;
	D3D_FEATURE_LEVEL fl_out{};
	HRESULT hr = pfnD3D11CreateDevice(
		nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
		D3D11_CREATE_DEVICE_SINGLETHREADED,
		&fl_in, 1, D3D11_SDK_VERSION,
		&im_->dev, &fl_out, &im_->ctx);
	if (FAILED(hr)) {
		*err = "D3D11CreateDevice failed (no DirectX 11 hardware GPU)";
		return false;
	}

	IDXGIDevice* dxgi_dev = nullptr;
	if (SUCCEEDED(im_->dev->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void**>(&dxgi_dev)))) {
		IDXGIAdapter* adapter = nullptr;
		if (SUCCEEDED(dxgi_dev->GetAdapter(&adapter))) {
			DXGI_ADAPTER_DESC desc{};
			if (SUCCEEDED(adapter->GetDesc(&desc))) {
				char name[256] = {};
				WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, name, sizeof name, nullptr, nullptr);
				adapter_name_ = name;
				vram_mb_ = (desc.DedicatedVideoMemory + desc.SharedSystemMemory) / (1024 * 1024);
			}
			adapter->Release();
		}
		dxgi_dev->Release();
	}

	std::string hlsl_src;
	for (const char* part : kComputeShaderParts)
		hlsl_src += part;

	// Cache compiled shader bytecode keyed by FNV-1a hash of HLSL source
	uint64_t h_val = 14695981039346656037ull;
	for (unsigned char c : hlsl_src) {
		h_val ^= c;
		h_val *= 1099511628211ull;
	}
	char cache_name[64];
	std::snprintf(cache_name, sizeof cache_name, "build/gpu_cs_%016llx.cso", static_cast<unsigned long long>(h_val));

	std::vector<uint8_t> cso_bytes;
	if (FILE* fp = std::fopen(cache_name, "rb")) {
		std::fseek(fp, 0, SEEK_END);
		long sz = std::ftell(fp);
		std::fseek(fp, 0, SEEK_SET);
		if (sz > 0) {
			cso_bytes.resize(static_cast<size_t>(sz));
			std::fread(cso_bytes.data(), 1, cso_bytes.size(), fp);
		}
		std::fclose(fp);
	}

	if (!cso_bytes.empty()) {
		hr = im_->dev->CreateComputeShader(cso_bytes.data(), cso_bytes.size(), nullptr, &im_->cs);
		if (FAILED(hr))
			cso_bytes.clear();
	}

	if (cso_bytes.empty()) {
		ID3DBlob* code = nullptr;
		ID3DBlob* errs = nullptr;
		hr = pfnD3DCompile(
			hlsl_src.data(), hlsl_src.size(),
			"gpu_first_run.hlsl", nullptr, nullptr, "CSMain", "cs_5_0",
			D3DCOMPILE_OPTIMIZATION_LEVEL0, 0, &code, &errs);
		if (FAILED(hr)) {
			std::string msg = "D3DCompile failed: ";
			if (errs) {
				msg.append(static_cast<const char*>(errs->GetBufferPointer()), errs->GetBufferSize());
				errs->Release();
			}
			*err = msg;
			return false;
		}
		if (errs) errs->Release();

		if (FILE* fp = std::fopen(cache_name, "wb")) {
			std::fwrite(code->GetBufferPointer(), 1, code->GetBufferSize(), fp);
			std::fclose(fp);
		}

		hr = im_->dev->CreateComputeShader(code->GetBufferPointer(), code->GetBufferSize(), nullptr, &im_->cs);
		code->Release();
		if (FAILED(hr)) {
			*err = "CreateComputeShader failed";
			return false;
		}
	}

	D3D11_BUFFER_DESC cbd{};
	cbd.ByteWidth = sizeof(GpuSimConstants);
	cbd.Usage = D3D11_USAGE_DEFAULT;
	cbd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	hr = im_->dev->CreateBuffer(&cbd, nullptr, &im_->cbuf);
	if (FAILED(hr)) {
		*err = "CreateBuffer (ConstantBuffer) failed";
		return false;
	}
	return true;
}

bool GpuFirstRunSim::run_batch(
	const std::vector<Track>& tracks,
	const RobotParams& rp,
	const std::vector<GpuDriveParams>& candidates,
	double max_time_s,
	uint32_t seed,
	std::vector<GpuSimResult>* out_results,
	GpuBatchStats* out_stats,
	std::string* err) {

	auto t_init0 = std::chrono::steady_clock::now();
	if (!init(err))
		return false;
	auto t_init1 = std::chrono::steady_clock::now();

	if (tracks.empty() || candidates.empty()) {
		*err = "tracks and candidates must not be empty";
		return false;
	}

	// 1. Pack all tracks (shapes, spatial grid, centerline pieces, expected marker sequences)
	std::vector<GpuTrackHeader> gpu_tracks;
	std::vector<GpuShape> gpu_shapes;
	std::vector<uint32_t> gpu_grid_cells; // pairs of (offset, count)
	std::vector<uint32_t> gpu_grid_indices;
	std::vector<GpuPiece> gpu_pieces;
	std::vector<uint32_t> gpu_expected_markers;

	gpu_tracks.reserve(tracks.size());
	for (const Track& trk : tracks) {
		GpuTrackHeader th{};
		th.shape_offset = static_cast<uint32_t>(gpu_shapes.size());
		th.shape_count  = static_cast<uint32_t>(trk.shapes().size());
		th.piece_offset = static_cast<uint32_t>(gpu_pieces.size());
		th.piece_count  = static_cast<uint32_t>(trk.pieces().size());
		th.length_m     = static_cast<float>(trk.length());

		Pose2 sp = trk.pose_at(trk.has_robot_start() ? trk.robot_start_s() : trk.start_s() - rp.sensor_x - 0.08);
		th.start_x  = static_cast<float>(sp.x);
		th.start_y  = static_cast<float>(sp.y);
		th.start_th = static_cast<float>(sp.th);

		double x0, y0, x1, y1;
		trk.bounds(&x0, &y0, &x1, &y1);
		const double cell = 0.05;
		const double gx0 = x0 - cell;
		const double gy0 = y0 - cell;
		const int gnx = static_cast<int>((x1 - gx0) / cell) + 2;
		const int gny = static_cast<int>((y1 - gy0) / cell) + 2;

		th.grid_cell_offset = static_cast<uint32_t>(gpu_grid_cells.size() / 2);
		th.gnx  = gnx;
		th.gny  = gny;
		th.gx0  = static_cast<float>(gx0);
		th.gy0  = static_cast<float>(gy0);
		th.cell = static_cast<float>(cell);

		std::vector<std::vector<uint32_t>> grid(static_cast<size_t>(gnx) * gny);
		const double margin = 0.020;
		for (size_t k = 0; k < trk.shapes().size(); k++) {
			const Shape& s = trk.shapes()[k];
			GpuShape gs{};
			gs.kind = static_cast<int32_t>(s.kind);
			gs.ax = static_cast<float>(s.a.x);
			gs.ay = static_cast<float>(s.a.y);
			gs.ux = static_cast<float>(s.u.x);
			gs.uy = static_cast<float>(s.u.y);
			gs.len = static_cast<float>(s.len);
			gs.hl = static_cast<float>(s.hl);
			gs.hw = static_cast<float>(s.hw);
			gs.R = static_cast<float>(s.R);
			gs.a0 = static_cast<float>(s.a0);
			gs.sweep = static_cast<float>(s.sweep);
			gs.minx = static_cast<float>(s.minx);
			gs.miny = static_cast<float>(s.miny);
			gs.maxx = static_cast<float>(s.maxx);
			gs.maxy = static_cast<float>(s.maxy);
			gs.r_in_sq = static_cast<float>(s.r_in_sq);
			gs.r_out_sq = static_cast<float>(s.r_out_sq);
			gpu_shapes.push_back(gs);

			int ix0 = std::max(0, static_cast<int>((s.minx - margin - gx0) / cell));
			int ix1 = std::min(gnx - 1, static_cast<int>((s.maxx + margin - gx0) / cell));
			int iy0 = std::max(0, static_cast<int>((s.miny - margin - gy0) / cell));
			int iy1 = std::min(gny - 1, static_cast<int>((s.maxy + margin - gy0) / cell));
			for (int ix = ix0; ix <= ix1; ix++)
				for (int iy = iy0; iy <= iy1; iy++)
					grid[static_cast<size_t>(iy) * gnx + ix].push_back(static_cast<uint32_t>(k));
		}

		for (const auto& cell_list : grid) {
			gpu_grid_cells.push_back(static_cast<uint32_t>(gpu_grid_indices.size()));
			gpu_grid_cells.push_back(static_cast<uint32_t>(cell_list.size()));
			gpu_grid_indices.insert(gpu_grid_indices.end(), cell_list.begin(), cell_list.end());
		}

		for (const PathPiece& p : trk.pieces()) {
			GpuPiece gp{};
			gp.arc = p.arc ? 1 : 0;
			gp.s0 = static_cast<float>(p.s0);
			gp.len = static_cast<float>(p.len);
			gp.p0x = static_cast<float>(p.p0.x);
			gp.p0y = static_cast<float>(p.p0.y);
			gp.th0 = static_cast<float>(p.th0);
			gp.radius = static_cast<float>(p.radius);
			gp.turn = p.turn;
			gp.cx = static_cast<float>(p.center.x);
			gp.cy = static_cast<float>(p.center.y);
			gp.ux = static_cast<float>(p.ux);
			gp.uy = static_cast<float>(p.uy);
			gp.a0 = static_cast<float>(p.a0);
			gp.sweep = static_cast<float>(p.sweep);
			gp.minx = static_cast<float>(p.minx);
			gp.miny = static_cast<float>(p.miny);
			gp.maxx = static_cast<float>(p.maxx);
			gp.maxy = static_cast<float>(p.maxy);
			gpu_pieces.push_back(gp);
		}

		std::string exp_str = trk.expected_markers();
		th.marker_seq_offset = static_cast<uint32_t>(gpu_expected_markers.size());
		th.marker_seq_count  = static_cast<uint32_t>(exp_str.size());
		for (char c : exp_str) {
			if (c == 'L') { gpu_expected_markers.push_back(1u); th.expected_L++; }
			else if (c == 'R') { gpu_expected_markers.push_back(2u); th.expected_R++; }
			else if (c == 'C') { gpu_expected_markers.push_back(3u); th.expected_C++; }
			else if (c == 'U') { gpu_expected_markers.push_back(4u); }
		}
		gpu_tracks.push_back(th);
	}

	const uint32_t num_tracks = static_cast<uint32_t>(tracks.size());
	const uint32_t num_cands  = static_cast<uint32_t>(candidates.size());
	const uint32_t total_runs = num_tracks * num_cands;

	// 2. Initialize GpuSimConstants
	GpuSimConstants cb{};
	cb.num_tracks = num_tracks;
	cb.num_candidates = num_cands;
	cb.total_runs = total_runs;
	cb.steps_per_dispatch = 100; // 50ms of virtual time per dispatch (avoids Windows 2s TDR)
	cb.delay_slots = static_cast<uint32_t>(std::clamp<int>(static_cast<int>(std::lround(rp.sensor_delay / 0.0005)), 0, 15));
	cb.base_seed = seed;
	cb.dt_ctrl = 0.0005f;
	cb.dt_phys = 0.0001f;

	const int num_wheels = rp.wheels.size() >= 4 ? 4 : 2;
	cb.mass_total = static_cast<float>(rp.mass + num_wheels * rp.wheel_mass);
	cb.com_x = static_cast<float>(rp.com[0]);
	cb.wheel_radius = static_cast<float>(rp.wheel_radius);
	cb.track_width = static_cast<float>(rp.track);
	double iw_spin = 0.5 * rp.wheel_mass * rp.wheel_radius * rp.wheel_radius;
	double iw_side = rp.wheel_mass * (3 * rp.wheel_radius * rp.wheel_radius + rp.wheel_width * rp.wheel_width) / 12.0;
	double izz_wheels = 0.0;
	if (rp.wheels.size() >= 4) {
		for (int i = 0; i < 4; i++) {
			double wx = rp.wheels[i][0] - rp.com[0];
			double wy = rp.wheels[i][1];
			izz_wheels += iw_side + rp.wheel_mass * (wx * wx + wy * wy);
			cb.wheel_xy[i][0] = static_cast<float>(rp.wheels[i][0]);
			cb.wheel_xy[i][1] = static_cast<float>(rp.wheels[i][1]);
			cb.wheel_xy[i][2] = rp.wheels[i][1] < 0 ? 1.0f : 0.0f;
			cb.wheel_xy[i][3] = 0.0f;
		}
		cb.wheel_j_side = static_cast<float>(2.0 * iw_spin + rp.motor.j_rotor * rp.motor.gear * rp.motor.gear);
	} else {
		double wy = 0.5 * rp.track;
		izz_wheels = 2.0 * (iw_side + rp.wheel_mass * (rp.com[0] * rp.com[0] + wy * wy));
		cb.wheel_xy[0][0] = 0.0f; cb.wheel_xy[0][1] = static_cast<float>(wy);  cb.wheel_xy[0][2] = 0.0f;
		cb.wheel_xy[1][0] = 0.0f; cb.wheel_xy[1][1] = static_cast<float>(wy);  cb.wheel_xy[1][2] = 0.0f;
		cb.wheel_xy[2][0] = 0.0f; cb.wheel_xy[2][1] = static_cast<float>(-wy); cb.wheel_xy[2][2] = 1.0f;
		cb.wheel_xy[3][0] = 0.0f; cb.wheel_xy[3][1] = static_cast<float>(-wy); cb.wheel_xy[3][2] = 1.0f;
		cb.wheel_j_side = static_cast<float>(iw_spin + rp.motor.j_rotor * rp.motor.gear * rp.motor.gear);
	}
	cb.izz_total = static_cast<float>(rp.inertia[2] + izz_wheels);
	cb.m_eff_tire = static_cast<float>(rp.wheel_mass + rp.motor.j_rotor * rp.motor.gear * rp.motor.gear / (rp.wheel_radius * rp.wheel_radius));
	cb.fz_per_wheel_base = static_cast<float>((cb.mass_total * 9.81) / 4.0);

	cb.kt = static_cast<float>(rp.motor.kt);
	cb.r_phase = static_cast<float>(rp.motor.r_phase);
	cb.gear = static_cast<float>(rp.motor.gear);
	cb.efficiency = static_cast<float>(rp.motor.efficiency);
	cb.motor_friction = static_cast<float>(rp.motor.friction);
	cb.motor_viscous = static_cast<float>(rp.motor.viscous);
	cb.current_alpha = static_cast<float>(1.0 - std::exp(-0.0001 / rp.motor.current_tau));
	cb.vmax = static_cast<float>(rp.motor.voltage_ratio * rp.vbat);

	// Firmware speed PI constants (foc.h)
	const float w_s = (2000.0f / 20.0f) * 2.0f * static_cast<float>(kPi);
	cb.spd_kp = (0.812e-7f * w_s) / 0.00646f;
	cb.spd_ki = cb.spd_kp / 0.00374f;
	cb.iq_limit = 5.0f;
	cb.mps_to_omega = (2.0f / 0.023f) * (39.0f / 11.0f);

	cb.tire_mu = static_cast<float>(rp.tire.mu);
	cb.tire_mu_viscous = static_cast<float>(rp.tire.mu_viscous);
	cb.slip_stiffness = static_cast<float>(rp.tire.slip_stiffness);
	cb.relaxation_length = static_cast<float>(rp.tire.relaxation_length);
	cb.damping_ratio = static_cast<float>(rp.tire.damping_ratio);
	cb.sensor_sigma = static_cast<float>(rp.sensor_sigma);
	cb.ir_white = static_cast<float>(rp.ir_white);
	cb.ir_black = static_cast<float>(rp.ir_black);
	cb.ir_noise = static_cast<float>(rp.ir_noise);
	cb.gyro_noise_dps = static_cast<float>(rp.gyro_noise_dps);
	cb.gyro_bias_dps = static_cast<float>(rp.gyro_bias_dps);
	cb.sensor_x = static_cast<float>(rp.sensor_x);

	for (int i = 0; i < 16; i++) {
		if (rp.ir_sensors.size() >= 16) {
			cb.sensor_xy[i][0] = static_cast<float>(rp.ir_sensors[i][0]);
			cb.sensor_xy[i][1] = static_cast<float>(rp.ir_sensors[i][1]);
		} else {
			cb.sensor_xy[i][0] = static_cast<float>(rp.sensor_x);
			cb.sensor_xy[i][1] = static_cast<float>(-(-1.0 + 2.0 * i / 15.0) * rp.sensor_half_width);
		}
	}
	float mark_sx = rp.ir_sensors.size() >= 16 ? static_cast<float>(rp.ir_sensors[0][0]) : static_cast<float>(rp.sensor_x);
	cb.sensor_xy[16][0] = mark_sx;
	cb.sensor_xy[16][1] = static_cast<float>(rp.marker_sensor_y);
	cb.sensor_xy[17][0] = mark_sx;
	cb.sensor_xy[17][1] = static_cast<float>(-rp.marker_sensor_y);

	// 3. Initialize GpuSimState for all (track, candidate) pairs
	std::vector<GpuSimState> init_states(total_runs);
	std::vector<float> init_hist_poses(static_cast<size_t>(total_runs) * 16 * 4, 0.0f);
	for (uint32_t tid = 0; tid < total_runs; tid++) {
		uint32_t t_idx = tid / num_cands;
		uint32_t c_idx = tid % num_cands;
		const GpuTrackHeader& th = gpu_tracks[t_idx];
		const GpuDriveParams& dp = candidates[c_idx];

		GpuSimState& st = init_states[tid];
		std::memset(&st, 0, sizeof(GpuSimState));
		st.x = th.start_x;
		st.y = th.start_y;
		st.th = th.start_th;
		for (int h = 0; h < 16; h++) {
			size_t base = (static_cast<size_t>(tid) * 16 + h) * 4;
			init_hist_poses[base + 0] = st.x;
			init_hist_poses[base + 1] = st.y;
			init_hist_poses[base + 2] = st.th;
			init_hist_poses[base + 3] = 0.0f;
		}
		st.prev_peak_idx = 8;
		st.force_full_scan = 1u;
		st.target_base_mps = dp.base_mps;
		st.filtered_atten = 1.0f;
		st.decel_active = dp.decel;
		st.marker_min_peak = 15;
		st.marker_max_peak = 0;
		st.recent_v = dp.base_mps;
		st.rng_state = seed + tid * 747796405u + 1013904223u;
	}

	// 4. Upload buffers to GPU
	ID3D11Buffer *b_tracks = nullptr, *b_shapes = nullptr, *b_cells = nullptr, *b_indices = nullptr, *b_pieces = nullptr, *b_markers = nullptr, *b_cands = nullptr;
	ID3D11ShaderResourceView *s_tracks = nullptr, *s_shapes = nullptr, *s_cells = nullptr, *s_indices = nullptr, *s_pieces = nullptr, *s_markers = nullptr, *s_cands = nullptr;
	ID3D11Buffer *b_states = nullptr, *b_hist = nullptr, *b_readback = nullptr;
	ID3D11UnorderedAccessView *u_states = nullptr, *u_hist = nullptr;

	auto cleanup = [&]() {
		if (u_hist) u_hist->Release();
		if (u_states) u_states->Release();
		if (b_readback) b_readback->Release();
		if (b_hist) b_hist->Release();
		if (b_states) b_states->Release();
		for (auto* s : {s_tracks, s_shapes, s_cells, s_indices, s_pieces, s_markers, s_cands}) if (s) s->Release();
		for (auto* b : {b_tracks, b_shapes, b_cells, b_indices, b_pieces, b_markers, b_cands}) if (b) b->Release();
	};

	if (!create_structured_srv(im_->dev, gpu_tracks.data(), sizeof(GpuTrackHeader), gpu_tracks.size(), &b_tracks, &s_tracks, err) ||
	    !create_structured_srv(im_->dev, gpu_shapes.data(), sizeof(GpuShape), gpu_shapes.size(), &b_shapes, &s_shapes, err) ||
	    !create_structured_srv(im_->dev, gpu_grid_cells.data(), sizeof(uint32_t) * 2, gpu_grid_cells.size() / 2, &b_cells, &s_cells, err) ||
	    !create_structured_srv(im_->dev, gpu_grid_indices.data(), sizeof(uint32_t), gpu_grid_indices.size(), &b_indices, &s_indices, err) ||
	    !create_structured_srv(im_->dev, gpu_pieces.data(), sizeof(GpuPiece), gpu_pieces.size(), &b_pieces, &s_pieces, err) ||
	    !create_structured_srv(im_->dev, gpu_expected_markers.data(), sizeof(uint32_t), gpu_expected_markers.size(), &b_markers, &s_markers, err) ||
	    !create_structured_srv(im_->dev, candidates.data(), sizeof(GpuDriveParams), candidates.size(), &b_cands, &s_cands, err)) {
		cleanup();
		return false;
	}

	D3D11_BUFFER_DESC ubd{};
	ubd.ByteWidth = static_cast<UINT>(sizeof(GpuSimState) * total_runs);
	ubd.Usage = D3D11_USAGE_DEFAULT;
	ubd.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
	ubd.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
	ubd.StructureByteStride = sizeof(GpuSimState);
	D3D11_SUBRESOURCE_DATA uinit{init_states.data(), 0, 0};
	if (FAILED(im_->dev->CreateBuffer(&ubd, &uinit, &b_states))) {
		*err = "CreateBuffer (UAV states) failed";
		cleanup();
		return false;
	}

	D3D11_UNORDERED_ACCESS_VIEW_DESC uavd{};
	uavd.Format = DXGI_FORMAT_UNKNOWN;
	uavd.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
	uavd.Buffer.FirstElement = 0;
	uavd.Buffer.NumElements = total_runs;
	if (FAILED(im_->dev->CreateUnorderedAccessView(b_states, &uavd, &u_states))) {
		*err = "CreateUnorderedAccessView failed";
		cleanup();
		return false;
	}

	D3D11_BUFFER_DESC hbd{};
	hbd.ByteWidth = static_cast<UINT>(sizeof(float) * 4 * 16 * total_runs);
	hbd.Usage = D3D11_USAGE_DEFAULT;
	hbd.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
	hbd.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
	hbd.StructureByteStride = sizeof(float) * 4;
	D3D11_SUBRESOURCE_DATA hinit{init_hist_poses.data(), 0, 0};
	if (FAILED(im_->dev->CreateBuffer(&hbd, &hinit, &b_hist))) {
		*err = "CreateBuffer (UAV hist_poses) failed";
		cleanup();
		return false;
	}
	D3D11_UNORDERED_ACCESS_VIEW_DESC huavd{};
	huavd.Format = DXGI_FORMAT_UNKNOWN;
	huavd.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
	huavd.Buffer.FirstElement = 0;
	huavd.Buffer.NumElements = total_runs * 16;
	if (FAILED(im_->dev->CreateUnorderedAccessView(b_hist, &huavd, &u_hist))) {
		*err = "CreateUnorderedAccessView (hist_poses) failed";
		cleanup();
		return false;
	}

	D3D11_BUFFER_DESC rbd{};
	rbd.ByteWidth = ubd.ByteWidth;
	rbd.Usage = D3D11_USAGE_STAGING;
	rbd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
	if (FAILED(im_->dev->CreateBuffer(&rbd, nullptr, &b_readback))) {
		*err = "CreateBuffer (Staging) failed";
		cleanup();
		return false;
	}

	im_->ctx->UpdateSubresource(im_->cbuf, 0, nullptr, &cb, 0, 0);
	auto t_upload1 = std::chrono::steady_clock::now();

	// 5. Bind & Dispatch in 1.0s slices until max_time_s or all finished
	im_->ctx->CSSetShader(im_->cs, nullptr, 0);
	im_->ctx->CSSetConstantBuffers(0, 1, &im_->cbuf);
	ID3D11ShaderResourceView* srvs[7] = {s_tracks, s_shapes, s_cells, s_indices, s_pieces, s_markers, s_cands};
	im_->ctx->CSSetShaderResources(0, 7, srvs);
	ID3D11UnorderedAccessView* uavs[2] = {u_states, u_hist};
	im_->ctx->CSSetUnorderedAccessViews(0, 2, uavs, nullptr);

	const uint32_t total_steps = static_cast<uint32_t>(std::ceil(max_time_s / 0.0005));
	const uint32_t num_dispatches = (total_steps + cb.steps_per_dispatch - 1) / cb.steps_per_dispatch;
	const uint32_t groups_x = (total_runs + 63) / 64;

	for (uint32_t d = 0; d < num_dispatches; d++) {
		im_->ctx->Dispatch(groups_x, 1, 1);
		if ((d + 1) % 8 == 0)
			im_->ctx->Flush();
		// Periodically check if all threads finished early (every 40 dispatches = 2.0s simulated)
		if ((d + 1) % 40 == 0 || d + 1 == num_dispatches) {
			im_->ctx->CopyResource(b_readback, b_states);
			D3D11_MAPPED_SUBRESOURCE mapped{};
			if (SUCCEEDED(im_->ctx->Map(b_readback, 0, D3D11_MAP_READ, 0, &mapped))) {
				const auto* st_ptr = static_cast<const GpuSimState*>(mapped.pData);
				bool any_running = false;
				for (uint32_t k = 0; k < total_runs; k++) {
					if (st_ptr[k].status == 0u) {
						any_running = true;
						break;
					}
				}
				im_->ctx->Unmap(b_readback, 0);
				if (!any_running)
					break;
			}
		}
	}

	// Final readback
	im_->ctx->CopyResource(b_readback, b_states);
	D3D11_MAPPED_SUBRESOURCE mapped{};
	if (FAILED(im_->ctx->Map(b_readback, 0, D3D11_MAP_READ, 0, &mapped))) {
		*err = "Map (Readback) failed";
		cleanup();
		return false;
	}
	const auto* final_states = static_cast<const GpuSimState*>(mapped.pData);
	auto t_dispatch1 = std::chrono::steady_clock::now();

	out_results->resize(total_runs);
	double sum_sim_s = 0.0;
	for (uint32_t tid = 0; tid < total_runs; tid++) {
		const GpuSimState& st = final_states[tid];
		const GpuTrackHeader& th = gpu_tracks[tid / num_cands];
		GpuSimResult& r = (*out_results)[tid];
		r.track_idx = static_cast<int>(tid / num_cands);
		r.cand_idx  = static_cast<int>(tid % num_cands);
		r.finished  = (st.status == 1u);
		r.line_lost = (st.status == 2u);
		r.lap_time_s = (st.finish_time_s > 0.0f) ? st.finish_time_s : (st.step_count * 0.0005f);
		r.odom_m = st.total_odom_m;
		r.max_lat_mm = st.max_lat_mm;
		r.max_beta_deg = st.max_beta_deg;
		r.weave_dps = (st.weave_cnt > 100u) ? std::sqrt(st.weave_sq_sum / static_cast<float>(st.weave_cnt)) : 0.0f;
		r.str_wz_rms = (st.str_cnt > 20u) ? std::sqrt(st.str_wz_sq_sum / static_cast<float>(st.str_cnt)) : 0.0f;
		r.str_lat_rms = (st.str_cnt > 20u) ? std::sqrt(st.str_lat_sq_sum / static_cast<float>(st.str_cnt)) : 0.0f;
		r.fail_v = st.recent_v;
		r.final_x = st.x;
		r.final_y = st.y;
		r.count_L = static_cast<int>(st.count_L);
		r.count_R = static_cast<int>(st.count_R);
		r.count_C = static_cast<int>(st.count_C);
		r.count_STOP = static_cast<int>(st.count_STOP);
		r.marker_mismatch_cnt = static_cast<int>(st.marker_mismatch_cnt);
		r.marker_ok = r.finished &&
		              (r.marker_mismatch_cnt == 0) &&
		              (r.count_L == th.expected_L) &&
		              (r.count_R == th.expected_R) &&
		              (r.count_C == th.expected_C) &&
		              (r.count_STOP == 2);
		sum_sim_s += st.step_count * 0.0005;
	}
	im_->ctx->Unmap(b_readback, 0);

	ID3D11UnorderedAccessView* null_uav[2] = {nullptr, nullptr};
	ID3D11ShaderResourceView* null_srv[7] = {};
	im_->ctx->CSSetUnorderedAccessViews(0, 2, null_uav, nullptr);
	im_->ctx->CSSetShaderResources(0, 7, null_srv);
	cleanup();

	if (out_stats) {
		out_stats->adapter_name = adapter_name_;
		out_stats->vram_mb = vram_mb_;
		out_stats->init_ms = std::chrono::duration<double, std::milli>(t_init1 - t_init0).count();
		out_stats->upload_ms = std::chrono::duration<double, std::milli>(t_upload1 - t_init1).count();
		out_stats->dispatch_ms = std::chrono::duration<double, std::milli>(t_dispatch1 - t_upload1).count();
		out_stats->total_sim_time_s = sum_sim_s;
		out_stats->total_runs = static_cast<int>(total_runs);
	}
	return true;
}

int run_gpu_cli(int argc, char** argv) {
	std::string track_path = "tracks/finals_1_5x_championship.json";
	std::string robot_path;
	int num_candidates = 64;
	int random_maps = 0;
	bool complex_random = true;
	double max_time_s = 0.0; // 0 = auto-compute from track length
	uint32_t seed = 42;

	GpuDriveParams base_dp{};
	double v_min = 0.0, v_max = 0.0;
	double kp_min = 0.0, kp_max = 0.0;
	double kd_min = 0.0, kd_max = 0.0;
	double atten_min = -1.0, atten_max = -1.0;

	for (int i = 2; i < argc; i++) {
		std::string a = argv[i];
		auto next = [&]() -> std::string {
			if (i + 1 >= argc) {
				std::fprintf(stderr, "missing value for %s\n", a.c_str());
				std::exit(2);
			}
			return argv[++i];
		};
		if (a == "--track") track_path = next();
		else if (a == "--robot") robot_path = next();
		else if (a == "--candidates" || a == "--trials" || a == "-n") num_candidates = std::max(1, std::atoi(next().c_str()));
		else if (a == "--random-maps" || a == "--maps") random_maps = std::max(0, std::atoi(next().c_str()));
		else if (a == "--max-time") max_time_s = std::atof(next().c_str());
		else if (a == "--seed") seed = static_cast<uint32_t>(std::strtoul(next().c_str(), nullptr, 10));
		else if (a == "--sweep-v") std::sscanf(next().c_str(), "%lf,%lf", &v_min, &v_max);
		else if (a == "--sweep-kp") std::sscanf(next().c_str(), "%lf,%lf", &kp_min, &kp_max);
		else if (a == "--sweep-kd") std::sscanf(next().c_str(), "%lf,%lf", &kd_min, &kd_max);
		else if (a == "--sweep-atten") std::sscanf(next().c_str(), "%lf,%lf", &atten_min, &atten_max);
		else if (a == "--set") {
			std::string kv = next();
			size_t eq = kv.find('=');
			if (eq != std::string::npos) {
				std::string k = kv.substr(0, eq);
				float v = static_cast<float>(std::atof(kv.c_str() + eq + 1));
				if (k == "base_mps") base_dp.base_mps = v;
				else if (k == "accel") base_dp.accel = v;
				else if (k == "decel") base_dp.decel = v;
				else if (k == "steer_kp") base_dp.steer_kp = v;
				else if (k == "steer_kd") base_dp.steer_kd = v;
				else if (k == "pos_atten_gain") base_dp.pos_atten_gain = v;
				else if (k == "yaw_gain_p") base_dp.yaw_gain_p = v;
				else if (k == "yaw_gain_i") base_dp.yaw_gain_i = v;
				else if (k == "pit_in_m") base_dp.pit_in_m = v;
				else if (k == "conv_gain") base_dp.conv_gain = v;
			}
		}
	}

	if (robot_path.empty()) {
		robot_path = fs::exists("robots/full.json") ? "robots/full.json" : "robots/default.json";
	}
	RobotParams rp;
	std::string err;
	{
		json::Value j;
		if (!json::parse_file(robot_path, &j, &err)) {
			std::fprintf(stderr, "robot load error: %s\n", err.c_str());
			return 1;
		}
		rp.load(j, fs::absolute(robot_path).parent_path().generic_string());
	}

	std::vector<Track> tracks;
	std::vector<std::string> track_labels;
	if (!track_path.empty() && track_path != "none" && track_path != "random") {
		Track t;
		if (!t.load(track_path, &err)) {
			std::fprintf(stderr, "track load error: %s\n", err.c_str());
			return 1;
		}
		tracks.push_back(std::move(t));
		track_labels.push_back(track_path);
	} else if (track_path == "random" && random_maps == 0) {
		random_maps = 4;
	}

	auto t_gen0 = std::chrono::steady_clock::now();
	for (int m = 0; m < random_maps; m++) {
		TrackGenOptions g;
		g.seed = seed + 1000u + static_cast<unsigned>(m) * 37u;
		g.complex_mode = complex_random;
		g.board_w = 9.15;
		g.board_h = 7.32;
		g.min_length = 45.0;
		g.max_length = 75.0;
		g.min_crossings = 1;
		std::string g_err;
		std::string json_txt = generate_track(g, &g_err);
		if (!json_txt.empty()) {
			Track rt;
			std::string lbl = "random_" + std::to_string(g.seed);
			if (rt.load_text(json_txt, lbl, &g_err)) {
				track_labels.push_back(lbl + " (" + std::to_string(static_cast<int>(rt.length())) + "m)");
				tracks.push_back(std::move(rt));
			}
		}
	}
	auto t_gen1 = std::chrono::steady_clock::now();
	double map_gen_ms = std::chrono::duration<double, std::milli>(t_gen1 - t_gen0).count();

	if (max_time_s <= 0.0) {
		double max_len = 35.0;
		for (const Track& t : tracks)
			max_len = std::max(max_len, t.length());
		max_time_s = std::max(30.0, max_len / 1.45 + 4.0);
	}

	// Build candidate list (Candidate 0 is always the exact base_dp; 1..N-1 sweep around it)
	std::vector<GpuDriveParams> candidates(num_candidates, base_dp);
	if (num_candidates > 1) {
		double v_lo  = (v_max > v_min)   ? v_min   : std::max(1.2, base_dp.base_mps - 0.30);
		double v_hi  = (v_max > v_min)   ? v_max   : (base_dp.base_mps + 0.60);
		double kp_lo = (kp_max > kp_min) ? kp_min  : std::max(8.0, base_dp.steer_kp * 0.65);
		double kp_hi = (kp_max > kp_min) ? kp_max  : (base_dp.steer_kp * 1.45);
		double kd_lo = (kd_max > kd_min) ? kd_min  : std::max(200.0, base_dp.steer_kd * 0.50);
		double kd_hi = (kd_max > kd_min) ? kd_max  : (base_dp.steer_kd * 1.60);
		double at_lo = (atten_max >= 0.0 && atten_max >= atten_min) ? atten_min : 0.0;
		double at_hi = (atten_max >= 0.0 && atten_max >= atten_min) ? atten_max : 0.25;

		std::mt19937 rng(seed);
		std::uniform_real_distribution<double> u01(0.0, 1.0);
		for (int c = 1; c < num_candidates; c++) {
			// Stratified / quasi-random sampling across (base_mps, steer_kp, steer_kd, pos_atten_gain, yaw_gain_p)
			double frac = (c + u01(rng) - 0.5) / static_cast<double>(num_candidates);
			frac = std::clamp(frac, 0.0, 1.0);
			candidates[c].base_mps = static_cast<float>(std::round((v_lo + frac * (v_hi - v_lo)) * 100.0) / 100.0);
			candidates[c].steer_kp = static_cast<float>(std::round((kp_lo + u01(rng) * (kp_hi - kp_lo)) * 10.0) / 10.0);
			candidates[c].steer_kd = static_cast<float>(std::round((kd_lo + u01(rng) * (kd_hi - kd_lo)) / 10.0) * 10.0);
			candidates[c].pos_atten_gain = static_cast<float>(std::round((at_lo + u01(rng) * (at_hi - at_lo)) * 100.0) / 100.0);
			candidates[c].yaw_gain_p = static_cast<float>(std::round((0.10 + u01(rng) * 0.45) * 100.0) / 100.0);
		}
	}

	GpuFirstRunSim gpu;
	std::vector<GpuSimResult> results;
	GpuBatchStats stats{};
	if (!gpu.run_batch(tracks, rp, candidates, max_time_s, seed, &results, &stats, &err)) {
		std::fprintf(stderr, "GPU simulation error: %s\n", err.c_str());
		return 1;
	}

	std::printf("====================================================================================================\n");
	std::printf(" [LineSim GPU 1st-Run Compute Engine]  Adapter: %s (%zu MB)\n", stats.adapter_name.c_str(), stats.vram_mb);
	std::printf("----------------------------------------------------------------------------------------------------\n");
	std::printf(" Tracks     : %zu (%s%s)\n", tracks.size(), track_labels.front().c_str(),
		random_maps > 0 ? (" + " + std::to_string(random_maps) + " dynamic random maps").c_str() : "");
	std::printf(" Candidates : %d gain/speed sets  ->  Total Parallel Runs: %d\n", num_candidates, stats.total_runs);
	std::printf(" Timing     : MapGen %.1f ms | ShaderInit %.1f ms | Upload %.2f ms | GPU Compute %.1f ms\n",
		map_gen_ms, stats.init_ms, stats.upload_ms, stats.dispatch_ms);
	double speedup = stats.dispatch_ms > 0.0 ? (stats.total_sim_time_s / (stats.dispatch_ms * 1e-3)) : 0.0;
	std::printf(" Throughput : %.1f s simulated driving in %.3f s  (%.0fx Real-Time)\n",
		stats.total_sim_time_s, stats.dispatch_ms * 1e-3, speedup);
	std::printf("====================================================================================================\n");

	// Aggregate across all tracks per candidate
	struct CandSummary {
		int cand_idx = 0;
		int pass_tracks = 0;
		int finish_tracks = 0;
		double avg_lap_s = 0.0;
		double avg_pace_10m = 0.0;
		double max_lat_mm = 0.0;
		double max_beta_deg = 0.0;
		double avg_weave_dps = 0.0;
		double avg_str_wz = 0.0;
		double score = 1e9;
	};
	std::vector<CandSummary> summaries(num_candidates);
	const int n_trk = static_cast<int>(tracks.size());

	for (int c = 0; c < num_candidates; c++) {
		CandSummary& cs = summaries[c];
		cs.cand_idx = c;
		double sum_lap = 0.0, sum_pace = 0.0, sum_weave = 0.0, sum_str_wz = 0.0;
		for (int t = 0; t < n_trk; t++) {
			const GpuSimResult& r = results[t * num_candidates + c];
			bool clean = r.finished && !r.line_lost && r.marker_ok && r.max_lat_mm <= 90.0f && r.max_beta_deg <= 25.0f;
			if (r.finished && !r.line_lost)
				cs.finish_tracks++;
			if (clean)
				cs.pass_tracks++;
			sum_lap += r.lap_time_s;
			sum_pace += 10.0 * r.lap_time_s / std::max(1.0, tracks[t].length());
			sum_weave += r.weave_dps;
			sum_str_wz += r.str_wz_rms;
			cs.max_lat_mm = std::max<double>(cs.max_lat_mm, r.max_lat_mm);
			cs.max_beta_deg = std::max<double>(cs.max_beta_deg, r.max_beta_deg);
		}
		cs.avg_lap_s = sum_lap / n_trk;
		cs.avg_pace_10m = sum_pace / n_trk;
		cs.avg_weave_dps = sum_weave / n_trk;
		cs.avg_str_wz = sum_str_wz / n_trk;
		// Rank by: 1) pass_tracks descending, 2) finish_tracks descending, 3) pace + stability penalty
		cs.score = (n_trk - cs.pass_tracks) * 1000.0 + (n_trk - cs.finish_tracks) * 500.0
		         + 10.0 * cs.avg_pace_10m + 0.02 * cs.avg_weave_dps + 0.03 * cs.avg_str_wz + 0.05 * cs.max_lat_mm;
	}

	std::sort(summaries.begin(), summaries.end(), [](const CandSummary& a, const CandSummary& b) {
		return a.score < b.score;
	});

	int pass_count = 0;
	for (const auto& s : summaries)
		if (s.pass_tracks == n_trk) pass_count++;

	std::printf(" Clean Pass Candidates (all %d track(s), lat<=90mm, beta<=25deg): %d / %d\n",
		n_trk, pass_count, num_candidates);
	if (!results.empty()) {
		const GpuSimResult& r0 = results[0];
		std::printf(" [Cand #0 on Track #0] fin=%d lost=%d mrk_ok=%d lap=%.3fs odom=%.2fm lat=%.1fmm beta=%.1fdeg L=%d R=%d C=%d STOP=%d mis=%d pos=(%.2f,%.2f)\n\n",
			r0.finished ? 1 : 0, r0.line_lost ? 1 : 0, r0.marker_ok ? 1 : 0,
			r0.lap_time_s, r0.odom_m, r0.max_lat_mm, r0.max_beta_deg,
			r0.count_L, r0.count_R, r0.count_C, r0.count_STOP, r0.marker_mismatch_cnt,
			r0.final_x, r0.final_y);
	}
	std::printf(" Rank | Cand |  v_base |  kp  |  kd  | atten | yaw_p | Pass | AvgLap | s/10m | MaxLat | MaxBeta | Weave | StrWz\n");
	std::printf("------+------+---------+------+------+-------+-------+------+--------+-------+--------+---------+-------+-------\n");

	int show_n = std::min(15, num_candidates);
	for (int i = 0; i < show_n; i++) {
		const CandSummary& s = summaries[i];
		const GpuDriveParams& dp = candidates[s.cand_idx];
		std::printf(" #%3d | %4d | %5.2fm/s | %4.1f | %4.0f | %5.2f | %5.2f | %2d/%-2d| %5.2fs | %5.2f | %4.1fmm | %5.1fdeg | %4.0f  | %4.1f\n",
			i + 1, s.cand_idx, dp.base_mps, dp.steer_kp, dp.steer_kd, dp.pos_atten_gain, dp.yaw_gain_p,
			s.pass_tracks, n_trk, s.avg_lap_s, s.avg_pace_10m, s.max_lat_mm, s.max_beta_deg, s.avg_weave_dps, s.avg_str_wz);
	}
	std::printf("====================================================================================================\n");
	std::fflush(stdout);
	return 0;
}
