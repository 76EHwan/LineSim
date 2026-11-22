#include "map_calib.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MC_PI      3.14159265358979323846f
#define MC_RAD2DEG 57.2957795130823208768f
#define MC_DEG2RAD 0.01745329251994329577f

static float wrap_pi(float a) {
	while (a > MC_PI) a -= 2.0f * MC_PI;
	while (a < -MC_PI) a += 2.0f * MC_PI;
	return a;
}

static float snap_grid45(float th, float th0) {
	const float step = 45.0f * MC_DEG2RAD;
	return th0 + roundf((th - th0) / step) * step;
}

/*
 * Reconstructs ZETIN course geometry directly onto nodes[].x, y using exact
 * 45-degree entry/exit headings and exact R = cfg->rule_arc_radius (0.25 m) arcs.
 */
static void reconstruct_zetin_map(MapCalibNode *nodes, int32_t n_nodes,
                                  const MapCalibMarker *markers, int32_t n_markers,
                                  const MapCalibConfig *cfg, float th0,
                                  MapCalibStats *st_out) {
	const float R = cfg->rule_arc_radius;
	const float arc45 = R * (45.0f * MC_DEG2RAD);

	int32_t i0 = markers[0].node_idx;
	if (i0 < 0) i0 = 0;
	if (i0 >= n_nodes) i0 = n_nodes - 1;
	int32_t i_end = markers[n_markers - 1].node_idx;
	if (i_end < i0) i_end = i0;
	if (i_end >= n_nodes) i_end = n_nodes - 1;

	/*
	 * Two-pass ZETIN reconstruction:
	 * Pass 0: reconstruct with unit straight scale and measure closure error (Ex, Ey)
	 *         plus straight-direction Gram matrix A.
	 * Pass 1: if closed_loop, solve 2x2 least-squares (a, b) so straight lengths
	 *         scale by (1 + a*ux + b*uy) to close the loop without shearing angles.
	 */
	float a_corr = 0.0f, b_corr = 0.0f;

	for (int pass = 0; pass < 2; ++pass) {
		float lx = cfg->start_x + cfg->sensor_x * cosf(th0);
		float ly = cfg->start_y + cfg->sensor_x * sinf(th0);
		float th = snap_grid45(th0, th0);
		float yaw_a = nodes[i0].x;

		/* On pass 1, restore nodes[].x? No: we only overwrite nodes[j].x on pass 1,
		   OR we read yaw from nodes[].th - drift? Even simpler: store heading in nodes[j].y
		   is not safe, so on pass 0 we only compute (lx, ly, th, Axx, Axy, Ayy) WITHOUT
		   overwriting nodes[].x, and on pass 1 we write nodes[].x, y! */
		if (pass == 0 && !cfg->closed_loop)
			continue;

		float Axx = 0.0f, Axy = 0.0f, Ayy = 0.0f;
		float sx_mark = lx + nodes[i0].s * cosf(th);
		float sy_mark = ly + nodes[i0].s * sinf(th);

		if (pass == 1) {
			for (int32_t j = 0; j <= i0; ++j) {
				nodes[j].x = lx + nodes[j].s * cosf(th);
				nodes[j].y = ly + nodes[j].s * sinf(th);
			}
		}
		lx = sx_mark;
		ly = sy_mark;

		int prev_dir = 0; /* -1: R, 0: Straight, +1: L */
		int32_t last_idx = i0;

		for (int32_t m = 1; m < n_markers; ++m) {
			int32_t idx_a = markers[m - 1].node_idx;
			int32_t idx_b = markers[m].node_idx;
			if (idx_a < 0) idx_a = 0;
			if (idx_b >= n_nodes) idx_b = n_nodes - 1;
			if (idx_b < idx_a) idx_b = idx_a;

			const float s_a = nodes[idx_a].s;
			const float s_b = nodes[idx_b].s;
			const float ds_seg = s_b - s_a;
			const int32_t prev_type = markers[m - 1].type;

			int next_dir = 0;
			if (prev_type == 1 || prev_type == 2) {
				const int x = (prev_type == 1) ? 1 : -1;
				next_dir = (prev_dir != x) ? x : 0;
			}

			const float yaw_b = (idx_b > idx_a) ? nodes[idx_b].x : yaw_a;
			const float dyaw_deg = (yaw_b - yaw_a) * MC_RAD2DEG;

			if (fabsf(dyaw_deg) >= 28.0f) {
				next_dir = (dyaw_deg > 0.0f) ? 1 : -1;
			} else if (next_dir != 0 && fabsf(dyaw_deg) < 18.0f) {
				next_dir = 0;
			}

			if (next_dir != 0 && ds_seg > 1e-4f) {
				const float k_yaw = (next_dir * dyaw_deg) / 45.0f;
				const float k_odom = ds_seg / arc45;
				const float k_est = (k_yaw > 0.4f) ? (0.5f * k_yaw + 0.5f * k_odom) : k_odom;
				int k = (int)roundf(k_est);
				if (k < 1) k = 1;
				if (k > 6) k = 6;

				const float phi = (float)(next_dir * k) * (45.0f * MC_DEG2RAD);
				const float cx = lx - (float)next_dir * R * sinf(th);
				const float cy = ly + (float)next_dir * R * cosf(th);
				const float rx = lx - cx;
				const float ry = ly - cy;

				if (pass == 1) {
					for (int32_t j = idx_a + 1; j <= idx_b; ++j) {
						float u = (nodes[j].s - s_a) / ds_seg;
						float ang = u * phi;
						float c = cosf(ang), s = sinf(ang);
						nodes[j].x = cx + rx * c - ry * s;
						nodes[j].y = cy + rx * s + ry * c;
					}
					lx = nodes[idx_b].x;
					ly = nodes[idx_b].y;
				} else {
					float c = cosf(phi), s = sinf(phi);
					lx = cx + rx * c - ry * s;
					ly = cy + rx * s + ry * c;
				}
				th += phi;
			} else {
				float yaw_mid = 0.5f * (yaw_a + yaw_b);
				th = snap_grid45(th + wrap_pi(yaw_mid - th), th0);
				const float ux = cosf(th);
				const float uy = sinf(th);
				float scale = 1.0f + a_corr * ux + b_corr * uy;
				if (scale < 0.92f) scale = 0.92f;
				if (scale > 1.08f) scale = 1.08f;

				if (pass == 0) {
					Axx += ds_seg * ux * ux;
					Axy += ds_seg * ux * uy;
					Ayy += ds_seg * uy * uy;
				} else {
					for (int32_t j = idx_a + 1; j <= idx_b; ++j) {
						float d = (nodes[j].s - s_a) * scale;
						nodes[j].x = lx + d * ux;
						nodes[j].y = ly + d * uy;
					}
				}
				lx += ds_seg * scale * ux;
				ly += ds_seg * scale * uy;
			}
			prev_dir = next_dir;
			last_idx = idx_b;
			yaw_a = yaw_b;
		}

		if (pass == 0) {
			const float u0x = cosf(th0), u0y = sinf(th0);
			const float n0x = -u0y, n0y = u0x;
			const float dx = lx - sx_mark;
			const float dy = ly - sy_mark;
			const float e_lat = dx * n0x + dy * n0y;
			const float along = dx * u0x + dy * u0y;
			if (st_out) st_out->closure_lat_mm = e_lat * 1000.0f;

			/* END sits ~0.35m behind START on the closing straight */
			float e_long = 0.0f;
			if (along > -0.20f) {
				e_long = along + 0.35f;
			} else if (along < -0.75f) {
				e_long = along + 0.35f;
			}
			const float Ex = e_lat * n0x + e_long * u0x;
			const float Ey = e_lat * n0y + e_long * u0y;
			const float det = Axx * Ayy - Axy * Axy;
			if (det > 1e-4f) {
				a_corr = ( Ayy * (-Ex) - Axy * (-Ey)) / det;
				b_corr = (-Axy * (-Ex) + Axx * (-Ey)) / det;
				if (a_corr < -0.05f) a_corr = -0.05f;
				if (a_corr >  0.05f) a_corr =  0.05f;
				if (b_corr < -0.05f) b_corr = -0.05f;
				if (b_corr >  0.05f) b_corr =  0.05f;
			}
		} else {
			/* Trailing nodes after the final marker (pit-in) along final 45-deg heading */
			const float s_last = nodes[last_idx].s;
			for (int32_t j = last_idx + 1; j < n_nodes; ++j) {
				float d = nodes[j].s - s_last;
				nodes[j].x = lx + d * cosf(th);
				nodes[j].y = ly + d * sinf(th);
			}
		}
	}
}

/*
 * Inverse-computes constant line curvature for curve span [idx_a .. idx_b]
 * turning from th_a to th_b (45-deg snapped), writes linear constant-curvature
 * headings into nodes[].y and ds_scale into nodes[].th.
 */
static void reconstruct_const_curvature_arc(MapCalibNode *nodes,
                                            int32_t idx_a, int32_t idx_b,
                                            float th_a, float th_b) {
	const float dphi = th_b - th_a;
	const float abs_phi = fabsf(dphi);
	const float s_a = nodes[idx_a].s;
	const float s_b = nodes[idx_b].s;
	if (idx_b <= idx_a || (s_b - s_a) < 1e-4f)
		return;

	/* Find 20% and 80% heading crossings in drift-corrected nodes[].x to exclude straight stubs */
	float s_20 = s_a + 0.2f * (s_b - s_a);
	float s_80 = s_a + 0.8f * (s_b - s_a);
	if (abs_phi >= 0.35f) {
		const float th_20 = th_a + 0.20f * dphi;
		const float th_80 = th_a + 0.80f * dphi;
		const float dir = (dphi > 0.0f) ? 1.0f : -1.0f;
		int found_20 = 0, found_80 = 0;
		for (int32_t k = idx_a + 1; k <= idx_b; ++k) {
			float x0 = nodes[k - 1].x;
			float x1 = nodes[k].x;
			if (!found_20 && (x1 - th_20) * dir >= 0.0f) {
				float denom = x1 - x0;
				float f = (fabsf(denom) > 1e-5f) ? (th_20 - x0) / denom : 0.5f;
				if (f < 0.0f) f = 0.0f;
				if (f > 1.0f) f = 1.0f;
				s_20 = nodes[k - 1].s + f * (nodes[k].s - nodes[k - 1].s);
				found_20 = 1;
			}
			if (found_20 && !found_80 && (x1 - th_80) * dir >= 0.0f) {
				float denom = x1 - x0;
				float f = (fabsf(denom) > 1e-5f) ? (th_80 - x0) / denom : 0.5f;
				if (f < 0.0f) f = 0.0f;
				if (f > 1.0f) f = 1.0f;
				s_80 = nodes[k - 1].s + f * (nodes[k].s - nodes[k - 1].s);
				found_80 = 1;
				break;
			}
		}
	}

	float l_arc_meas = (s_80 - s_20) / 0.60f;
	if (l_arc_meas < 0.05f || l_arc_meas > (s_b - s_a) + 0.05f) {
		l_arc_meas = s_b - s_a;
		s_20 = s_a + 0.2f * l_arc_meas;
	}
	float s_tan_a = s_20 - 0.20f * l_arc_meas;
	float s_tan_b = s_tan_a + l_arc_meas;
	if (s_tan_a < s_a) s_tan_a = s_a;
	if (s_tan_b > s_b) s_tan_b = s_b;
	const float l_span = s_tan_b - s_tan_a;
	if (l_span < 1e-4f)
		return;

	for (int32_t k = idx_a + 1; k <= idx_b; ++k) {
		float sk = nodes[k].s;
		if (sk <= s_tan_a) {
			nodes[k].y = th_a;
		} else if (sk >= s_tan_b) {
			nodes[k].y = th_b;
		} else {
			float u = (sk - s_tan_a) / l_span;
			nodes[k].y = th_a + u * dphi;
		}
	}
}

/*
 * Splits curve gap [idx_a .. idx_b] at any S-curve inflection points (>= 18 deg reversal)
 * and reconstructs each sub-arc with inverse-computed constant curvature.
 */
static void reconstruct_curve_gap(MapCalibNode *nodes,
                                  int32_t idx_a, int32_t idx_b,
                                  float th_a, float th_b, float th0) {
	if (idx_b <= idx_a + 1)
		return;

	const float rev_thresh = 18.0f * MC_DEG2RAD;
	int32_t sub_start = idx_a;
	float sub_th = th_a;

	int dir = 0;
	float peak_th = nodes[idx_a].x;
	int32_t peak_idx = idx_a;

	for (int32_t k = idx_a + 1; k < idx_b; ++k) {
		float th_k = nodes[k].x;
		if (dir == 0) {
			float d = th_k - peak_th;
			if (fabsf(d) >= 12.0f * MC_DEG2RAD) {
				dir = (d > 0.0f) ? 1 : -1;
				peak_th = th_k;
				peak_idx = k;
			}
		} else {
			if (dir > 0) {
				if (th_k > peak_th) { peak_th = th_k; peak_idx = k; }
				if ((peak_th - th_k) >= rev_thresh) {
					float th_mid = snap_grid45(peak_th, th0);
					if (fabsf(th_mid - sub_th) >= 30.0f * MC_DEG2RAD && peak_idx > sub_start) {
						reconstruct_const_curvature_arc(nodes, sub_start, peak_idx,
						                                sub_th, th_mid);
						sub_start = peak_idx;
						sub_th = th_mid;
					}
					dir = -1;
					peak_th = th_k;
					peak_idx = k;
				}
			} else {
				if (th_k < peak_th) { peak_th = th_k; peak_idx = k; }
				if ((th_k - peak_th) >= rev_thresh) {
					float th_mid = snap_grid45(peak_th, th0);
					if (fabsf(th_mid - sub_th) >= 30.0f * MC_DEG2RAD && peak_idx > sub_start) {
						reconstruct_const_curvature_arc(nodes, sub_start, peak_idx,
						                                sub_th, th_mid);
						sub_start = peak_idx;
						sub_th = th_mid;
					}
					dir = 1;
					peak_th = th_k;
					peak_idx = k;
				}
			}
		}
	}

	reconstruct_const_curvature_arc(nodes, sub_start, idx_b,
	                                sub_th, th_b);
}

/*
 * Rule-based ZETIN map (1st run): every marker-to-marker segment is a straight or one arc of
 * k x 45 deg with R = rule_arc_radius. Arc lengths come from the rule, straight lengths from
 * odometry. Straights that start at an arc exit read ~18 mm long (the marker fires at different
 * distances after the curvature change at arc entry and exit). Headings are exact multiples of
 * 45 deg, so the only free quantities are the straight lengths: a weighted least-squares
 * correction closes the loop laterally (hard) and pulls paired crossing ('C') marks together (soft).
 * Measured on 11 sim runs / 6 tracks: line rms 35 mm, worst point 117 mm.
 */
#define RM_MAX_SEG    256
#define RM_MAX_EQ     33
#define RM_LAG        0.040f   /* marker event after the curvature change [m] */
#define RM_LAG_C      0.060f   /* crossing mark after the crossing point [m] */
#define RM_STR_OFF    0.018f   /* straight after an arc exit reads this much long [m] */
#define RM_SIG_STR0   0.017f   /* straight length error [m] = SIG0 + SIGK * length */
#define RM_SIG_STRK   0.002f
#define RM_SIG_CROSS  0.100f   /* crossing coincidence tolerance [m] */

static int rm_snap_k45(float dyaw_deg, float seg_d, float R) {
	if (fabsf(dyaw_deg) < 22.0f || seg_d <= 0.005f) return 0;
	const float arc45 = R * (45.0f * MC_DEG2RAD);
	if (fabsf(dyaw_deg) < 34.0f && seg_d > 1.6f * arc45) return 0;
	float k_est = 0.5f * (fabsf(dyaw_deg) / (45.0f * 1.05f)) + 0.5f * (seg_d / (arc45 * 1.08f));
	int k = (int)roundf(k_est);
	if (k < 1) k = 1;
	if (k > 6) k = 6;
	return (dyaw_deg > 0.0f) ? k : -k;
}

typedef struct {
	float len[RM_MAX_SEG];   /* straight length [m] (arcs: R * |phi|) */
	float th[RM_MAX_SEG];    /* heading at segment start [rad] */
	int   k45[RM_MAX_SEG];   /* 0 = straight, else signed number of 45 deg steps */
	float ex[RM_MAX_SEG], ey[RM_MAX_SEG], eth[RM_MAX_SEG];   /* event point after the segment */
	int   n;
} RuleMap;

/* Walks the segments from the sensor-bar start; boundaries sit RM_LAG before each event. */
static void rm_trace(RuleMap *rm, float bx, float by, float R) {
	float x = bx, y = by;
	for (int j = 0; j < rm->n; ++j) {
		const float th = rm->th[j];
		if (rm->k45[j] == 0) {
			x += rm->len[j] * cosf(th);
			y += rm->len[j] * sinf(th);
		} else {
			const float phi = (float)rm->k45[j] * (45.0f * MC_DEG2RAD);
			const float turn = (phi > 0.0f) ? 1.0f : -1.0f;
			const float cx = x - turn * R * sinf(th), cy = y + turn * R * cosf(th);
			const float rx = x - cx, ry = y - cy, c = cosf(phi), s = sinf(phi);
			x = cx + rx * c - ry * s;
			y = cy + rx * s + ry * c;
		}
		const float th_end = (j + 1 < rm->n) ? rm->th[j + 1]
			: rm->th[j] + (float)rm->k45[j] * (45.0f * MC_DEG2RAD);
		rm->ex[j] = x + RM_LAG * cosf(th_end);
		rm->ey[j] = y + RM_LAG * sinf(th_end);
		rm->eth[j] = th_end;
	}
}

/* Point on the traced line at line distance d from the bar start (d may run past the end). */
static void rm_point(const RuleMap *rm, float bx, float by, float R, float d, float *px, float *py) {
	float x = bx, y = by;
	for (int j = 0; j < rm->n; ++j) {
		const float th = rm->th[j];
		const float L = rm->len[j];
		if (d <= L || j == rm->n - 1) {
			const float u = (d < 0.0f) ? 0.0f : d;
			if (rm->k45[j] == 0 || d > L) {
				/* straight piece, or past the last arc: continue along the exit heading */
				float t = th;
				float base_x = x, base_y = y, run = u;
				if (rm->k45[j] != 0) {
					const float phi = (float)rm->k45[j] * (45.0f * MC_DEG2RAD);
					const float turn = (phi > 0.0f) ? 1.0f : -1.0f;
					const float cx = x - turn * R * sinf(th), cy = y + turn * R * cosf(th);
					const float rx = x - cx, ry = y - cy, c = cosf(phi), s = sinf(phi);
					base_x = cx + rx * c - ry * s;
					base_y = cy + rx * s + ry * c;
					t = th + phi;
					run = u - L;
				}
				*px = base_x + run * cosf(t);
				*py = base_y + run * sinf(t);
			} else {
				const float phi = (float)rm->k45[j] * (45.0f * MC_DEG2RAD) * (u / L);
				const float turn = (rm->k45[j] > 0) ? 1.0f : -1.0f;
				const float cx = x - turn * R * sinf(th), cy = y + turn * R * cosf(th);
				const float rx = x - cx, ry = y - cy, c = cosf(phi), s = sinf(phi);
				*px = cx + rx * c - ry * s;
				*py = cy + rx * s + ry * c;
			}
			return;
		}
		/* advance to the end of segment j */
		if (rm->k45[j] == 0) {
			x += L * cosf(th);
			y += L * sinf(th);
		} else {
			const float phi = (float)rm->k45[j] * (45.0f * MC_DEG2RAD);
			const float turn = (phi > 0.0f) ? 1.0f : -1.0f;
			const float cx = x - turn * R * sinf(th), cy = y + turn * R * cosf(th);
			const float rx = x - cx, ry = y - cy, c = cosf(phi), s = sinf(phi);
			x = cx + rx * c - ry * s;
			y = cy + rx * s + ry * c;
		}
		d -= L;
	}
	*px = x;
	*py = y;
}

/* Solves the small dense system M z = r (q x q, Gaussian elimination with partial pivoting). */
static int rm_solve(float M[RM_MAX_EQ][RM_MAX_EQ], float *r, int q) {
	for (int c = 0; c < q; ++c) {
		int p = c;
		for (int i = c + 1; i < q; ++i)
			if (fabsf(M[i][c]) > fabsf(M[p][c])) p = i;
		if (fabsf(M[p][c]) < 1e-12f) return 0;
		if (p != c) {
			for (int k = 0; k < q; ++k) { float t = M[c][k]; M[c][k] = M[p][k]; M[p][k] = t; }
			float t = r[c]; r[c] = r[p]; r[p] = t;
		}
		for (int i = c + 1; i < q; ++i) {
			const float f = M[i][c] / M[c][c];
			for (int k = c; k < q; ++k) M[i][k] -= f * M[c][k];
			r[i] -= f * r[c];
		}
	}
	for (int c = q - 1; c >= 0; --c) {
		float v = r[c];
		for (int k = c + 1; k < q; ++k) v -= M[c][k] * r[k];
		r[c] = v / M[c][c];
	}
	return 1;
}

static RuleMap g_rm;   /* last rule map (MapCalib_RuleFromLog reads it) */

static int rule_zetin_map(MapCalibNode *nodes, int32_t n_nodes,
                          MapCalibMarker *markers, int32_t n_markers,
                          const MapCalibConfig *cfg, MapCalibStats *st) {
	static float A[RM_MAX_EQ][RM_MAX_SEG];
	static float Mq[RM_MAX_EQ][RM_MAX_EQ];
	static float rq[RM_MAX_EQ];
	const float R = cfg->rule_arc_radius;
	const float th0 = cfg->start_th;
	if (n_markers < 2 || n_markers > RM_MAX_SEG) return 0;

	/* segments: 0 = robot start -> first mark, j = mark j-1 -> mark j */
	g_rm.n = n_markers;
	static float seg_d[RM_MAX_SEG];
	float th = th0;
	int prev_dir = 0;   /* -1: R 곡선, 0: 직선, +1: L 곡선 (직전 구간) */
	for (int j = 0; j < n_markers; ++j) {
		int32_t ib = markers[j].node_idx;
		int32_t ia = (j == 0) ? 0 : markers[j - 1].node_idx;
		if (ia < 0) ia = 0;
		if (ib >= n_nodes) ib = n_nodes - 1;
		if (ib < ia) ib = ia;
		/* 여러 회차를 융합한 거리가 있으면 그것을 쓴다 (노드 위치는 비례 배치) */
		const float d = (markers[j].seg_dist_m > 0.0f) ? markers[j].seg_dist_m : (nodes[ib].s - nodes[ia].s);
		/* segment yaw from the firmware's own marker log (the viewer's sampled heading lags the marks) */
		const float dyaw = markers[j].seg_yaw_deg;
		/* 구간 종류는 ZETIN 마커 규칙으로 정한다: 곡선 시작은 자기 쪽 마커, 곡선->직선은 마지막 회전 쪽 마커.
		   직전 마커가 X(L/R)이고 직전 구간이 X 곡선이면 이번 구간은 직선, 아니면 X 곡선.
		   C/STOP 마커는 직전 방향을 유지. yaw 가 규칙과 크게 어긋나면 45도 스냅으로 되돌린다. */
		int k = 0;
		if (j > 0) {
			const int32_t pt = markers[j - 1].type;
			int expd;
			if (pt == 1 || pt == 2) {
				const int x = (pt == 1) ? 1 : -1;
				expd = (prev_dir == x) ? 0 : x;
			} else {
				expd = (j > 1) ? prev_dir : 0;
			}
			if (expd != 0) {
				if (dyaw * (float)expd < -30.0f) {
					k = rm_snap_k45(dyaw, d, R);
				} else {
					const float arc45 = R * (45.0f * MC_DEG2RAD);
					const float k_est = 0.5f * (fabsf(dyaw) / (45.0f * 1.05f)) + 0.5f * (d / (arc45 * 1.08f));
					int kk = (int)roundf(k_est);
					if (kk < 1) kk = 1;
					if (kk > 6) kk = 6;
					k = expd * kk;
				}
			} else if (fabsf(dyaw) > 60.0f) {
				k = rm_snap_k45(dyaw, d, R);
			}
		}
		prev_dir = (k > 0) ? 1 : ((k < 0) ? -1 : 0);
		g_rm.k45[j] = k;
		seg_d[j] = d;
	}

	for (int j = 0; j < n_markers; ++j) {
		const int k = g_rm.k45[j];
		const float d = seg_d[j];
		g_rm.th[j] = th;
		if (k != 0) {
			g_rm.len[j] = R * fabsf((float)k) * (45.0f * MC_DEG2RAD);
			th += (float)k * (45.0f * MC_DEG2RAD);
		} else {
			float L = d;
			if (j == 0) L -= RM_LAG;                                   /* first boundary LAG before the mark */
			else if (g_rm.k45[j - 1] != 0) L -= RM_STR_OFF;              /* straight after an arc exit */
			g_rm.len[j] = (L > 0.01f) ? L : 0.01f;
		}
	}
	const float bx = cfg->start_x + cfg->sensor_x * cosf(th0);
	const float by = cfg->start_y + cfg->sensor_x * sinf(th0);
	rm_trace(&g_rm, bx, by, R);

	/* least squares on the straight lengths */
	int q = 0;
	const float nx = -sinf(th0), ny = cosf(th0);
	/* 폐루프 조건은 스냅된 총 회전이 360도의 배수일 때만 (규칙 분류가 틀렸으면 닫지 않는다) */
	int k_sum = 0;
	for (int j = 0; j < g_rm.n; ++j) k_sum += g_rm.k45[j];
	if (cfg->closed_loop && (k_sum % 8) == 0) {
		for (int j = 0; j < g_rm.n; ++j)
			A[q][j] = (g_rm.k45[j] == 0 && j > 0) ? cosf(g_rm.th[j]) * nx + sinf(g_rm.th[j]) * ny : 0.0f;
		const int last = g_rm.n - 1;
		const float lx = g_rm.ex[last] - RM_LAG * cosf(g_rm.eth[last]);
		const float ly = g_rm.ey[last] - RM_LAG * sinf(g_rm.eth[last]);
		rq[q] = -((lx - bx) * nx + (ly - by) * ny);
		if (st) st->closure_lat_mm = -rq[q] * 1000.0f;
		++q;
	}
	const int q_hard = q;
	{
		uint8_t used[RM_MAX_SEG] = {0};
		for (int a = 0; a < g_rm.n && q + 2 <= RM_MAX_EQ; ++a) {
			if (markers[a].type != 3 || used[a]) continue;
			const float pax = g_rm.ex[a] - RM_LAG_C * cosf(g_rm.eth[a]);
			const float pay = g_rm.ey[a] - RM_LAG_C * sinf(g_rm.eth[a]);
			int best = -1;
			float best_d = 0.80f;
			for (int b = a + 1; b < g_rm.n; ++b) {
				if (markers[b].type != 3 || used[b]) continue;
				if (nodes[markers[b].node_idx].s - nodes[markers[a].node_idx].s < 1.0f) continue;
				const float pbx = g_rm.ex[b] - RM_LAG_C * cosf(g_rm.eth[b]);
				const float pby = g_rm.ey[b] - RM_LAG_C * sinf(g_rm.eth[b]);
				const float dd = hypotf(pax - pbx, pay - pby);
				if (dd < best_d) { best_d = dd; best = b; }
			}
			if (best < 0) continue;
			used[a] = used[best] = 1;
			const float pbx = g_rm.ex[best] - RM_LAG_C * cosf(g_rm.eth[best]);
			const float pby = g_rm.ey[best] - RM_LAG_C * sinf(g_rm.eth[best]);
			for (int comp = 0; comp < 2; ++comp) {
				for (int j = 0; j < g_rm.n; ++j) {
					const int between = (g_rm.k45[j] == 0 && j > a && j <= best);
					A[q][j] = between ? (comp == 0 ? cosf(g_rm.th[j]) : sinf(g_rm.th[j])) : 0.0f;
				}
				rq[q] = (comp == 0) ? (pax - pbx) : (pay - pby);
				++q;
			}
			if (st) st->crossing_pairs++;
		}
	}
	if (q > 0) {
		/* dl = S A' (A S A' + Rn)^-1 r,  S = diag(sigma_j^2) */
		for (int i = 0; i < q; ++i) {
			for (int k = 0; k < q; ++k) {
				float acc = 0.0f;
				for (int j = 0; j < g_rm.n; ++j) {
					if (g_rm.k45[j] != 0 || j == 0) continue;
					const float sg = RM_SIG_STR0 + RM_SIG_STRK * g_rm.len[j];
					acc += A[i][j] * sg * sg * A[k][j];
				}
				Mq[i][k] = acc;
			}
			Mq[i][i] += (i < q_hard) ? 1e-8f : RM_SIG_CROSS * RM_SIG_CROSS;
		}
		if (rm_solve(Mq, rq, q)) {
			for (int j = 1; j < g_rm.n; ++j) {
				if (g_rm.k45[j] != 0) continue;
				const float sg = RM_SIG_STR0 + RM_SIG_STRK * g_rm.len[j];
				float acc = 0.0f;
				for (int i = 0; i < q; ++i) acc += A[i][j] * rq[i];
				g_rm.len[j] += sg * sg * acc;
				if (g_rm.len[j] < 0.01f) g_rm.len[j] = 0.01f;
			}
			rm_trace(&g_rm, bx, by, R);
		}
	}

	/* nodes: proportional position inside their segment, on the line (boundaries LAG before the marks) */
	float seg_start = -RM_LAG;   /* line distance of the event that starts segment j (robot start = 0) */
	float line_acc = 0.0f;       /* line distance of the start of segment j */
	for (int j = 0; j < g_rm.n; ++j) {
		const int32_t ia = (j == 0) ? 0 : markers[j - 1].node_idx;
		const int32_t ib = markers[j].node_idx;
		const float sa = nodes[ia < 0 ? 0 : ia].s, sb = nodes[ib >= n_nodes ? n_nodes - 1 : ib].s;
		const float line_a = (j == 0) ? 0.0f : line_acc + RM_LAG;   /* event j-1 on the line */
		const float line_b = line_acc + g_rm.len[j] + RM_LAG;          /* event j */
		for (int32_t i = (j == 0 ? 0 : ia + 1); i <= ib && i < n_nodes; ++i) {
			const float u = (sb - sa > 1e-5f) ? (nodes[i].s - sa) / (sb - sa) : 1.0f;
			rm_point(&g_rm, bx, by, R, line_a + u * (line_b - line_a), &nodes[i].x, &nodes[i].y);
		}
		line_acc += g_rm.len[j];
		(void)seg_start;
	}
	{
		const int32_t il = markers[g_rm.n - 1].node_idx;
		const float s_last = nodes[il].s;
		for (int32_t i = il + 1; i < n_nodes; ++i)
			rm_point(&g_rm, bx, by, R, line_acc + RM_LAG + (nodes[i].s - s_last), &nodes[i].x, &nodes[i].y);
	}
	for (int j = 0; j < g_rm.n; ++j) {
		markers[j].x = g_rm.ex[j];
		markers[j].y = g_rm.ey[j];
	}
	if (st) st->odom_scale = 1.0f;
	return 1;
}

void MapCalib_Run(MapCalibNode *nodes, int32_t n_nodes,
                  MapCalibMarker *markers, int32_t n_markers,
                  const MapCalibConfig *cfg, MapCalibStats *stats_out) {
	MapCalibStats st = {0.0f, 1.0f, 0.0f, 0};
	if (!nodes || n_nodes <= 1 || !cfg) {
		if (stats_out) *stats_out = st;
		return;
	}

	/* Unwrap heading via delta angle so -180..+180 wrapped IMU inputs work cleanly */
	{
		float prev_raw = nodes[0].th;
		float unwrapped = nodes[0].th;
		for (int32_t i = 1; i < n_nodes; ++i) {
			float cur_raw = nodes[i].th;
			unwrapped += wrap_pi(cur_raw - prev_raw);
			prev_raw = cur_raw;
			nodes[i].th = unwrapped;
		}
	}

	const float total_s = nodes[n_nodes - 1].s;
	const float th0 = cfg->start_th;
	const float th_end = nodes[n_nodes - 1].th;

	/* Stage 1: Linear gyro drift removal -> store drift-corrected heading in nodes[i].x */
	float drift_total = 0.0f;
	float drift_per_m = 0.0f;
	if (cfg->closed_loop && total_s > 0.5f) {
		drift_total = wrap_pi(th_end - th0);
		if (fabsf(drift_total) < 0.35f) {
			drift_per_m = drift_total / total_s;
		} else {
			drift_total = 0.0f;
		}
	}
	st.gyro_drift_deg = drift_total * MC_RAD2DEG;

	for (int32_t i = 0; i < n_nodes; ++i) {
		nodes[i].x = nodes[i].th - drift_per_m * nodes[i].s;
	}

	const int used_zetin = (!cfg->path_only && cfg->rule_arc_radius > 0.0f && markers && n_markers >= 2);
	/* 마커 로그(회차별 yaw)가 있으면 모든 회차에서 규칙 기반 ZETIN 맵 복원 (R 0.25 원호, 직선 길이 최소제곱) */
	if (cfg->mark_yaw_valid && cfg->rule_arc_radius > 0.0f && markers && n_markers >= 2
	    && rule_zetin_map(nodes, n_nodes, markers, n_markers, cfg, &st)) {
		st.rule_map = 1;
		if (stats_out) *stats_out = st;
		return;
	}
	/* Stage 2 + Stage 3 + Stage 4: Vector reconstruction (always run so 1st drive also has vector map) */
	{
		/*
		 * Stage 2:
		 * - Map mode (!path_only): 45-deg straight snapping + constant-arc reconstruction
		 * - Path mode (path_only): 45-deg straight locking + vector gyro shortcut preservation
		 */
		int32_t lo = 0, hi = 0;
		int32_t prev_str_idx = -1;
		float prev_grid = snap_grid45(nodes[0].x, th0);
		nodes[0].y = prev_grid;

		for (int32_t i = 0; i < n_nodes; ++i) {
			const float si = nodes[i].s;
			while (lo + 1 < i && (si - nodes[lo].s) > 0.07f) ++lo;
			while (hi + 1 < n_nodes && (nodes[hi].s - si) < 0.07f) ++hi;

			const float span = nodes[hi].s - nodes[lo].s;
			const float rate = (span >= 0.06f) ? (fabsf(nodes[hi].x - nodes[lo].x) / span) : 99.0f;
			const float grid = snap_grid45(nodes[i].x, th0);
			const float err = grid - nodes[i].x;
			const int is_straight = (rate < 1.2f && fabsf(err) < 15.0f * MC_DEG2RAD) ? 1 : 0;

			if (is_straight) {
				nodes[i].y = grid;
				if (prev_str_idx >= 0 && i > prev_str_idx + 1) {
					const float dphi_gap = fabsf(grid - prev_grid);
					const float s_a = nodes[prev_str_idx].s;
					const float s_b = nodes[i].s;
					const float l_meas = s_b - s_a;
					if (cfg->path_only) {
						const float err_a = prev_grid - nodes[prev_str_idx].x;
						const float err_b = grid - nodes[i].x;
						for (int32_t k = prev_str_idx + 1; k < i; ++k) {
							float u = (l_meas > 1e-4f) ? ((nodes[k].s - s_a) / l_meas) : 0.0f;
							nodes[k].y = nodes[k].x + err_a + u * (err_b - err_a);
						}
					} else if (dphi_gap >= 0.35f && l_meas > 1e-4f) {
						/* Curve gap: split S-curve inflections and reconstruct constant-curvature arcs */
						reconstruct_curve_gap(nodes, prev_str_idx, i,
						                      prev_grid, grid, th0);
					} else {
						for (int32_t k = prev_str_idx + 1; k < i; ++k)
							nodes[k].y = grid;
					}
				} else if (prev_str_idx < 0 && i > 0) {
					for (int32_t k = 0; k < i; ++k)
						nodes[k].y = grid;
				}
				prev_str_idx = i;
				prev_grid = grid;
			} else {
				nodes[i].y = nodes[i].x;
			}
		}
		if (prev_str_idx >= 0 && prev_str_idx + 1 < n_nodes) {
			for (int32_t k = prev_str_idx + 1; k < n_nodes; ++k)
				nodes[k].y = prev_grid;
		}

	/* Stage 3: Vector re-integration of axle + sensor offset */
	float odom_scale = 1.0f;
	if (cfg->closed_loop && total_s > 0.5f) {
		float ax0 = cfg->start_x, ay0 = cfg->start_y;
		float pth = nodes[0].y;
		for (int32_t i = 1; i < n_nodes; ++i) {
			float th_c = nodes[i].y;
			float th_mid = 0.5f * (pth + th_c);
			ax0 += nodes[i].ds * cosf(th_mid);
			ay0 += nodes[i].ds * sinf(th_mid);
			pth = th_c;
		}
		const float u0x = cosf(th0), u0y = sinf(th0);
		const float along0 = (ax0 - cfg->start_x) * u0x + (ay0 - cfg->start_y) * u0y;
		if (along0 > 0.30f) {
			float est = 1.0f - (along0 - 0.30f) / total_s;
			if (est >= 0.95f && est <= 1.02f) {
				odom_scale = est;
			}
		}
	}
	st.odom_scale = odom_scale;

	float ax = cfg->start_x;
	float ay = cfg->start_y;
	{
		float prev_th = nodes[0].y;
		nodes[0].x = ax + cfg->sensor_x * cosf(prev_th);
		nodes[0].y = ay + cfg->sensor_x * sinf(prev_th);
		for (int32_t i = 1; i < n_nodes; ++i) {
			float th_c = nodes[i].y;
			float th_mid = 0.5f * (prev_th + th_c);
			float ds_c = nodes[i].ds * odom_scale;
			ax += ds_c * cosf(th_mid);
			ay += ds_c * sinf(th_mid);
			nodes[i].x = ax + cfg->sensor_x * cosf(th_c);
			nodes[i].y = ay + cfg->sensor_x * sinf(th_c);
			prev_th = th_c;
		}
	}
	}

	/* Stage 4: Closed-loop lateral/overshoot closure (skipped when ZETIN pass already closed straights) */
	if (cfg->closed_loop && total_s > 0.5f) {
		const float u0x = cosf(th0), u0y = sinf(th0);
		const float n0x = -u0y, n0y = u0x;
		const float dx = nodes[n_nodes - 1].x - nodes[0].x;
		const float dy = nodes[n_nodes - 1].y - nodes[0].y;
		const float e_lat = dx * n0x + dy * n0y;
		const float along = dx * u0x + dy * u0y;
		st.closure_lat_mm = e_lat * 1000.0f;

		const float e_long = (along > 0.0f) ? (along + 0.30f) : 0.0f;
		for (int32_t i = 0; i < n_nodes; ++i) {
			float u = nodes[i].s / total_s;
			nodes[i].x -= u * (e_lat * n0x + e_long * u0x);
			nodes[i].y -= u * (e_lat * n0y + e_long * u0y);
		}
	}

	if (used_zetin) {
		float *vec_xy = (float *)malloc((size_t)n_nodes * 2U * sizeof(float));
		if (vec_xy) {
			for (int32_t i = 0; i < n_nodes; ++i) {
				vec_xy[2 * i] = nodes[i].x;
				vec_xy[2 * i + 1] = nodes[i].y;
				nodes[i].x = nodes[i].th - drift_per_m * nodes[i].s;
			}
			float vec_odom_scale = st.odom_scale;
			float vec_closure_lat = st.closure_lat_mm;
			reconstruct_zetin_map(nodes, n_nodes, markers, n_markers, cfg, th0, &st);
			st.odom_scale = vec_odom_scale;
			st.closure_lat_mm = vec_closure_lat;
			for (int32_t m = 1; m < n_markers; ++m) {
				int32_t ia = markers[m - 1].node_idx;
				int32_t ib = markers[m].node_idx;
				if (ia < 0) ia = 0;
				if (ib >= n_nodes) ib = n_nodes - 1;
				if (ib <= ia) continue;
				float ex_a = nodes[ia].x - vec_xy[2 * ia];
				float ey_a = nodes[ia].y - vec_xy[2 * ia + 1];
				float ex_b = nodes[ib].x - vec_xy[2 * ib];
				float ey_b = nodes[ib].y - vec_xy[2 * ib + 1];
				float sa = nodes[ia].s;
				float ds = nodes[ib].s - sa;
				for (int32_t i = ia + 1; i < ib; ++i) {
					float u = (ds > 1e-4f) ? ((nodes[i].s - sa) / ds) : 0.5f;
					float vx_al = vec_xy[2 * i] + ex_a * (1.0f - u) + ex_b * u;
					float vy_al = vec_xy[2 * i + 1] + ey_a * (1.0f - u) + ey_b * u;
					float d = hypotf(nodes[i].x - vx_al, nodes[i].y - vy_al);
					float w_vec = (d <= 0.12f) ? 0.35f : 0.20f;
					nodes[i].x = (1.0f - w_vec) * nodes[i].x + w_vec * vx_al;
					nodes[i].y = (1.0f - w_vec) * nodes[i].y + w_vec * vy_al;
				}
			}
			free(vec_xy);
		}
	}

	/* Stage 5: Crossing-pair ('C') local relaxation */
	if (markers && n_markers > 1 && total_s > 0.5f) {
		int32_t cross_idx[32];
		int32_t n_cross = 0;
		for (int32_t i = 0; i < n_markers && n_cross < 32; ++i) {
			if (markers[i].type == 3 && markers[i].node_idx >= 0 && markers[i].node_idx < n_nodes) {
				cross_idx[n_cross++] = i;
			}
		}
		uint8_t used[32] = {0};
		for (int32_t a = 0; a < n_cross; ++a) {
			if (used[a]) continue;
			int32_t ia = markers[cross_idx[a]].node_idx;
			float best_d2 = 0.65f * 0.65f;
			int32_t best_b = -1;
			for (int32_t b = a + 1; b < n_cross; ++b) {
				if (used[b]) continue;
				int32_t ib = markers[cross_idx[b]].node_idx;
				if (fabsf(nodes[ib].s - nodes[ia].s) < 1.0f) continue;
				float ddx = nodes[ib].x - nodes[ia].x;
				float ddy = nodes[ib].y - nodes[ia].y;
				float d2 = ddx * ddx + ddy * ddy;
				if (d2 < best_d2) {
					best_d2 = d2;
					best_b = b;
				}
			}
			if (best_b >= 0) {
				used[a] = 1;
				used[best_b] = 1;
				st.crossing_pairs++;
				int32_t ib = markers[cross_idx[best_b]].node_idx;
				float mx = 0.5f * (nodes[ib].x - nodes[ia].x);
				float my = 0.5f * (nodes[ib].y - nodes[ia].y);
				float sa = nodes[ia].s, sb = nodes[ib].s;
				float half_span = 0.35f * (sb - sa);
				if (half_span < 0.5f) half_span = 0.5f;
				for (int32_t k = 0; k < n_nodes; ++k) {
					float sk = nodes[k].s;
					float da = fabsf(sk - sa);
					float db = fabsf(sk - sb);
					float wa = (da < half_span) ? (1.0f - da / half_span) : 0.0f;
					float wb = (db < half_span) ? (1.0f - db / half_span) : 0.0f;
					float w = wa - wb;
					nodes[k].x += w * mx;
					nodes[k].y += w * my;
				}
			}
		}

		for (int32_t i = 0; i < n_markers; ++i) {
			int32_t idx = markers[i].node_idx;
			if (idx >= 0 && idx < n_nodes) {
				markers[i].x = nodes[idx].x;
				markers[i].y = nodes[idx].y;
			}
		}
	}

	if (stats_out) *stats_out = st;
}

/* 마커 로그만으로 규칙 기반 맵 복원 (실차 SD 맵 로그 / 시뮬 driveN_markers.csv). */
int32_t MapCalib_RuleFromLog(const MapLogMark *marks, int32_t n, uint8_t closed_loop,
                             int32_t *seg_k45, float *seg_len, MapCalibStats *stats_out) {
	static MapCalibNode nodes[RM_MAX_SEG + 1];
	static MapCalibMarker mk[RM_MAX_SEG];
	MapCalibStats st = {0.0f, 1.0f, 0.0f, 0, 0};
	if (!marks || n < 2 || n > RM_MAX_SEG) return 0;
	float s = 0.0f;
	memset(&nodes[0], 0, sizeof nodes[0]);
	for (int32_t j = 0; j < n; ++j) {
		const float d = (marks[j].dist_m > 0.0f) ? marks[j].dist_m : 0.0f;
		s += d;
		memset(&nodes[j + 1], 0, sizeof nodes[j + 1]);
		nodes[j + 1].ds = d;
		nodes[j + 1].s = s;
		memset(&mk[j], 0, sizeof mk[j]);
		mk[j].node_idx = j + 1;
		mk[j].type = marks[j].type;
		mk[j].seg_yaw_deg = marks[j].seg_yaw_deg;
	}
	MapCalibConfig cfg;
	memset(&cfg, 0, sizeof cfg);
	cfg.rule_arc_radius = 0.25f;
	cfg.closed_loop = closed_loop;
	cfg.mark_yaw_valid = 1;
	if (!rule_zetin_map(nodes, n + 1, mk, n, &cfg, &st)) return 0;
	for (int32_t j = 0; j < n; ++j) {
		if (seg_k45) seg_k45[j] = g_rm.k45[j];
		if (seg_len) seg_len[j] = g_rm.len[j];
	}
	st.rule_map = 1;
	if (stats_out) *stats_out = st;
	return n;
}
