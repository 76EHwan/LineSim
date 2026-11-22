/*
 * MCU-optimized 2D vector map calibration (gyro drift, wheel-slip scale,
 * closed-loop closure, and crossing-pair relaxation).
 *
 * Pure C99, single-precision float, zero dynamic allocation (heap-free) so it
 * can run directly on an STM32H7 linetracer MCU at the END mark in < 0.5 ms.
 */
#ifndef LINESIM_MAP_CALIB_H
#define LINESIM_MAP_CALIB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float ds;   /* odometry step length [m] */
	float th;   /* raw unwrapped heading [rad] */
	float s;    /* cumulative odometry distance [m] */
	float x;    /* calibrated line x [m] (output) */
	float y;    /* calibrated line y [m] (output) */
} MapCalibNode;

typedef struct {
	int32_t node_idx;   /* index into nodes[] */
	int32_t type;       /* 1:L, 2:R, 3:C, 4:U */
	float seg_yaw_deg;  /* firmware yaw change since the previous mark [deg] (input, rule map) */
	float seg_dist_m;   /* > 0: segment length to use instead of the node odometry (fused drives) */
	float x;            /* calibrated marker x [m] (output) */
	float y;            /* calibrated marker y [m] (output) */
} MapCalibMarker;

typedef struct {
	float start_x;          /* initial axle x [m] */
	float start_y;          /* initial axle y [m] */
	float start_th;         /* initial heading [rad] */
	float sensor_x;         /* axle -> sensor bar offset [m] */
	float rule_arc_radius;  /* ZETIN line arc radius [m] (0.25f; <= 0 disables scale fit) */
	uint8_t closed_loop;    /* 1 if robot stopped at END mark with matching heading */
	uint8_t path_only;      /* 0: map reconstruction (45-deg snap), 1: path reconstruction */
	uint8_t mark_yaw_valid; /* 1: markers[].seg_yaw_deg holds the firmware log (rule-based 1st-run map) */
} MapCalibConfig;

typedef struct {
	float gyro_drift_deg;   /* removed total heading drift [deg] */
	float odom_scale;       /* estimated wheel-slip scale factor */
	float closure_lat_mm;   /* residual lateral closure error before relaxation [mm] */
	int32_t crossing_pairs; /* number of paired C-markers used for relaxation */
	uint8_t rule_map;       /* 1: rule-based ZETIN map from the marker log was used */
} MapCalibStats;

/* One entry of a marker log (real-robot SD map log or sim driveN_markers.csv). */
typedef struct {
	int32_t type;        /* 1:L, 2:R, 3:C, 4:STOP */
	float dist_m;        /* odometry since the previous mark [m] */
	float seg_yaw_deg;   /* firmware yaw change since the previous mark [deg] */
} MapLogMark;

/*
 * Calibrates `nodes[0..n_nodes-1]` and `markers[0..n_markers-1]` in place.
 * Complexity: O(n_nodes + n_markers * crossing_pairs), zero heap allocation.
 */
void MapCalib_Run(MapCalibNode *nodes, int32_t n_nodes,
                  MapCalibMarker *markers, int32_t n_markers,
                  const MapCalibConfig *cfg, MapCalibStats *stats_out);

/*
 * Rule-based ZETIN map from a marker log alone. Segment j ends at mark j (segment 0: robot
 * start -> first mark). seg_k45[j]: 0 = straight, +-k = k * 45 deg arc (R 0.25 m, + = left);
 * seg_len[j]: line length [m]. The line starts at the robot's sensor bar. closed_loop: the run
 * ended on the goal mark of a closed course (lateral closure constraint). Returns n or 0.
 */
int32_t MapCalib_RuleFromLog(const MapLogMark *marks, int32_t n, uint8_t closed_loop,
                             int32_t *seg_k45, float *seg_len, MapCalibStats *stats_out);

#ifdef __cplusplus
}
#endif

#endif /* LINESIM_MAP_CALIB_H */
