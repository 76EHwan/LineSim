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
} MapCalibConfig;

typedef struct {
	float gyro_drift_deg;   /* removed total heading drift [deg] */
	float odom_scale;       /* estimated wheel-slip scale factor */
	float closure_lat_mm;   /* residual lateral closure error before relaxation [mm] */
	int32_t crossing_pairs; /* number of paired C-markers used for relaxation */
} MapCalibStats;

/*
 * Calibrates `nodes[0..n_nodes-1]` and `markers[0..n_markers-1]` in place.
 * Complexity: O(n_nodes + n_markers * crossing_pairs), zero heap allocation.
 */
void MapCalib_Run(MapCalibNode *nodes, int32_t n_nodes,
                  MapCalibMarker *markers, int32_t n_markers,
                  const MapCalibConfig *cfg, MapCalibStats *stats_out);

#ifdef __cplusplus
}
#endif

#endif /* LINESIM_MAP_CALIB_H */
