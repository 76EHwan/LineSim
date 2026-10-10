/*
 * The subset of the CMSIS-DSP API used by the firmware, implemented with the
 * same equations as CMSIS-DSP (float32 PID in incremental form, Clarke/Park).
 */
#ifndef SIM_ARM_MATH_H
#define SIM_ARM_MATH_H

#include "sim_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef float float32_t;
typedef double float64_t;

#ifndef PI
#define PI 3.14159265358979f
#endif

typedef enum {
	ARM_MATH_SUCCESS = 0, ARM_MATH_ARGUMENT_ERROR = -1, ARM_MATH_LENGTH_ERROR = -2,
	ARM_MATH_SIZE_MISMATCH = -3, ARM_MATH_NANINF = -4, ARM_MATH_SINGULAR = -5, ARM_MATH_TEST_FAILURE = -6
} arm_status;

typedef struct {
	float32_t A0;        /* Kp + Ki + Kd */
	float32_t A1;        /* -Kp - 2Kd */
	float32_t A2;        /* Kd */
	float32_t state[3];  /* x[n-1], x[n-2], y[n-1] */
	float32_t Kp;
	float32_t Ki;
	float32_t Kd;
} arm_pid_instance_f32;

static __inline void arm_pid_init_f32(arm_pid_instance_f32 *S, int32_t resetStateFlag) {
	S->A0 = S->Kp + S->Ki + S->Kd;
	S->A1 = (-S->Kp) - ((float32_t) 2.0f * S->Kd);
	S->A2 = S->Kd;
	if (resetStateFlag) {
		S->state[0] = S->state[1] = S->state[2] = 0.0f;
	}
}

static __inline void arm_pid_reset_f32(arm_pid_instance_f32 *S) {
	S->state[0] = S->state[1] = S->state[2] = 0.0f;
}

static __inline float32_t arm_pid_f32(arm_pid_instance_f32 *S, float32_t in) {
	float32_t out = (S->A0 * in) + (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);
	S->state[1] = S->state[0];
	S->state[0] = in;
	S->state[2] = out;
	return out;
}

static __inline float32_t arm_sin_f32(float32_t x) { return sinf(x); }
static __inline float32_t arm_cos_f32(float32_t x) { return cosf(x); }

static __inline void arm_sin_cos_f32(float32_t theta_deg, float32_t *pSinVal, float32_t *pCosVal) {
	float32_t r = theta_deg * (PI / 180.0f);
	*pSinVal = sinf(r);
	*pCosVal = cosf(r);
}

static __inline arm_status arm_sqrt_f32(float32_t in, float32_t *pOut) {
	if (in >= 0.0f) { *pOut = sqrtf(in); return ARM_MATH_SUCCESS; }
	*pOut = 0.0f;
	return ARM_MATH_ARGUMENT_ERROR;
}

static __inline void arm_clarke_f32(float32_t Ia, float32_t Ib, float32_t *pIalpha, float32_t *pIbeta) {
	*pIalpha = Ia;
	*pIbeta = (0.57735026919f * Ia) + (1.15470053838f * Ib);
}

static __inline void arm_inv_clarke_f32(float32_t Ialpha, float32_t Ibeta, float32_t *pIa, float32_t *pIb) {
	*pIa = Ialpha;
	*pIb = (-0.5f * Ialpha) + (0.8660254039f * Ibeta);
}

static __inline void arm_park_f32(float32_t Ialpha, float32_t Ibeta, float32_t *pId, float32_t *pIq,
		float32_t sinVal, float32_t cosVal) {
	*pId = Ialpha * cosVal + Ibeta * sinVal;
	*pIq = -Ialpha * sinVal + Ibeta * cosVal;
}

static __inline void arm_inv_park_f32(float32_t Id, float32_t Iq, float32_t *pIalpha, float32_t *pIbeta,
		float32_t sinVal, float32_t cosVal) {
	*pIalpha = Id * cosVal - Iq * sinVal;
	*pIbeta = Id * sinVal + Iq * cosVal;
}

#ifdef __cplusplus
}
#endif
#endif /* SIM_ARM_MATH_H */
