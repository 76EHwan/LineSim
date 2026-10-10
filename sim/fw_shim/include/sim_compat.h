/*
 * GCC/ARM idioms used by the firmware, mapped for MSVC C.
 * Included by every C file in the firmware library (via sim_fw_prelude.h or directly).
 */
#ifndef SIM_COMPAT_H
#define SIM_COMPAT_H

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

/* Pull in every standard header the firmware uses *before* any keyword macro below,
 * so their include guards are already set when the firmware includes them. */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#define __attribute__(x)

static __inline int sim_popcount32(uint32_t v) {
	v = v - ((v >> 1) & 0x55555555u);
	v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
	return (int) ((((v + (v >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24);
}
#define __builtin_popcount(x) sim_popcount32((uint32_t)(x))

#endif /* SIM_COMPAT_H */
