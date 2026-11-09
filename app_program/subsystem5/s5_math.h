/*
 *      s5_math.h
 *      Subsystem 5 -- the few float functions the bypass needs (nyx,
 *      2026-11-07). The firmware is linked without libm, so sin, cos,
 *      atan2 and sqrt are done here. Accurate to about 1e-5, plenty for
 *      millimetres over a metre. Pure: the host tests use the same code.
 */

#ifndef S5_MATH_H
#define S5_MATH_H

#include <stdint.h>

#define S5_PI_F      (3.14159265f)
#define S5_DEG2RAD_F (S5_PI_F / 180.0f)
#define S5_RAD2DEG_F (180.0f / S5_PI_F)

float s5_absf(float x);
float s5_clampf(float x, float lo, float hi);
float s5_wrapf(float a); /* to -pi .. pi */
float s5_sinf(float a);
float s5_cosf(float a);
float s5_tanf(float a); /* |a| < 85 deg */
float s5_atanf(float x);
float s5_atan2f(float y, float x);
float s5_sqrtf(float x); /* 0 for x <= 0 */
int32_t s5_roundf(float x);

#endif /* S5_MATH_H */
