/************************************************
 * Q16.16 fixed-point math (backup / stage E)
 * Default path remains float; enable via USE_FIXED_POINT=1
 * Realtime path: no Serial.print / hostPrintf
 ************************************************/
#pragma once

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define Q16_16_FRAC_BITS  16
#define Q16_16_SCALE      (1 << 16)

typedef int32_t q16_t;

#define FLOAT_TO_Q16(x)   ((q16_t)(( (x) >= 0.0f) ? ((x) * (float)Q16_16_SCALE + 0.5f) \
                                                  : ((x) * (float)Q16_16_SCALE - 0.5f)))
#define Q16_TO_FLOAT(x)   ((float)(x) / (float)Q16_16_SCALE)
#define INT_TO_Q16(x)     ((q16_t)((int32_t)(x) << Q16_16_FRAC_BITS))
#define Q16_ADD(a, b)     ((q16_t)((a) + (b)))
#define Q16_SUB(a, b)     ((q16_t)((a) - (b)))

static inline q16_t q16_mul(q16_t a, q16_t b) {
  return (q16_t)(((int64_t)a * (int64_t)b) >> Q16_16_FRAC_BITS);
}

static inline q16_t q16_div(q16_t a, q16_t b) {
  if (b == 0) {
    return (a >= 0) ? INT32_MAX : INT32_MIN;
  }
  return (q16_t)(((int64_t)a << Q16_16_FRAC_BITS) / (int64_t)b);
}

static inline q16_t q16_clamp(q16_t x, q16_t lo, q16_t hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline q16_t q16_abs(q16_t x) {
  return (x < 0) ? (q16_t)(-x) : x;
}

static inline q16_t ema_update_q16(q16_t current, q16_t raw, q16_t alpha_q16) {
  q16_t one_m = Q16_SUB(INT_TO_Q16(1), alpha_q16);
  return Q16_ADD(q16_mul(one_m, current), q16_mul(alpha_q16, raw));
}

static inline q16_t calc_rpm_window_q16(int64_t dc, uint32_t dt_us, int32_t cpr) {
  if (dt_us == 0 || cpr == 0) return 0;
  int64_t num = dc * 60000000LL;
  int64_t den = (int64_t)cpr * (int64_t)dt_us;
  if (den == 0) return 0;
  int64_t rpm_q16 = (num << Q16_16_FRAC_BITS) / den;
  if (rpm_q16 > INT32_MAX) return INT32_MAX;
  if (rpm_q16 < INT32_MIN) return INT32_MIN;
  return (q16_t)rpm_q16;
}

typedef struct {
  q16_t kp, ki, kd;
  q16_t i_acc;
  q16_t i_lim;
  q16_t out_lo, out_hi;
  q16_t prev_err;
} pid_q16_t;

static inline void pid_q16_reset(pid_q16_t* p) {
  if (!p) return;
  p->i_acc = 0;
  p->prev_err = 0;
}

static inline q16_t pid_update_q16(pid_q16_t* p, q16_t target, q16_t feedback, q16_t dt_q16) {
  if (!p) return 0;
  q16_t err = Q16_SUB(target, feedback);
  q16_t p_term = q16_mul(p->kp, err);
  p->i_acc = q16_clamp(Q16_ADD(p->i_acc, q16_mul(q16_mul(p->ki, err), dt_q16)),
                       Q16_SUB(0, p->i_lim), p->i_lim);
  q16_t d_term = 0;
  if (dt_q16 != 0) {
    d_term = q16_mul(p->kd, q16_div(Q16_SUB(err, p->prev_err), dt_q16));
  }
  p->prev_err = err;
  return q16_clamp(Q16_ADD(Q16_ADD(p_term, p->i_acc), d_term), p->out_lo, p->out_hi);
}

#ifdef __cplusplus
}
#endif