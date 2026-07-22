/************************************************
 * 板载轻量整定记录（TUNE）：仅写 RAM，实时路径禁打印。
 * 供闭环 computePulseClosed 推样；TUNE?/TEST DONE 再摘要。
 ************************************************/
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifndef TUNE_RING_N
#define TUNE_RING_N 200  // ~5s @40Hz（400Hz 控制每 10 拍记 1 点）
#endif
#ifndef TUNE_DECIMATE
#define TUNE_DECIMATE 10
#endif

struct TuneSample {
  float err;
  float ff;
  float u;
  float rpm;
  float target;
  float pid_i;
  uint16_t pulse;
  uint8_t i_sat;  // |pid_i| >= 0.95*i_lim
};

struct TuneStats {
  bool rec_on;
  uint32_t n;           // 累计推样（含未入环的统计拍）
  uint32_t ring_n;      // 环内有效点数
  float sum_err;
  float sum_u;
  float sum_ff;
  float sum_pulse;
  float sum_rpm;
  float sum_tgt;
  float sum_pid_i;
  uint32_t i_sat_n;
  float last_err;
  float last_ff;
  float last_u;
  float last_rpm;
  float last_tgt;
  float last_pid_i;
  float last_i_lim;
  uint16_t last_pulse;
  float map_rpm_max;    // 调用方写入：前馈图最高 rpm
};

static TuneSample g_tune_ring[TUNE_RING_N];
static uint16_t g_tune_w = 0;
static uint16_t g_tune_fill = 0;
static uint8_t g_tune_div = 0;
static TuneStats g_tune = {};

inline void tuneRecReset(void) {
  g_tune_w = 0;
  g_tune_fill = 0;
  g_tune_div = 0;
  float map_keep = g_tune.map_rpm_max;
  bool on = g_tune.rec_on;
  memset(&g_tune, 0, sizeof(g_tune));
  g_tune.rec_on = on;
  g_tune.map_rpm_max = map_keep;
}

inline void tuneRecSet(bool on) {
  g_tune.rec_on = on;
  if (on) tuneRecReset();
}

inline bool tuneRecOn(void) { return g_tune.rec_on; }

inline void tuneSetMapRpmMax(float rpm_max) { g_tune.map_rpm_max = rpm_max; }

/** 实时路径调用：只写 RAM，无打印。decimate 后入环，每拍更新均值累加。 */
inline void tuneRecPush(float err, float ff, float u, uint16_t pulse,
                        float rpm, float target, float pid_i, float i_lim) {
  if (!g_tune.rec_on) return;
  const bool sat = (i_lim > 1.0f) && (fabsf(pid_i) >= 0.95f * i_lim);
  g_tune.n++;
  g_tune.sum_err += err;
  g_tune.sum_u += u;
  g_tune.sum_ff += ff;
  g_tune.sum_pulse += (float)pulse;
  g_tune.sum_rpm += rpm;
  g_tune.sum_tgt += target;
  g_tune.sum_pid_i += pid_i;
  if (sat) g_tune.i_sat_n++;
  g_tune.last_err = err;
  g_tune.last_ff = ff;
  g_tune.last_u = u;
  g_tune.last_rpm = rpm;
  g_tune.last_tgt = target;
  g_tune.last_pid_i = pid_i;
  g_tune.last_i_lim = i_lim;
  g_tune.last_pulse = pulse;

  if (++g_tune_div < TUNE_DECIMATE) return;
  g_tune_div = 0;
  TuneSample& s = g_tune_ring[g_tune_w];
  s.err = err;
  s.ff = ff;
  s.u = u;
  s.rpm = rpm;
  s.target = target;
  s.pid_i = pid_i;
  s.pulse = pulse;
  s.i_sat = sat ? 1 : 0;
  g_tune_w = (uint16_t)((g_tune_w + 1) % TUNE_RING_N);
  if (g_tune_fill < TUNE_RING_N) g_tune_fill++;
  g_tune.ring_n = g_tune_fill;
}

inline const TuneStats& tuneStats(void) { return g_tune; }

inline uint16_t tuneRingCount(void) { return g_tune_fill; }

/** 按时间顺序取环内样本（oldest→newest）。idx 0..count-1 */
inline bool tuneRingAt(uint16_t idx, TuneSample* out) {
  if (!out || idx >= g_tune_fill) return false;
  uint16_t base = (g_tune_fill < TUNE_RING_N) ? 0 : g_tune_w;
  uint16_t i = (uint16_t)((base + idx) % TUNE_RING_N);
  *out = g_tune_ring[i];
  return true;
}

/**
 * 填一句建议到 buf（≤hint_cap-1）。
 * 返回建议码：0=ok 1=ff_short 2=i_sat 3=ki_small 4=need_data
 */
inline int tuneHint(char* buf, size_t hint_cap) {
  if (!buf || hint_cap < 8) return 4;
  buf[0] = '\0';
  const TuneStats& t = g_tune;
  if (t.n < 20) {
    snprintf(buf, hint_cap, "need_more_samples");
    return 4;
  }
  const float inv = 1.0f / (float)t.n;
  const float mean_err = t.sum_err * inv;
  const float mean_u = t.sum_u * inv;
  const float mean_tgt = t.sum_tgt * inv;
  const float sat_frac = (float)t.i_sat_n * inv;
  const float map_max = t.map_rpm_max;
  const float abs_err = fabsf(mean_err);
  const float err_frac = (mean_tgt > 100.0f) ? (abs_err / mean_tgt) : 0.0f;

  if (map_max > 1.0f && mean_tgt > map_max * 1.02f) {
    snprintf(buf, hint_cap,
             "FF_SHORT map_max=%.0f<tgt=%.0f -> LEARN to >=8000",
             (double)map_max, (double)mean_tgt);
    return 1;
  }
  if (sat_frac >= 0.40f && err_frac >= 0.02f) {
    snprintf(buf, hint_cap,
             "I_SAT frac=%.2f i=%.0f/%.0f u=%.1f -> raise i_lim or LEARN",
             (double)sat_frac, (double)t.last_pid_i, (double)t.last_i_lim,
             (double)mean_u);
    return 2;
  }
  if (err_frac >= 0.03f && fabsf(mean_u) < 15.0f) {
    snprintf(buf, hint_cap, "Ki_SMALL mean_err=%.0f mean_u=%.1f -> Ki up or LEARN",
             (double)mean_err, (double)mean_u);
    return 3;
  }
  if (err_frac < 0.02f) {
    snprintf(buf, hint_cap, "OK mean_err=%.1f (%.2f%%)",
             (double)mean_err, (double)(err_frac * 100.0f));
    return 0;
  }
  snprintf(buf, hint_cap, "CHECK mean_err=%.0f u=%.1f ff=%.0f (prefer LEARN if FF thin)",
           (double)mean_err, (double)mean_u, (double)(t.sum_ff * inv));
  return 1;
}
