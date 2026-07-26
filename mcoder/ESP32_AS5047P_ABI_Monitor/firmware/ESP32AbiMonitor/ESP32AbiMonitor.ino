/************************************************
 * ESP32-S3 + AS5047P ABI 转速监控（独立工程）
 *
 * 监控 ON 后状态机（弹射场景：可先静止武装，电机突然转起来再记）：
 *   IDLE  → |RPM|>20 进入 STAGING，样本写入「临时数据池」
 *   STAGING → 短窗内净转角达标：真实转动 → 向前追最近 BACKTRACK_N 点写入主记录，
 *             再继续记 REC_DURATION_MS（默认 1s），然后本段结束
 *           → 噪声（掉速/反向抖动/超时未确认）：丢弃数据池，回 IDLE
 *   RECORD → 已确认，写主记录，满时长自动停本段
 *
 * 板内主记录：多段同一缓冲，用 seg 区分；PC 可单文件或按段拆文件。
 ************************************************/
#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "ble_nus.h"
#include "cap_long.h"
#include "snap_bin.h"
#include "sd_card.h"
#include "sd_usb_msc.h"

static const int PIN_A = 15;
static const int PIN_B = 16;
static const int PIN_I = 17;
static const int ABI_STEPS_PER_REV = 4000;

static const uint32_t SAMPLE_HZ = 2000;  // 2kHz → 记 1s ≈ 2000 点
static const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_HZ;
static uint32_t g_rec_duration_ms = 1000;  // 正式段默认 1s@2kHz（弹射够用；可用 REC MS 改）
static const float RPM_GATE = 10.0f;       // |RPM|>10 且 I 过 1 圈 → 触发
static const uint16_t BACKTRACK_N = 400;   // 触发时向前保留 400 点
static const uint32_t NOISE_BELOW_MS = 80;       // 旧路径残留
static const uint32_t STAGING_TIMEOUT_MS = 8000; // 旧路径残留
static const int64_t CONFIRM_COUNTS = (int64_t)ABI_STEPS_PER_REV;  // 旧路径残留
static const int VEL_WINDOW = 8;

// 临时池：回溯+确认窗；容量略留余量。放堆/PSRAM，避免 .bss 撑爆 DRAM
static const size_t POOL_CAP = 2000;

typedef struct __attribute__((packed)) {
  uint32_t t_ms;     // 板载 millis（绝对时间 = session_unix_t0 + (t_ms - board_t0)）
  int16_t rpm_x10;
  uint16_t seg;
  uint32_t index_n;  // I 过零累计圈数
} LogSample;

/** 每次 I 上升沿（一转过零）一条，供回放分析转速变化规律 */
typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  uint32_t dt_ms;    // 距上次 I 的间隔；0=本段首脉冲
  uint32_t index_n;  // 本沿之后的累计圈数
  int16_t rpm_x10;   // 当时 A/B 滤波转速
  int16_t rpm_i_x10; // 由 dt 推算：60000/dt（带符号）
  uint16_t seg;
} IndexEvent;

enum RecPhase : uint8_t {
  PH_IDLE = 0,
  PH_STAGING = 1,   // 临时池
  PH_RECORD = 2,    // 已确认，主记录
};

static const size_t LOG_CAP_PSRAM = 300000;      // 给 CAPTURE 留 PSRAM（原 60 万太大）
static const size_t LOG_CAP_PSRAM_MID = 180000;
static const size_t LOG_CAP_INTERNAL = 6000;
static const size_t ILOG_CAP = 4000;   // I 事件主缓冲
static const size_t IPOOL_CAP = 2000;   // staging 临时 I 池
static LogSample* g_log = nullptr;
static size_t g_log_cap = 0;
static volatile size_t g_log_w = 0;
static volatile size_t g_log_n = 0;
static volatile uint32_t g_log_drop = 0;
static bool g_log_in_psram = false;
static portMUX_TYPE g_log_mux = portMUX_INITIALIZER_UNLOCKED;

static LogSample* g_pool = nullptr;
static size_t g_pool_n = 0;

static IndexEvent* g_ilog = nullptr;
static size_t g_ilog_cap = 0;
static size_t g_ilog_w = 0;
static size_t g_ilog_n = 0;
static uint32_t g_ilog_drop = 0;
static IndexEvent* g_ipool = nullptr;
static size_t g_ipool_n = 0;
static uint32_t g_index_evt_seen = 0;  // 已写成 I 事件的 irq 计数
static uint32_t g_last_i_ms = 0;       // 上次 I 沿时间（算 dt）
static portMUX_TYPE g_ilog_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool g_armed = false;  // MONITOR START 武装探测
static volatile RecPhase g_phase = PH_IDLE;
static volatile uint16_t g_seg_cur = 0;
static volatile uint16_t g_seg_count = 0;
static volatile uint32_t g_stage_start_ms = 0;
static volatile uint32_t g_rec_start_ms = 0;  // 确认后计时
static volatile int64_t g_stage_counts0 = 0;
static volatile uint32_t g_stage_index0 = 0;  // STAGING 起点 I 圈数；+1 确认
static volatile int g_stage_sign = 0;  // 本段主方向 +1/-1
static volatile uint32_t g_below_ms = 0;
static volatile uint32_t g_flip_cnt = 0;

static volatile bool g_evt_pending = false;
static volatile uint8_t g_evt_code = 0;  // 1=confirm 2=noise 3=done 4=pool_ovf
static volatile uint8_t g_noise_why = 0; // 2=flip 3=below 4=timeout 5=ovf
static volatile uint16_t g_evt_seg = 0;
static volatile uint32_t g_evt_n = 0;
static volatile float g_evt_revs = 0;
static volatile bool g_dumping = false;  // dump 中暂停遥测，避免冲乱 D/I 行
static volatile bool g_sample_freeze = false;  // RECORD 结束后立刻停采样算 FPU，防导出前复位
static volatile bool g_need_auto_dump = false;
static volatile uint16_t g_auto_dump_seg = 0;
static volatile uint32_t g_auto_dump_expect = 0;
static uint32_t g_ready_ms = 0;           // READY 发出时刻；超时未拉则板子自推
static volatile bool g_dump_started = false;

// 时间锚点：手机连接时建立；TIME <unix_ms> 对时后可算绝对时间
static uint32_t g_session_board_t0 = 0;
static uint64_t g_session_unix_t0 = 0;  // 0=未对时
static bool g_time_synced = false;

static pcnt_unit_handle_t g_pcnt = nullptr;
static esp_timer_handle_t g_timer = nullptr;

static portMUX_TYPE g_snap_mux = portMUX_INITIALIZER_UNLOCKED;
struct Snap {
  int64_t counts;
  float rpm;
  int8_t dir;
  uint32_t t_ms;
  uint32_t seq;
  uint32_t sample_hz_meas;
  float revs_total;   // A/B 有符号净圈数
  float revs_abs;     // A/B 路程圈数
  uint32_t index_n;   // I 过零次数 = I 计圈（绝对值）
  int32_t index_signed;  // I 有符号圈数（按过零时方向累加）
};
static Snap g_snap = {};

static int64_t cb_counts_base = 0;
static int64_t g_counts_zero = 0;      // REVS CLEAR 零点
static double g_revs_abs_acc = 0.0;    // 绝对路程圈数
static int64_t g_abs_counts_last = 0;
static bool g_abs_inited = false;
static int64_t cb_hist_c[VEL_WINDOW];
static uint32_t cb_hist_us[VEL_WINDOW];
static int cb_hist_i = 0;
static int cb_hist_n = 0;
static float cb_rpm_filt = 0.0f;
static uint32_t cb_n = 0;
static uint32_t cb_last_diag_ms = 0;
static uint32_t meas_hz = 0;
static volatile uint32_t g_index_irq = 0;       // I 上升沿累计（=过零圈数）
static volatile int32_t g_index_signed = 0;     // 有符号 I 圈数
static uint32_t g_index_seen = 0;               // sampleCb 已消费的 irq 计数
static volatile float g_rpm_for_index = 0.0f;   // ISR 读方向用

static void IRAM_ATTR onIndexIsr(void*) {
  g_index_irq++;
  // 按当前转速符号记有符号圈
  if (g_rpm_for_index >= 0.0f) g_index_signed++;
  else g_index_signed--;
}

static void revsClear() {
  int raw = 0;
  if (g_pcnt) pcnt_unit_get_count(g_pcnt, &raw);
  int64_t counts = cb_counts_base + (int64_t)raw;
  g_counts_zero = counts;
  g_revs_abs_acc = 0.0;
  g_abs_counts_last = counts;
  g_abs_inited = true;
  noInterrupts();
  g_index_irq = 0;
  g_index_signed = 0;
  interrupts();
  g_index_seen = 0;
  g_index_evt_seen = 0;
  g_last_i_ms = 0;
}

static uint32_t sessionRel(uint32_t t_ms) {
  if (g_session_board_t0 == 0) return 0;
  return t_ms - g_session_board_t0;
}

static uint64_t sampleUnixMs(uint32_t t_ms) {
  if (!g_time_synced || g_session_unix_t0 == 0) return 0;
  return g_session_unix_t0 + (uint64_t)sessionRel(t_ms);
}

static void sessionBegin(const char* why) {
  g_session_board_t0 = millis();
  // 未收到 TIME 前 unix 用 0；手机连上后会发 TIME
  if (!g_time_synced) g_session_unix_t0 = 0;
  hostPrintf("# SESSION %s board_t0=%lu unix_t0=%llu synced=%d\n", why,
             (unsigned long)g_session_board_t0, (unsigned long long)g_session_unix_t0,
             g_time_synced ? 1 : 0);
}

/** 当前墙钟（PC 对时后）；未对时返回 0 */
static uint64_t wallUnixMsNow() { return sampleUnixMs(millis()); }

/** 格式化为本地 CST 文件名时间戳：YYYYMMDD_HHMMSS；未对时用 b<millis> */
static bool formatWallStamp(char* out, size_t n, uint64_t unix_ms) {
  if (!out || n < 16) return false;
  if (unix_ms == 0) {
    snprintf(out, n, "b%lu", (unsigned long)millis());
    return false;
  }
  time_t sec = (time_t)(unix_ms / 1000ULL);
  setenv("TZ", "CST-8", 1);  // 东八区，无夏令时
  tzset();
  struct tm t;
  if (!localtime_r(&sec, &t)) {
    snprintf(out, n, "u%llu", (unsigned long long)(unix_ms / 1000ULL));
    return false;
  }
  snprintf(out, n, "%04d%02d%02d_%02d%02d%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  return true;
}

static void applyHostTime(uint64_t unix_ms) {
  // 对齐到既有会话锚点：unix_t0 + (now - board_t0) ≈ 手机/PC 当前时刻
  if (g_session_board_t0 == 0) g_session_board_t0 = millis();
  uint32_t rel = millis() - g_session_board_t0;
  g_session_unix_t0 = (unix_ms > rel) ? (unix_ms - (uint64_t)rel) : unix_ms;
  g_time_synced = true;
  char stamp[24];
  formatWallStamp(stamp, sizeof(stamp), wallUnixMsNow());
  hostPrintf("# TIME ok stamp=%s unix_now=%llu board_t0=%lu (SD filenames use this clock)\n",
             stamp, (unsigned long long)wallUnixMsNow(), (unsigned long)g_session_board_t0);
}

static bool poolAlloc() {
  if (g_pool) return true;
  size_t bytes = POOL_CAP * sizeof(LogSample);
  // 定时回调会写 pool：只用内部 RAM（写 PSRAM 易异常）
  void* p = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!p) p = malloc(bytes);
  g_pool = (LogSample*)p;
  return g_pool != nullptr;
}

static bool ilogAlloc() {
  if (g_ilog && g_ipool) return true;
  size_t ib = ILOG_CAP * sizeof(IndexEvent);
  void* p = heap_caps_malloc(ib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!p) p = malloc(ib);
  g_ilog = (IndexEvent*)p;
  g_ilog_cap = g_ilog ? ILOG_CAP : 0;
  g_ilog_w = 0;
  g_ilog_n = 0;
  g_ilog_drop = 0;

  size_t pb = IPOOL_CAP * sizeof(IndexEvent);
  void* q = heap_caps_malloc(pb, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!q) q = malloc(pb);
  g_ipool = (IndexEvent*)q;
  g_ipool_n = 0;
  return g_ilog != nullptr && g_ipool != nullptr;
}

static void ipoolClear() { g_ipool_n = 0; }

static void ilogClear() {
  portENTER_CRITICAL(&g_ilog_mux);
  g_ilog_w = 0;
  g_ilog_n = 0;
  g_ilog_drop = 0;
  portEXIT_CRITICAL(&g_ilog_mux);
  ipoolClear();
  g_last_i_ms = 0;
  g_index_evt_seen = g_index_irq;
}

static void ilogPushEvt(const IndexEvent& e) {
  if (!g_ilog || g_ilog_cap == 0) return;
  portENTER_CRITICAL(&g_ilog_mux);
  g_ilog[g_ilog_w] = e;
  g_ilog_w = (g_ilog_w + 1) % g_ilog_cap;
  if (g_ilog_n < g_ilog_cap) g_ilog_n++;
  else g_ilog_drop++;
  portEXIT_CRITICAL(&g_ilog_mux);
}

static void ipoolPushEvt(const IndexEvent& e) {
  if (!g_ipool || g_ipool_n >= IPOOL_CAP) return;
  g_ipool[g_ipool_n++] = e;
}

static void ipoolCommitToIlog() {
  for (size_t i = 0; i < g_ipool_n; ++i) ilogPushEvt(g_ipool[i]);
  ipoolClear();
}

static int16_t rpmToX10(float rpm) {
  float r = rpm;
  if (r > 3276.0f) r = 3276.0f;
  if (r < -3276.0f) r = -3276.0f;
  return (int16_t)lroundf(r * 10.0f);
}

/** 采样中检测 I 跳动：STAGING→临时池，RECORD→主 I 日志 */
static void captureIndexEdges(uint32_t now_ms, float rpm, uint32_t index_n) {
  if (index_n <= g_index_evt_seen) {
    g_index_evt_seen = index_n;  // 允许清零后回绕对齐
    return;
  }
  if (g_phase != PH_STAGING && g_phase != PH_RECORD) {
    g_index_evt_seen = index_n;
    g_last_i_ms = now_ms;
    return;
  }

  // 通常每采样最多 1 沿；高速时可能跨多沿
  uint32_t n_edges = index_n - g_index_evt_seen;
  if (n_edges > 8) n_edges = 8;  // 防护
  for (uint32_t k = 0; k < n_edges; ++k) {
    uint32_t this_n = g_index_evt_seen + 1 + k;
    uint32_t dt = 0;
    if (g_last_i_ms != 0 && now_ms >= g_last_i_ms) dt = now_ms - g_last_i_ms;
    g_last_i_ms = now_ms;

    float rpm_i = 0.0f;
    if (dt > 0) {
      rpm_i = 60000.0f / (float)dt;
      if (rpm < 0.0f) rpm_i = -rpm_i;
    }

    IndexEvent e;
    e.t_ms = now_ms;
    e.dt_ms = dt;
    e.index_n = this_n;
    e.rpm_x10 = rpmToX10(rpm);
    e.rpm_i_x10 = rpmToX10(rpm_i);
    e.seg = g_seg_cur;

    if (g_phase == PH_STAGING) ipoolPushEvt(e);
    else ilogPushEvt(e);
  }
  g_index_evt_seen = index_n;
}

static bool logAlloc() {
  if (g_log) return true;
  Serial.printf("# logAlloc INTERNAL-only (spiram_free=%u internal_free=%u)\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  // 短记录放 DRAM：定时器写 PSRAM + USB DUMP 易崩（本次 log_n=37 后 DUMP 复位）
  size_t cap = LOG_CAP_INTERNAL;
  void* p = heap_caps_malloc(cap * sizeof(LogSample), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  g_log_in_psram = false;
  if (!p) {
    cap = 4000;
    p = malloc(cap * sizeof(LogSample));
  }
  if (!p) return false;
  g_log = (LogSample*)p;
  g_log_cap = cap;
  g_log_w = 0;
  g_log_n = 0;
  g_log_drop = 0;
  return true;
}

static void logClear() {
  portENTER_CRITICAL(&g_log_mux);
  g_log_w = 0;
  g_log_n = 0;
  g_log_drop = 0;
  portEXIT_CRITICAL(&g_log_mux);
  g_seg_count = 0;
  g_seg_cur = 0;
  ilogClear();
  revsClear();
}

static void logPushMain(uint32_t t_ms, float rpm, uint16_t seg, uint32_t index_n) {
  if (!g_log || g_log_cap == 0) return;
  LogSample s;
  s.t_ms = t_ms;
  s.seg = seg;
  s.rpm_x10 = rpmToX10(rpm);
  s.index_n = index_n;
  portENTER_CRITICAL(&g_log_mux);
  g_log[g_log_w] = s;
  g_log_w = (g_log_w + 1) % g_log_cap;
  if (g_log_n < g_log_cap) g_log_n++;
  else g_log_drop++;
  portEXIT_CRITICAL(&g_log_mux);
}

static void poolClear() { g_pool_n = 0; }

static void poolPush(uint32_t t_ms, float rpm, uint16_t seg, uint32_t index_n) {
  if (!g_pool) return;
  // 池将满：提交已有数据，勿清空扔掉
  if (g_pool_n >= POOL_CAP) {
    g_evt_seg = g_seg_cur;
    poolCommitToMain();
    g_evt_n = (uint32_t)g_log_n;
    g_phase = PH_IDLE;
    g_seg_cur = 0;
    g_armed = false;
    bleMuteHost(false);
    g_noise_why = 5;
    g_evt_code = 5;
    g_evt_pending = true;
    // 不永久 freeze；等 DUMP 时再停 timer
    return;
  }
  g_pool[g_pool_n].t_ms = t_ms;
  g_pool[g_pool_n].rpm_x10 = rpmToX10(rpm);
  g_pool[g_pool_n].seg = seg;
  g_pool[g_pool_n].index_n = index_n;
  g_pool_n++;
}

/** 真实转动：向前追最近 BACKTRACK_N 点写入主记录（池不足则全交） */
static void poolCommitToMain() {
  if (!g_pool) {
    poolClear();
    return;
  }
  size_t start = 0;
  size_t n = g_pool_n;
  if (n > BACKTRACK_N) {
    start = n - BACKTRACK_N;
    n = BACKTRACK_N;
  }
  for (size_t i = 0; i < n; ++i) {
    LogSample s = g_pool[start + i];
    portENTER_CRITICAL(&g_log_mux);
    g_log[g_log_w] = s;
    g_log_w = (g_log_w + 1) % g_log_cap;
    if (g_log_n < g_log_cap) g_log_n++;
    else g_log_drop++;
    portEXIT_CRITICAL(&g_log_mux);
  }
  g_evt_n = (uint32_t)n;
  poolClear();
  ipoolCommitToIlog();  // staging 期间的 I 跳动一并追溯提交
}

static void discardStaging(uint8_t why) {
  poolClear();
  ipoolClear();
  g_phase = PH_IDLE;
  g_seg_cur = 0;
  g_below_ms = 0;
  g_flip_cnt = 0;
  g_evt_code = 2;
  g_noise_why = why;
  g_evt_pending = true;
  bleMuteHost(false);  // 丢弃后恢复 BLE 状态通道
}

/** 停转/超时/翻转：有数据则提交并进入 RECORD 记满时长（停转也继续记，不删）；无数据才丢。 */
static void salvageStagingOrDiscard(uint8_t why, uint32_t now_ms, int64_t net) {
  // 弹射很短：只要进过 STAGING 且有一点转动痕迹就保留并记满
  const bool keep = (g_pool_n >= 8) || (net >= 8) || (net >= (CONFIRM_COUNTS / 8));
  if (!keep) {
    g_evt_seg = g_seg_cur;
    g_evt_n = (uint32_t)((net > 0) ? net : (int64_t)g_pool_n);
    discardStaging(why);
    return;
  }
  // 有实质数据：提交临时池 → 继续记满 REC 时长（停转也采完既定窗口）
  g_evt_revs = (float)net / (float)ABI_STEPS_PER_REV;
  g_evt_seg = g_seg_cur;
  poolCommitToMain();
  g_evt_n = (uint32_t)g_log_n;
  g_phase = PH_RECORD;
  g_rec_start_ms = now_ms;
  g_below_ms = 0;
  g_flip_cnt = 0;
  g_noise_why = why;
  g_evt_code = 1;  // 当作确认，进入 RECORD
  g_evt_pending = true;
  // 保持 armed，记满 g_rec_duration_ms 后再停（期间 RPM=0 也写点）
}

static void beginStaging(uint32_t now_ms, int64_t counts, float rpm, uint32_t index_n) {
  bleMuteHost(true);  // 记录窗口内禁止 host*→BLE（ISR 安全：仅置位）
  g_seg_count = (uint16_t)(g_seg_count + 1);
  g_seg_cur = g_seg_count;
  g_phase = PH_STAGING;
  g_stage_start_ms = now_ms;
  g_stage_counts0 = counts;
  g_stage_sign = (rpm >= 0.0f) ? 1 : -1;
  g_below_ms = 0;
  g_flip_cnt = 0;
  poolClear();
  ipoolClear();
  g_last_i_ms = 0;  // 本段第一个 I：dt=0
  poolPush(now_ms, rpm, g_seg_cur, index_n);
}

static bool pcntBegin() {
  pcnt_unit_config_t ucfg = {};
  ucfg.high_limit = 30000;
  ucfg.low_limit = -30000;
  ucfg.flags.accum_count = true;
  if (pcnt_new_unit(&ucfg, &g_pcnt) != ESP_OK) return false;
  pcnt_glitch_filter_config_t filter = {};
  filter.max_glitch_ns = 1000;
  pcnt_unit_set_glitch_filter(g_pcnt, &filter);

  pcnt_chan_config_t ch_a = {};
  ch_a.edge_gpio_num = PIN_A;
  ch_a.level_gpio_num = PIN_B;
  pcnt_channel_handle_t chan_a = nullptr;
  if (pcnt_new_channel(g_pcnt, &ch_a, &chan_a) != ESP_OK) return false;
  pcnt_chan_config_t ch_b = {};
  ch_b.edge_gpio_num = PIN_B;
  ch_b.level_gpio_num = PIN_A;
  pcnt_channel_handle_t chan_b = nullptr;
  if (pcnt_new_channel(g_pcnt, &ch_b, &chan_b) != ESP_OK) return false;

  pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                               PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                               PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_unit_enable(g_pcnt);
  pcnt_unit_clear_count(g_pcnt);
  pcnt_unit_start(g_pcnt);
  pinMode(PIN_I, INPUT_PULLUP);
  // I：每转一圈一个脉冲；用上升沿，避免 CHANGE 计成 2
  attachInterruptArg(PIN_I, onIndexIsr, nullptr, RISING);
  return true;
}

static void sampleCb(void* arg) {
  (void)arg;
  // 必须在任何 float 之前返回：主循环 dump/printf 与定时器 FPU 并发会 Coprocessor 复位
  if (g_sample_freeze || g_dumping) return;
  uint32_t t0 = micros();
  int raw = 0;
  if (g_pcnt) pcnt_unit_get_count(g_pcnt, &raw);
  if (raw > 20000 || raw < -20000) {
    cb_counts_base += raw;
    pcnt_unit_clear_count(g_pcnt);
    raw = 0;
  }
  int64_t counts = cb_counts_base + (int64_t)raw;
  uint32_t now_us = t0;
  uint32_t now_ms = millis();

  // 正式 SNAP 记录中：仍更新本地转速窗（将来电机控制用）；BIN 只存 counts
  if (snapIsRecording()) {
    uint32_t index_n = g_index_irq;
    snapOnSampleCounts(counts, index_n, now_us);
    // 本地 live 速度（与下方非记录路径相同算法）
    cb_hist_c[cb_hist_i] = counts;
    cb_hist_us[cb_hist_i] = now_us;
    cb_hist_i = (cb_hist_i + 1) % VEL_WINDOW;
    if (cb_hist_n < VEL_WINDOW) cb_hist_n++;
    float rpm_loc = 0.0f;
    if (cb_hist_n >= VEL_WINDOW) {
      int i_old = cb_hist_i;
      int64_t dc = counts - cb_hist_c[i_old];
      uint32_t dt = now_us - cb_hist_us[i_old];
      if (dt > 0) {
        rpm_loc = ((float)dc / (float)ABI_STEPS_PER_REV) * (1e6f / (float)dt) * 60.0f;
      }
    }
    cb_rpm_filt = 0.85f * cb_rpm_filt + 0.15f * rpm_loc;
    g_rpm_for_index = cb_rpm_filt;
    portENTER_CRITICAL(&g_snap_mux);
    g_snap.counts = counts;
    g_snap.t_ms = now_ms;
    g_snap.index_n = index_n;
    g_snap.seq++;
    portEXIT_CRITICAL(&g_snap_mux);
    return;
  }

  if (!g_abs_inited) {
    g_abs_counts_last = counts;
    g_counts_zero = counts;
    g_abs_inited = true;
  } else {
    int64_t dc = counts - g_abs_counts_last;
    if (dc < 0) dc = -dc;
    g_revs_abs_acc += (double)dc / (double)ABI_STEPS_PER_REV;
    g_abs_counts_last = counts;
  }
  float revs_total = (float)((double)(counts - g_counts_zero) / (double)ABI_STEPS_PER_REV);
  float revs_abs = (float)g_revs_abs_acc;

  cb_hist_c[cb_hist_i] = counts;
  cb_hist_us[cb_hist_i] = now_us;
  cb_hist_i = (cb_hist_i + 1) % VEL_WINDOW;
  if (cb_hist_n < VEL_WINDOW) cb_hist_n++;

  float rpm = 0.0f;
  if (cb_hist_n >= VEL_WINDOW) {
    int i_old = cb_hist_i;
    int64_t dc = counts - cb_hist_c[i_old];
    uint32_t dt = now_us - cb_hist_us[i_old];
    if (dt > 0) {
      rpm = ((float)dc / (float)ABI_STEPS_PER_REV) * (1e6f / (float)dt) * 60.0f;
    }
  }
  cb_rpm_filt = 0.85f * cb_rpm_filt + 0.15f * rpm;
  if (fabsf(cb_rpm_filt) < 0.3f) cb_rpm_filt = 0.0f;
  g_rpm_for_index = cb_rpm_filt;

  uint32_t index_n = g_index_irq;
  int32_t index_signed = g_index_signed;
  g_index_seen = index_n;

  int8_t dir = 0;
  if (cb_rpm_filt > 0.5f) dir = 1;
  else if (cb_rpm_filt < -0.5f) dir = -1;

  const float arpm = fabsf(cb_rpm_filt);
  const uint32_t dt_sample_ms = SAMPLE_PERIOD_US / 1000UL;

  // 主路径：武装环缓 → |RPM|>10 且 I+1圈 → 回溯400 + 再记2s（snap）
  // 注意：触发后 g_armed_ring=false，但仍须继续 snapOnSampleCounts，否则正式段不写点且会误入旧 STAGING 挤死 BLE
  if (snapIsRecording()) {
    snapOnSampleCounts(counts, index_n, now_us);
    goto snap_out;
  }
  if (g_armed && snapIsArmed()) {
    snapOnSampleCounts(counts, index_n, now_us);
    if (g_phase == PH_IDLE) {
      if (arpm > RPM_GATE) {
        bleMuteHost(true);  // STAGING/RECORD：少灌 BLE，防 Windows 断链
        g_phase = PH_STAGING;
        g_stage_start_ms = now_ms;
        g_stage_index0 = index_n;
        g_stage_counts0 = counts;
        g_seg_count = (uint16_t)(g_seg_count + 1);
        g_seg_cur = g_seg_count;
        g_evt_code = 6;  // staging
        g_evt_pending = true;
      }
    } else if (g_phase == PH_STAGING) {
      // 弹射约 0.3s：进入 STAGING 后绝不因掉转速回 IDLE 丢弃。
      // 掉速 / 短时 / I+1 / 环缓够 → 立刻触发正式段，记满 REC 时长（停转也继续采点）。
      const bool rpm_drop = (arpm < (RPM_GATE * 0.5f));
      const bool short_hold = (now_ms - g_stage_start_ms) >= 200u;  // 0.2s 覆盖弹射主过程
      const bool i_plus = ((int32_t)(index_n - g_stage_index0) >= 1);
      const bool ring_ok = (snapRingCount() >= 120);
      const bool drop_salvage = rpm_drop && (snapRingCount() >= 40);
      if (i_plus || short_hold || ring_ok || drop_salvage) {
        if (snapTriggerFromRing(BACKTRACK_N, g_rec_duration_ms)) {
          g_phase = PH_RECORD;
          g_rec_start_ms = now_ms;
          g_evt_revs = 1.0f;
          g_evt_seg = g_seg_cur;
          g_evt_n = (uint32_t)snapCount();
          g_evt_code = 1;
          g_evt_pending = true;
        }
      }
    }
    goto snap_out;
  }

  // 旧 MONITOR 主缓冲路径（仅非 snap 武装时）
  if (g_phase == PH_RECORD && !snapIsRecording()) {
    logPushMain(now_ms, cb_rpm_filt, g_seg_cur, index_n);
    if ((now_ms - g_rec_start_ms) >= g_rec_duration_ms) {
      g_evt_seg = g_seg_cur;
      g_evt_n = (uint32_t)g_log_n;
      g_evt_code = 3;
      g_evt_pending = true;
      g_phase = PH_IDLE;
      g_seg_cur = 0;
      g_armed = false;
      bleMuteHost(false);
    }
  } else if (g_armed && !snapIsArmed()) {
    if (g_phase == PH_IDLE) {
      if (arpm > RPM_GATE) {
        beginStaging(now_ms, counts, cb_rpm_filt, index_n);
      }
    } else if (g_phase == PH_STAGING) {
      poolPush(now_ms, cb_rpm_filt, g_seg_cur, index_n);
      if (g_phase != PH_STAGING) {
        g_noise_why = 5;
        goto snap_out;
      }

      int64_t dcounts = counts - g_stage_counts0;
      int64_t net = (g_stage_sign >= 0) ? dcounts : -dcounts;
      if (net < 0) net = -net;
      if (net >= CONFIRM_COUNTS) {
        g_evt_revs = (float)net / (float)ABI_STEPS_PER_REV;
        g_evt_seg = g_seg_cur;
        poolCommitToMain();
        g_phase = PH_RECORD;
        g_rec_start_ms = now_ms;
        g_evt_code = 1;
        g_evt_pending = true;
        goto snap_out;
      }

      int sgn = (cb_rpm_filt >= 0.0f) ? 1 : -1;
      if (arpm > RPM_GATE && sgn != g_stage_sign) {
        g_flip_cnt++;
        if (g_flip_cnt >= 24) {
          salvageStagingOrDiscard(2, now_ms, net);
          goto snap_out;
        }
      } else if (arpm > RPM_GATE) {
        g_flip_cnt = 0;
      }

      if (arpm <= RPM_GATE) {
        g_below_ms += dt_sample_ms ? dt_sample_ms : 1;
        if (g_below_ms >= NOISE_BELOW_MS) {
          salvageStagingOrDiscard(3, now_ms, net);
          goto snap_out;
        }
      } else {
        g_below_ms = 0;
      }

      if ((now_ms - g_stage_start_ms) >= STAGING_TIMEOUT_MS) {
        salvageStagingOrDiscard(4, now_ms, net);
        goto snap_out;
      }
    }
  }

snap_out:
  // 长采：直采原始转速/计数，不做门限、噪声、确认判断（与 MONITOR 无关）
  if (capIsRunning()) {
    capPushIsr(now_ms, rpm, counts, index_n);
  }
  // 每次 I 跳动记一条（仅 STAGING/RECORD；丢弃路径 phase 已 IDLE）
  captureIndexEdges(now_ms, cb_rpm_filt, index_n);
  portENTER_CRITICAL(&g_snap_mux);
  g_snap.counts = counts;
  g_snap.rpm = cb_rpm_filt;
  g_snap.dir = dir;
  g_snap.t_ms = now_ms;
  g_snap.seq++;
  g_snap.revs_total = revs_total;
  g_snap.revs_abs = revs_abs;
  g_snap.index_n = index_n;
  g_snap.index_signed = index_signed;
  portEXIT_CRITICAL(&g_snap_mux);

  cb_n++;
  if (now_ms - cb_last_diag_ms >= 500) {
    if (cb_n > 0) meas_hz = (uint32_t)((cb_n * 1000UL) / (now_ms - cb_last_diag_ms));
    cb_n = 0;
    cb_last_diag_ms = now_ms;
    portENTER_CRITICAL(&g_snap_mux);
    g_snap.sample_hz_meas = meas_hz;
    portEXIT_CRITICAL(&g_snap_mux);
  }
  (void)t0;
}

static bool sampleTimerBegin() {
  esp_timer_create_args_t args = {};
  args.callback = &sampleCb;
  args.name = "abi2k";
  if (esp_timer_create(&args, &g_timer) != ESP_OK) return false;
  return esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US) == ESP_OK;
}

static Snap takeSnap() {
  Snap s;
  portENTER_CRITICAL(&g_snap_mux);
  s = g_snap;
  portEXIT_CRITICAL(&g_snap_mux);
  return s;
}

static const char* phaseName(RecPhase p) {
  switch (p) {
    case PH_STAGING:
      return "STAGING";
    case PH_RECORD:
      return "RECORD";
    default:
      return "IDLE";
  }
}

static void emitTelem(const Snap& s) {
  if (g_dumping || g_sample_freeze) return;
  size_t n = 0, drop = 0;
  portENTER_CRITICAL(&g_log_mux);
  n = g_log_n;
  drop = g_log_drop;
  portEXIT_CRITICAL(&g_log_mux);

  uint32_t remain = 0;
  if (g_phase == PH_RECORD) {
    uint32_t e = millis() - g_rec_start_ms;
    remain = (e < g_rec_duration_ms) ? (g_rec_duration_ms - e) : 0;
  }

  // 整数格式化，避免 main 与 sampleCb 双端 FPU → Coprocessor 复位
  int rx = rpmToX10(s.rpm);
  int neg = 0;
  if (rx < 0) {
    neg = 1;
    rx = -rx;
  }
  int whole = rx / 10;
  int frac = rx % 10;
  int rxi = rpmToX10(s.revs_total);
  int negi = 0;
  if (rxi < 0) {
    negi = 1;
    rxi = -rxi;
  }
  int wholer = rxi / 10;
  int fracr = rxi % 10;
  int rxia = rpmToX10(s.revs_abs);
  if (rxia < 0) rxia = -rxia;
  int wholea = rxia / 10;
  int fraca = rxia % 10;
  int revs_x10 = 0;
  if (g_phase == PH_STAGING) {
    int64_t d = s.counts - g_stage_counts0;
    if (d < 0) d = -d;
    revs_x10 = (int)((d * 10) / ABI_STEPS_PER_REV);
  }

  char line[220];
  uint32_t t_rel = sessionRel(s.t_ms);
  uint64_t unix_ms = sampleUnixMs(s.t_ms);
  snprintf(line, sizeof(line),
           "L,%lu,%s%d.%d,%d,%d,%u,%lu,%lu,%lld,%u,%u,%u,%d.%d,%lu,%u,%s%d.%d,%d.%d,%lu,%ld,%lu,%llu\n",
           (unsigned long)s.t_ms, neg ? "-" : "", whole, frac, (int)s.dir, g_armed ? 1 : 0,
           (unsigned)n, (unsigned long)drop, (unsigned long)s.sample_hz_meas, (long long)s.counts,
           (unsigned)g_seg_cur, (unsigned)g_phase, (unsigned)g_pool_n, revs_x10 / 10,
           revs_x10 % 10, (unsigned long)remain, (unsigned)g_seg_count, negi ? "-" : "", wholer,
           fracr, wholea, fraca, (unsigned long)s.index_n, (long)s.index_signed,
           (unsigned long)t_rel, (unsigned long long)unix_ms);
  Serial.print(line);

  // 完全分离：未武装时才发 BLE 实时转速；一 MONITOR START 就停发 L，
  // 只靠 # QUIET / # CONFIRM / # RECORD done / # CD15 / # BLE PULL READY。
  // USB 串口始终有完整 L, 供 PC 调试。
  if (bleConnected() && !bleHostMuted() && !g_armed && g_phase == PH_IDLE &&
      !snapIsRecording()) {
    uint16_t snap_n = snapCount();
    char short_l[96];
    snprintf(short_l, sizeof(short_l), "L,%s%d.%d,%d,%d,%u,%lu,%u\n", neg ? "-" : "", whole, frac,
             (int)s.dir, g_armed ? 1 : 0, (unsigned)g_phase, (unsigned long)remain,
             (unsigned)snap_n);
    bleSendLine(short_l);
  }
}

/** BLE 回传：单行一包 + 强制等 ACK + 最小间隔（宁慢勿断，防协议栈饿死）。 */
static uint16_t g_ble_pkt_ms = 180;      // 无 ACK / 超时后的包间隔
static uint16_t g_ble_ack_wait_ms = 600;  // 等 DUMP ACK 上限（给 Windows 轮询时间）
static uint16_t g_ble_dump_max = 120;    // BLE 最多点数（再多降采样）
static uint16_t g_ble_min_gap_ms = 50;   // 即使收到 ACK 也至少间隔这么久再发下一包
static volatile bool g_dump_ack = false;
static volatile bool g_dump_abort = false;
static uint32_t g_ble_pkt_seq = 0;
/** 记录结束后待主机拉取的段（0=无）；避免 RECORD 刚结束就灌 BLE 导致断链 */
static uint16_t g_ble_pull_seg = 0;

static void bleDumpTakeAck() { g_dump_ack = true; }

/** 在 dump 循环里取 RX：ACK 放行；MONITOR/LOG 等则中止回传，恢复实时。 */
static void bleDumpPollRx() {
  char line[96];
  while (bleTakeRxLine(line, sizeof(line))) {
    if (!strncasecmp(line, "DUMP ACK", 8) || !strcasecmp(line, "ACK") ||
        !strncasecmp(line, "ACK,", 4) || !strcasecmp(line, "DUMP NEXT")) {
      g_dump_ack = true;
    } else if (!strncasecmp(line, "TIME", 4) || !strcasecmp(line, "ABI?") ||
               !strncasecmp(line, "BLE RATE", 8) || !strncasecmp(line, "PING", 4) ||
               !strcasecmp(line, "DIAG") || !strcasecmp(line, "SNAP?")) {
      // 重连对时/诊断：吞掉但不中止 BIN 回传（旧逻辑 TIME 会 abort → 像「清空」）
    } else if (!strncasecmp(line, "MONITOR", 7) || !strncasecmp(line, "LOG", 3) ||
               !strcasecmp(line, "DUMP ABORT") || !strcasecmp(line, "ABORT")) {
      g_dump_abort = true;
    }
  }
}

/** 发完一包后：优先等主机 ACK；超时则长间隔再继续。始终给 BLE 栈 yield。 */
static bool bleDumpPace() {
  if (!bleConnected() || g_dump_abort) return false;
  g_dump_ack = false;
  uint32_t t0 = millis();
  while (bleConnected() && !g_dump_abort) {
    bleDumpPollRx();
    if (g_dump_ack) {
      uint32_t e = millis() - t0;
      if (e < g_ble_min_gap_ms) delay(g_ble_min_gap_ms - e);
      return bleConnected() && !g_dump_abort;
    }
    uint32_t elapsed = millis() - t0;
    if (elapsed >= g_ble_ack_wait_ms) break;
    delay(5);  // 让出给 NimBLE / 看门狗
  }
  if (g_dump_abort) return false;
  // 超时未 ACK：再退避，避免背靠背灌包
  delay(g_ble_pkt_ms);
  return bleConnected() && !g_dump_abort;
}

static bool bleDumpSendLine(const char* line) {
  if (!line || !bleConnected()) return false;
  bleSendLine(line);
  g_ble_pkt_seq++;
  return bleDumpPace();
}

/** 导出标记行：via_ble 时只走 BLE，避免 USB/BLE 双通道互相污染。 */
static void dumpMark(bool use_ble, const char* fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (use_ble) {
    if (!bleConnected()) return;
    bleDumpSendLine(buf);
  } else {
    Serial.print(buf);
  }
}

/** only_seg==0 导出全部；否则只导出该段。
 *  via_ble=false：仅串口全速（PC USB）
 *  via_ble=true：BLE 单行分包 + ACK/慢速（手机 / PC 蓝牙）
 */
static void dumpLog(size_t max_n, uint16_t only_seg, bool via_ble) {
  g_dump_started = true;
  g_dumping = true;
  g_dump_abort = false;
  // 停采样：与 DUMP 并发用 FPU → Coprocessor exception 复位丢数据
  if (g_timer) esp_timer_stop(g_timer);
  g_sample_freeze = true;
  const bool use_ble = via_ble && bleConnected();
  if (via_ble && !use_ble) {
    if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
    g_dumping = false;
    g_sample_freeze = false;
    return;
  }
  g_ble_pkt_seq = 0;
  if (!g_log || g_log_cap == 0) {
    dumpMark(use_ble, "# LOG empty (no buffer)\n");
    dumpMark(use_ble, "D END 0\n");
    dumpMark(use_ble, "I END 0\n");
    if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
    g_dumping = false;
    g_sample_freeze = false;
    return;
  }
  size_t n, w;
  portENTER_CRITICAL(&g_log_mux);
  n = g_log_n;
  w = g_log_w;
  portEXIT_CRITICAL(&g_log_mux);
  if (n == 0) {
    dumpMark(use_ble, "# LOG empty\n");
    dumpMark(use_ble,
             "# HINT: live RPM != record. Send MONITOR START while spinning, "
             "wait CONFIRM + RECORD done, then DUMP.\n");
    hostPrintf("# MONITOR armed=%d phase=%s log_n=0 (dump empty)\n", g_armed ? 1 : 0,
               phaseName(g_phase));
    dumpMark(use_ble, "D END 0\n");
    dumpMark(use_ble, "I END 0\n");
    if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
    g_dumping = false;
    g_sample_freeze = false;
    return;
  } else {
    if (max_n == 0 || max_n > n) max_n = n;
    size_t start = (w + g_log_cap - n) % g_log_cap;

    // 统计本段点数 → BLE 降采样上限
    uint32_t expect = 0;
    for (size_t i = 0; i < max_n; ++i) {
      LogSample s = g_log[(start + i) % g_log_cap];
      if (only_seg != 0 && s.seg != only_seg) continue;
      expect++;
    }
    uint32_t stride = 1;
    if (use_ble && expect > g_ble_dump_max && g_ble_dump_max > 0) {
      stride = (expect + g_ble_dump_max - 1) / g_ble_dump_max;
    }

    uint32_t out_total = (stride > 1) ? ((expect + stride - 1) / stride) : expect;
    if (use_ble) {
      dumpMark(true,
               "# BLE dump mode=paced ack_wait=%ums pkt_ms=%ums max=%u stride=%lu "
               "expect≈%lu out~%lu\n",
               (unsigned)g_ble_ack_wait_ms, (unsigned)g_ble_pkt_ms, (unsigned)g_ble_dump_max,
               (unsigned long)stride, (unsigned long)expect, (unsigned long)out_total);
      dumpMark(true, "# DATA_START|%lu\n", (unsigned long)out_total);
      dumpMark(true, "# DUMP PROG n=0 total=%lu pct=0\n", (unsigned long)out_total);
    }

    if (only_seg) {
      dumpMark(use_ble, "# LOG DUMP seg=%u %s\n", (unsigned)only_seg,
               use_ble ? "ble=1 paced" : "ble=0 (USB full speed)");
    } else {
      dumpMark(use_ble, "# LOG DUMP by-seg n=%u segs≈%u %s\n", (unsigned)max_n,
               (unsigned)g_seg_count, use_ble ? "ble=1 paced" : "ble=0 (USB full speed)");
    }

    uint16_t cur_seg = 0xFFFF;
    uint32_t seg_count_pts = 0;
    uint32_t total = 0;
    uint32_t seg_out = 0;
    uint32_t keep_i = 0;
    bool ble_aborted = false;
    uint32_t last_ka_ms = millis();

    for (size_t i = 0; i < max_n; ++i) {
      size_t idx = (start + i) % g_log_cap;
      LogSample s = g_log[idx];
      if (only_seg != 0 && s.seg != only_seg) continue;

      if (use_ble && stride > 1) {
        if ((keep_i++ % stride) != 0) continue;
      }

      if (s.seg != cur_seg) {
        if (cur_seg != 0xFFFF) {
          dumpMark(use_ble, "S END %u n=%lu\n", (unsigned)cur_seg, (unsigned long)seg_count_pts);
          seg_out++;
        }
        cur_seg = s.seg;
        seg_count_pts = 0;
        dumpMark(use_ble, "S BEGIN %u board_t0=%lu unix_t0=%llu\n", (unsigned)cur_seg,
                 (unsigned long)g_session_board_t0, (unsigned long long)g_session_unix_t0);
      }
      // 整数格式化 RPM（禁止 float，防 Coprocessor exception）
      int rx = (int)s.rpm_x10;
      int neg = 0;
      if (rx < 0) {
        neg = 1;
        rx = -rx;
      }
      int whole = rx / 10;
      int frac = rx % 10;
      int dir = (s.rpm_x10 > 0) ? 1 : (s.rpm_x10 < 0 ? -1 : 0);
      uint32_t t_rel = sessionRel(s.t_ms);
      uint64_t unix_ms = sampleUnixMs(s.t_ms);
      if (use_ble) {
        if (!bleConnected()) {
          ble_aborted = true;
          Serial.println("# DUMP BLE aborted: link lost");
          break;
        }
        char ble[80];
        snprintf(ble, sizeof(ble), "D,%lu,%s%d.%d,%d,%u,%lu\n", (unsigned long)s.t_ms,
                 neg ? "-" : "", whole, frac, dir, (unsigned)s.seg, (unsigned long)s.index_n);
        if (!bleDumpSendLine(ble)) {
          ble_aborted = true;
          Serial.println("# DUMP BLE aborted: link lost");
          break;
        }
      } else {
        // USB：放慢灌包，避免 CDC 堵死/只收到几行就断
        char line[72];
        snprintf(line, sizeof(line), "D,%lu,%s%d.%d,%d,%u,%lu\n", (unsigned long)s.t_ms,
                 neg ? "-" : "", whole, frac, dir, (unsigned)s.seg, (unsigned long)s.index_n);
        Serial.print(line);
        if ((total & 0x07) == 0x07) {
          Serial.flush();
          yield();
          delay(2);
        }
      }
      seg_count_pts++;
      total++;
      if (use_ble && (total % 20) == 0) {
        uint32_t pct = out_total ? (total * 100UL / out_total) : 0;
        if (pct > 100) pct = 100;
        Serial.printf("# DUMP BLE progress %lu/%lu pct=%lu\n", (unsigned long)total,
                      (unsigned long)out_total, (unsigned long)pct);
        char prog[80];
        snprintf(prog, sizeof(prog), "# DUMP PROG n=%lu total=%lu pct=%lu\n",
                 (unsigned long)total, (unsigned long)out_total, (unsigned long)pct);
        if (!bleDumpSendLine(prog)) {
          ble_aborted = true;
          break;
        }
      }
    }
    if (cur_seg != 0xFFFF) {
      dumpMark(use_ble, "S END %u n=%lu\n", (unsigned)cur_seg, (unsigned long)seg_count_pts);
      seg_out++;
    }
    if (ble_aborted) {
      dumpMark(true, "# DUMP BLE truncated n=%lu\n", (unsigned long)total);
    }
    dumpMark(use_ble, "D END %lu segs=%lu\n", (unsigned long)total, (unsigned long)seg_out);
    if (use_ble) dumpMark(true, "# DATA_END|%lu\n", (unsigned long)total);
  }

  // USB：不推 I 事件（易拉长 DUMP 再崩）；PC 只靠 D 点即可分析
  if (!use_ble) {
    dumpMark(false, "# INDEX EVENTS skipped (USB short)\n");
    dumpMark(false, "I END 0\n");
    if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
    g_dumping = false;
    g_sample_freeze = false;
    return;
  }

  size_t in = 0, iw = 0;
  portENTER_CRITICAL(&g_ilog_mux);
  in = g_ilog_n;
  iw = g_ilog_w;
  portEXIT_CRITICAL(&g_ilog_mux);
  dumpMark(use_ble, "# INDEX EVENTS n=%u drop=%lu filter_seg=%u\n", (unsigned)in,
           (unsigned long)g_ilog_drop, (unsigned)only_seg);
  if (in == 0 || !g_ilog) {
    dumpMark(use_ble, "I END 0\n");
    if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
    g_dumping = false;
    g_sample_freeze = false;
    return;
  }
  size_t istart = (iw + g_ilog_cap - in) % g_ilog_cap;
  uint32_t itotal = 0;
  for (size_t i = 0; i < in; ++i) {
    if (use_ble && !bleConnected()) break;
    size_t idx = (istart + i) % g_ilog_cap;
    IndexEvent e = g_ilog[idx];
    if (only_seg != 0 && e.seg != only_seg) continue;
    int rx = (int)e.rpm_x10;
    int neg = 0;
    if (rx < 0) {
      neg = 1;
      rx = -rx;
    }
    int whole = rx / 10, frac = rx % 10;
    int rxi = (int)e.rpm_i_x10;
    int negi = 0;
    if (rxi < 0) {
      negi = 1;
      rxi = -rxi;
    }
    int wholei = rxi / 10, fraci = rxi % 10;
    uint32_t t_rel = sessionRel(e.t_ms);
    uint64_t unix_ms = sampleUnixMs(e.t_ms);
    char line[160];
    snprintf(line, sizeof(line), "I,%u,%lu,%lu,%s%d.%d,%s%d.%d,%lu,%u,%lu,%llu\n",
             (unsigned)itotal, (unsigned long)e.t_ms, (unsigned long)e.index_n, neg ? "-" : "",
             whole, frac, negi ? "-" : "", wholei, fraci, (unsigned long)e.dt_ms, (unsigned)e.seg,
             (unsigned long)t_rel, (unsigned long long)unix_ms);
    if (use_ble) {
      if (!bleDumpSendLine(line)) break;
    } else {
      Serial.print(line);
      if ((itotal & 0x1F) == 0x1F) {
        yield();
        delay(1);
      }
    }
    itotal++;
  }
  dumpMark(use_ble, "I END %lu\n", (unsigned long)itotal);
  if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
  g_dumping = false;
  g_sample_freeze = false;
}

static void requestAutoDump(uint16_t seg) {
  g_auto_dump_seg = seg;
  g_auto_dump_expect = g_evt_n;  // 已在 ISR 记好，勿再扫缓冲（减主循环负担）
  g_need_auto_dump = true;
}

static bool ensureMonBuffers() {
  if (!g_log && !logAlloc()) {
    Serial.println("# WARN logAlloc failed");
    return false;
  }
  if (!g_pool && !poolAlloc()) {
    Serial.println("# WARN poolAlloc failed");
    return false;
  }
  if (!g_ilog && !ilogAlloc()) {
    Serial.println("# WARN ilogAlloc failed");
    return false;
  }
  return true;
}

static void monitorStart() {
  // 残留 U 盘模式会挡住武装；自动关掉
  if (sdMscIsOn()) {
    hostPrintln("# MONITOR: USB DISK was ON → OFF");
    sdMscOff();
  }
  // 残留 freeze/dumping 会导致 sampleCb 直接 return，永远不记
  if (g_sample_freeze || g_dumping) {
    hostPrintf("# MONITOR: clear stuck freeze=%d dumping=%d\n", g_sample_freeze ? 1 : 0,
               g_dumping ? 1 : 0);
  }
  g_sample_freeze = false;
  g_dumping = false;
  g_dump_abort = false;
  if (snapIsRecording()) {
    hostPrintln("# MONITOR busy (snap recording)");
    return;
  }
  if (!snapArmBegin()) {
    hostPrintln("# MONITOR fail: snap arm/ring alloc");
    char d[120];
    snapDiagLine(d, sizeof(d));
    hostPrintln(d);
    return;
  }
  g_armed = true;
  g_phase = PH_IDLE;
  g_seg_cur = 0;
  g_dump_started = false;
  g_ready_ms = 0;
  static uint32_t s_monitor_start_count = 0;
  s_monitor_start_count++;
  char stamp[24];
  bool synced = formatWallStamp(stamp, sizeof(stamp), wallUnixMsNow());
  // 武装说明走串口；BLE 只发短标记，随后静音实时流
  hostPrintf("# MONITOR armed: ESP32 auto |rpm|>%.0f → I+1/1.2s/ring → REC %lums\n", RPM_GATE,
             (unsigned long)g_rec_duration_ms);
  hostPrintf("# MONITOR time stamp=%s synced=%d\n", stamp, synced ? 1 : 0);
  hostPrintf("# MONITOR START count=%lu (clears previous RAM snap)\n",
             (unsigned long)s_monitor_start_count);
  {
    char d[120];
    snapDiagLine(d, sizeof(d));
    hostPrintln(d);
  }
  if (bleConnected()) {
    char s[72];
    snprintf(s, sizeof(s), "# MONITOR START count=%lu\n", (unsigned long)s_monitor_start_count);
    bleSendLine(s);
    bleSendLine("# MONITOR armed — BLE live RPM OFF until STOP/done\n");
  }
  // 武装后禁止 host*→BLE 灌包；事件仍用 bleSendLine 越过 mute
  bleMuteHost(true);
  if (g_timer) {
    esp_timer_stop(g_timer);
    esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
  }
}

/** 电机已转起来后：点一下 → RAM 采满 → 事后 SD 存档 */
static void recNow() {
  if (sdMscIsOn()) {
    hostPrintln("# REC blocked — USB DISK ON (send USB DISK OFF first)");
    return;
  }
  if (snapIsRecording()) {
    hostPrintln("# REC NOW busy (already recording)");
    return;
  }
  g_armed = false;
  g_phase = PH_IDLE;
  g_seg_cur = 0;
  g_sample_freeze = false;
  g_dumping = false;
  g_dump_started = false;
  g_ready_ms = 0;
  bleMuteHost(false);
  if (!snapRecStart(g_rec_duration_ms, SAMPLE_HZ)) {
    hostPrintln("# REC NOW fail (snap buffer)");
    return;
  }
  if (g_timer) {
    esp_timer_stop(g_timer);
    esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
  }
  hostPrintf("# REC NOW %lums @%luHz → INTERNAL RAM then SD SAVE\n",
             (unsigned long)g_rec_duration_ms, (unsigned long)SAMPLE_HZ);
}

/** RAM snap → /snap_*.bin（记完后存档，不在 ISR 写卡） */
static void snapArchiveToSd() {
  if (sdMscIsOn()) {
    hostPrintln("# SD SAVE blocked — USB DISK ON (PC owns card; USB DISK OFF first)");
    return;
  }
  if (snapIsRecording()) {
    hostPrintln("# SD SAVE busy recording");
    return;
  }
  size_t len = snapDataLen();
  const uint8_t* p = snapDataBytes();
  if (!p || len == 0 || snapCount() == 0) {
    hostPrintln("# SD SAVE fail: no snap data");
    return;
  }
  if (!sdReady() && !sdBegin()) {
    hostPrintln("# SD SAVE fail: no card — data still in RAM");
    if (bleConnected()) {
      delay(400);
      hostPrintln("# BLE PULL READY src=RAM — send DUMP BIN BLE (no SD archive)");
    } else {
      hostPrintln("# → try DUMP BIN (USB) or insert SD + SD SAVE");
    }
    return;
  }
  uint64_t unix_now = wallUnixMsNow();
  char stamp[24];
  bool synced = formatWallStamp(stamp, sizeof(stamp), unix_now);
  // /snap_YYYYMMDD_HHMMSS_n.bin  （未对时则为 /snap_b<millis>_n.bin）
  char path[64];
  snprintf(path, sizeof(path), "/snap_%s_%u.bin", stamp, (unsigned)snapCount());
  g_dumping = true;
  g_sample_freeze = true;
  if (g_timer) esp_timer_stop(g_timer);
  bool ok = sdWriteBinary(path, p, len);
  if (ok) {
    uint32_t crc = snapCrc32(p, len);
    char meta_path[64];
    snprintf(meta_path, sizeof(meta_path), "/snap_%s_%u.txt", stamp, (unsigned)snapCount());
    char meta[320];
    snprintf(meta, sizeof(meta),
             "stamp=%s\n"
             "iso_local=%s\n"
             "unix_ms=%llu\n"
             "synced=%d\n"
             "n=%u\n"
             "bytes=%u\n"
             "crc=0x%08lX\n"
             "hz=2000\n"
             "board_ms=%lu\n"
             "file=%s\n",
             stamp, stamp, (unsigned long long)unix_now, synced ? 1 : 0, (unsigned)snapCount(),
             (unsigned)len, (unsigned long)crc, (unsigned long)millis(), path);
    sdWriteText(meta_path, meta);
    hostPrintf("# SD SAVE OK %s n=%u bytes=%u crc=0x%08lX time=%s synced=%d\n", path,
               (unsigned)snapCount(), (unsigned)len, (unsigned long)crc, stamp, synced ? 1 : 0);
    if (!synced) {
      hostPrintln("# WARN no PC TIME yet — filename uses board millis; reconnect UI to sync clock");
    }
    // 取数策略：SD=存档；BLE 传输期间不读卡（SPI 争用易断链）。
    // 有 BLE 连接时跳过 USB MSC，保留 RAM 供 DUMP BIN BLE；无 BLE 则 U 盘读卡。
    if (bleConnected()) {
      hostPrintln("# → BLE linked: skip USB DISK ON; RAM kept (=SD payload)");
      // 给 SD SPI / BLE 栈一点喘息，再通知手机拉（避免立刻 DUMP 空/断）
      delay(400);
      hostPrintln("# BLE PULL READY src=RAM — send DUMP BIN BLE (safer than SD SPI TX)");
    } else {
      hostPrintln("# → auto USB DISK ON (Native USB = U-disk; CH343 = serial)");
      sdMscOn();
      hostPrintln("# → if still JTAG: unplug/replug Native USB once");
    }
  } else {
    hostPrintln("# SD SAVE fail: write error — data still in RAM");
    if (bleConnected()) {
      delay(400);
      hostPrintln("# BLE PULL READY src=RAM — send DUMP BIN BLE (SD write failed)");
    }
  }
  if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
  g_sample_freeze = false;
  g_dumping = false;
}

static void dumpBin() {
  if (snapIsRecording()) {
    hostPrintln("# DUMP BIN busy (still recording)");
    return;
  }
  if (!snapDumpReady()) {
    hostPrintln("# DUMP BIN wait — need # SNAP DUMP READY (ALIVE ~2s)");
    return;
  }
  g_dumping = true;
  g_sample_freeze = true;
  if (g_timer) esp_timer_stop(g_timer);
  snapDumpBinary();
  if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
  g_sample_freeze = false;
  g_dumping = false;
}

/** BLE hex 分包：B,<seq>,<HEX> 每包等 ACK。从 RAM 读，不碰 SD。 */
static bool bleSendBinHexChunks(const uint8_t* data, size_t len, uint32_t* seq) {
  static const char* HEXDIG = "0123456789ABCDEF";
  const size_t CHUNK = 72;  // 144 hex + 前缀，适配 MTU≈247
  size_t off = 0;
  char line[200];
  while (off < len) {
    if (!bleConnected() || g_dump_abort) return false;
    size_t m = len - off;
    if (m > CHUNK) m = CHUNK;
    int pos = snprintf(line, sizeof(line), "B,%lu,", (unsigned long)(*seq));
    for (size_t i = 0; i < m && pos + 2 < (int)sizeof(line) - 2; ++i) {
      uint8_t b = data[off + i];
      line[pos++] = HEXDIG[b >> 4];
      line[pos++] = HEXDIG[b & 0x0F];
    }
    line[pos++] = '\n';
    line[pos] = '\0';
    if (!bleDumpSendLine(line)) return false;
    (*seq)++;
    off += m;
  }
  return true;
}

/**
 * BLE 二进制回传公共路径：payload=原始 SnapPoint 字节（与 SD 文件内容一致）。
 * 协议：# BIN BLE BEGIN … / B,<seq>,<hex> / # BIN BLE END …（每包等 DUMP ACK）
 * src 仅用于标记：RAM / SD
 */
static void dumpBinBlePayload(const uint8_t* payload, size_t plen, uint16_t n, uint16_t hz,
                              const char* src) {
  if (!bleConnected()) {
    hostPrintln("# DUMP BIN BLE fail: BLE not connected");
    return;
  }
  if (!payload || n == 0 || plen == 0) {
    hostPrintln("# DUMP BIN BLE fail: empty payload");
    return;
  }
  if (plen != (size_t)n * snapPointSize()) {
    // SD 裸文件按点阵对齐；尾部残片丢弃
    n = (uint16_t)(plen / snapPointSize());
    plen = (size_t)n * snapPointSize();
    if (n == 0) {
      hostPrintln("# DUMP BIN BLE fail: payload not snap points");
      return;
    }
  }
  if (hz == 0) hz = 2000;

  uint32_t file_magic = snapFileMagic();
  uint32_t crc = snapCrc32(payload, plen);
  uint8_t head[8];
  memcpy(head, &file_magic, 4);
  memcpy(head + 4, &n, 2);
  memcpy(head + 6, &hz, 2);
  uint8_t crc_le[4];
  memcpy(crc_le, &crc, 4);

  g_dumping = true;
  g_sample_freeze = true;
  g_dump_abort = false;
  bleMuteHost(true);
  if (g_timer) esp_timer_stop(g_timer);

  uint16_t saved_gap = g_ble_min_gap_ms;
  uint16_t saved_ack = g_ble_ack_wait_ms;
  g_ble_min_gap_ms = 35;
  g_ble_ack_wait_ms = 900;

  const char* src_tag = (src && src[0]) ? src : "RAM";
  char mark[200];
  snprintf(mark, sizeof(mark),
           "# BIN BLE BEGIN src=%s n=%u hz=%u steps=%ld pt=%u bytes=%u crc=0x%08lX\n", src_tag,
           (unsigned)n, (unsigned)hz, (long)snapStepsPerRev(), (unsigned)snapPointSize(),
           (unsigned)(8 + plen + 4), (unsigned long)crc);
  Serial.print(mark);
  bool ok = bleDumpSendLine(mark);

  uint32_t seq = 0;
  if (ok) ok = bleSendBinHexChunks(head, sizeof(head), &seq);
  if (ok) ok = bleSendBinHexChunks(payload, plen, &seq);
  if (ok) ok = bleSendBinHexChunks(crc_le, sizeof(crc_le), &seq);

  snprintf(mark, sizeof(mark),
           "# BIN BLE END src=%s n=%u chunks=%lu ok=%d crc=0x%08lX\n", src_tag, (unsigned)n,
           (unsigned long)seq, ok ? 1 : 0, (unsigned long)crc);
  Serial.print(mark);
  if (bleConnected() && !g_dump_abort) bleDumpSendLine(mark);

  g_ble_min_gap_ms = saved_gap;
  g_ble_ack_wait_ms = saved_ack;
  if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
  bleMuteHost(false);
  g_sample_freeze = false;
  g_dumping = false;
  g_dump_abort = false;
}

/** 从 SD 最新 snap_*.bin 拉：先整文件读入 RAM 再 BLE 传（避免 SPI/BLE 交错） */
static void dumpBinBleFromSd() {
  if (!bleConnected()) {
    hostPrintln("# DUMP BIN BLE SD fail: BLE not connected");
    return;
  }
  if (snapIsRecording()) {
    hostPrintln("# DUMP BIN BLE SD busy recording");
    return;
  }
  if (sdMscIsOn()) {
    hostPrintln("# DUMP BIN BLE SD blocked — USB DISK ON");
    return;
  }
  char path[64];
  uint32_t fsz = 0;
  if (!sdFindLatestSnap(path, sizeof(path), &fsz)) {
    hostPrintln("# DUMP BIN BLE SD fail: no snap_*.bin on card");
    return;
  }
  g_dumping = true;
  g_sample_freeze = true;
  if (g_timer) esp_timer_stop(g_timer);
  uint8_t* buf = nullptr;
  size_t len = 0;
  bool rd = sdReadEntire(path, &buf, &len);
  if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
  g_sample_freeze = false;
  g_dumping = false;
  if (!rd || !buf || len == 0) {
    if (buf) free(buf);
    hostPrintf("# DUMP BIN BLE SD fail: read %s\n", path);
    return;
  }
  hostPrintf("# DUMP BIN BLE SD file=%s bytes=%u → BLE\n", path, (unsigned)len);
  uint16_t n = (uint16_t)(len / snapPointSize());
  dumpBinBlePayload(buf, (size_t)n * snapPointSize(), n, 2000, "SD");
  free(buf);
}

/**
 * BLE 优先读 INTERNAL RAM；空则自动改拉 SD 最新 snap（手机可用）。
 */
static void dumpBinBle() {
  if (!bleConnected()) {
    hostPrintln("# DUMP BIN BLE fail: BLE not connected");
    return;
  }
  if (snapIsRecording()) {
    hostPrintln("# DUMP BIN BLE busy recording");
    return;
  }
  const uint8_t* payload = snapDataBytes();
  size_t plen = snapDataLen();
  uint16_t n = snapCount();
  uint16_t hz = (uint16_t)snapHz();
  if (payload && n > 0 && plen > 0) {
    dumpBinBlePayload(payload, plen, n, hz, "RAM");
    return;
  }
  char d[140];
  snapDiagLine(d, sizeof(d));
  hostPrintln(d);
  if (snapRingCount() > 0 && n == 0) {
    hostPrintf("# DUMP BIN BLE: empty SNAP (ring=%u) — try SD fallback\n",
               (unsigned)snapRingCount());
  } else {
    hostPrintln("# DUMP BIN BLE: empty RAM — try SD fallback");
  }
  dumpBinBleFromSd();
}

static void dumpHex() {
  if (snapIsRecording()) {
    hostPrintln("# HEX DUMP busy (still recording)");
    return;
  }
  if (!snapDumpReady()) {
    hostPrintln("# HEX DUMP wait — need # SNAP DUMP READY");
    return;
  }
  g_dumping = true;
  g_sample_freeze = true;
  if (g_timer) esp_timer_stop(g_timer);
  snapDumpHex();
  if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
  g_sample_freeze = false;
  g_dumping = false;
}

static void monitorStop() {
  g_armed = false;
  snapArmEnd();
  if (g_phase == PH_STAGING) {
    g_evt_seg = g_seg_cur;
    discardStaging(2);
    hostPrintln("# MONITOR disarm: staging discarded");
  } else if (g_phase == PH_RECORD) {
    uint16_t seg = g_seg_cur;
    g_evt_seg = seg;
    g_phase = PH_IDLE;
    g_seg_cur = 0;
    hostPrintln("# MONITOR disarm: record stopped → auto dump");
    requestAutoDump(seg);
  } else {
    hostPrintln("# MONITOR disarm");
  }
}

static void handleCmd(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  char* p = line + strlen(line);
  while (p > line && (p[-1] == ' ' || p[-1] == '\t')) *--p = '\0';
  if (*line == '\0') return;

  // 直采优先：勿被 REC MS 前缀误吞（旧 bug：REC NOW → 只改时长）
  // 记录中/武装中禁止 REC NOW，防止重连误令清空 RAM
  if (!strcasecmp(line, "REC") || !strncasecmp(line, "REC NOW", 7) || !strcasecmp(line, "SNAP") ||
      !strcasecmp(line, "REC SNAP")) {
    if (snapIsRecording() || g_armed || snapCount() > 0) {
      hostPrintf("# REC NOW ignored (rec=%d armed=%d snap=%u) — use MONITOR path\n",
                 snapIsRecording() ? 1 : 0, g_armed ? 1 : 0, (unsigned)snapCount());
      return;
    }
    recNow();
    return;
  }
  if (!strcasecmp(line, "DUMP BIN BLE SD") || !strcasecmp(line, "BIN DUMP BLE SD") ||
      !strcasecmp(line, "BLE BIN SD") || !strcasecmp(line, "DUMP SD BLE")) {
    dumpBinBleFromSd();
    return;
  }
  if (!strcasecmp(line, "DUMP BIN BLE") || !strcasecmp(line, "BIN DUMP BLE") ||
      !strcasecmp(line, "BLE BIN")) {
    dumpBinBle();
    return;
  }
  if (!strcasecmp(line, "DUMP BIN") || !strcasecmp(line, "BIN DUMP")) {
    dumpBin();  // USB 串口二进制；蓝牙请用 DUMP BIN BLE（读 RAM，不读 SD）
    return;
  }
  if (!strcasecmp(line, "HEX DUMP") || !strcasecmp(line, "DUMP HEX")) {
    dumpHex();
    return;
  }
  if (!strcasecmp(line, "SNAP?")) {
    // 短状态：手机据此决定 RAM 还是 SD
    uint16_t n = snapCount();
    size_t bytes = snapDataLen();
    int valid = (n > 0 && bytes > 0) ? 1 : 0;
    char s[120];
    snprintf(s, sizeof(s), "# SNAP src=RAM valid=%d n=%u bytes=%u ring=%u ready=%d\n", valid,
             (unsigned)n, (unsigned)bytes, (unsigned)snapRingCount(), snapDumpReady() ? 1 : 0);
    hostPrintln(s);
    if (bleConnected()) bleSendLine(s);
    return;
  }
  if (!strcasecmp(line, "LOG?") || !strcasecmp(line, "SNAP STATUS") || !strcasecmp(line, "DIAG") ||
      !strcasecmp(line, "STATUS")) {
    snapPrintStatus();
    char d[160];
    snapDiagLine(d, sizeof(d));
    hostPrintln(d);
    hostPrintf("# DIAG2 armed=%d phase=%s freeze=%d dumping=%d rpm=%.1f idx=%lu rec_ms=%lu\n",
               g_armed ? 1 : 0, phaseName(g_phase), g_sample_freeze ? 1 : 0, g_dumping ? 1 : 0,
               (double)takeSnap().rpm, (unsigned long)takeSnap().index_n,
               (unsigned long)g_rec_duration_ms);
    return;
  }
  if (!strcasecmp(line, "SD?") || !strcasecmp(line, "SD STATUS")) {
    sdPrintStatus();
    return;
  }
  if (!strcasecmp(line, "SD INIT") || !strcasecmp(line, "SD BEGIN")) {
    if (sdMscIsOn()) {
      hostPrintln("# SD INIT blocked — USB DISK ON (USB DISK OFF first)");
      return;
    }
    g_dumping = true;
    g_sample_freeze = true;
    if (g_timer) esp_timer_stop(g_timer);
    sdBegin();
    if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
    g_sample_freeze = false;
    g_dumping = false;
    return;
  }
  if (!strcasecmp(line, "SD TEST")) {
    if (sdMscIsOn()) {
      hostPrintln("# SD TEST blocked — USB DISK ON");
      return;
    }
    g_dumping = true;
    g_sample_freeze = true;
    if (g_timer) esp_timer_stop(g_timer);
    sdSelfTest();
    if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
    g_sample_freeze = false;
    g_dumping = false;
    return;
  }
  if (!strcasecmp(line, "SD LIST") || !strcasecmp(line, "SD LS")) {
    if (sdMscIsOn()) {
      hostPrintln("# SD LIST blocked — USB DISK ON (read files on PC)");
      return;
    }
    g_dumping = true;
    g_sample_freeze = true;
    if (g_timer) esp_timer_stop(g_timer);
    sdListRoot(30);
    if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
    g_sample_freeze = false;
    g_dumping = false;
    return;
  }
  if (!strcasecmp(line, "SD SAVE") || !strcasecmp(line, "SNAP SD")) {
    snapArchiveToSd();
    return;
  }
  if (!strcasecmp(line, "USB DISK") || !strcasecmp(line, "USB DISK?") ||
      !strcasecmp(line, "MSC?") || !strcasecmp(line, "MSC")) {
    sdMscPrintStatus();
    return;
  }
  if (!strcasecmp(line, "USB DISK ON") || !strcasecmp(line, "MSC ON") ||
      !strcasecmp(line, "USB MSC ON")) {
    if (snapIsRecording()) {
      hostPrintln("# USB DISK blocked — still recording");
      return;
    }
    g_dumping = true;
    g_sample_freeze = true;
    if (g_timer) esp_timer_stop(g_timer);
    sdMscOn();
    // MSC 开着时保持 freeze：不写卡、少打扰
    if (!sdMscIsOn()) {
      if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
      g_sample_freeze = false;
      g_dumping = false;
    }
    return;
  }
  if (!strcasecmp(line, "USB DISK OFF") || !strcasecmp(line, "MSC OFF") ||
      !strcasecmp(line, "USB MSC OFF")) {
    sdMscOff();
    if (g_timer) esp_timer_start_periodic(g_timer, SAMPLE_PERIOD_US);
    g_sample_freeze = false;
    g_dumping = false;
    return;
  }
  if (!strncasecmp(line, "REC MS", 6) || !strncasecmp(line, "REC TIME", 8)) {
    const char* a = line;
    if (!strncasecmp(a, "REC MS", 6)) a += 6;
    else a += 8;
    while (*a == ' ') ++a;
    if (*a) {
      int v = atoi(a);
      if (v < 500) v = 500;
      if (v > 3000) v = 3000;
      v = (v / 100) * 100;
      if (v < 500) v = 500;
      g_rec_duration_ms = (uint32_t)v;
    }
    // 只改「下次」时长，绝不清 RAM / 不停记录
    hostPrintf("# REC MS=%lu (next shot only; snap=%u rec=%d)\n",
               (unsigned long)g_rec_duration_ms, (unsigned)snapCount(),
               snapIsRecording() ? 1 : 0);
    return;
  }
  if (!strncasecmp(line, "DUMP ACK", 8) || !strcasecmp(line, "ACK") ||
      !strcasecmp(line, "DUMP NEXT")) {
    bleDumpTakeAck();
    return;
  }
  if (!strncasecmp(line, "BLE DUMP", 8)) {
    const char* a = line + 8;
    while (*a == ' ') ++a;
    if (!strncasecmp(a, "MS", 2)) {
      int v = atoi(a + 2);
      if (v < 20) v = 20;
      if (v > 500) v = 500;
      g_ble_pkt_ms = (uint16_t)v;
      g_ble_ack_wait_ms = (uint16_t)(v * 2 + 40);
      if (g_ble_ack_wait_ms > 800) g_ble_ack_wait_ms = 800;
      hostPrintf("# OK ble_ms=%u ack_wait=%u\n", (unsigned)g_ble_pkt_ms,
                 (unsigned)g_ble_ack_wait_ms);
    } else if (!strncasecmp(a, "MAX", 3)) {
      int v = atoi(a + 3);
      if (v < 100) v = 100;
      if (v > 4000) v = 4000;
      g_ble_dump_max = (uint16_t)v;
      hostPrintf("# OK ble_max=%u\n", (unsigned)g_ble_dump_max);
    } else {
      hostPrintf("# OK ble_ms=%u ack_wait=%u max=%u\n",
                 (unsigned)g_ble_pkt_ms, (unsigned)g_ble_ack_wait_ms, (unsigned)g_ble_dump_max);
    }
    return;
  }
  if (!strcasecmp(line, "BLE ON") || !strcasecmp(line, "BLE START")) {
    bleBegin();
    hostPrintf("# BLE ON heap_internal=%u\n",
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    return;
  }
  if (!strcasecmp(line, "PING")) {
    // 不灌 BLE，避免心跳冲掉遥测 Notify
    hostSerialPrintln("# PONG");
    return;
  }
  if (!strcasecmp(line, "FW?") || !strcasecmp(line, "VERSION")) {
    hostPrintln("# FW=monitor-v40-counts-bin snap=counts→host ble_pull=RAM|SD pt=16");
    return;
  }
  if (!strncasecmp(line, "TIME", 4)) {
    const char* a = line + 4;
    while (*a == ' ') ++a;
    uint64_t u = strtoull(a, nullptr, 10);
    if (u > 0) applyHostTime(u);
    else hostSerialPrintln("# TIME need unix_ms");
    return;
  }
  if (!strcasecmp(line, "HELP") || !strcasecmp(line, "?")) {
    hostPrintln("# CMD: MONITOR START|STOP | REC NOW | SD SAVE | DUMP BIN BLE | USB DISK …");
    hostPrintln("# 触发: 环缓 → |rpm|>10 且 I+1圈 → 回溯400+2s → RAM→SD；BLE 从 RAM 拉");
    hostPrintln("# DUMP BIN BLE=读RAM(不读SD) | USB DISK ON=Native USB 读卡");
    hostPrintln("# SPI SD: CS=10 SCK=12 MOSI=11 MISO=13 @3.3V");
    return;
  }
  if (!strncasecmp(line, "CAPTURE", 7) || !strncasecmp(line, "CAP ", 4)) {
    const char* a = line;
    if (!strncasecmp(a, "CAPTURE", 7)) a += 7;
    else a += 3;
    while (*a == ' ') ++a;
    if (!strcasecmp(a, "STOP")) {
      capStop();
      hostPrintf("# CAPTURE stop n=%u dur_ms=%lu\n", (unsigned)capCount(),
                 (unsigned long)capDurationMs());
    } else if (!strncasecmp(a, "START", 5)) {
      a += 5;
      while (*a == ' ') ++a;
      uint32_t sec = 30;
      if (*a) {
        float v = atof(a);
        if (v >= 30.0f && v <= 100.0f) sec = (uint32_t)(v + 0.5f);
        else if (v > 100.0f && v <= 100000.0f) sec = (uint32_t)((v / 1000.0f) + 0.5f);  // ms
      }
      if (sec < 30) sec = 30;
      if (sec > 100) sec = 100;
      if (!capStart(sec * 1000UL)) {
        size_t free_b = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        hostPrintf("# CAPTURE fail: no buffer (spiram_free=%u). Reboot after flash capture-v2.\n",
                   (unsigned)free_b);
      } else {
        hostPrintf("# CAPTURE START %lus RAW@2kHz (no gate/filter/MONITOR) buf≈%us\n",
                   (unsigned long)((capRemainMs() + 999) / 1000UL),
                   (unsigned)(capCapacity() / SAMPLE_HZ));
        hostPrintf("# CAPTURE: twist freely — stop/start/accel all saved. Then CAPTURE DUMP\n");
        hostPrintf("# CAPTURE cap=%u remain_ms=%lu\n", (unsigned)capCapacity(),
                   (unsigned long)capRemainMs());
      }
    } else if (!strncasecmp(a, "DUMP", 4)) {
      if (capIsRunning()) {
        hostPrintln("# CAPTURE still running — STOP first or wait done");
      } else {
        const char* p = a + 4;
        while (*p == ' ') ++p;
        uint16_t stride = 10;  // 默认抽稀 ≈200Hz，USB 稳
        if (!strncasecmp(p, "FULL", 4) || !strcasecmp(p, "1") || !strcasecmp(p, "ALL")) {
          stride = 1;
        } else if (*p) {
          int v = atoi(p);
          if (v >= 1 && v <= 50) stride = (uint16_t)v;
        }
        capDumpUsb(stride);
      }
    } else if (!strncasecmp(a, "NEXT", 4)) {
      capDumpNext();
    } else {
      hostPrintf("# CAPTURE run=%d n=%u/%u dur_ms=%lu remain_ms=%lu isr=%lu drop=%lu\n",
                 capIsRunning() ? 1 : 0, (unsigned)capCount(), (unsigned)capCapacity(),
                 (unsigned long)capDurationMs(), (unsigned long)capRemainMs(),
                 (unsigned long)capIsrHits(), (unsigned long)capRingDrop());
    }
    return;
  }
  if (!strcasecmp(line, "REVS CLEAR") || !strcasecmp(line, "INDEX CLEAR")) {
    revsClear();
    hostPrintln("# REVS/INDEX cleared");
    return;
  }
  if (!strcasecmp(line, "READ") || !strcasecmp(line, "LOG READ")) {
    // 对齐 00ACC：主机主动拉取（等同 LOG DUMP BLE）
    uint16_t only_seg = g_ble_pull_seg;
    dumpLog(0, only_seg, true);
    if (only_seg != 0) g_ble_pull_seg = 0;
    return;
  }
  if (!strncasecmp(line, "MONITOR", 7)) {
    const char* a = line + 7;
    while (*a == ' ') ++a;
    if (!strcasecmp(a, "START") || !strcasecmp(a, "ON") || !strcasecmp(a, "1")) {
      monitorStart();
    } else if (!strcasecmp(a, "STOP") || !strcasecmp(a, "OFF") || !strcasecmp(a, "0")) {
      monitorStop();
    } else {
      hostPrintf("# MONITOR armed=%d phase=%s seg=%u pool=%u log_n=%u segs=%u\n",
                 g_armed ? 1 : 0, phaseName(g_phase), (unsigned)g_seg_cur, (unsigned)g_pool_n,
                 (unsigned)g_log_n, (unsigned)g_seg_count);
    }
    return;
  }
  if (!strncasecmp(line, "LOG", 3)) {
    const char* a = line + 3;
    while (*a == ' ') ++a;
    if (!strcasecmp(a, "CLEAR")) {
      logClear();
      poolClear();
      hostPrintln("# LOG cleared (main+pool)");
    } else if (!strncasecmp(a, "DUMP", 4)) {
      // LOG DUMP USB → 仅串口；LOG DUMP BLE → 仅蓝牙；默认：串口 +（若已连）蓝牙各推一份
      bool do_usb = true;
      bool do_ble = true;
      const char* b = a + 4;
      while (*b == ' ') ++b;
      if (!strncasecmp(b, "USB", 3)) {
        do_ble = false;
        b += 3;
        while (*b == ' ') ++b;
      } else if (!strncasecmp(b, "BLE", 3)) {
        do_usb = false;
        b += 3;
        while (*b == ' ') ++b;
      }
      uint16_t only_seg = 0;
      // LOG DUMP … SEG n；USB 若刚 RECORD 完可默认拉该段
      if (!strncasecmp(b, "SEG", 3)) {
        b += 3;
        while (*b == ' ') ++b;
        only_seg = (uint16_t)strtoul(b, nullptr, 10);
        while (*b && *b != ' ') ++b;
        while (*b == ' ') ++b;
      } else if (g_ble_pull_seg != 0) {
        only_seg = g_ble_pull_seg;
      }
      size_t max_n = 0;
      if (*b) max_n = (size_t)strtoul(b, nullptr, 10);
      if (do_usb) {
        Serial.printf("# AUTO DUMP BEGIN seg=%u expect_D=%lu (PC pull)\n", (unsigned)only_seg,
                      (unsigned long)g_auto_dump_expect);
        dumpLog(max_n, only_seg, false);
        Serial.printf("# AUTO DUMP END seg=%u\n", (unsigned)only_seg);
        if (only_seg != 0 && only_seg == g_ble_pull_seg) g_ble_pull_seg = 0;
      }
      if (do_ble) {
        dumpLog(max_n, only_seg, true);
        if (only_seg != 0 && only_seg == g_ble_pull_seg) g_ble_pull_seg = 0;
      }
    } else {
      hostPrintf("# LOG n=%u cap=%u drop=%lu pool=%u segs=%u phase=%s | I_n=%u I_drop=%lu\n",
                 (unsigned)g_log_n, (unsigned)g_log_cap, (unsigned long)g_log_drop,
                 (unsigned)g_pool_n, (unsigned)g_seg_count, phaseName(g_phase),
                 (unsigned)g_ilog_n, (unsigned long)g_ilog_drop);
    }
    return;
  }
  if (!strcasecmp(line, "ABI?")) {
    Snap s = takeSnap();
    hostPrintf(
        "# ABI hz=%lu meas=%lu rpm=%.2f armed=%d phase=%s | revs_abi=%.3f revs_abs=%.3f "
        "I_n=%lu I_signed=%ld (GPIO17)\n",
        (unsigned long)SAMPLE_HZ, (unsigned long)s.sample_hz_meas, s.rpm, g_armed ? 1 : 0,
        phaseName(g_phase), s.revs_total, s.revs_abs, (unsigned long)s.index_n,
        (long)s.index_signed);
    return;
  }
  if (!strncasecmp(line, "BLE RATE", 8)) {
    bleSetRate((uint16_t)atoi(line + 8));
    hostPrintf("# BLE RATE %u\n", (unsigned)bleTelemHz());
    return;
  }
  hostPrintf("# ERR unknown: %s\n", line);
}

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(921600);
  delay(300);
  Serial.println();
  Serial.println("# ESP32 AS5047P ABI Monitor FW=monitor-v40-counts-bin");
  Serial.println("# Policy: idle=BLE live RPM; MONITOR armed=BLE telem OFF (events only)");
  Serial.println("# Shot: once STAGING/triggered, keep full REC window even if RPM→0");
  Serial.println("# Android tip: dump = Notify + DUMP ACK; avoid continuous GATT READ");
  Serial.println("# Policy: arm → trigger → SD snap_YYYYMMDD_HHMMSS_n.bin (+ .txt meta)");
  Serial.println("# PC connect sends TIME → wall clock for SD filenames (CST-8)");

  // 长采/MONITOR 大缓冲按需分配，保证 SD FatFS 有内部堆
  Serial.println("# CAPTURE/MONITOR buffers deferred (heap for SD)");

  snapSetStepsPerRev(ABI_STEPS_PER_REV);
  if (!snapAlloc()) Serial.println("# WARN snap alloc failed");
  else Serial.println("# SNAP RAM buffer 5000x12 INTERNAL (~60KB) OK");

  if (!sdBegin()) Serial.println("# WARN SD not ready — check wiring, then SD INIT / SD TEST");
  else sdSelfTest();

  if (!pcntBegin()) Serial.println("# FATAL PCNT failed");
  else if (!sampleTimerBegin()) Serial.println("# FATAL timer failed");
  else Serial.println("# ABI PCNT + 2kHz OK");
  revsClear();

  // 主路径要蓝牙搜得到：上电即广播 OEZ-ABI（堆紧张时仍可 SD；失败再 BLE ON）
  bleBegin();
  Serial.printf("# BLE name=%s (auto on boot)\n", BLE_DEVICE_NAME);
  Serial.printf("# heap after setup internal=%u spiram=%u\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  Serial.printf("# USB DISK available=%u — after SAVE: USB DISK ON\n",
                (unsigned)sdMscAvailable());
  Serial.println("# ready. MONITOR START → ring → trigger → SD; BLE→DUMP BIN BLE(src=RAM) / USB→DISK ON");
}

void loop() {
  static bool s_snap_was_rec = false;
  bool snap_rec = snapIsRecording();
  if (s_snap_was_rec && !snap_rec) {
    // 正式段记满：解除武装（ALIVE/SD 仍继续）
    g_armed = false;
    g_phase = PH_IDLE;
    g_seg_cur = 0;
    bleMuteHost(false);
    g_evt_n = (uint32_t)snapCount();
    g_evt_code = 3;
    g_evt_pending = true;
  }
  s_snap_was_rec = snap_rec;

  snapPollDone();
  if (snapTakeArchiveEdge()) {
    snapArchiveToSd();
  }
  capPoll();      // 长采：DRAM 环 → PSRAM
  capDumpPoll();  // 长采 DUMP：ACK 分块非阻塞

  if (bleTakeConnectEdge()) {
    // 重连必须恢复实时遥测：中止可能卡住的 BLE 回传
    g_dump_abort = true;
    g_dumping = false;
    sessionBegin("ble_connect");
  }

  if (capTakeDoneEdge()) {
    hostPrintf("# CAPTURE DONE n=%u dur_ms=%lu — send CAPTURE DUMP to pull\n",
               (unsigned)capCount(), (unsigned long)capDurationMs());
  }

  // 长采剩余时间：每 0.5s 报一次（USB；可启停电机，倒计时照走）
  if (capIsRunning()) {
    static uint32_t last_cap_prog_ms = 0;
    uint32_t nowp = millis();
    if (nowp - last_cap_prog_ms >= 500) {
      last_cap_prog_ms = nowp;
      uint32_t rem = capRemainMs();
      uint32_t rem_s = (rem + 999) / 1000UL;
      Serial.printf("# CAPTURE PROG remain_s=%lu remain_ms=%lu n=%u\n",
                    (unsigned long)rem_s, (unsigned long)rem, (unsigned)capCount());
    }
  }

  if (g_evt_pending) {
    g_evt_pending = false;
    uint8_t c = g_evt_code;
    if (c == 1) {
      // 串口详单；BLE 在 mute 下只发短行（防大包 Notify 断链）
      hostPrintf("# CONFIRM |rpm|>%.0f & I+1rev seg=%u snap_n=%lu → +REC %lums (backtrack≤%u)\n",
                 RPM_GATE, (unsigned)g_evt_seg, (unsigned long)g_evt_n,
                 (unsigned long)g_rec_duration_ms, (unsigned)BACKTRACK_N);
      if (bleConnected()) {
        char s[72];
        snprintf(s, sizeof(s), "# CONFIRM seg=%u n=%lu\n", (unsigned)g_evt_seg,
                 (unsigned long)g_evt_n);
        bleSendLine(s);  // 单行状态，可越过 mute
      }
      g_noise_why = 0;
    } else if (c == 2) {
      const char* why = "unknown";
      if (g_noise_why == 2) why = "flip";
      else if (g_noise_why == 3) why = "below_gate";
      else if (g_noise_why == 4) why = "timeout";
      else if (g_noise_why == 5) why = "pool_ovf";
      hostPrintf("# NOISE discard seg=%u why=%s counts=%lu\n", (unsigned)g_evt_seg, why,
                 (unsigned long)g_evt_n);
      if (bleConnected()) bleSendLine("# NOISE discard\n");
    } else if (c == 3) {
      hostPrintf("# RECORD done n=%lu → ALIVE then SD SAVE (disarmed)\n",
                 (unsigned long)g_evt_n);
      if (bleConnected()) {
        // 短标记：手机专门认 CD15 / RECORD done 开 15s 倒计时
        char s[96];
        snprintf(s, sizeof(s), "# RECORD done n=%lu → SD SAVE\n", (unsigned long)g_evt_n);
        bleSendLine(s);
        snprintf(s, sizeof(s), "# CD15 n=%lu\n", (unsigned long)g_evt_n);
        bleSendLine(s);
      }
      // 旧 log 路径才自推 LOG DUMP；snap 有 RAM 点数时走 SD SAVE / DUMP BIN BLE，勿发 AUTO DUMP
      if (snapCount() == 0 && g_log_n > 0 && g_evt_n == (uint32_t)g_log_n) {
        requestAutoDump(g_evt_seg);
      }
    } else if (c == 6) {
      hostPrintf("# STAGING |rpm|>%.0f waiting I+1rev ring=%u\n", RPM_GATE,
                 (unsigned)snapRingCount());
      // 短行通知手机立刻静默 GATT 轮询（PC 无此问题）；勿发长 STAGING 文案
      if (bleConnected()) bleSendLine("# QUIET\n");
    } else if (c == 5) {
      const char* why = "stop";
      if (g_noise_why == 2) why = "flip";
      else if (g_noise_why == 3) why = "rpm_stop";
      else if (g_noise_why == 4) why = "timeout";
      hostPrintf("# SALVAGE keep seg=%u log_n=%lu why=%s (data NOT deleted)\n",
                 (unsigned)g_evt_seg, (unsigned long)g_evt_n, why);
      requestAutoDump(g_evt_seg);
    } else if (c == 4) {
      hostPrintln("# NOISE discard: pool overflow");
    }
  }

  if (g_need_auto_dump) {
    g_need_auto_dump = false;
    uint16_t seg = g_auto_dump_seg;
    // 按段精确点数（含触发前 backtrack + RECORD 段）
    uint32_t expect = 0;
    {
      size_t n, w;
      portENTER_CRITICAL(&g_log_mux);
      n = g_log_n;
      w = g_log_w;
      portEXIT_CRITICAL(&g_log_mux);
      if (g_log && n > 0 && seg != 0) {
        size_t start = (w + g_log_cap - n) % g_log_cap;
        for (size_t i = 0; i < n; ++i) {
          if (g_log[(start + i) % g_log_cap].seg == seg) expect++;
        }
      } else {
        expect = g_auto_dump_expect;
      }
    }
    g_auto_dump_expect = expect;
    g_ble_pull_seg = seg;
    g_dump_started = false;
    g_dumping = false;
    // READY 阶段保持采样/遥测，避免「电机在转、板子已冻死」；DUMP 时再停
    g_sample_freeze = false;

    Serial.printf("# RECORD_DONE|%lu\n", (unsigned long)expect);
    Serial.printf(
        "# AUTO DUMP READY seg=%u expect_D=%lu (backtrack+record) — send LOG DUMP USB SEG %u\n",
        (unsigned)seg, (unsigned long)expect, (unsigned)seg);
    if (bleConnected()) {
      char ble_msg[140];
      snprintf(ble_msg, sizeof(ble_msg), "# RECORD_DONE|%lu\n", (unsigned long)expect);
      bleSendLine(ble_msg);
      snprintf(ble_msg, sizeof(ble_msg),
               "# AUTO DUMP READY seg=%u expect_D=%lu (send READ or LOG DUMP BLE SEG %u)\n",
               (unsigned)seg, (unsigned long)expect, (unsigned)seg);
      bleSendLine(ble_msg);
    }
    g_ready_ms = millis();
    if (expect == 0) {
      Serial.println("# WARN READY but expect_D=0 (log empty for seg) — nothing to dump");
      g_ready_ms = 0;
    }
  }

  // PC 超时未拉：板子自行慢速导出（防「没有记录到数据」）
  if (g_ready_ms != 0 && !g_dump_started && g_auto_dump_seg != 0 && g_auto_dump_expect > 0) {
    if ((millis() - g_ready_ms) >= 1200) {
      uint16_t seg = g_auto_dump_seg;
      g_ready_ms = 0;
      Serial.printf("# AUTO DUMP FALLBACK seg=%u expect_D=%lu (PC silent)\n", (unsigned)seg,
                    (unsigned long)g_auto_dump_expect);
      Serial.printf("# AUTO DUMP BEGIN seg=%u expect_D=%lu (board fallback)\n", (unsigned)seg,
                    (unsigned long)g_auto_dump_expect);
      dumpLog(0, seg, false);
      Serial.printf("# AUTO DUMP END seg=%u\n", (unsigned)seg);
      g_ble_pull_seg = 0;
    }
  } else if (g_dump_started) {
    g_ready_ms = 0;
  }

  static char usb_line[128];
  static size_t usb_len = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (usb_len > 0) {
        usb_line[usb_len] = '\0';
        handleCmd(usb_line);
        usb_len = 0;
      }
    } else if (usb_len + 1 < sizeof(usb_line)) {
      usb_line[usb_len++] = c;
    } else {
      usb_len = 0;
    }
  }

  char ble_line[128];
  if (bleTakeRxLine(ble_line, sizeof(ble_line))) handleCmd(ble_line);

  // 武装期间不往 BLE 灌 # MON（实时流已分离）；仅串口诊断
  static uint32_t last_mon_diag_ms = 0;
  uint32_t now_diag = millis();
  if (g_armed && !g_dumping && g_phase == PH_IDLE && !snapIsRecording() &&
      (now_diag - last_mon_diag_ms) >= 1000) {
    last_mon_diag_ms = now_diag;
    Snap sdiag = takeSnap();
    Serial.printf("# MON phase=%s rpm=%.0f snap=%u ring=%u rec=%d idx=%lu\n", phaseName(g_phase),
                  (double)fabsf(sdiag.rpm), (unsigned)snapCount(), (unsigned)snapRingCount(),
                  snapIsRecording() ? 1 : 0, (unsigned long)sdiag.index_n);
  }

  static uint32_t last_telem_ms = 0;
  uint32_t now = millis();
  uint16_t want_hz = bleTelemHz();
  if (capIsDumping()) {
    // 不发 L
  } else if (snapIsRecording() || g_phase == PH_STAGING || g_phase == PH_RECORD) {
    // 记录窗口：loop 仍可走 emitTelem→Serial，但 BLE 短帧已在 emitTelem 内关闭
    // 降到 1Hz 减轻 USB；BLE 侧无包
    uint32_t period = 1000;
    if (now - last_telem_ms >= period) {
      last_telem_ms = now;
      emitTelem(takeSnap());
    }
  } else {
    if (capIsRunning()) want_hz = 1;
    uint32_t period = 1000UL / (uint32_t)want_hz;
    if (period < 20) period = 20;
    if (now - last_telem_ms >= period) {
      last_telem_ms = now;
      emitTelem(takeSnap());
    }
  }
}
