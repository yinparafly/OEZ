/************************************************
 * ESP32-S3：AS5047P + 航模电调转速控制
 *
 * 电调：GPIO9，默认 50Hz，脉宽 1000~2000us（上电先最低油门）
 *   FREQ <50..400> 可改刷新率（测完请回 50）
 * 策略：台阶辨识前馈图 f_inv + PI；profile: noload / flap
 *
 * 指令（行末 \n）：
 *   START | STOP | ESTOP | PING
 *   RPM <0..6000> | RPMMAX <rpm> | SOFT ON|OFF | SOFT RATE <rpm/s>
 *   MODE OPEN|CLOSED|LEARN|SAFE
 *   PROFILE noload|flap
 *   LEARN START | LEARN ABORT | LEARN MAXUS <us>
 *   PID <kp> <ki> <kd> | PID? | PID SAVE
 *   PHASE ZERO | PHASE? | MOVE CW|CCW <deg> [rpm]
 *   STOPAT <deg>|OFF | GOTO <deg> [CW|CCW|AUTO] [rpm]
 *   ADAPT ON|OFF | PWM <1000..2000>
 ************************************************/
#include <SPI.h>
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---- 引脚 ----
static const int PIN_CS   = 10;
static const int PIN_SCLK = 12;
static const int PIN_MISO = 13;
static const int PIN_MOSI = 11;
static const int PIN_ESC_PWM = 9;

// ---- 时序 ----
static const uint32_t SERIAL_BAUD = 921600;
// 100Hz 时角度差分 (±180°/样本) 理论测速上限≈3000RPM；6000 目标需 >200Hz
static const uint32_t CTRL_HZ = 250;
static const uint32_t CTRL_MS = 1000 / CTRL_HZ;
static const uint32_t HOST_TIMEOUT_MS = 1500;

// ---- 电调 PWM（默认 50Hz，可用 FREQ 命令改到 400）----
static const uint32_t ESC_PWM_HZ_DEFAULT = 50;
static const uint16_t ESC_PULSE_MIN_US = 1000;
static const uint16_t ESC_PULSE_MAX_US = 2000;
static const uint8_t  ESC_PWM_RES_BITS = 14;
// 电机允许最大转速（对应 6S）；目标/超速/学习截止均按此约束。当前 3S 调试达不到 6000，属预期。
static const float    MOTOR_RPM_ABS_MAX = 6000.0f;
static const float    TARGET_RPM_MAX_DEFAULT = 6000.0f;

uint32_t esc_pwm_hz = ESC_PWM_HZ_DEFAULT;
uint32_t esc_period_us = 1000000UL / ESC_PWM_HZ_DEFAULT;
float target_rpm_max = TARGET_RPM_MAX_DEFAULT;

// ---- 学习 ----
static const uint16_t LEARN_STEP_US = 50;
// 学习油门硬顶=电调最大 2000μs；未达转速限制前不得提前结束（禁止再卡在 1600）
static const uint16_t LEARN_MAX_US_DEFAULT = ESC_PULSE_MAX_US;
static const uint32_t LEARN_SETTLE_MS = 2000;  // 每台阶停留，再升下一档
static const int      MAP_MAX_POINTS = 32;
static const float    LEARN_RPM_STOP_RATIO = 0.98f;  // 达电机最大转速的 98% 才因转速停

static const uint16_t ENC_CPR = 16384;  // AS5047P 14bit：一圈 16384 点
static const float    ENC_DEG_PER_COUNT = 360.0f / (float)ENC_CPR;
static const uint16_t REG_NOP = 0x0000;
static const uint16_t REG_ERRFL = 0x0001;
static const uint16_t REG_DIAAGC = 0x3FFC;
static const uint16_t REG_ANGLECOM = 0x3FFF;
static const float TWO_PI_F = 6.28318530717958647692f;

enum CtrlMode : uint8_t {
  MODE_SAFE = 0,
  MODE_OPEN = 1,
  MODE_CLOSED = 2,
  MODE_LEARN = 3,
  MODE_MEASURE = 4
};
enum RunState : uint8_t { RUN_IDLE = 0, RUN_RUNNING = 1, RUN_ESTOP = 2 };
enum ProfileId : uint8_t { PROF_NOLOAD = 0, PROF_FLAP = 1 };

struct AngleSample {
  uint16_t raw;
  bool ef;
  uint8_t agc;
  bool mag_low;
  bool mag_high;
};

struct ThrottleMap {
  int n;
  uint16_t pulse[MAP_MAX_POINTS];
  float rpm[MAP_MAX_POINTS];
  bool valid;
};

SPIClass* spi = &SPI;
Preferences prefs;

float last_deg = NAN;
uint32_t last_ms = 0;
float rpm_filt = 0.0f;       // 滤波后带符号转速（符号=编码器瞬时方向）
float rpm_signed_stable = 0.0f;  // 符号锁定后的转速（单向转时不乱跳正负）
int rpm_sign_lock = 0;       // -1/0/+1，高速后锁定方向
uint8_t rpm_sign_flip_cnt = 0;
uint8_t last_agc = 0;
bool last_mag_low = false;
bool last_mag_high = false;

uint16_t esc_pulse_us = ESC_PULSE_MIN_US;
float target_cmd = 0.0f;      // 用户设定
float target_ramped = 0.0f;   // 缓启动后
bool soft_enable = true;
float soft_rate_rpm_s = 600.0f;

CtrlMode ctrl_mode = MODE_OPEN;
RunState run_state = RUN_IDLE;
ProfileId profile_id = PROF_NOLOAD;

float kp = 0.08f;
float ki = 0.25f;
float kd = 0.0f;
float pid_i = 0.0f;
float pid_prev_err = 0.0f;
bool adapt_on = false;

ThrottleMap maps[2];
uint16_t learn_max_us = LEARN_MAX_US_DEFAULT;
uint16_t learn_pulse = ESC_PULSE_MIN_US;
uint32_t learn_step_t0 = 0;
int learn_idx = 0;
bool learn_active = false;

uint16_t measure_pulse = ESC_PULSE_MIN_US;
bool measure_active = false;
bool esccal_high = false;

// 相位零点 / 相对转动 / 停机相位（deg：用户坐标系，CW=编码器角度增加）
float phase_zero_deg = 0.0f;
int esc_sense = 1;             // +1=给油门时编码器角度增加(CW), -1=减少；单向电调只沿此向转
bool move_active = false;
int move_dir = 1;              // 实际转动方向（恒等于 esc_sense）
float move_target_deg = 0.0f;
float move_accum_deg = 0.0f;
float move_crawl_rpm = 180.0f;
uint32_t move_wrong_ms = 0;
bool stopat_on = false;
float stopat_deg = 0.0f;
float stopat_tol_deg = 4.0f;
float stopat_prev_rel = NAN;
// 自动辨识转向（用固定脉宽，避免 RPM→脉宽在未学习时过低转不动）
bool sense_learn = false;
float sense_accum = 0.0f;
uint32_t sense_t0 = 0;
uint16_t sense_pulse_us = 1400;
static const uint32_t SENSE_MS = 1800;
static const uint16_t SENSE_PULSE_DEFAULT = 1400;
static const uint16_t CRAWL_PULSE_MIN = 1250;  // 开环爬行最低脉宽，保证能转起来

uint32_t last_host_ms = 0;
bool host_seen = false;

// ---------------- SPI / 编码器 ----------------
uint16_t evenParityBit(uint16_t value15) {
  uint16_t x = value15 & 0x7FFF;
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

uint16_t buildReadCmd(uint16_t addr14) {
  uint16_t cmd = (1u << 14) | (addr14 & 0x3FFF);
  cmd |= (evenParityBit(cmd) << 15);
  return cmd;
}

bool checkEvenParity(uint16_t frame) {
  return evenParityBit(frame) == ((frame >> 15) & 1);
}

uint16_t spiFrame(uint16_t mosi) {
  digitalWrite(PIN_CS, LOW);
  uint16_t miso = spi->transfer16(mosi);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(1);
  return miso;
}

void parseDiaagc(uint16_t dia) {
  if (!checkEvenParity(dia)) return;
  last_agc = dia & 0xFF;
  last_mag_high = (dia >> 10) & 1;
  last_mag_low = (dia >> 11) & 1;
}

AngleSample readAngleCom() {
  static bool primed = false;
  static const uint16_t CMD_ANGLE = buildReadCmd(REG_ANGLECOM);
  static const uint16_t CMD_DIA = buildReadCmd(REG_DIAAGC);
  static uint16_t diag_div = 0;
  AngleSample s = {};

  if (!primed) {
    spiFrame(CMD_DIA);
    parseDiaagc(spiFrame(CMD_ANGLE));
    primed = true;
  }
  if (++diag_div >= 50) {
    diag_div = 0;
    spiFrame(CMD_DIA);
    parseDiaagc(spiFrame(CMD_ANGLE));
  }
  uint16_t rx = spiFrame(CMD_ANGLE);
  s.ef = (rx >> 14) & 1;
  s.raw = rx & 0x3FFF;
  s.agc = last_agc;
  s.mag_low = last_mag_low;
  s.mag_high = last_mag_high;
  return s;
}

float unwrapDeltaDeg(float cur, float prev) {
  float d = cur - prev;
  if (d > 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

/** 由角度差分得到转速；单向高速时锁定符号，避免噪声造成 ±RPM 乱跳 */
float updateRpmFromDelta(float ddeg, float dt) {
  if (dt <= 0.0005f) return fabsf(rpm_signed_stable);
  float rpm_inst = (ddeg / 360.0f) / dt * 60.0f;
  // 拒绝明显不可能的尖峰（噪声/丢样）
  if (fabsf(rpm_inst) > (target_rpm_max * 1.5f + 500.0f)) {
    rpm_inst = rpm_filt;
  }
  rpm_filt = 0.85f * rpm_filt + 0.15f * rpm_inst;

  float mag = fabsf(rpm_filt);
  int s = (rpm_filt > 0.0f) ? 1 : ((rpm_filt < 0.0f) ? -1 : 0);

  // 低速：不锁定，跟滤波符号
  if (mag < 80.0f) {
    rpm_sign_lock = 0;
    rpm_sign_flip_cnt = 0;
    rpm_signed_stable = rpm_filt;
    return mag;
  }

  // 高速：锁定主方向；偶发反向差分需连续多次才翻转
  if (rpm_sign_lock == 0) {
    if (s != 0) rpm_sign_lock = s;
  } else if (s != 0 && s != rpm_sign_lock) {
    rpm_sign_flip_cnt++;
    if (rpm_sign_flip_cnt >= 8) {  // ~80ms @100Hz
      rpm_sign_lock = s;
      rpm_sign_flip_cnt = 0;
    }
  } else {
    rpm_sign_flip_cnt = 0;
  }

  int use = (rpm_sign_lock != 0) ? rpm_sign_lock : s;
  if (use == 0) use = 1;
  rpm_signed_stable = (float)use * mag;
  return mag;  // 控制/学习用非负大小
}

float wrap360(float d) {
  while (d < 0.0f) d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  return d;
}

float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

float phaseRelFromAbs(float abs_deg) {
  return wrap360(abs_deg - phase_zero_deg);
}

// ---------------- ESC PWM ----------------
uint32_t pulseUsToDuty(uint16_t pulse_us) {
  const uint32_t max_duty = (1u << ESC_PWM_RES_BITS) - 1u;
  uint32_t period = esc_period_us;
  if (period < 1) period = 1;
  if (pulse_us > period) pulse_us = (uint16_t)period;
  return (uint32_t)(((uint64_t)pulse_us * max_duty) / period);
}

void applyEscPulse(uint16_t pulse_us) {
  if (pulse_us < ESC_PULSE_MIN_US) pulse_us = ESC_PULSE_MIN_US;
  if (pulse_us > ESC_PULSE_MAX_US) pulse_us = ESC_PULSE_MAX_US;
  esc_pulse_us = pulse_us;
  ledcWrite(PIN_ESC_PWM, pulseUsToDuty(pulse_us));
}

bool setEscPwmFreq(uint32_t hz) {
  if (hz < 50) hz = 50;
  if (hz > 400) hz = 400;
  // 先拉最低油门再改频，避免改频瞬间毛刺
  applyEscPulse(ESC_PULSE_MIN_US);
  ledcDetach(PIN_ESC_PWM);
  esc_pwm_hz = hz;
  esc_period_us = 1000000UL / hz;
  if (!ledcAttach(PIN_ESC_PWM, esc_pwm_hz, ESC_PWM_RES_BITS)) {
    // 回退 50Hz
    esc_pwm_hz = ESC_PWM_HZ_DEFAULT;
    esc_period_us = 1000000UL / esc_pwm_hz;
    ledcAttach(PIN_ESC_PWM, esc_pwm_hz, ESC_PWM_RES_BITS);
    applyEscPulse(ESC_PULSE_MIN_US);
    return false;
  }
  applyEscPulse(ESC_PULSE_MIN_US);
  return true;
}

void setupEscPwmMinFirst() {
  pinMode(PIN_ESC_PWM, OUTPUT);
  digitalWrite(PIN_ESC_PWM, LOW);
  esc_pwm_hz = ESC_PWM_HZ_DEFAULT;
  esc_period_us = 1000000UL / esc_pwm_hz;
  ledcAttach(PIN_ESC_PWM, esc_pwm_hz, ESC_PWM_RES_BITS);
  applyEscPulse(ESC_PULSE_MIN_US);
}

// ---------------- 前馈图 ----------------
void mapClear(ThrottleMap& m) {
  m.n = 0;
  m.valid = false;
}

void mapInitLinear(ThrottleMap& m) {
  m.n = 2;
  m.pulse[0] = ESC_PULSE_MIN_US;
  m.rpm[0] = 0.0f;
  m.pulse[1] = ESC_PULSE_MAX_US;
  m.rpm[1] = target_rpm_max;
  m.valid = true;
}

float mapRpmFromPulse(const ThrottleMap& m, uint16_t pulse) {
  if (!m.valid || m.n < 2) {
    return target_rpm_max * (float)(pulse - ESC_PULSE_MIN_US) /
           (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US);
  }
  if (pulse <= m.pulse[0]) return m.rpm[0];
  if (pulse >= m.pulse[m.n - 1]) return m.rpm[m.n - 1];
  for (int i = 0; i < m.n - 1; i++) {
    if (pulse >= m.pulse[i] && pulse <= m.pulse[i + 1]) {
      float t = (float)(pulse - m.pulse[i]) / (float)(m.pulse[i + 1] - m.pulse[i]);
      return m.rpm[i] + t * (m.rpm[i + 1] - m.rpm[i]);
    }
  }
  return m.rpm[m.n - 1];
}

uint16_t mapPulseFromRpm(const ThrottleMap& m, float rpm) {
  if (rpm <= 0.0f) return ESC_PULSE_MIN_US;
  if (!m.valid || m.n < 2) {
    float span = (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US);
    return (uint16_t)(ESC_PULSE_MIN_US + (rpm / target_rpm_max) * span + 0.5f);
  }
  if (rpm <= m.rpm[0]) return m.pulse[0];
  if (rpm >= m.rpm[m.n - 1]) return m.pulse[m.n - 1];
  for (int i = 0; i < m.n - 1; i++) {
    if (rpm >= m.rpm[i] && rpm <= m.rpm[i + 1]) {
      float den = m.rpm[i + 1] - m.rpm[i];
      float t = (den > 1e-3f) ? ((rpm - m.rpm[i]) / den) : 0.0f;
      return (uint16_t)(m.pulse[i] + t * (m.pulse[i + 1] - m.pulse[i]) + 0.5f);
    }
  }
  return m.pulse[m.n - 1];
}

ThrottleMap& activeMap() { return maps[profile_id]; }

const char* profileName() {
  return (profile_id == PROF_FLAP) ? "flap" : "noload";
}

const char* modeName() {
  switch (ctrl_mode) {
    case MODE_SAFE: return "SAFE";
    case MODE_OPEN: return "OPEN";
    case MODE_CLOSED: return "CLOSED";
    case MODE_LEARN: return "LEARN";
    case MODE_MEASURE: return "MEASURE";
  }
  return "?";
}

void saveMapToNvs(ProfileId id) {
  char key[16];
  snprintf(key, sizeof(key), "map%d", (int)id);
  prefs.putBytes(key, &maps[id], sizeof(ThrottleMap));
}

void loadMapsFromNvs() {
  mapInitLinear(maps[PROF_NOLOAD]);
  mapInitLinear(maps[PROF_FLAP]);
  for (int id = 0; id < 2; id++) {
    char key[16];
    snprintf(key, sizeof(key), "map%d", id);
    ThrottleMap tmp;
    size_t n = prefs.getBytes(key, &tmp, sizeof(tmp));
    if (n == sizeof(tmp) && tmp.n >= 2 && tmp.n <= MAP_MAX_POINTS) {
      maps[id] = tmp;
      maps[id].valid = true;
    }
  }
  kp = prefs.getFloat("kp", kp);
  ki = prefs.getFloat("ki", ki);
  kd = prefs.getFloat("kd", kd);
  phase_zero_deg = prefs.getFloat("ph0", 0.0f);
  esc_sense = prefs.getInt("esense", 1);
  if (esc_sense != 1 && esc_sense != -1) esc_sense = 1;
}

void savePhaseZeroToNvs() {
  prefs.putFloat("ph0", phase_zero_deg);
}

void saveEscSenseToNvs() {
  prefs.putInt("esense", esc_sense);
}

void savePidToNvs() {
  prefs.putFloat("kp", kp);
  prefs.putFloat("ki", ki);
  prefs.putFloat("kd", kd);
}

// ---------------- 控制 ----------------
void clearPid() {
  pid_i = 0.0f;
  pid_prev_err = 0.0f;
}

void doEstop(const char* reason) {
  run_state = RUN_ESTOP;
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  clearPid();
  learn_active = false;
  measure_active = false;
  move_active = false;
  sense_learn = false;
  esccal_high = false;
  if (ctrl_mode == MODE_LEARN || ctrl_mode == MODE_MEASURE) ctrl_mode = MODE_OPEN;
  applyEscPulse(ESC_PULSE_MIN_US);
  Serial.printf("# ESTOP %s pulse=%u\n", reason, (unsigned)esc_pulse_us);
}

void doStop(bool immediate) {
  target_cmd = 0.0f;
  if (immediate || !soft_enable) {
    target_ramped = 0.0f;
    applyEscPulse(ESC_PULSE_MIN_US);
  }
  clearPid();
  run_state = RUN_IDLE;
  learn_active = false;
  move_active = false;
  sense_learn = false;
  if (ctrl_mode == MODE_LEARN) ctrl_mode = MODE_OPEN;
  Serial.printf("# ACK STOP soft=%d pulse=%u\n", soft_enable && !immediate, (unsigned)esc_pulse_us);
}

void doStart() {
  if (run_state == RUN_ESTOP) {
    Serial.println("# ERR clear ESTOP first with STOP");
    return;
  }
  if (ctrl_mode == MODE_SAFE) {
    Serial.println("# ERR MODE SAFE; set MODE OPEN|CLOSED");
    return;
  }
  if (ctrl_mode == MODE_LEARN || ctrl_mode == MODE_MEASURE) {
    Serial.println("# ERR cannot START in LEARN/MEASURE");
    return;
  }
  run_state = RUN_RUNNING;
  if (!soft_enable) target_ramped = target_cmd;
  clearPid();
  last_host_ms = millis();
  host_seen = true;
  Serial.printf("# ACK START target=%.1f mode=%s\n", (double)target_cmd, modeName());
}

void updateSoftRamp(float dt) {
  if (!soft_enable) {
    target_ramped = target_cmd;
    return;
  }
  float max_step = soft_rate_rpm_s * dt;
  float err = target_cmd - target_ramped;
  if (err > max_step) target_ramped += max_step;
  else if (err < -max_step) target_ramped -= max_step;
  else target_ramped = target_cmd;
}

void adaptStep(float err, float dt) {
  if (!adapt_on || run_state != RUN_RUNNING || ctrl_mode != MODE_CLOSED) return;
  // 有界 MIT 式：误差大则略增 Kp
  float dkp = 0.00002f * err * err * ((err > 0) ? 1.0f : -1.0f);
  (void)dt;
  kp += dkp;
  if (kp < 0.01f) kp = 0.01f;
  if (kp > 0.5f) kp = 0.5f;
}

uint16_t computePulseOpen(float rpm_tgt) {
  // 正常开环：严格按前馈图，禁止再抬最低油门（否则设 600 也会被抬到 ~1250μs→实测三千转）
  return mapPulseFromRpm(activeMap(), rpm_tgt);
}

uint16_t computePulseCrawl(float rpm_tgt) {
  uint16_t p = mapPulseFromRpm(activeMap(), rpm_tgt);
  if (rpm_tgt > 1.0f && p < CRAWL_PULSE_MIN) p = CRAWL_PULSE_MIN;
  return p;
}

uint16_t computePulseClosed(float rpm_tgt, float rpm_meas, float dt) {
  uint16_t ff = mapPulseFromRpm(activeMap(), rpm_tgt);
  // 转速用绝对值做闭环（符号只表示编码器转向，不是“负目标转速”）
  float meas = fabsf(rpm_meas);
  float err = rpm_tgt - meas;
  adaptStep(err, dt);
  pid_i += err * dt;
  // 积分抗饱和
  if (pid_i > 800.0f) pid_i = 800.0f;
  if (pid_i < -800.0f) pid_i = -800.0f;
  float derr = (dt > 1e-4f) ? ((err - pid_prev_err) / dt) : 0.0f;
  pid_prev_err = err;
  float u = kp * err + ki * pid_i + kd * derr;
  long pulse = (long)ff + (long)(u + (u >= 0 ? 0.5f : -0.5f));
  if (pulse < ESC_PULSE_MIN_US) pulse = ESC_PULSE_MIN_US;
  if (pulse > ESC_PULSE_MAX_US) pulse = ESC_PULSE_MAX_US;
  return (uint16_t)pulse;
}

void learnBegin() {
  if (run_state == RUN_RUNNING) {
    Serial.println("# ERR STOP before LEARN");
    return;
  }
  // 强制扫到电调满油门；未达转速限制前一直升高油门（不再接受 1600 提前结束）
  learn_max_us = ESC_PULSE_MAX_US;
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  clearPid();
  run_state = RUN_IDLE;
  ctrl_mode = MODE_LEARN;
  learn_active = true;
  learn_pulse = ESC_PULSE_MIN_US;
  learn_idx = 0;
  learn_step_t0 = millis();
  mapClear(activeMap());
  activeMap().pulse[0] = ESC_PULSE_MIN_US;
  activeMap().rpm[0] = 0.0f;
  activeMap().n = 1;
  applyEscPulse(ESC_PULSE_MIN_US);
  float rpm_stop = target_rpm_max * LEARN_RPM_STOP_RATIO;
  Serial.printf(
      "# ACK LEARN START profile=%s max_us=%u rpm_stop=%.0f rpm_limit=%.0f step=%uus settle=%lums\n",
      profileName(), (unsigned)learn_max_us, (double)rpm_stop, (double)target_rpm_max,
      (unsigned)LEARN_STEP_US, (unsigned long)LEARN_SETTLE_MS);
  Serial.println("# Learn stops only when: RPM>=rpm_stop OR pulse>=2000 OR map full");
  Serial.printf("# LEARN LOG begin t_ms=%lu pulse=%u\n",
                (unsigned long)millis(), (unsigned)learn_pulse);
}

void learnAbort() {
  learn_active = false;
  ctrl_mode = MODE_OPEN;
  applyEscPulse(ESC_PULSE_MIN_US);
  target_cmd = 0;
  target_ramped = 0;
  run_state = RUN_IDLE;
  Serial.println("# LEARN ABORT");
}

void suggestPidFromMap() {
  ThrottleMap& m = activeMap();
  if (!m.valid || m.n < 2) {
    Serial.println("# SUGGEST PID need >=2 points");
    return;
  }
  // 用中段斜率估计 plant gain [RPM/us]
  int i0 = m.n / 4;
  int i1 = (3 * m.n) / 4;
  if (i1 <= i0) {
    i0 = 0;
    i1 = m.n - 1;
  }
  float dpulse = (float)(m.pulse[i1] - m.pulse[i0]);
  float drpm = m.rpm[i1] - m.rpm[i0];
  if (dpulse < 1.0f) dpulse = 1.0f;
  float gain = drpm / dpulse;  // rpm per us
  if (gain < 0.05f) gain = 0.05f;
  // PI 输出单位=us，误差=rpm → kp ≈ 目标带宽 / gain
  float kp_s = 0.35f / gain;
  float ki_s = kp_s * 2.5f;
  if (kp_s < 0.02f) kp_s = 0.02f;
  if (kp_s > 0.4f) kp_s = 0.4f;
  if (ki_s < 0.05f) ki_s = 0.05f;
  if (ki_s > 1.5f) ki_s = 1.5f;
  Serial.printf("# SUGGEST gain=%.3f rpm/us  PID kp=%.4f ki=%.4f kd=0\n",
                (double)gain, (double)kp_s, (double)ki_s);
  Serial.printf("# HINT apply: PID %.4f %.4f 0\n", (double)kp_s, (double)ki_s);
}

void dumpActiveMap() {
  ThrottleMap& m = activeMap();
  Serial.printf("# MAP BEGIN profile=%s points=%d valid=%d\n",
                profileName(), m.n, m.valid ? 1 : 0);
  for (int i = 0; i < m.n; ++i) {
    Serial.printf("# MAP point i=%d pulse=%u rpm=%.1f\n",
                  i, (unsigned)m.pulse[i], (double)m.rpm[i]);
  }
  Serial.println("# MAP END");
}

void measureBegin() {
  if (run_state == RUN_RUNNING) {
    Serial.println("# ERR STOP before MEASURE");
    return;
  }
  learn_active = false;
  target_cmd = 0;
  target_ramped = 0;
  clearPid();
  run_state = RUN_IDLE;
  ctrl_mode = MODE_MEASURE;
  measure_active = true;
  measure_pulse = ESC_PULSE_MIN_US;
  applyEscPulse(measure_pulse);
  Serial.println("# MEASURE START — use PWM / MEASURE +50|-50 / HOLD / AUTO / SAVE");
  Serial.println("# OBSERVE: increase pulse, watch rpm on telemetry");
}

void measureAbort() {
  measure_active = false;
  learn_active = false;
  ctrl_mode = MODE_OPEN;
  measure_pulse = ESC_PULSE_MIN_US;
  applyEscPulse(ESC_PULSE_MIN_US);
  Serial.println("# MEASURE ABORT");
}

void measureSetPulse(long us) {
  if (us < ESC_PULSE_MIN_US) us = ESC_PULSE_MIN_US;
  if (us > ESC_PULSE_MAX_US) us = ESC_PULSE_MAX_US;
  measure_pulse = (uint16_t)us;
  // 学习中只由 learnTick 出油门，避免与手动 PWM 打架
  if (ctrl_mode == MODE_MEASURE && !learn_active) {
    applyEscPulse(measure_pulse);
  }
  Serial.printf("# MEASURE pulse=%u\n", (unsigned)measure_pulse);
}

void measureHold(float rpm_meas) {
  ThrottleMap& m = activeMap();
  if (m.n >= MAP_MAX_POINTS) {
    Serial.println("# ERR map full");
    return;
  }
  // 同脉宽则覆盖
  int idx = -1;
  for (int i = 0; i < m.n; i++) {
    if (m.pulse[i] == measure_pulse) {
      idx = i;
      break;
    }
  }
  if (idx < 0) {
    idx = m.n++;
    m.pulse[idx] = measure_pulse;
  }
  m.rpm[idx] = fabsf(rpm_meas);
  // 按脉宽排序（简单插入排序）
  for (int i = 1; i < m.n; i++) {
    uint16_t tp = m.pulse[i];
    float tr = m.rpm[i];
    int j = i - 1;
    while (j >= 0 && m.pulse[j] > tp) {
      m.pulse[j + 1] = m.pulse[j];
      m.rpm[j + 1] = m.rpm[j];
      j--;
    }
    m.pulse[j + 1] = tp;
    m.rpm[j + 1] = tr;
  }
  m.valid = (m.n >= 2);
  Serial.printf("# HOLD pulse=%u rpm=%.1f points=%d\n",
                (unsigned)measure_pulse, (double)rpm_meas, m.n);
}

void measureSave() {
  ThrottleMap& m = activeMap();
  m.valid = (m.n >= 2);
  if (!m.valid) {
    Serial.println("# ERR MEASURE SAVE need >=2 HOLD points");
    return;
  }
  saveMapToNvs(profile_id);
  suggestPidFromMap();
  measure_active = false;
  learn_active = false;
  applyEscPulse(ESC_PULSE_MIN_US);
  measure_pulse = ESC_PULSE_MIN_US;
  ctrl_mode = MODE_CLOSED;
  clearPid();
  Serial.printf("# MEASURE SAVE profile=%s points=%d -> MODE CLOSED\n",
                profileName(), m.n);
}

void measureClear() {
  mapClear(activeMap());
  Serial.println("# MEASURE CLEAR");
}

void escCalHigh() {
  // 标准油门行程校准：先输出最高油门，再给电调上电
  esccal_high = true;
  measure_active = false;
  learn_active = false;
  run_state = RUN_IDLE;
  ctrl_mode = MODE_SAFE;
  applyEscPulse(ESC_PULSE_MAX_US);
  Serial.printf("# ACK ESCCAL HIGH pulse=%u — hold max; power ESC battery now\n",
                (unsigned)esc_pulse_us);
  Serial.println("# Next: after beeps, send ESCCAL LOW");
}

void escCalLow() {
  esccal_high = false;
  applyEscPulse(ESC_PULSE_MIN_US);
  Serial.printf("# ACK ESCCAL LOW pulse=%u — wait long beep, then ESCCAL DONE\n",
                (unsigned)esc_pulse_us);
}

void escCalDone() {
  esccal_high = false;
  applyEscPulse(ESC_PULSE_MIN_US);
  ctrl_mode = MODE_OPEN;
  Serial.printf("# ACK ESCCAL DONE pulse=%u mode=OPEN\n", (unsigned)esc_pulse_us);
}

void learnTick(float rpm_meas) {
  if (!learn_active) return;
  uint32_t now = millis();
  applyEscPulse(learn_pulse);

  uint32_t elapsed = now - learn_step_t0;
  // 停留期间周期性回报，避免 UI 以为卡住
  static uint32_t last_hb = 0;
  if ((now - last_hb) >= 500) {
    last_hb = now;
    uint32_t remain = (elapsed < LEARN_SETTLE_MS) ? (LEARN_SETTLE_MS - elapsed) : 0;
    Serial.printf("# LEARN settle pulse=%u rpm=%.1f remain_ms=%lu\n",
                  (unsigned)learn_pulse, (double)rpm_meas, (unsigned long)remain);
  }

  if (elapsed < LEARN_SETTLE_MS) return;

  // 记录点（转速存绝对值，便于前馈图）
  float rpm_abs = fabsf(rpm_meas);
  if (learn_idx == 0 && learn_pulse == ESC_PULSE_MIN_US) {
    activeMap().pulse[0] = ESC_PULSE_MIN_US;
    activeMap().rpm[0] = rpm_abs < 30.0f ? 0.0f : rpm_abs;
    activeMap().n = 1;
  } else if (activeMap().n < MAP_MAX_POINTS) {
    int i = activeMap().n;
    activeMap().pulse[i] = learn_pulse;
    activeMap().rpm[i] = rpm_abs;
    activeMap().n++;
  }

  Serial.printf("# ACK LEARN point pulse=%u rpm=%.1f n=%d next_in=%lums\n",
                (unsigned)learn_pulse, (double)rpm_abs, activeMap().n,
                (unsigned long)LEARN_SETTLE_MS);

  float rpm_stop = target_rpm_max * LEARN_RPM_STOP_RATIO;
  const char* stop_reason = nullptr;
  if (rpm_abs >= rpm_stop) {
    stop_reason = "rpm_cap";  // 已接近设定最大转速，不必再推油门
  } else if (learn_pulse >= learn_max_us) {
    stop_reason = "pulse_cap";  // 脉宽到学习上限（默认=电调 2000）
  } else if (activeMap().n >= MAP_MAX_POINTS) {
    stop_reason = "map_full";
  }

  if (stop_reason != nullptr) {
    activeMap().valid = (activeMap().n >= 2);
    saveMapToNvs(profile_id);
    dumpActiveMap();
    suggestPidFromMap();
    learn_active = false;
    applyEscPulse(ESC_PULSE_MIN_US);
    target_cmd = 0;
    target_ramped = 0;
    run_state = RUN_IDLE;
    ctrl_mode = MODE_CLOSED;
    clearPid();
    float rpm_hi = 0.0f;
    uint16_t pulse_hi = learn_pulse;
    for (int i = 0; i < activeMap().n; ++i) {
      if (activeMap().rpm[i] > rpm_hi) rpm_hi = activeMap().rpm[i];
      if (activeMap().pulse[i] > pulse_hi) pulse_hi = activeMap().pulse[i];
    }
    float cover = (target_rpm_max > 1.0f) ? (100.0f * rpm_hi / target_rpm_max) : 0.0f;
    int at_full_throttle = (pulse_hi >= ESC_PULSE_MAX_US) ? 1 : 0;
    int can_raise = (strcmp(stop_reason, "rpm_cap") != 0 && pulse_hi < ESC_PULSE_MAX_US) ? 1 : 0;
    // 记录本次学习在最高油门下的实测转速
    prefs.putFloat("lrpm", rpm_hi);
    prefs.putUInt("lpus", (uint32_t)pulse_hi);
    prefs.putFloat("llim", target_rpm_max);
    Serial.printf(
        "# ACK LEARN DONE profile=%s points=%d rpm_max=%.1f pulse_max=%u "
        "rpm_limit=%.0f cover=%.1f%% reason=%s full_throttle=%d can_raise=%d -> MODE CLOSED\n",
        profileName(), activeMap().n, (double)rpm_hi, (unsigned)pulse_hi,
        (double)target_rpm_max, (double)cover, stop_reason, at_full_throttle, can_raise);
    Serial.printf(
        "# LEARN RESULT rpm_peak=%.1f pulse_peak=%u rpm_limit=%.0f reason=%s\n",
        (double)rpm_hi, (unsigned)pulse_hi, (double)target_rpm_max, stop_reason);
    if (at_full_throttle) {
      Serial.printf(
          "# EVAL: ESC full throttle %uus → measured peak RPM=%.1f (limit=%.0f, cover=%.1f%%)\n",
          (unsigned)ESC_PULSE_MAX_US, (double)rpm_hi, (double)target_rpm_max, (double)cover);
    } else if (strcmp(stop_reason, "rpm_cap") == 0) {
      Serial.printf(
          "# EVAL: reached rpm_limit~%.0f at pulse=%u — do not raise throttle further\n",
          (double)target_rpm_max, (unsigned)pulse_hi);
    } else if (can_raise) {
      Serial.printf(
          "# EVAL: rpm_peak=%.0f < limit=%.0f and pulse<%u — unexpected early stop\n",
          (double)rpm_hi, (double)target_rpm_max, (unsigned)ESC_PULSE_MAX_US);
    }
    Serial.println("# NEXT: apply PID, low RPM closed-loop test, then raise setpoint gradually");
    return;
  }

  learn_pulse = (uint16_t)(learn_pulse + LEARN_STEP_US);
  if (learn_pulse > learn_max_us) learn_pulse = learn_max_us;
  learn_idx++;
  learn_step_t0 = now;
  Serial.printf("# LEARN next pulse=%u\n", (unsigned)learn_pulse);
}

void phaseMoveFinish(const char* why) {
  move_active = false;
  sense_learn = false;
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  clearPid();
  run_state = RUN_IDLE;
  applyEscPulse(ESC_PULSE_MIN_US);
  Serial.printf("# ACK MOVE DONE reason=%s accum=%.2f target=%.2f\n",
                why, (double)move_accum_deg, (double)move_target_deg);
}

/** 单向路径：只沿 esc_sense 转到目标相对角，返回需转过的度数 (0..360] */
float uniTravelDeg(float from_rel, float to_rel) {
  if (esc_sense > 0) {
    float d = wrap360(to_rel - from_rel);
    return (d < 0.5f) ? 0.0f : d;
  }
  float d = wrap360(from_rel - to_rel);  // CCW distance
  return (d < 0.5f) ? 0.0f : d;
}

/** 用户期望 CW(+) / CCW(-) 相对转角 → 单向路径长度，方向固定为 esc_sense */
bool phaseMoveUserDelta(float user_delta_cw, float rpm) {
  if (isnan(last_deg)) {
    Serial.println("# ERR no angle yet");
    return false;
  }
  float rel = phaseRelFromAbs(last_deg);
  float target = wrap360(rel + user_delta_cw);
  float travel = uniTravelDeg(rel, target);
  // 多圈：用户要转超过一圈时，在单向路径上补整圈
  float turns = floorf(fabsf(user_delta_cw) / 360.0f);
  if (turns > 0.0f) {
    // 若用户方向与电调同向，保留整圈；反向则只保证最终相位（不加整圈，避免空转）
    bool same = (user_delta_cw >= 0.0f && esc_sense > 0) ||
                (user_delta_cw < 0.0f && esc_sense < 0);
    if (same) travel += turns * 360.0f;
  }
  if (travel < 0.5f) {
    Serial.printf("# ACK MOVE skip already at target rel=%.2f (user wanted Δ=%.2f)\n",
                  (double)rel, (double)user_delta_cw);
    return false;
  }
  Serial.printf(
      "# UNI userΔcw=%.2f -> target=%.2f travel=%.2f via=%s (esc_sense=%s)\n",
      (double)user_delta_cw, (double)target, (double)travel,
      esc_sense > 0 ? "CW" : "CCW", esc_sense > 0 ? "+encoder" : "-encoder");
  return phaseMoveBegin(esc_sense, travel, rpm);
}

bool phaseGotoRel(float target_rel, float rpm) {
  if (isnan(last_deg)) {
    Serial.println("# ERR no angle yet");
    return false;
  }
  target_rel = wrap360(target_rel);
  float rel = phaseRelFromAbs(last_deg);
  float travel = uniTravelDeg(rel, target_rel);
  if (travel < 0.5f) {
    Serial.printf("# ACK GOTO already at %.2f (rel=%.2f)\n", (double)target_rel, (double)rel);
    return false;
  }
  Serial.printf("# GOTO target=%.2f from=%.2f travel=%.2f via=%s\n",
                (double)target_rel, (double)rel, (double)travel,
                esc_sense > 0 ? "CW" : "CCW");
  return phaseMoveBegin(esc_sense, travel, rpm);
}

bool phaseMoveBegin(int dir, float deg, float rpm) {
  if (run_state == RUN_ESTOP) {
    Serial.println("# ERR clear ESTOP first");
    return false;
  }
  if (ctrl_mode == MODE_LEARN || ctrl_mode == MODE_MEASURE) {
    Serial.println("# ERR abort LEARN/MEASURE before MOVE");
    return false;
  }
  if (deg < 0.5f) deg = 0.5f;
  if (deg > 7200.0f) deg = 7200.0f;
  if (rpm < 30.0f) rpm = 30.0f;
  if (rpm > 800.0f) rpm = 800.0f;
  if (rpm > target_rpm_max) rpm = target_rpm_max;
  // 强制单向：忽略传入反向，只用 esc_sense
  move_dir = (esc_sense >= 0) ? 1 : -1;
  move_target_deg = deg;
  move_accum_deg = 0.0f;
  move_crawl_rpm = rpm;
  move_wrong_ms = 0;
  move_active = true;
  sense_learn = false;
  learn_active = false;
  measure_active = false;
  ctrl_mode = MODE_OPEN;
  clearPid();
  target_cmd = move_crawl_rpm;
  // 相位爬行也立刻给够油门，避免缓启动期间几乎不动
  target_ramped = move_crawl_rpm;
  run_state = RUN_RUNNING;
  applyEscPulse(computePulseCrawl(move_crawl_rpm));
  Serial.printf("# ACK MOVE via=%s travel=%.2f rpm=%.1f esc_sense=%d pulse~%u\n",
                move_dir > 0 ? "CW" : "CCW", (double)deg, (double)rpm, esc_sense,
                (unsigned)esc_pulse_us);
  return true;
}

void senseLearnBegin(float rpm_or_pulse) {
  if (run_state == RUN_ESTOP) {
    Serial.println("# ERR clear ESTOP first");
    return;
  }
  // 参数：若 >=1000 当作脉宽 μs；否则当作 RPM（再换算，并抬到可转脉宽）
  uint16_t pulse;
  if (rpm_or_pulse >= 1000.0f) {
    pulse = (uint16_t)rpm_or_pulse;
  } else {
    float rpm = rpm_or_pulse;
    if (rpm < 100.0f) rpm = 100.0f;
    if (rpm > 800.0f) rpm = 800.0f;
    pulse = computePulseCrawl(rpm);
  }
  if (pulse < CRAWL_PULSE_MIN) pulse = CRAWL_PULSE_MIN;
  if (pulse > 1700) pulse = 1700;  // 辨识上限，仍留余量
  sense_pulse_us = pulse;
  sense_learn = true;
  sense_accum = 0.0f;
  sense_t0 = millis();
  move_active = false;
  learn_active = false;
  measure_active = false;
  ctrl_mode = MODE_OPEN;
  clearPid();
  // 立即给油，不走缓启动
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  run_state = RUN_RUNNING;
  applyEscPulse(sense_pulse_us);
  Serial.printf("# ACK SENSE AUTO pulse=%uus for %lums (fixed throttle, no soft)\n",
                (unsigned)sense_pulse_us, (unsigned long)SENSE_MS);
}

void phaseTick(float abs_deg, float ddeg) {
  float rel = phaseRelFromAbs(abs_deg);

  if (sense_learn) {
    applyEscPulse(sense_pulse_us);  // 辨识期间强制固定脉宽
    sense_accum += ddeg;
    if ((millis() - sense_t0) >= SENSE_MS) {
      if (fabsf(sense_accum) < 3.0f) {
        Serial.printf(
            "# ERR SENSE AUTO: little motion accum=%.2f pulse=%u — try SENSE AUTO 1350\n",
            (double)sense_accum, (unsigned)sense_pulse_us);
        sense_learn = false;
        doStop(true);
        return;
      }
      esc_sense = (sense_accum >= 0.0f) ? 1 : -1;
      saveEscSenseToNvs();
      sense_learn = false;
      doStop(true);
      Serial.printf("# ACK SENSE=%d (%s when throttle) accum=%.2f pulse=%u saved\n",
                    esc_sense, esc_sense > 0 ? "CW/encoder+" : "CCW/encoder-",
                    (double)sense_accum, (unsigned)sense_pulse_us);
    }
    return;
  }

  if (move_active) {
    move_accum_deg += ddeg;
    bool wrong = (move_dir > 0 && ddeg < -0.05f) || (move_dir < 0 && ddeg > 0.05f);
    if (wrong) {
      move_wrong_ms += CTRL_MS;
      if (move_wrong_ms > 800) {
        // 自动纠正电调方向并重试剩余角度
        float remain = move_target_deg - fabsf(move_accum_deg);
        if (remain < 0.5f) remain = move_target_deg;
        esc_sense = -esc_sense;
        saveEscSenseToNvs();
        Serial.printf("# WARN flipped esc_sense -> %d, retry remain=%.2f\n",
                      esc_sense, (double)remain);
        phaseMoveBegin(esc_sense, remain, move_crawl_rpm);
        return;
      }
    } else if ((move_dir > 0 && ddeg > 0.0f) || (move_dir < 0 && ddeg < 0.0f)) {
      move_wrong_ms = 0;
    }
    bool reached = (move_dir > 0 && move_accum_deg >= move_target_deg) ||
                   (move_dir < 0 && move_accum_deg <= -move_target_deg);
    if (reached) {
      phaseMoveFinish("reached");
      return;
    }
  }

  if (stopat_on && run_state == RUN_RUNNING && !move_active && !sense_learn) {
    if (!isnan(stopat_prev_rel)) {
      float err = wrap180(rel - stopat_deg);
      float prev_err = wrap180(stopat_prev_rel - stopat_deg);
      bool crossed = (prev_err > 0.0f && err <= 0.0f) || (prev_err < 0.0f && err >= 0.0f);
      bool near = fabsf(err) <= stopat_tol_deg;
      // 单向穿越：仅在与 esc_sense 一致的穿越有效
      bool sense_ok = (esc_sense > 0 && ddeg >= -0.02f) || (esc_sense < 0 && ddeg <= 0.02f);
      if ((crossed || near) && sense_ok) {
        Serial.printf("# ACK STOPAT hit target=%.2f rel=%.2f\n",
                      (double)stopat_deg, (double)rel);
        doStop(true);
      }
    }
    stopat_prev_rel = rel;
  } else {
    stopat_prev_rel = rel;
  }
}

void controlTick(float rpm_meas, float dt) {
  // 主机超时（学习中不因超时急停，避免油门被拉回 1000 再爬升）
  if (host_seen && run_state == RUN_RUNNING && !learn_active) {
    if ((millis() - last_host_ms) > HOST_TIMEOUT_MS) {
      doEstop("host_timeout");
      return;
    }
  }

  float overspeed = target_rpm_max * 1.10f;
  // 学习扫表时允许到转速上限；超速保护只在正常运行
  if (!learn_active && fabsf(rpm_meas) > overspeed && run_state == RUN_RUNNING) {
    doEstop("overspeed");
    return;
  }

  // 学习标志优先于 mode：防止 MODE/杂讯把 mode 改掉后油门被 IDLE 路径拉低
  if (learn_active) {
    if (ctrl_mode != MODE_LEARN) ctrl_mode = MODE_LEARN;
    learnTick(rpm_meas);
    return;
  }

  if (ctrl_mode == MODE_LEARN) {
    // learn_active 已假但仍挂 LEARN：收尾到 OPEN，避免卡在空模式
    ctrl_mode = MODE_OPEN;
  }

  if (sense_learn) {
    applyEscPulse(sense_pulse_us);
    return;
  }

  if (ctrl_mode == MODE_MEASURE) {
    applyEscPulse(measure_pulse);
    return;
  }

  if (esccal_high) {
    applyEscPulse(ESC_PULSE_MAX_US);
    return;
  }

  if (ctrl_mode == MODE_SAFE || run_state != RUN_RUNNING) {
    if (run_state != RUN_RUNNING) {
      updateSoftRamp(dt);
      if (target_ramped <= 0.5f) {
        applyEscPulse(ESC_PULSE_MIN_US);
      } else {
        // 停止斜坡下降过程
        applyEscPulse(computePulseOpen(target_ramped));
      }
    } else {
      applyEscPulse(ESC_PULSE_MIN_US);
    }
    return;
  }

  updateSoftRamp(dt);

  uint16_t pulse;
  if (ctrl_mode == MODE_CLOSED) {
    pulse = computePulseClosed(target_ramped, rpm_meas, dt);
  } else {
    pulse = computePulseOpen(target_ramped);
  }
  applyEscPulse(pulse);
}

// ---------------- 串口指令 ----------------
void handleCommandLine(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  char* end = line + strlen(line);
  while (end > line && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r')) *--end = '\0';
  if (*line == '\0' || *line == '#') return;

  last_host_ms = millis();
  host_seen = true;

  if (strcasecmp(line, "PING") == 0) {
    Serial.println("# ACK PING");
    return;
  }
  if (strcasecmp(line, "START") == 0) {
    doStart();
    return;
  }
  if (strcasecmp(line, "STOP SOFT") == 0) {
    if (run_state == RUN_ESTOP) {
      run_state = RUN_IDLE;
      Serial.println("# ACK ESTOP cleared");
    }
    doStop(false);
    return;
  }
  if (strcasecmp(line, "STOP") == 0 || strcasecmp(line, "STOP NOW") == 0) {
    if (run_state == RUN_ESTOP) {
      run_state = RUN_IDLE;
      Serial.println("# ACK ESTOP cleared");
    }
    // 默认立即停转（UI 停止按钮）；缓停用 STOP SOFT
    doStop(true);
    return;
  }
  if (strcasecmp(line, "ESTOP") == 0) {
    doEstop("cmd");
    return;
  }
  if (strncasecmp(line, "RPMMAX", 6) == 0) {
    char* p = line + 6;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    float v = strtof(p, nullptr);
    if (v < 100.0f) v = 100.0f;
    if (v > MOTOR_RPM_ABS_MAX) v = MOTOR_RPM_ABS_MAX;
    target_rpm_max = v;
    if (target_cmd > target_rpm_max) target_cmd = target_rpm_max;
    Serial.printf("# ACK RPMMAX=%.0f (learn stops ~%.0f, overspeed=%.0f)\n",
                  (double)target_rpm_max, (double)(target_rpm_max * LEARN_RPM_STOP_RATIO),
                  (double)(target_rpm_max * 1.10f));
    return;
  }
  if (strncasecmp(line, "RPM", 3) == 0) {
    char* p = line + 3;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    float rpm = strtof(p, nullptr);
    if (rpm < 0) rpm = 0;
    if (rpm > target_rpm_max) rpm = target_rpm_max;
    target_cmd = rpm;
    Serial.printf("# ACK RPM=%.1f (max=%.0f)\n", (double)target_cmd, (double)target_rpm_max);
    return;
  }
  if (strncasecmp(line, "SOFT RATE", 9) == 0) {
    float r = strtof(line + 9, nullptr);
    if (r < 50.0f) r = 50.0f;
    if (r > 5000.0f) r = 5000.0f;
    soft_rate_rpm_s = r;
    Serial.printf("# ACK SOFT RATE=%.1f\n", (double)soft_rate_rpm_s);
    return;
  }
  if (strcasecmp(line, "SOFT ON") == 0 || strcasecmp(line, "SOFT=ON") == 0) {
    soft_enable = true;
    Serial.println("# ACK SOFT ON");
    return;
  }
  if (strcasecmp(line, "SOFT OFF") == 0 || strcasecmp(line, "SOFT=OFF") == 0) {
    soft_enable = false;
    Serial.println("# ACK SOFT OFF");
    return;
  }
  if (strncasecmp(line, "SOFT", 4) == 0 && (line[4] == '\0' || line[4] == ' ' || line[4] == '=' || line[4] == ':')) {
    char* p = line + 4;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    soft_enable = (strncasecmp(p, "ON", 2) == 0);
    Serial.printf("# ACK SOFT %s\n", soft_enable ? "ON" : "OFF");
    return;
  }
  if (strncasecmp(line, "MODE", 4) == 0) {
    char* p = line + 4;
    while (*p == ' ') ++p;
    // 学习中禁止 MODE 切换（会清 learn_active，导致油门掉回 1000 再被别的路径拉起）
    if (learn_active || ctrl_mode == MODE_LEARN) {
      if (strncasecmp(p, "SAFE", 4) == 0) {
        learnAbort();
        ctrl_mode = MODE_SAFE;
        doStop(true);
        Serial.printf("# ACK MODE %s (aborted LEARN)\n", modeName());
        return;
      }
      Serial.println("# ERR LEARN active — use LEARN ABORT or STOP first");
      return;
    }
    if (strncasecmp(p, "OPEN", 4) == 0) ctrl_mode = MODE_OPEN;
    else if (strncasecmp(p, "CLOSED", 6) == 0) ctrl_mode = MODE_CLOSED;
    else if (strncasecmp(p, "MEASURE", 7) == 0) {
      measureBegin();
      return;
    } else if (strncasecmp(p, "LEARN", 5) == 0) {
      Serial.println("# ERR use LEARN START or MEASURE AUTO");
      return;
    } else if (strncasecmp(p, "SAFE", 4) == 0) {
      ctrl_mode = MODE_SAFE;
      doStop(true);
    } else {
      Serial.println("# ERR MODE");
      return;
    }
    measure_active = false;
    learn_active = false;
    clearPid();
    Serial.printf("# ACK MODE %s\n", modeName());
    return;
  }
  if (strncasecmp(line, "PROFILE", 7) == 0) {
    char* p = line + 7;
    while (*p == ' ') ++p;
    if (strncasecmp(p, "flap", 4) == 0) profile_id = PROF_FLAP;
    else profile_id = PROF_NOLOAD;
    Serial.printf("# ACK PROFILE %s valid=%d points=%d\n",
                  profileName(), activeMap().valid, activeMap().n);
    return;
  }
  if (strcasecmp(line, "PHASE ZERO") == 0 || strcasecmp(line, "PHASE0") == 0) {
    if (isnan(last_deg)) {
      Serial.println("# ERR no angle yet");
      return;
    }
    phase_zero_deg = last_deg;
    savePhaseZeroToNvs();
    stopat_prev_rel = NAN;
    Serial.printf("# ACK PHASE ZERO abs=%.3f -> rel=0  (saved)\n", (double)phase_zero_deg);
    return;
  }
  if (strcasecmp(line, "PHASE?") == 0 || strcasecmp(line, "PHASE") == 0) {
    float absd = isnan(last_deg) ? 0.0f : last_deg;
    float rel = phaseRelFromAbs(absd);
    Serial.printf("# ACK PHASE abs=%.3f rel=%.3f zero=%.3f esc_sense=%d stopat=%s/%.1f move=%d\n",
                  (double)absd, (double)rel, (double)phase_zero_deg, esc_sense,
                  stopat_on ? "ON" : "OFF", (double)stopat_deg, move_active ? 1 : 0);
    return;
  }
  if (strncasecmp(line, "SENSE", 5) == 0) {
    char* p = line + 5;
    while (*p == ' ') ++p;
    if (*p == '\0' || strcasecmp(p, "AUTO") == 0 || strncasecmp(p, "AUTO", 4) == 0) {
      float arg = (float)SENSE_PULSE_DEFAULT;
      char* q = p;
      if (strncasecmp(q, "AUTO", 4) == 0) q += 4;
      while (*q == ' ') ++q;
      if (*q) arg = strtof(q, nullptr);
      senseLearnBegin(arg);
      return;
    }
    if (p[0] == '+' || strcasecmp(p, "CW") == 0 || strcasecmp(p, "1") == 0) {
      esc_sense = 1;
    } else if (p[0] == '-' || strcasecmp(p, "CCW") == 0 || strcasecmp(p, "-1") == 0) {
      esc_sense = -1;
    } else {
      Serial.println("# ERR SENSE AUTO|+|−|CW|CCW");
      return;
    }
    saveEscSenseToNvs();
    Serial.printf("# ACK SENSE=%d (%s) saved\n", esc_sense,
                  esc_sense > 0 ? "CW/encoder+" : "CCW/encoder-");
    return;
  }
  if (strncasecmp(line, "MOVE ", 5) == 0) {
    char* p = line + 5;
    while (*p == ' ') ++p;
    float user_delta = 0.0f;
    if (strncasecmp(p, "CW", 2) == 0) {
      p += 2;
      while (*p == ' ') ++p;
      user_delta = strtof(p, &p);  // CW = +
    } else if (strncasecmp(p, "CCW", 3) == 0) {
      p += 3;
      while (*p == ' ') ++p;
      user_delta = -strtof(p, &p);  // CCW = −
    } else {
      Serial.println("# ERR MOVE CW|CCW <deg> [rpm]  (uni ESC auto-converts path)");
      return;
    }
    while (*p == ' ') ++p;
    float rpm = (*p != '\0') ? strtof(p, nullptr) : move_crawl_rpm;
    phaseMoveUserDelta(user_delta, rpm);
    return;
  }
  if (strncasecmp(line, "STOPAT", 6) == 0) {
    char* p = line + 6;
    while (*p == ' ') ++p;
    if (*p == '\0' || strcasecmp(p, "OFF") == 0) {
      stopat_on = false;
      Serial.println("# ACK STOPAT OFF");
      return;
    }
    float deg = strtof(p, nullptr);
    stopat_deg = wrap360(deg);
    stopat_on = true;
    stopat_prev_rel = NAN;
    Serial.printf("# ACK STOPAT ON deg=%.2f tol=%.1f via=%s\n",
                  (double)stopat_deg, (double)stopat_tol_deg,
                  esc_sense > 0 ? "CW" : "CCW");
    return;
  }
  if (strncasecmp(line, "GOTO ", 5) == 0) {
    char* p = line + 5;
    while (*p == ' ') ++p;
    float target = strtof(p, &p);
    target = wrap360(target);
    while (*p == ' ') ++p;
    // 忽略 CW/CCW/AUTO：单向电调只沿 esc_sense
    if (strncasecmp(p, "CW", 2) == 0) p += 2;
    else if (strncasecmp(p, "CCW", 3) == 0) p += 3;
    else if (strncasecmp(p, "AUTO", 4) == 0) p += 4;
    while (*p == ' ') ++p;
    float rpm = (*p != '\0') ? strtof(p, nullptr) : move_crawl_rpm;
    phaseGotoRel(target, rpm);
    return;
  }
  if (strcasecmp(line, "LEARN START") == 0) {
    // 自动台阶学习（测量模式的自动版）
    if (ctrl_mode != MODE_MEASURE) measureBegin();
    learnBegin();
    return;
  }
  if (strncasecmp(line, "LEARN MAXUS", 11) == 0) {
    // 学习油门上限固定为电调最大；忽略更低值（避免再被设成 1600）
    learn_max_us = ESC_PULSE_MAX_US;
    Serial.printf("# ACK LEARN MAXUS=%u (fixed to ESC max; ignore lower)\n",
                  (unsigned)learn_max_us);
    return;
  }
  if (strcasecmp(line, "LEARN ABORT") == 0) {
    learnAbort();
    return;
  }
  if (strcasecmp(line, "MEASURE START") == 0 || strcasecmp(line, "MEASURE") == 0) {
    measureBegin();
    return;
  }
  if (strcasecmp(line, "MEASURE ABORT") == 0 || strcasecmp(line, "MEASURE STOP") == 0) {
    measureAbort();
    return;
  }
  if (strcasecmp(line, "MEASURE HOLD") == 0 || strcasecmp(line, "HOLD") == 0) {
    if (ctrl_mode != MODE_MEASURE) {
      Serial.println("# ERR enter MEASURE first");
      return;
    }
    measureHold(rpm_filt);
    return;
  }
  if (strcasecmp(line, "MEASURE SAVE") == 0) {
    measureSave();
    return;
  }
  if (strcasecmp(line, "MEASURE CLEAR") == 0) {
    measureClear();
    return;
  }
  if (strcasecmp(line, "MEASURE AUTO") == 0) {
    if (ctrl_mode != MODE_MEASURE) measureBegin();
    learnBegin();
    return;
  }
  if (strncasecmp(line, "MEASURE +", 9) == 0 || strncasecmp(line, "MEASURE -", 9) == 0) {
    if (ctrl_mode != MODE_MEASURE) measureBegin();
    long delta = strtol(line + 8, nullptr, 10);  // includes sign after space? "MEASURE +50"
    // line is "MEASURE +50" — parse from after MEASURE
    char* p = line + 7;
    while (*p == ' ') ++p;
    delta = strtol(p, nullptr, 10);
    measureSetPulse((long)measure_pulse + delta);
    return;
  }
  if (strcasecmp(line, "ESCCAL HIGH") == 0) {
    escCalHigh();
    return;
  }
  if (strcasecmp(line, "ESCCAL LOW") == 0) {
    escCalLow();
    return;
  }
  if (strcasecmp(line, "ESCCAL DONE") == 0) {
    escCalDone();
    return;
  }
  if (strcasecmp(line, "PID?") == 0 || strcasecmp(line, "PID") == 0) {
    Serial.printf("# PID kp=%.4f ki=%.4f kd=%.4f adapt=%d\n",
                  (double)kp, (double)ki, (double)kd, adapt_on ? 1 : 0);
    return;
  }
  if (strcasecmp(line, "PID SAVE") == 0) {
    savePidToNvs();
    Serial.println("# ACK PID SAVE");
    return;
  }
  if (strncasecmp(line, "PID", 3) == 0) {
    char* p = line + 3;
    float a = strtof(p, &p);
    float b = strtof(p, &p);
    float c = strtof(p, &p);
    kp = a; ki = b; kd = c;
    clearPid();
    Serial.printf("# ACK PID kp=%.4f ki=%.4f kd=%.4f\n", (double)kp, (double)ki, (double)kd);
    return;
  }
  if (strncasecmp(line, "ADAPT", 5) == 0) {
    char* p = line + 5;
    while (*p == ' ') ++p;
    adapt_on = (strncasecmp(p, "ON", 2) == 0);
    Serial.printf("# ACK ADAPT %s\n", adapt_on ? "ON" : "OFF");
    return;
  }
  if (strncasecmp(line, "FREQ", 4) == 0) {
    char* p = line + 4;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    if (*p == '\0' || strcasecmp(p, "?") == 0) {
      Serial.printf("# FREQ %lu Hz period=%luus\n",
                    (unsigned long)esc_pwm_hz, (unsigned long)esc_period_us);
      return;
    }
    long hz = strtol(p, nullptr, 10);
    if (hz < 50 || hz > 400) {
      Serial.println("# ERR FREQ range 50..400");
      return;
    }
    // 建议按 50 步进；仍允许任意整数以便调试
    bool ok = setEscPwmFreq((uint32_t)hz);
    Serial.printf("# ACK FREQ %lu ok=%d period=%luus pulse=%u\n",
                  (unsigned long)esc_pwm_hz, ok ? 1 : 0,
                  (unsigned long)esc_period_us, (unsigned)esc_pulse_us);
    return;
  }
  if (strncasecmp(line, "PWM", 3) == 0) {
    long us = strtol(line + 3, nullptr, 10);
    if (us < ESC_PULSE_MIN_US || us > ESC_PULSE_MAX_US) {
      Serial.println("# ERR PWM range");
      return;
    }
    // 自动学习期间禁止主机 PWM 抢控（UI 滑条被遥测带动时会误发）
    if (ctrl_mode == MODE_LEARN || learn_active) {
      Serial.println("# ERR ignore PWM during LEARN");
      return;
    }
    if (ctrl_mode == MODE_MEASURE) {
      measureSetPulse(us);
      return;
    }
    run_state = RUN_RUNNING;
    ctrl_mode = MODE_OPEN;
    applyEscPulse((uint16_t)us);
    Serial.printf("# ACK PWM=%ld\n", us);
    return;
  }
  Serial.printf("# ERR unknown: %s\n", line);
}

void pollSerialCommands() {
  static char buf[96];
  static size_t len = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handleCommandLine(buf);
        len = 0;
      }
      continue;
    }
    if (len + 1 < sizeof(buf)) buf[len++] = c;
    else len = 0;
  }
}

// ---------------- setup / loop ----------------
void setup() {
  setupEscPwmMinFirst();

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  Serial.begin(SERIAL_BAUD);
  delay(20);

  prefs.begin("escctl", false);
  loadMapsFromNvs();

  spi->begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  spiFrame(buildReadCmd(REG_ERRFL));
  spiFrame(buildReadCmd(REG_NOP));

  Serial.println("# AS5047P + ESC Ready (ESP32-S3)");
  Serial.printf("# sample_hz=%lu ctrl_hz=%lu baud=%lu esc_pwm_hz=%lu pin=%d\n",
                (unsigned long)CTRL_HZ, (unsigned long)CTRL_HZ,
                (unsigned long)SERIAL_BAUD, (unsigned long)esc_pwm_hz, PIN_ESC_PWM);
  Serial.println("# format: t_ms,raw,deg(rel),rad,rpm,ef,agc,magL,magH,pulse_us,target_rpm,mode,kp,ki,kd,profile,run");
  Serial.println("# cmds: START STOP ESTOP RPM PHASE MOVE STOPAT GOTO LEARN MEASURE ...");
  Serial.println("# PHASE: ZERO; MOVE CW/CCW auto→uni path; SENSE AUTO detect ESC dir");
  Serial.printf("# phase_zero=%.3f esc_sense=%d rpm_max=%.0f\n",
                (double)phase_zero_deg, esc_sense, (double)target_rpm_max);
  Serial.printf("# ENC AS5047P cpr=%u deg/count=%.6f | rpm=(dcount/cpr)/dt*60\n",
                (unsigned)ENC_CPR, (double)ENC_DEG_PER_COUNT);
  Serial.printf("# ESC min=%uus armed low | profile=%s\n",
                (unsigned)ESC_PULSE_MIN_US, profileName());
}

void loop() {
  pollSerialCommands();

  static uint32_t next_ms = 0;
  uint32_t now = millis();
  if ((int32_t)(now - next_ms) < 0) return;
  next_ms = now + CTRL_MS;

  AngleSample s = readAngleCom();
  float abs_deg = (float)s.raw * ENC_DEG_PER_COUNT;
  float deg = phaseRelFromAbs(abs_deg);  // 遥测 deg = 用户相对相位（零点后）
  float rad = deg * (TWO_PI_F / 360.0f);

  float rpm_mag = 0.0f;
  float dt = CTRL_MS * 0.001f;
  float ddeg = 0.0f;
  if (!isnan(last_deg) && now > last_ms) {
    dt = (now - last_ms) * 0.001f;
    if (dt > 0.0005f) {
      ddeg = unwrapDeltaDeg(abs_deg, last_deg);  // 必须用绝对角，相对相位跨 0° 会误判正负
      rpm_mag = updateRpmFromDelta(ddeg, dt);
    }
  } else {
    rpm_mag = fabsf(rpm_signed_stable);
  }
  last_deg = abs_deg;
  last_ms = now;

  phaseTick(abs_deg, ddeg);
  controlTick(rpm_mag, dt);  // 闭环用转速大小（≥0）

  int mode_i = (int)ctrl_mode;
  int run_i = (int)run_state;
  int prof_i = (int)profile_id;

  // 遥测 rpm：符号锁定后的值（单向转时稳定为正或稳定为负，不会正负乱跳）
  Serial.printf("%lu,%u,%.3f,%.6f,%.2f,%u,%u,%u,%u,%u,%.1f,%d,%.4f,%.4f,%.4f,%d,%d\n",
                (unsigned long)now,
                (unsigned)s.raw,
                deg,
                rad,
                rpm_signed_stable,
                (unsigned)s.ef,
                (unsigned)s.agc,
                (unsigned)s.mag_low,
                (unsigned)s.mag_high,
                (unsigned)esc_pulse_us,
                (double)target_ramped,
                mode_i,
                (double)kp,
                (double)ki,
                (double)kd,
                prof_i,
                run_i);
}
