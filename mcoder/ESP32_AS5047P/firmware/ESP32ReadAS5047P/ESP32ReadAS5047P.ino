/************************************************
 * ESP32-S3：AS5047P + 航模电调转速控制
 *
 * 【钉死术语 — 禁止写「取用」】
 *   采集数据（采集频率）= SPI 读编码器角度入缓冲/解缠          → 目标 2000 Hz
 *   使用数据（使用频率）= 用采集点求速度（差分/窗口），与采集同层全用
 *                       → 目标 2000 Hz（采了 2k、用 2k；≥方案建议 1200）
 *   输出数据（输出频率）= PID → 写 ESC/PWM 的控制输出           → 目标 400 Hz
 *   硬约束【实时路径禁打印】：2kHz encSampleCb / 400Hz motorControlTask 内
 *     禁止 Serial.print / hostPrintf 阻塞写出；若调用则入异步日志环，由低优 telemTask 排出。
 *   数据流：采集2k → 使用2k(求速全用) → 输出400(最新RPM→ESC)
 *   禁止把 400 叫采集/使用；勿把使用频率擅自降到 1200（除非用户另说）。
 *
 * 电调：GPIO9，默认 FREQ 400（与输出同相），脉宽 1000~2000us（上电先最低油门）
 *   FREQ <50..600> 可改刷新率（>500Hz 周期<2000μs，高油门夹断）
 * 扑翼霍尔（YL-57 / LM393+A3144，VCC=5V）：
 *   GPIO4 = 输出盘 0°（下扑）DO；GPIO5 = ~180°（上举）DO
 * 减速比：小齿 12·14 / 大齿 59·79 → (59/12)*(79/14)=4661/168≈27.7440476
 *
 * 盘相位传感模型（霍尔稀疏绝对 + 磁编连续）：
 *   霍尔沿 → latch disc_ref∈{0,180} 与 motor_counts_at_hall；
 *   之后 disc = wrap(disc_ref + esc_sense·Δmotor_deg / gear)；
 *   提前制动用编码器推算的盘相位，霍尔仅复位/确认。
 *
 * 调度（阶段 A/B）：
 *   采集+使用：esp_timer @2000Hz → encSampleCb（SPI+解缠+窗差分+EMA）→ EncShared；零打印
 *   输出：esp_timer @400Hz → 信号量 → motorControlTask@Core1（只读快照/PID/ESC；零直打串口）
 *   telemTask@Core0 低优：USB 遥测≤20Hz（Serial.write 批量）+ hostDrainAsyncLog
 *   loop：仅命令解析 + 轻量 drain
 *   阶段 D/E 备份：enc_ring.h + q16_math.h；USE_FIXED_POINT 默认 0（浮点主路）
 *
 * 指令：START STOP ESTOP PING | ENC? CTRL? TELEM? LOAD? | CORE? FIXED? Q16CMP?|RESET
 *   TELEM ON|OFF|? | LOAD RESET | BLE ON|OFF|? | FLIGHT ON|OFF|?
 *   HOST ON|OFF|? | HOST TO <sec>           （台架：可关/改主机超时；默认 1.5s）
 *   TEST START <name> | TEST STOP | TEST?   （板内分段测试，见 onboardTest）
 *   TUNE REC ON|OFF | TUNE? | TUNE DUMP     （板载整定环形记录，见 tune_rec.h）
 *   HOLD PHASE <deg> [crawl_rpm] [stop_ms] | PHASE HOLD … | PHASE <0|90|180> …
 *     扑翼盘相位保持：0°=霍尔 GPIO4 下扑；180°=GPIO5 上举；90°=半行程（编码器/减速比反算）
 *     stop_ms>0：从当前转速闭环软降→爬行，再提前制动；壁钟≈stop_ms（如 3000）
 *   RPM / MODE / FREQ / BLE RATE|LITE …
 * 蓝牙：OEZ-RPM；测试默认 ON；FLIGHT ON → 关 BLE + 禁主机超时 ESTOP
 * TEST 运行中同样禁用 HOST_TIMEOUT ESTOP；段内自判在转（enc+hall），主机只发 START/STOP。
 * HOST OFF / HOST TO 0 亦禁用超时（台架）；HOST ON 恢复默认 1.5s。
 ************************************************/
#include <SPI.h>
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esc_dshot.h"
#include "ble_host.h"
#include "q16_math.h"
#include "enc_ring.h"
#include "tune_rec.h"

// 备份：Q16/环缓。默认 0=浮点主路径（采集2k/使用2k/输出400）；改为 1 启用定点求速+EMA+环缓写入
#ifndef USE_FIXED_POINT
#define USE_FIXED_POINT 0
#endif

// 测试固件默认开 BLE；飞行构建可改为 0（不 bleBegin）
#ifndef FEATURE_BLE_DEFAULT
#define FEATURE_BLE_DEFAULT 1
#endif
// ---- 引脚 ----
static const int PIN_CS   = 10;
static const int PIN_SCLK = 12;
static const int PIN_MISO = 13;
static const int PIN_MOSI = 11;
static const int PIN_ESC_PWM = 9;
// 扑翼输出盘霍尔（A3144 模块 DO→GPIO，VCC=5V；默认触发=低，同厂商 51 样例）
static const int PIN_HALL_DOWN = 4;  // 0° 下扑起点
static const int PIN_HALL_UP   = 5;  // ~180° 上举
// 两级减速：电机→12/59→14/79→输出盘；精确分数 4661/168（勿再近似 27.744）
static const int32_t GEAR_TEETH_PINION1 = 12;
static const int32_t GEAR_TEETH_WHEEL1  = 59;
static const int32_t GEAR_TEETH_PINION2 = 14;
static const int32_t GEAR_TEETH_WHEEL2  = 79;
static const int32_t GEAR_RATIO_NUM = GEAR_TEETH_WHEEL1 * GEAR_TEETH_WHEEL2;  // 4661
static const int32_t GEAR_RATIO_DEN = GEAR_TEETH_PINION1 * GEAR_TEETH_PINION2;  // 168
// (59/12)*(79/14) = 4661/168 ≈ 27.744047619…
static const float GEAR_RATIO =
    (float)GEAR_RATIO_NUM / (float)GEAR_RATIO_DEN;

// ---- 时序 ----
static const uint32_t SERIAL_BAUD = 921600;
// 输出频率 400Hz（≠采集/使用）。周期 2500µs；esp_timer→信号量唤醒 Core1（µs 级），
// 避免 vTaskDelayUntil(2.5ms) 在 tick=1ms 时落成 2~3ms。见总纲 §6.1。
static const uint32_t CTRL_HZ = 400;                              // 输出频率 Hz
static const uint32_t CTRL_PERIOD_US = 1000000UL / CTRL_HZ;       // 2500
// USB 遥测帧率：每秒发送多少「完整 CSV 遥测行」（一帧=一行全字段），不是一帧内算 N 次。
// 旧稿「100Hz」=每秒 100 帧；现默认 20Hz=每秒 20 帧（周期 50ms），每拍最多 1 行。
static const uint32_t USB_TELEM_HZ = 20;
static const uint32_t HOST_TIMEOUT_DEFAULT_MS = 1500;             // 上电默认；HOST ON 恢复
// 运行时可改：0=禁用（HOST OFF）；FLIGHT/TEST/LEARN 亦豁免。见 controlTick。
uint32_t host_timeout_ms = HOST_TIMEOUT_DEFAULT_MS;

// ---- 采集+使用：高频编码器（与输出解耦）----
// 采集=SPI+解缠 @2k；使用=窗差分求速同回调 @2k（采了 2k、用 2k）。
// 方案原文使用频率建议≥1200；本工程保持 2000，勿降到 1200/400。
// Nyquist @2k ≈0.5rev/样本 → 60000RPM。
static const uint32_t ENC_SAMPLE_HZ = 2000;                       // 采集/使用目标 Hz
static const uint32_t ENC_SAMPLE_PERIOD_US = 1000000UL / ENC_SAMPLE_HZ;
static const uint32_t ENC_SPI_HZ = 8000000;
static const int      ENC_HIST = 16;
static const int      ENC_VEL_WINDOW = 8;                         // 使用侧测速窗(@2kHz=4ms)
static const uint32_t ENC_DIAG_DIV = 100;
static const uint32_t ENC_SIGN_FLIP_SAMPLES = ENC_SAMPLE_HZ / 12;
// 默认：portMUX 快照 + 浮点求速/PID。USE_FIXED_POINT=1 时采集侧定点求速+环缓（输出仍读 EncShared）。

// 共享快照：采集/使用回调写、输出任务读，portMUX 保护。
struct EncShared {
  int64_t counts;
  uint16_t raw;
  bool ef;
  uint8_t agc;
  bool mag_low;
  bool mag_high;
  float rpm_signed;    // 使用层滤波后带符号 rpm
  uint32_t t_us;
  uint32_t seq;        // 采集序号 → ENC? meas_hz
  uint32_t busy_acc;
  uint32_t busy_cnt;
  uint32_t busy_max;   // 窗内回调耗时 max(µs)，encTakeBusy 时清零
};

// ---- 电调 PWM（默认与输出同相 400Hz；FREQ 可改 50..600）----
static const uint32_t ESC_PWM_HZ_DEFAULT = 400;
static const uint32_t ESC_PWM_HZ_MAX = 600;
static const uint16_t ESC_PULSE_MIN_US = 1000;
static const uint16_t ESC_PULSE_MAX_US = 2000;
static const uint8_t  ESC_PWM_RES_BITS = 14;
static const float    MOTOR_RPM_ABS_MAX = 12000.0f;
static const float    TARGET_RPM_MAX_DEFAULT = 12000.0f;

enum EscProto : uint8_t { ESC_PROTO_PWM = 0, ESC_PROTO_DSHOT = 1 };
EscProto esc_proto = ESC_PROTO_PWM;
EscDshot g_dshot;

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
// LEARN RAMP：开环斜坡辨识（脉宽连续上升，比台阶更快）
static const float    RAMP_US_PER_S = 80.0f;         // 斜坡速率：80us/s
static const uint32_t RAMP_SAMPLE_MS = 100;           // 每 ~100ms 采样一次
static const uint16_t RAMP_SAMPLE_PULSE_STEP = 25;    // 脉宽增量达此值才记入图

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

// 转速分区 PID 表：按 rpm 插值 kp/ki/kd，类似 EFI 点火/喷油 MAP
static const int GAIN_MAX_POINTS = 12;
struct GainMap {
  int n;
  float rpm[GAIN_MAX_POINTS];
  float kp[GAIN_MAX_POINTS];
  float ki[GAIN_MAX_POINTS];
  float kd[GAIN_MAX_POINTS];
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
float soft_rate_up_rpm_s = 600.0f;    // 指令上升斜率（加速）
float soft_rate_down_rpm_s = 600.0f;  // 指令下降斜率（减速，空载可设更慢）
// 兼容旧名：部分代码/日志仍读写 soft_rate_rpm_s，始终与 up 同步含义见 SOFT RATE 命令
float soft_rate_rpm_s = 600.0f;
// 指令斜坡瞬时加速度 [rpm/s]：由 updateSoftRamp 写入，供 S2 加速度前馈使用
float soft_cmd_accel_rpm_s = 0.0f;
// S2：脉宽域加速度前馈 Δpulse = ka * soft_cmd_accel_rpm_s；单位 us/(rpm/s)
float ka_us_per_rpms = 0.0f;
// S3：加减速增益 / 减速积分限制（指令加速度 soft_cmd_accel 判别）
bool adg_on = false;
bool adg_dual = false;           // false=缩放当前 GainMap/全局 PID；true=用 ACCEL/DECEL 绝对值
float adg_ki_decel_scale = 0.35f;
float adg_kp_decel_scale = 1.0f;
float adg_i_lim_decel = 250.0f;
float adg_i_lim_accel = 800.0f;
float adg_kp_accel = 0.08f;
float adg_ki_accel = 0.25f;
float adg_kp_decel = 0.06f;
float adg_ki_decel = 0.08f;

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
GainMap gains[2];               // 与 maps[2] 并行：每个 profile 一张分区 PID 表
bool gain_schedule_on = true;   // 关闭时闭环用全局 kp/ki/kd
uint16_t learn_max_us = LEARN_MAX_US_DEFAULT;
uint16_t learn_pulse = ESC_PULSE_MIN_US;
uint32_t learn_step_t0 = 0;
int learn_idx = 0;
bool learn_active = false;
bool learn_mode_ramp = false;         // true=LEARN RAMP（斜坡），false=LEARN START（台阶）
float learn_ramp_pulse_f = (float)ESC_PULSE_MIN_US;  // 亚微秒累积，避免整数截断卡步
uint32_t learn_ramp_last_sample_ms = 0;
uint16_t learn_ramp_last_pulse = ESC_PULSE_MIN_US;

uint16_t measure_pulse = ESC_PULSE_MIN_US;
bool measure_active = false;
// OPEN 下 PWM 指令锁存：否则 controlTick 会按 target_ramped 覆盖开环探点
bool open_pwm_hold = false;
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
// 扑翼盘相位保持：单向爬行 + 提前提前制动（不可等霍尔沿再停，惯量会冲过）
// 0°=GPIO4 下扑；180°=GPIO5 上举；90°=盘上合成（编码器/减速比，无专用霍尔）
bool holdph_active = false;
float holdph_target_deg = 0.0f;       // 盘相位目标 [0,360)
float holdph_tol_deg = 5.0f;          // 到位容差（盘°）
float holdph_crawl_rpm = 120.0f;      // 电机轴爬行 RPM（盘更慢 ≈ rpm/GEAR）
uint16_t holdph_pulse = ESC_PULSE_MIN_US;
uint16_t holdph_max_pulse = 1450;     // 接近段油门上限（仅爬行）
uint32_t holdph_t0_ms = 0;
uint32_t holdph_timeout_ms = 30000;   // 总超时 → ESTOP 1000µs
uint32_t holdph_brake_ms = 0;         // 进入制动的时刻
uint32_t holdph_dn0 = 0;
uint32_t holdph_up0 = 0;
float holdph_prev_disc = NAN;
float holdph_prev_remain = NAN;
bool holdph_hall_seen = false;        // 制动后霍尔确认（非停机触发）
// 提前量：effective_lead = lead_deg + ω_disc*lead_ms/1000 + k*|rpm_motor|
float holdph_lead_deg = 10.0f;        // 基础提前盘角 °（NVS hplead）
float holdph_lead_ms = 100.0f;        // 按当前盘角速度折算的提前时间 ms（NVS hpleadt）
float holdph_lead_rpm_k = 0.03f;      // 额外盘° / 电机RPM（NVS hpleadk）
float holdph_stop_rpm = 40.0f;        // 判停电机 RPM 阈值
// 定时软降：HOLD PHASE <deg> [crawl] [stop_ms]；stop_ms=0 → 旧爬行
uint32_t holdph_profile_ms = 0;       // 期望壁钟停机时间（软降+爬行接近）
uint32_t holdph_soft_ms = 0;          // 软降段时长（profile 的前段）
float holdph_rpm_start = 0.0f;        // 进入时电机 |RPM|
bool holdph_soft_done = false;        // 软降结束 → 允许 lead 提前制动
enum HoldPhState : uint8_t {
  HOLDPH_ST_APPROACH = 0,  // 单向软接近（含可选软降）
  HOLDPH_ST_BRAKE = 1,     // 已切 1000µs，惯量滑行
  HOLDPH_ST_SETTLE = 2     // 低速确认到位
};
uint8_t holdph_state = HOLDPH_ST_APPROACH;
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
bool flight_mode = false;  // FLIGHT ON：关 BLE + 禁用主机超时 ESTOP
bool test_active = false;  // 板内 TEST 分段：禁 HOST_TIMEOUT（同 FLIGHT）
static bool tune_owned_by_test = false;  // TEST 自动开 TUNE，结束再关
// 闭环最近一拍（20Hz 遥测尾列 err,ff,u；仅 CLOSED 有意义）
float g_closed_err = 0.0f;
float g_closed_ff = 0.0f;
float g_closed_u = 0.0f;

// ---- 板内 TEST（写死参数；主机只发 START/STOP/?）----
static const float    TEST_RPM_SPIN_MIN = 90.0f;
static const float    TEST_SPIN_FRAC_MIN = 0.70f;
static const uint32_t TEST_SPIN_SAMPLE_MS = 50;
static const uint32_t TEST_SPIN_WINDOW_MS = 3000;
static const uint8_t  TEST_SPIN_MAX_SAMP = 64;
static const float    TEST_SOFT_RATE_UP = 1000.0f;
static const float    TEST_SOFT_RATE_DOWN = 800.0f;
static const float    TEST_RPMMAX = 12000.0f;
static const float    TEST_SMOKE_RPM = 2000.0f;
static const uint32_t TEST_SMOKE_HOLD_MS = 12000;  // 10–15s 取中
static const float    TEST_SMOKE_ERR_FRAC = 0.15f;
static const float    TEST_STEP_RPM = 7500.0f;
static const uint32_t TEST_STEP_HOLD_MS = 20000;
static const float    TEST_STEP_ERR_FRAC = 0.15f;
static const float    TEST_IDLE_RPM_MAX = 30.0f;
static const uint32_t TEST_IDLE_WINDOW_MS = 2000;
static const uint32_t TEST_RAMP_TIMEOUT_MS = 20000;
static const float    TEST_RAMP_NEAR_RPM = 80.0f;

enum TestKind : uint8_t {
  TEST_KIND_NONE = 0,
  TEST_KIND_SMOKE2000,
  TEST_KIND_STEP7500,
  TEST_KIND_IDLE
};
enum TestPhase : uint8_t {
  TEST_PH_IDLE = 0,
  TEST_PH_ARM,
  TEST_PH_RAMP,
  TEST_PH_SPIN,
  TEST_PH_HOLD,
  TEST_PH_STEP_RAMP,
  TEST_PH_STEP_SPIN,
  TEST_PH_STEP_HOLD,
  TEST_PH_STOPPING,
  TEST_PH_DONE
};

static TestKind  test_kind = TEST_KIND_NONE;
static TestPhase test_phase = TEST_PH_IDLE;
static uint32_t  test_phase_t0 = 0;
static float     test_set_rpm = 0.0f;
static float     test_err_frac = 0.15f;
static uint32_t  test_hold_ms = 0;
static char      test_name[16] = "";
static char      test_reason[40] = "";
static float     test_spin_buf[TEST_SPIN_MAX_SAMP];
static uint8_t   test_spin_n = 0;
static uint32_t  test_spin_above = 0;
static uint32_t  test_spin_last_ms = 0;
static uint32_t  test_hall0_dn = 0;
static uint32_t  test_hall0_up = 0;
static float     test_hold_sum = 0.0f;
static float     test_hold_err_sum = 0.0f;
static uint32_t  test_hold_n = 0;
static uint16_t  test_pulse_peak = 0;
static bool      test_pulse_sat = false;
static float     test_enc_rpm = 0.0f;
static float     test_err_mean = 0.0f;
static int       test_hall_d = 0;
static bool      test_ok = false;
static bool      test_finish_pending = false;  // ESTOP 路径避免重入

// 输出任务 / 核诊断
static SemaphoreHandle_t g_ctrl_sem = nullptr;
static esp_timer_handle_t g_ctrl_timer = nullptr;
static TaskHandle_t g_ctrl_task = nullptr;
static volatile uint32_t g_ctrl_core_id = 0;
static volatile uint32_t g_enc_cb_core_id = 0;
static volatile uint32_t g_loop_core_id = 0;
float ctrl_out_hz_meas = 0.0f;  // 实测输出频率（CTRL?）
static volatile uint32_t g_ctrl_tick_cnt = 0;

// ---- 负载快照（LOAD? / LOAD RESET；对比浮点/定点、遥测、BLE 等工况）----
static volatile bool g_usb_telem_on = true;           // TELEM ON|OFF（CSV 写出）
static volatile uint32_t g_loop_iters = 0;            // loop() 空转告警粗估
float    loop_hz_meas = 0.0f;                        // 主 loop 迭代率（越高越空闲）
float    enc_busy_avg_us = 0.0f;                     // 采集回调平均耗时 µs
uint32_t enc_busy_max_us = 0;                        // 采集回调最大耗时 µs（自 RESET）
uint32_t enc_busy_max_window_us = 0;                 // 最近统计窗内 max
float    ctrl_period_avg_us = 0.0f;                  // 控制唤醒间隔均值（期望 2500）
uint32_t ctrl_period_max_us = 0;                     // 控制唤醒间隔 max（自 RESET）
float    ctrl_busy_avg_us = 0.0f;                    // 控制任务一拍耗时均值 µs
uint32_t ctrl_busy_max_us = 0;                       // 控制任务一拍耗时 max（自 RESET）
uint32_t ctrl_overrun_cnt = 0;                       // 周期 > 期望+slack 次数（自 RESET）
static const uint32_t CTRL_OVERRUN_SLACK_US = 500;    // >3000µs 记 overrun（期望 2500）
static portMUX_TYPE g_load_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t g_ctrl_last_wake_us = 0;
static uint32_t g_ctrl_period_acc_us = 0;
static uint32_t g_ctrl_period_n = 0;
static uint32_t g_ctrl_busy_acc_us = 0;
static uint32_t g_ctrl_busy_n = 0;

// ---- 扑翼霍尔 / 输出盘相位（相对电机编码器反算）----
// GPIO4/5：ESP32-S3 DevKit 丝印 IO4/IO5 = Arduino GPIO4/5；非 USB(19/20)/UART0(43/44)/strapping
volatile bool hall_active_low = true;  // 厂商 51 样例：if(DO==0) 触发；可用 HALL ACTIVE 改
uint8_t hall_dn_lvl = 1;               // 原始读数 0/1（遥测用，主循环读）
uint8_t hall_up_lvl = 1;
uint8_t hall_dn_prev = 1;              // 主循环边沿备份（对照 ISR）
uint8_t hall_up_prev = 1;
uint32_t hall_dn_count = 0;            // 主循环消费后的累计触发次数
uint32_t hall_up_count = 0;
// ISR：IDF gpio_isr + ANYEDGE，软件判进入有效电平（避免 Arduino attachInterrupt 漏计）
static const uint32_t HALL_IRQ_DEBOUNCE_US = 5000;  // 同脚最短间隔 5ms，滤磁滞/触点抖
volatile uint32_t hall_dn_irq_cnt = 0;
volatile uint32_t hall_up_irq_cnt = 0;
volatile uint32_t hall_dn_irq_us = 0;  // 最近一次有效沿 micros()
volatile uint32_t hall_up_irq_us = 0;
static uint32_t hall_dn_irq_seen = 0;  // 主循环已处理到的 irq 计数
static uint32_t hall_up_irq_seen = 0;
volatile uint32_t hall_dn_irq_last_us = 0;
volatile uint32_t hall_up_irq_last_us = 0;
// 盘相位 latch 防抖：同标记两次绝对复位至少隔这么多电机 counts（≈40° 盘）
// = 0.40 * gear * CPR ≈ 0.40 * 4661/168 * 16384
static const int64_t HALL_LATCH_MIN_MOTOR_COUNTS =
    (int64_t)GEAR_RATIO_NUM * (int64_t)ENC_CPR * 40LL / ((int64_t)GEAR_RATIO_DEN * 360LL);
// 主循环 digitalRead 采样：转盘时应有低电平占比>0；poll_edge=软件边沿备份计数
static uint32_t hall_dn_samp_n = 0;
static uint32_t hall_dn_low_n = 0;
static uint32_t hall_up_samp_n = 0;
static uint32_t hall_up_low_n = 0;
static uint32_t hall_dn_poll_edge = 0;
static uint32_t hall_up_poll_edge = 0;
static bool hall_gpio_isr_ok = false;
float motor_unwrapped_deg = 0.0f;  // 累计电机角（°），用于盘相位
// —— 盘相位：霍尔绝对复位 + 磁编连续推算 ——
// 过 GPIO4/5（或 HALL CAL）时 latch；之后仅靠编码器积分，霍尔再过时纠正。
bool flap_calibrated = false;
float disc_ref_deg = 0.0f;              // 上次霍尔绝对盘角：0°(GPIO4) 或 180°(GPIO5)
float disc_ref_motor_unwrap = 0.0f;     // latch 时电机展开角 °
int64_t disc_ref_motor_counts = 0;      // latch 时电机累计 counts
// 盘相位常值偏置（NVS hphoff）：修 0↔180 装反/标反；加在 latch+积分之后
float disc_phase_offset_deg = 0.0f;
// 兼容旧名：flap_zero_* = 把 disc 写成「相对 0° 的电机展开参考」时的等价零点
// （仅诊断；主路径用 disc_ref + Δmotor）
float flap_zero_motor_unwrap = 0.0f;
float last_hd_motor_rel = NAN;     // 触发时电机相对相位
float last_hu_motor_rel = NAN;
float last_hd_motor_abs = NAN;
float last_hu_motor_abs = NAN;
uint16_t last_hd_raw = 0;
uint16_t last_hu_raw = 0;
float last_hd_disc = NAN;          // 触发时盘相位（GPIO4 锁存后应为 0）
float last_hu_disc = NAN;          // 触发时盘相位（GPIO5 锁存后应为 180）
uint32_t last_hd_ms = 0;
uint32_t last_hu_ms = 0;
float last_hd_motor_unwrap = NAN;  // 上次下扑(0°)沿电机展开角
float last_hu_motor_unwrap = NAN;  // 上次上举(180°)沿电机展开角
// 霍尔实测：同标记两次过点 = 输出盘 1 转
float out_hz_meas = NAN;           // 最近一次同标记周期 → Hz
float out_rpm_meas = NAN;          // = out_hz_meas * 60
float gear_ratio_meas = NAN;       // 一盘转内电机转数 ≈27.744
float out_hz_meas_dn = NAN;        // 仅 0°→0°
float out_hz_meas_up = NAN;        // 仅 180°→180°
float gear_ratio_meas_dn = NAN;
float gear_ratio_meas_up = NAN;
uint8_t gear_meas_src = 0;         // 1=DOWN 2=UP

// —— 定点（整型）减速比/频率：全程整数运算，避免浮点/打印截断误差 ——
// AS5047P 每圈 16384 counts；设计比 4661/168（精确）
static const int64_t ENC_CPR_I = 16384;
// round(4661/168 * 1e6) = round(27744047.619…) = 27744048
static const int64_t GEAR_DESIGN_MICRO =
    (GEAR_RATIO_NUM * 1000000LL + GEAR_RATIO_DEN / 2) / GEAR_RATIO_DEN;
int64_t motor_unwrapped_counts = 0;   // 整型累计编码器计数（raw 差分 ±8192 环绕已处理）
int last_raw_i = -1;                   // 上拍 raw（<0=未初始化）
int64_t last_hd_motor_unwrap_counts = 0;
int64_t last_hu_motor_unwrap_counts = 0;
bool has_hd_counts = false;
bool has_hu_counts = false;
int64_t gear_micro_meas = -1;         // 最近一次同标记两过点：减速比 ×1e6
int64_t out_hz_micro_meas = -1;       // 最近一次：盘频 Hz ×1e6
int64_t gear_micro_dn = -1;           // 0°→0° 定点减速比 ×1e6
int64_t gear_micro_up = -1;           // 180°→180° 定点减速比 ×1e6
int64_t out_hz_micro_dn = -1;         // 0°→0° 定点盘频 ×1e6
int64_t out_hz_micro_up = -1;         // 180°→180° 定点盘频 ×1e6

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

// ================= 高频编码器采样（esp_timer 解耦）=================
// EncShared 结构已在上方配置区声明。
static portMUX_TYPE g_enc_mux = portMUX_INITIALIZER_UNLOCKED;
static EncShared g_enc = {};
static esp_timer_handle_t g_enc_timer = nullptr;
static uint16_t g_cmd_angle = 0;
static uint16_t g_cmd_dia = 0;

// —— 回调私有状态（仅采样回调访问，无需加锁）——
static int      enc_cb_last_raw = -1;
static int64_t  enc_cb_counts = 0;
static int64_t  enc_hist_counts[ENC_HIST] = {0};
static uint32_t enc_hist_us[ENC_HIST] = {0};
static int      enc_hist_head = 0;
static int      enc_hist_fill = 0;
static float    enc_cb_rpm_filt = 0.0f;
static float    enc_cb_rpm_stable = 0.0f;
static int      enc_cb_sign_lock = 0;
static uint32_t enc_cb_flip_cnt = 0;
static uint16_t enc_cb_diag_div = 0;
static uint8_t  enc_cb_agc = 0;
static bool     enc_cb_mag_low = false;
static bool     enc_cb_mag_high = false;

#if USE_FIXED_POINT
static enc_ring_t g_enc_ring;
static q16_t      enc_cb_rpm_filt_q16 = 0;
static q16_t      enc_cb_rpm_stable_q16 = 0;
static const q16_t ENC_EMA_ALPHA_Q16 = FLOAT_TO_Q16(0.10f);
#endif

// 浮点主路旁路：同采样再算 Q16，累积 |Δrpm|（禁打印；查询用 Q16CMP?）
#ifndef Q16_SHADOW_COMPARE
#if !USE_FIXED_POINT
#define Q16_SHADOW_COMPARE 1
#else
#define Q16_SHADOW_COMPARE 0
#endif
#endif
#if Q16_SHADOW_COMPARE
static const q16_t ENC_EMA_ALPHA_Q16_SHADOW = FLOAT_TO_Q16(0.10f);
static q16_t   shadow_rpm_filt_q16 = 0;
static q16_t   shadow_rpm_stable_q16 = 0;
static int     shadow_sign_lock = 0;
static uint32_t shadow_flip_cnt = 0;
static portMUX_TYPE g_q16cmp_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t q16cmp_n = 0;
static double   q16cmp_sum_abs = 0.0;
static float    q16cmp_max_abs = 0.0f;
static float    q16cmp_last_f = 0.0f;
static float    q16cmp_last_q = 0.0f;

static inline void q16cmpStatsReset() {
  portENTER_CRITICAL(&g_q16cmp_mux);
  q16cmp_n = 0;
  q16cmp_sum_abs = 0.0;
  q16cmp_max_abs = 0.0f;
  // keep last_f/last_q for readability
  portEXIT_CRITICAL(&g_q16cmp_mux);
}

static inline void q16cmpFullReset() {
  q16cmpStatsReset();
  shadow_rpm_filt_q16 = 0;
  shadow_rpm_stable_q16 = 0;
  shadow_sign_lock = 0;
  shadow_flip_cnt = 0;
}

static inline void q16cmpShadowUpdate(bool have_win, int64_t dc, uint32_t dt_us,
                                      float rpm_float_stable) {
  q16_t rpm_inst_q = shadow_rpm_filt_q16;
  if (have_win && dt_us > 0) {
    rpm_inst_q = calc_rpm_window_q16(dc, dt_us, (int32_t)ENC_CPR_I);
  }
  q16_t lim_q = FLOAT_TO_Q16(target_rpm_max * 1.5f + 500.0f);
  if (q16_abs(rpm_inst_q) > lim_q) rpm_inst_q = shadow_rpm_filt_q16;
  shadow_rpm_filt_q16 = ema_update_q16(shadow_rpm_filt_q16, rpm_inst_q, ENC_EMA_ALPHA_Q16_SHADOW);
  float mag = Q16_TO_FLOAT(q16_abs(shadow_rpm_filt_q16));
  int sgn = (shadow_rpm_filt_q16 > 0) ? 1 : ((shadow_rpm_filt_q16 < 0) ? -1 : 0);
  if (mag < 80.0f) {
    shadow_sign_lock = 0;
    shadow_flip_cnt = 0;
    shadow_rpm_stable_q16 = shadow_rpm_filt_q16;
  } else {
    if (shadow_sign_lock == 0) {
      if (sgn != 0) shadow_sign_lock = sgn;
    } else if (sgn != 0 && sgn != shadow_sign_lock) {
      if (++shadow_flip_cnt >= ENC_SIGN_FLIP_SAMPLES) {
        shadow_sign_lock = sgn;
        shadow_flip_cnt = 0;
      }
    } else {
      shadow_flip_cnt = 0;
    }
    int use = (shadow_sign_lock != 0) ? shadow_sign_lock : sgn;
    if (use == 0) use = 1;
    shadow_rpm_stable_q16 = FLOAT_TO_Q16((float)use * mag);
  }
  float rpm_q = Q16_TO_FLOAT(shadow_rpm_stable_q16);
  float d = fabsf(rpm_float_stable - rpm_q);
  portENTER_CRITICAL(&g_q16cmp_mux);
  q16cmp_n++;
  q16cmp_sum_abs += (double)d;
  if (d > q16cmp_max_abs) q16cmp_max_abs = d;
  q16cmp_last_f = rpm_float_stable;
  q16cmp_last_q = rpm_q;
  portEXIT_CRITICAL(&g_q16cmp_mux);
}
#endif


// —— 供命令/遥测读取的实测统计（loop 侧写）——
float    enc_sample_hz_meas = 0.0f;  // 实测采样率
float    enc_cpu_pct = 0.0f;         // 采样任务 CPU 占用估计(%)
float    enc_loop_hz = 0.0f;         // 主循环空转频率（越高越空闲）

static inline void encReadDiagInline() {
  spiFrame(g_cmd_dia);
  uint16_t d = spiFrame(g_cmd_angle);
  if (checkEvenParity(d)) {
    enc_cb_agc = d & 0xFF;
    enc_cb_mag_high = (d >> 10) & 1;
    enc_cb_mag_low = (d >> 11) & 1;
  }
}

// esp_timer 周期回调（采集+使用同层；ESP_TIMER_TASK；禁止 Serial/hostPrintf）
static void encSampleCb(void* /*arg*/) {
  hostEnterRealtimeCb();
  g_enc_cb_core_id = (uint32_t)xPortGetCoreID();
  uint32_t t0 = micros();

  if (++enc_cb_diag_div >= ENC_DIAG_DIV) {
    enc_cb_diag_div = 0;
    encReadDiagInline();
  }
  uint16_t rx = spiFrame(g_cmd_angle);
  bool ef = (rx >> 14) & 1;
  uint16_t raw = rx & 0x3FFF;
  uint32_t now = t0;

  // 整型差分解缠（±8192 环绕）累加到 counts
  if (enc_cb_last_raw >= 0) {
    int draw = (int)raw - enc_cb_last_raw;
    if (draw > 8192) draw -= 16384;
    else if (draw < -8192) draw += 16384;
    enc_cb_counts += draw;
  }
  enc_cb_last_raw = (int)raw;

  // 写历史环
#if USE_FIXED_POINT
  // 备份路径：环缓 + Q16 窗差求速 + EMA（禁打印）
  enc_ring_push(&g_enc_ring, now, enc_cb_counts);
  enc_ring_sample_t peek[ENC_VEL_WINDOW + 1];
  uint32_t npeek = enc_ring_peek_n(&g_enc_ring, peek, (uint32_t)(ENC_VEL_WINDOW + 1));
  q16_t rpm_inst_q = enc_cb_rpm_filt_q16;
  if (npeek >= 2) {
    uint32_t i0 = 0;
    uint32_t i1 = npeek - 1;
    int64_t dc = peek[i1].counts - peek[i0].counts;
    uint32_t dt_us = peek[i1].t_us - peek[i0].t_us;
    if (dt_us > 0) {
      rpm_inst_q = calc_rpm_window_q16(dc, dt_us, (int32_t)ENC_CPR_I);
    }
  }
  q16_t lim_q = FLOAT_TO_Q16(target_rpm_max * 1.5f + 500.0f);
  if (q16_abs(rpm_inst_q) > lim_q) rpm_inst_q = enc_cb_rpm_filt_q16;
  enc_cb_rpm_filt_q16 = ema_update_q16(enc_cb_rpm_filt_q16, rpm_inst_q, ENC_EMA_ALPHA_Q16);
  float mag = Q16_TO_FLOAT(q16_abs(enc_cb_rpm_filt_q16));
  int sgn = (enc_cb_rpm_filt_q16 > 0) ? 1 : ((enc_cb_rpm_filt_q16 < 0) ? -1 : 0);
  if (mag < 80.0f) {
    enc_cb_sign_lock = 0;
    enc_cb_flip_cnt = 0;
    enc_cb_rpm_stable_q16 = enc_cb_rpm_filt_q16;
  } else {
    if (enc_cb_sign_lock == 0) {
      if (sgn != 0) enc_cb_sign_lock = sgn;
    } else if (sgn != 0 && sgn != enc_cb_sign_lock) {
      if (++enc_cb_flip_cnt >= ENC_SIGN_FLIP_SAMPLES) {
        enc_cb_sign_lock = sgn;
        enc_cb_flip_cnt = 0;
      }
    } else {
      enc_cb_flip_cnt = 0;
    }
    int use = (enc_cb_sign_lock != 0) ? enc_cb_sign_lock : sgn;
    if (use == 0) use = 1;
    enc_cb_rpm_stable_q16 = FLOAT_TO_Q16((float)use * mag);
  }
  enc_cb_rpm_filt = Q16_TO_FLOAT(enc_cb_rpm_filt_q16);
  enc_cb_rpm_stable = Q16_TO_FLOAT(enc_cb_rpm_stable_q16);
#else
  // 主路径：浮点历史窗 + EMA（采集2k/使用2k）
  int cur = enc_hist_head;
  enc_hist_counts[cur] = enc_cb_counts;
  enc_hist_us[cur] = now;
  enc_hist_head = (enc_hist_head + 1) % ENC_HIST;
  if (enc_hist_fill < ENC_HIST) enc_hist_fill++;

  float rpm_inst = enc_cb_rpm_filt;
  int w = ENC_VEL_WINDOW;
  if (w > enc_hist_fill - 1) w = enc_hist_fill - 1;
  bool have_win = false;
  int64_t dc_win = 0;
  uint32_t dt_win = 0;
  if (w >= 1) {
    int idx = (cur - w + ENC_HIST) % ENC_HIST;
    int64_t dc = enc_cb_counts - enc_hist_counts[idx];
    uint32_t dt_us = now - enc_hist_us[idx];
    if (dt_us > 0) {
      have_win = true;
      dc_win = dc;
      dt_win = dt_us;
      rpm_inst = (float)dc * 60000000.0f / ((float)ENC_CPR_I * (float)dt_us);
    }
  }

  float lim = target_rpm_max * 1.5f + 500.0f;
  if (fabsf(rpm_inst) > lim) rpm_inst = enc_cb_rpm_filt;
  enc_cb_rpm_filt = 0.90f * enc_cb_rpm_filt + 0.10f * rpm_inst;

  float mag = fabsf(enc_cb_rpm_filt);
  int sgn = (enc_cb_rpm_filt > 0.0f) ? 1 : ((enc_cb_rpm_filt < 0.0f) ? -1 : 0);
  if (mag < 80.0f) {
    enc_cb_sign_lock = 0;
    enc_cb_flip_cnt = 0;
    enc_cb_rpm_stable = enc_cb_rpm_filt;
  } else {
    if (enc_cb_sign_lock == 0) {
      if (sgn != 0) enc_cb_sign_lock = sgn;
    } else if (sgn != 0 && sgn != enc_cb_sign_lock) {
      if (++enc_cb_flip_cnt >= ENC_SIGN_FLIP_SAMPLES) {
        enc_cb_sign_lock = sgn;
        enc_cb_flip_cnt = 0;
      }
    } else {
      enc_cb_flip_cnt = 0;
    }
    int use = (enc_cb_sign_lock != 0) ? enc_cb_sign_lock : sgn;
    if (use == 0) use = 1;
    enc_cb_rpm_stable = (float)use * mag;
  }
#if Q16_SHADOW_COMPARE
  // 同窗 (dc,dt) 旁路 Q16：控制仍用浮点；仅统计 |Δ|
  q16cmpShadowUpdate(have_win, dc_win, dt_win, enc_cb_rpm_stable);
#endif
#endif

  uint32_t busy = micros() - t0;
  portENTER_CRITICAL(&g_enc_mux);
  g_enc.counts = enc_cb_counts;
  g_enc.raw = raw;
  g_enc.ef = ef;
  g_enc.agc = enc_cb_agc;
  g_enc.mag_low = enc_cb_mag_low;
  g_enc.mag_high = enc_cb_mag_high;
  g_enc.rpm_signed = enc_cb_rpm_stable;
  g_enc.t_us = now;
  g_enc.seq++;
  g_enc.busy_acc += busy;
  g_enc.busy_cnt++;
  if (busy > g_enc.busy_max) g_enc.busy_max = busy;
  portEXIT_CRITICAL(&g_enc_mux);
  hostExitRealtimeCb();
}

// 读快照（loop 侧调用）
static inline void encGetSnapshot(EncShared* out) {
  portENTER_CRITICAL(&g_enc_mux);
  *out = g_enc;
  portEXIT_CRITICAL(&g_enc_mux);
}

// 取并清零采样耗时累计（供 CPU / LOAD 估计）
static inline void encTakeBusy(uint32_t* acc, uint32_t* cnt, uint32_t* mx) {
  portENTER_CRITICAL(&g_enc_mux);
  *acc = g_enc.busy_acc;
  *cnt = g_enc.busy_cnt;
  *mx = g_enc.busy_max;
  g_enc.busy_acc = 0;
  g_enc.busy_cnt = 0;
  g_enc.busy_max = 0;
  portEXIT_CRITICAL(&g_enc_mux);
}

static void loadStatsReset() {
  portENTER_CRITICAL(&g_load_mux);
  enc_busy_max_us = 0;
  enc_busy_max_window_us = 0;
  enc_busy_avg_us = 0.0f;
  ctrl_period_max_us = 0;
  ctrl_period_avg_us = 0.0f;
  ctrl_busy_max_us = 0;
  ctrl_busy_avg_us = 0.0f;
  ctrl_overrun_cnt = 0;
  g_ctrl_last_wake_us = 0;
  g_ctrl_period_acc_us = 0;
  g_ctrl_period_n = 0;
  g_ctrl_busy_acc_us = 0;
  g_ctrl_busy_n = 0;
  portEXIT_CRITICAL(&g_load_mux);
  uint32_t a, c, m;
  encTakeBusy(&a, &c, &m);  // 清窗内累计
  (void)a;
  (void)c;
  (void)m;
  hostAsyncLogDropReset();
}

// 启动高频采样（SPI 已在 setup 里 begin/beginTransaction）
void encoderBegin() {
  g_cmd_angle = buildReadCmd(REG_ANGLECOM);
  g_cmd_dia = buildReadCmd(REG_DIAAGC);
  // 预热：先发一次诊断+角度命令，使后续帧返回有效数据
  encReadDiagInline();
  spiFrame(g_cmd_angle);
  enc_cb_last_raw = -1;
  enc_cb_counts = 0;
  enc_hist_head = 0;
  enc_hist_fill = 0;
#if USE_FIXED_POINT
  enc_ring_init(&g_enc_ring);
  enc_cb_rpm_filt_q16 = 0;
  enc_cb_rpm_stable_q16 = 0;
#endif
#if Q16_SHADOW_COMPARE
  q16cmpFullReset();
#endif

  const esp_timer_create_args_t args = {
      .callback = &encSampleCb,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "enc_sample",
      .skip_unhandled_events = true,
  };
  if (esp_timer_create(&args, &g_enc_timer) == ESP_OK) {
    esp_timer_start_periodic(g_enc_timer, ENC_SAMPLE_PERIOD_US);
  }
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

float discEstFromMotor() {
  // 传感模型：霍尔 latch 绝对盘角 + 磁编连续推算
  //   disc = wrap( disc_ref + esc_sense * (motor_now − motor_at_hall) / gear + offset )
  // gear = 4661/168；esc_sense 使盘角随驱动方向递增；offset 修 0↔180 标反
  if (!flap_calibrated || GEAR_RATIO < 1.0f) return NAN;
  float d_motor =
      (float)(motor_unwrapped_counts - disc_ref_motor_counts) * ENC_DEG_PER_COUNT;
  float d_disc = (float)esc_sense * d_motor / GEAR_RATIO;
  return wrap360(disc_ref_deg + d_disc + disc_phase_offset_deg);
}

/** 盘上单向剩余角：盘相位已定义为随油门方向递增，始终 target−disc（勿再套 uniTravelDeg/esc_sense） */
static float discRemainFwd(float disc, float target) {
  float d = wrap360(target - disc);
  return (d < 0.5f) ? 0.0f : d;
}

/**
 * 霍尔（或 HALL CAL）绝对复位：记下 disc_ref 与当时电机计数。
 * 之后盘相位只靠磁编；再过霍尔时再次调用纠正漂移。
 */
static void discLatchHall(float abs_disc_deg) {
  abs_disc_deg = wrap360(abs_disc_deg);
  disc_ref_deg = abs_disc_deg;
  disc_ref_motor_unwrap = motor_unwrapped_deg;
  disc_ref_motor_counts = motor_unwrapped_counts;
  // 等价「盘 0° 时的电机展开角」：便于旧日志对照
  // zero_motor = motor_now − esc_sense * disc_ref * gear
  flap_zero_motor_unwrap =
      motor_unwrapped_deg - (float)esc_sense * abs_disc_deg * GEAR_RATIO;
  flap_calibrated = true;
}

/** 同霍尔抖动时拒绝重复 latch；换 0↔180 标记时行程门限减半 */
static bool discLatchHallAllowed(float abs_disc_deg) {
  if (!flap_calibrated) return true;
  int64_t dc = motor_unwrapped_counts - disc_ref_motor_counts;
  if (dc < 0) dc = -dc;
  float dref = fabsf(wrap180(abs_disc_deg - disc_ref_deg));
  int64_t need = HALL_LATCH_MIN_MOTOR_COUNTS;
  if (dref > 90.0f) need = HALL_LATCH_MIN_MOTOR_COUNTS / 2;
  return dc >= need;
}

bool hallIsActive(uint8_t lvl) {
  return hall_active_low ? (lvl == 0) : (lvl == 1);
}

/** IDF GPIO ISR：ANYEDGE 后读电平，只计「进入有效电平」的一侧（=软件判下降/上升） */
static void IRAM_ATTR hallDnIsr(void* /*arg*/) {
  const int lvl = gpio_get_level((gpio_num_t)PIN_HALL_DOWN);
  const bool hit = hall_active_low ? (lvl == 0) : (lvl == 1);
  if (!hit) return;
  uint32_t t = micros();
  uint32_t last = hall_dn_irq_last_us;
  if ((uint32_t)(t - last) < HALL_IRQ_DEBOUNCE_US) return;
  hall_dn_irq_last_us = t;
  hall_dn_irq_us = t;
  hall_dn_irq_cnt++;
}

static void IRAM_ATTR hallUpIsr(void* /*arg*/) {
  const int lvl = gpio_get_level((gpio_num_t)PIN_HALL_UP);
  const bool hit = hall_active_low ? (lvl == 0) : (lvl == 1);
  if (!hit) return;
  uint32_t t = micros();
  uint32_t last = hall_up_irq_last_us;
  if ((uint32_t)(t - last) < HALL_IRQ_DEBOUNCE_US) return;
  hall_up_irq_last_us = t;
  hall_up_irq_us = t;
  hall_up_irq_cnt++;
}

void hallAttachIrqs() {
  // 不用 Arduino attachInterrupt：直接 gpio_config + gpio_isr_handler_add（ANYEDGE）
  gpio_config_t io = {};
  io.pin_bit_mask = (1ULL << PIN_HALL_DOWN) | (1ULL << PIN_HALL_UP);
  io.mode = GPIO_MODE_INPUT;
  // 去掉内部上拉：内部~45k 上拉到 3.3V 与外部高阻分压(100k/200k,戴维南≈67k)叠加，
  // 会把 DO 拉低时的脚电压抬到≈2V(>VIL 0.8V)，导致永远读高。悬空电平交给外部分压决定。
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_ANYEDGE;
  esp_err_t cfg = gpio_config(&io);

  static bool isr_service = false;
  if (!isr_service) {
    esp_err_t e = gpio_install_isr_service(0);
    if (e == ESP_OK || e == ESP_ERR_INVALID_STATE) {
      isr_service = true;
    }
  }

  gpio_isr_handler_remove((gpio_num_t)PIN_HALL_DOWN);
  gpio_isr_handler_remove((gpio_num_t)PIN_HALL_UP);
  esp_err_t a0 = gpio_isr_handler_add((gpio_num_t)PIN_HALL_DOWN, hallDnIsr, nullptr);
  esp_err_t a1 = gpio_isr_handler_add((gpio_num_t)PIN_HALL_UP, hallUpIsr, nullptr);
  gpio_intr_enable((gpio_num_t)PIN_HALL_DOWN);
  gpio_intr_enable((gpio_num_t)PIN_HALL_UP);
  hall_gpio_isr_ok = isr_service && (cfg == ESP_OK) && (a0 == ESP_OK) && (a1 == ESP_OK);
}

void hallSetup() {
  hall_dn_irq_cnt = 0;
  hall_up_irq_cnt = 0;
  hall_dn_irq_seen = 0;
  hall_up_irq_seen = 0;
  hall_dn_samp_n = hall_dn_low_n = 0;
  hall_up_samp_n = hall_up_low_n = 0;
  hall_dn_poll_edge = hall_up_poll_edge = 0;
  hall_dn_irq_last_us = micros();
  hall_up_irq_last_us = hall_dn_irq_last_us;
  hallAttachIrqs();
  hall_dn_lvl = (uint8_t)gpio_get_level((gpio_num_t)PIN_HALL_DOWN);
  hall_up_lvl = (uint8_t)gpio_get_level((gpio_num_t)PIN_HALL_UP);
  hall_dn_prev = hall_dn_lvl;
  hall_up_prev = hall_up_lvl;
}

void hallCalibrateNow() {
  // 语义：当前物理位 = 盘 0°（应在 GPIO4 下扑点执行；非任意姿）
  discLatchHall(0.0f);
  last_hd_disc = 0.0f;
  last_hd_motor_unwrap = motor_unwrapped_deg;
  last_hd_motor_unwrap_counts = motor_unwrapped_counts;
  has_hd_counts = true;
}

/** 把定点 ×1e6 值格式化为「整数.六位」；<0 视为无数据打印 -1.000000 */
static void fmtMicro(char* buf, size_t n, int64_t micro) {
  if (micro < 0) {
    snprintf(buf, n, "-1.000000");
    return;
  }
  snprintf(buf, n, "%lld.%06lld", (long long)(micro / 1000000LL),
           (long long)(micro % 1000000LL));
}

/**
 * 同相位标记两次过点：测盘频 + 电机转数（应≈ GEAR_RATIO）。
 * 浮点值保留（供旧逻辑/遥测），减速比与频率另用整型定点计算，避免浮点误差。
 */
static void hallMeasureFullTurn(uint32_t now, uint32_t prev_ms, float prev_unwrap,
                                int64_t prev_counts, bool has_prev_counts,
                                float* out_hz_slot, float* gear_slot,
                                int64_t* out_hz_micro_slot, int64_t* gear_micro_slot,
                                uint8_t src) {
  if (prev_ms == 0 || isnan(prev_unwrap)) return;
  uint32_t dt_ms = now - prev_ms;
  // 1Hz→1000ms … 6Hz→167ms；放宽上下限防误触发/卡死
  if (dt_ms < 50 || dt_ms > 30000) return;
  float hz = 1000.0f / (float)dt_ms;
  float motor_revs = fabsf(motor_unwrapped_deg - prev_unwrap) / 360.0f;
  if (motor_revs < 0.5f) return;  // 一盘转至少半圈电机，滤抖
  *out_hz_slot = hz;
  *gear_slot = motor_revs;
  out_hz_meas = hz;
  out_rpm_meas = hz * 60.0f;
  gear_ratio_meas = motor_revs;
  gear_meas_src = src;
  // —— 定点整型：减速比 = Δcounts/16384，频率 = 1000/dt_ms，均 ×1e6 后整数除（四舍五入）——
  if (has_prev_counts && dt_ms > 0) {
    int64_t dcounts = motor_unwrapped_counts - prev_counts;
    if (dcounts < 0) dcounts = -dcounts;
    if (dcounts >= (ENC_CPR_I / 2)) {  // 至少半圈电机，与浮点滤抖一致
      int64_t gm_micro = (dcounts * 1000000LL + ENC_CPR_I / 2) / ENC_CPR_I;
      int64_t oh_micro = (1000000000LL + (int64_t)dt_ms / 2) / (int64_t)dt_ms;
      *gear_micro_slot = gm_micro;
      *out_hz_micro_slot = oh_micro;
      gear_micro_meas = gm_micro;
      out_hz_micro_meas = oh_micro;
    }
  }
}

void hallPoll(uint32_t now, float motor_rel, float motor_abs, uint16_t raw) {
  // 电平采样（诊断低占比）+ 软件边沿备份；主事件仍由 GPIO ISR 累计
  hall_dn_lvl = (uint8_t)gpio_get_level((gpio_num_t)PIN_HALL_DOWN);
  hall_up_lvl = (uint8_t)gpio_get_level((gpio_num_t)PIN_HALL_UP);
  hall_dn_samp_n++;
  hall_up_samp_n++;
  if (hall_dn_lvl == 0) hall_dn_low_n++;
  if (hall_up_lvl == 0) hall_up_low_n++;

  // 主循环边沿备份：进入有效电平记 poll_edge（宽脉冲时与 irq 对照；窄脉冲可能漏）
  if (!hallIsActive(hall_dn_prev) && hallIsActive(hall_dn_lvl)) {
    hall_dn_poll_edge++;
  }
  if (!hallIsActive(hall_up_prev) && hallIsActive(hall_up_lvl)) {
    hall_up_poll_edge++;
  }
  hall_dn_prev = hall_dn_lvl;
  hall_up_prev = hall_up_lvl;

  uint32_t dn_irq = hall_dn_irq_cnt;
  uint32_t up_irq = hall_up_irq_cnt;

  // —— 0° 下扑：两次过 0 点 = 输出盘一整转 ——
  while (hall_dn_irq_seen != dn_irq) {
    hall_dn_irq_seen++;
    const bool latch_ok = discLatchHallAllowed(0.0f);
    if (latch_ok && hall_dn_count >= 1) {
      hallMeasureFullTurn(now, last_hd_ms, last_hd_motor_unwrap,
                          last_hd_motor_unwrap_counts, has_hd_counts,
                          &out_hz_meas_dn, &gear_ratio_meas_dn,
                          &out_hz_micro_dn, &gear_micro_dn, 1);
    }
    if (!latch_ok) continue;  // 抖动沿：吃掉 IRQ，不重复 latch/测比
    hall_dn_count++;
    last_hd_ms = now;
    last_hd_motor_rel = motor_rel;
    last_hd_motor_abs = motor_abs;
    last_hd_raw = raw;
    last_hd_motor_unwrap = motor_unwrapped_deg;
    last_hd_motor_unwrap_counts = motor_unwrapped_counts;
    has_hd_counts = true;
    // GPIO4 = 物理 0°：绝对复位；此后磁编推算盘相位
    discLatchHall(0.0f);
    last_hd_disc = 0.0f;
    {
      char b_oh[24], b_g[24], b_d[24];
      fmtMicro(b_oh, sizeof(b_oh), out_hz_micro_dn);
      fmtMicro(b_g, sizeof(b_g), gear_micro_dn);
      fmtMicro(b_d, sizeof(b_d), GEAR_DESIGN_MICRO);
      hostPrintf(
          "# HALL DOWN t_ms=%lu motor_rel=%.3f raw=%u "
          "out_hz=%s gear_meas=%s design=%s src=0deg count=%lu irq_us=%lu "
          "latch_ref=0 motor_cnt=%lld\n",
          (unsigned long)now, (double)motor_rel, (unsigned)raw,
          b_oh, b_g, b_d, (unsigned long)hall_dn_count,
          (unsigned long)hall_dn_irq_us,
          (long long)disc_ref_motor_counts);
    }
  }

  // —— 180° 上举：两次过 180 点 = 输出盘一整转 ——
  while (hall_up_irq_seen != up_irq) {
    hall_up_irq_seen++;
    const bool latch_ok = discLatchHallAllowed(180.0f);
    if (latch_ok && hall_up_count >= 1) {
      hallMeasureFullTurn(now, last_hu_ms, last_hu_motor_unwrap,
                          last_hu_motor_unwrap_counts, has_hu_counts,
                          &out_hz_meas_up, &gear_ratio_meas_up,
                          &out_hz_micro_up, &gear_micro_up, 2);
    }
    if (!latch_ok) continue;
    hall_up_count++;
    last_hu_ms = now;
    last_hu_motor_rel = motor_rel;
    last_hu_motor_abs = motor_abs;
    last_hu_raw = raw;
    last_hu_motor_unwrap = motor_unwrapped_deg;
    last_hu_motor_unwrap_counts = motor_unwrapped_counts;
    has_hu_counts = true;
    // GPIO5 = 物理 180°：绝对复位（纠正仅靠 GPIO4 零点时的 ~180° 错位/漂移）
    float disc_before = discEstFromMotor();
    discLatchHall(180.0f);
    last_hu_disc = 180.0f;
    {
      char b_oh[24], b_g[24], b_d[24];
      fmtMicro(b_oh, sizeof(b_oh), out_hz_micro_up);
      fmtMicro(b_g, sizeof(b_g), gear_micro_up);
      fmtMicro(b_d, sizeof(b_d), GEAR_DESIGN_MICRO);
      hostPrintf(
          "# HALL UP t_ms=%lu motor_rel=%.3f raw=%u "
          "out_hz=%s gear_meas=%s design=%s src=180deg count=%lu irq_us=%lu "
          "latch_ref=180 disc_before=%.2f motor_cnt=%lld\n",
          (unsigned long)now, (double)motor_rel, (unsigned)raw,
          b_oh, b_g, b_d, (unsigned long)hall_up_count,
          (unsigned long)hall_up_irq_us,
          (double)(isnan(disc_before) ? -1.0f : disc_before),
          (long long)disc_ref_motor_counts);
    }
  }
}

void hallPrintStatus() {
  float disc = discEstFromMotor();
  float dn_low_pct =
      hall_dn_samp_n ? (100.0f * (float)hall_dn_low_n / (float)hall_dn_samp_n) : 0.0f;
  float up_low_pct =
      hall_up_samp_n ? (100.0f * (float)hall_up_low_n / (float)hall_up_samp_n) : 0.0f;
  hostPrintf(
      "# HALL dn_pin=%d up_pin=%d active=%s irq=%s isr_ok=%d dn_lvl=%u up_lvl=%u "
      "dn_cnt=%lu up_cnt=%lu irq_dn=%lu irq_up=%lu cal=%d gear=%d/%d=%.6f "
      "(12*14 / 59*79)\n",
      PIN_HALL_DOWN, PIN_HALL_UP, hall_active_low ? "LOW" : "HIGH",
      "ANYEDGE+SW", hall_gpio_isr_ok ? 1 : 0,
      (unsigned)hall_dn_lvl, (unsigned)hall_up_lvl,
      (unsigned long)hall_dn_count, (unsigned long)hall_up_count,
      (unsigned long)hall_dn_irq_cnt, (unsigned long)hall_up_irq_cnt,
      flap_calibrated ? 1 : 0, (int)GEAR_RATIO_NUM, (int)GEAR_RATIO_DEN,
      (double)GEAR_RATIO);
  hostPrintf(
      "# HALL SAMP dn_low%%=%.2f (%lu/%lu) up_low%%=%.2f (%lu/%lu) "
      "poll_dn=%lu poll_up=%lu (spinning: low%% should be >0 if DO reaches pad)\n",
      (double)dn_low_pct, (unsigned long)hall_dn_low_n, (unsigned long)hall_dn_samp_n,
      (double)up_low_pct, (unsigned long)hall_up_low_n, (unsigned long)hall_up_samp_n,
      (unsigned long)hall_dn_poll_edge, (unsigned long)hall_up_poll_edge);
  hostPrintf(
      "# HALL LATCH ref=%.1f motor_unwrap=%.3f counts=%lld esc_sense=%d | "
      "last_dn disc=%.3f | last_up disc=%.3f | disc_now=%.3f "
      "(model: disc=wrap(ref+sense*dmotor/gear); Halls reset, enc continuous)\n",
      (double)disc_ref_deg, (double)disc_ref_motor_unwrap,
      (long long)disc_ref_motor_counts, esc_sense,
      (double)(isnan(last_hd_disc) ? -1.0f : last_hd_disc),
      (double)(isnan(last_hu_disc) ? -1.0f : last_hu_disc),
      (double)(isnan(disc) ? -1.0f : disc));
  hostPrintf(
      "# HALL last_dn motor_rel=%.3f disc=%.3f | last_up motor_rel=%.3f disc=%.3f | "
      "disc_now=%.3f\n",
      (double)(isnan(last_hd_motor_rel) ? -1.0f : last_hd_motor_rel),
      (double)(isnan(last_hd_disc) ? -1.0f : last_hd_disc),
      (double)(isnan(last_hu_motor_rel) ? -1.0f : last_hu_motor_rel),
      (double)(isnan(last_hu_disc) ? -1.0f : last_hu_disc),
      (double)(isnan(disc) ? -1.0f : disc));
  // 定点误差（相对设计值）：ppm = (gear_micro - design)*1e6 / design（整数）
  long long err_ppm = -999999;
  if (gear_micro_meas > 0) {
    err_ppm = (long long)((gear_micro_meas - GEAR_DESIGN_MICRO) * 1000000LL /
                          GEAR_DESIGN_MICRO);
  }
  char m_oh[24], m_g[24], m_d[24], m_dnh[24], m_dni[24], m_uph[24], m_upi[24];
  fmtMicro(m_oh, sizeof(m_oh), out_hz_micro_meas);
  fmtMicro(m_g, sizeof(m_g), gear_micro_meas);
  fmtMicro(m_d, sizeof(m_d), GEAR_DESIGN_MICRO);
  fmtMicro(m_dnh, sizeof(m_dnh), out_hz_micro_dn);
  fmtMicro(m_dni, sizeof(m_dni), gear_micro_dn);
  fmtMicro(m_uph, sizeof(m_uph), out_hz_micro_up);
  fmtMicro(m_upi, sizeof(m_upi), gear_micro_up);
  hostPrintf(
      "# HALL MEAS out_hz=%s out_rpm=%.4f gear_meas=%s design=%s err_ppm=%lld "
      "src=%s dn_hz=%s dn_i=%s up_hz=%s up_i=%s\n",
      m_oh, (double)(isnan(out_rpm_meas) ? -1.0f : out_rpm_meas), m_g, m_d, err_ppm,
      gear_meas_src == 1 ? "0deg" : (gear_meas_src == 2 ? "180deg" : "?"),
      m_dnh, m_dni, m_uph, m_upi);
}

// ---------------- ESC PWM / DShot ----------------
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
  // 高频 PWM：周期可能 <2000μs（如 600Hz≈1667μs），必须夹断到周期内，否则占空比饱和
  if (esc_proto == ESC_PROTO_PWM && esc_period_us > 0) {
    uint16_t max_hi = ESC_PULSE_MAX_US;
    if (esc_period_us <= 50) {
      max_hi = (uint16_t)esc_period_us;
    } else if (esc_period_us < (uint32_t)ESC_PULSE_MAX_US + 50u) {
      max_hi = (uint16_t)(esc_period_us - 50u);  // 留一点低电平间隙
    }
    if (pulse_us > max_hi) pulse_us = max_hi;
  }
  esc_pulse_us = pulse_us;
  if (esc_proto == ESC_PROTO_DSHOT) {
    uint16_t thr = pulseUsToDshot(pulse_us, ESC_PULSE_MIN_US, ESC_PULSE_MAX_US);
    dshotSetThrottle(g_dshot, thr, false);
  } else {
    ledcWrite(PIN_ESC_PWM, pulseUsToDuty(pulse_us));
  }
}

bool setEscProtoPwm() {
  // 先停转再切
  if (esc_proto == ESC_PROTO_DSHOT) {
    dshotSetThrottle(g_dshot, 0, false);
    delay(20);
    dshotEnd(g_dshot);
  }
  esc_proto = ESC_PROTO_PWM;
  pinMode(PIN_ESC_PWM, OUTPUT);
  digitalWrite(PIN_ESC_PWM, LOW);
  esc_pwm_hz = ESC_PWM_HZ_DEFAULT;
  esc_period_us = 1000000UL / esc_pwm_hz;
  if (!ledcAttach(PIN_ESC_PWM, esc_pwm_hz, ESC_PWM_RES_BITS)) {
    return false;
  }
  applyEscPulse(ESC_PULSE_MIN_US);
  return true;
}

bool setEscProtoDshot(DshotRate rate) {
  // 卸 LEDC，上 RMT DShot
  applyEscPulse(ESC_PULSE_MIN_US);
  ledcDetach(PIN_ESC_PWM);
  pinMode(PIN_ESC_PWM, OUTPUT);
  digitalWrite(PIN_ESC_PWM, LOW);
  if (!dshotBegin(g_dshot, PIN_ESC_PWM, rate)) {
    // 失败回 PWM
    setEscProtoPwm();
    return false;
  }
  esc_proto = ESC_PROTO_DSHOT;
  dshotSetThrottle(g_dshot, 0, false);
  esc_pulse_us = ESC_PULSE_MIN_US;
  return true;
}

bool setEscPwmFreq(uint32_t hz) {
  if (esc_proto != ESC_PROTO_PWM) {
    return false;
  }
  if (hz < 50) hz = 50;
  if (hz > ESC_PWM_HZ_MAX) hz = ESC_PWM_HZ_MAX;
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
  esc_proto = ESC_PROTO_PWM;
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
  ka_us_per_rpms = prefs.getFloat("ka", 0.0f);
  if (ka_us_per_rpms < 0.0f) ka_us_per_rpms = 0.0f;
  if (ka_us_per_rpms > 2.0f) ka_us_per_rpms = 2.0f;
  adg_on = prefs.getBool("adg_on", false);
  adg_dual = prefs.getBool("adg_dual", false);
  adg_ki_decel_scale = prefs.getFloat("adg_kis", 0.35f);
  adg_kp_decel_scale = prefs.getFloat("adg_kps", 1.0f);
  adg_i_lim_decel = prefs.getFloat("adg_ild", 250.0f);
  adg_i_lim_accel = prefs.getFloat("adg_ila", 800.0f);
  adg_kp_accel = prefs.getFloat("adg_kpa", adg_kp_accel);
  adg_ki_accel = prefs.getFloat("adg_kia", adg_ki_accel);
  adg_kp_decel = prefs.getFloat("adg_kpd", adg_kp_decel);
  adg_ki_decel = prefs.getFloat("adg_kid", adg_ki_decel);
  phase_zero_deg = prefs.getFloat("ph0", 0.0f);
  esc_sense = prefs.getInt("esense", 1);
  if (esc_sense != 1 && esc_sense != -1) esc_sense = 1;
  holdph_lead_deg = prefs.getFloat("hplead", 10.0f);
  holdph_lead_ms = prefs.getFloat("hpleadt", 100.0f);
  holdph_lead_rpm_k = prefs.getFloat("hpleadk", 0.03f);
  disc_phase_offset_deg = prefs.getFloat("hphoff", 0.0f);
  disc_phase_offset_deg = wrap360(disc_phase_offset_deg);
  if (holdph_lead_deg < 1.0f) holdph_lead_deg = 1.0f;
  if (holdph_lead_deg > 60.0f) holdph_lead_deg = 60.0f;
  if (holdph_lead_ms < 0.0f) holdph_lead_ms = 0.0f;
  if (holdph_lead_ms > 500.0f) holdph_lead_ms = 500.0f;
  if (holdph_lead_rpm_k < 0.0f) holdph_lead_rpm_k = 0.0f;
  if (holdph_lead_rpm_k > 0.2f) holdph_lead_rpm_k = 0.2f;
}

// ---------------- 分区 PID（GainMap） ----------------
void gainMapClear(GainMap& g) {
  g.n = 0;
  g.valid = false;
}

GainMap& activeGainMap() { return gains[profile_id]; }

/** 按 |rpm| 线性插值分区 kp/ki/kd；无有效表时返回 false（调用者应回退全局 PID）。 */
bool interpGain(float rpm, float* kp_out, float* ki_out, float* kd_out) {
  GainMap& g = activeGainMap();
  if (!g.valid || g.n < 1) return false;
  float r = fabsf(rpm);
  if (g.n == 1 || r <= g.rpm[0]) {
    *kp_out = g.kp[0];
    *ki_out = g.ki[0];
    *kd_out = g.kd[0];
    return true;
  }
  if (r >= g.rpm[g.n - 1]) {
    *kp_out = g.kp[g.n - 1];
    *ki_out = g.ki[g.n - 1];
    *kd_out = g.kd[g.n - 1];
    return true;
  }
  for (int i = 0; i < g.n - 1; i++) {
    if (r >= g.rpm[i] && r <= g.rpm[i + 1]) {
      float den = g.rpm[i + 1] - g.rpm[i];
      float t = (den > 1e-3f) ? ((r - g.rpm[i]) / den) : 0.0f;
      *kp_out = g.kp[i] + t * (g.kp[i + 1] - g.kp[i]);
      *ki_out = g.ki[i] + t * (g.ki[i + 1] - g.ki[i]);
      *kd_out = g.kd[i] + t * (g.kd[i + 1] - g.kd[i]);
      return true;
    }
  }
  *kp_out = g.kp[g.n - 1];
  *ki_out = g.ki[g.n - 1];
  *kd_out = g.kd[g.n - 1];
  return true;
}

void saveGainMapToNvs(ProfileId id) {
  char key[16];
  snprintf(key, sizeof(key), "gmap%d", (int)id);
  prefs.putBytes(key, &gains[id], sizeof(GainMap));
}

void loadGainMapsFromNvs() {
  gainMapClear(gains[PROF_NOLOAD]);
  gainMapClear(gains[PROF_FLAP]);
  for (int id = 0; id < 2; id++) {
    char key[16];
    snprintf(key, sizeof(key), "gmap%d", id);
    GainMap tmp;
    size_t n = prefs.getBytes(key, &tmp, sizeof(tmp));
    if (n == sizeof(tmp) && tmp.n >= 1 && tmp.n <= GAIN_MAX_POINTS) {
      gains[id] = tmp;
      gains[id].valid = true;
    }
  }
  gain_schedule_on = prefs.getBool("gainon", true);
}

void dumpGainMap() {
  GainMap& g = activeGainMap();
  hostPrintf("# GAIN BEGIN profile=%s points=%d valid=%d schedule=%s\n",
                profileName(), g.n, g.valid ? 1 : 0, gain_schedule_on ? "ON" : "OFF");
  for (int i = 0; i < g.n; ++i) {
    hostPrintf("# GAIN point rpm=%.0f kp=%.4f ki=%.4f kd=%.4f\n",
                  (double)g.rpm[i], (double)g.kp[i], (double)g.ki[i], (double)g.kd[i]);
  }
  hostPrintln("# GAIN END");
}

void savePhaseZeroToNvs() {
  prefs.putFloat("ph0", phase_zero_deg);
}

void saveHoldPhaseLeadToNvs() {
  prefs.putFloat("hplead", holdph_lead_deg);
  prefs.putFloat("hpleadt", holdph_lead_ms);
  prefs.putFloat("hpleadk", holdph_lead_rpm_k);
  prefs.putFloat("hphoff", disc_phase_offset_deg);
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

/** CTRL?/FLIGHT?/HOST? 共用状态串（配置值；FLIGHT/TEST 豁免见 controlTick）。 */
static const char* hostTimeoutStatusStr() {
  if (flight_mode) return "disabled(flight)";
  if (test_active) return "disabled(test)";
  if (host_timeout_ms == 0) return "disabled(HOST OFF)";
  static char buf[24];
  snprintf(buf, sizeof(buf), "%.3fs", (double)host_timeout_ms / 1000.0);
  return buf;
}

// ---------------- 板内 TEST 状态机（判定写死；打印仅状态切换，走 async log）----------------
static float testMedianInplace(float* a, uint8_t n) {
  if (n == 0) return 0.0f;
  // 插入排序（n≤64）
  for (uint8_t i = 1; i < n; ++i) {
    float v = a[i];
    int j = (int)i - 1;
    while (j >= 0 && a[j] > v) {
      a[j + 1] = a[j];
      --j;
    }
    a[j + 1] = v;
  }
  if (n & 1) return a[n / 2];
  return 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

static void testSpinReset() {
  test_spin_n = 0;
  test_spin_above = 0;
  test_spin_last_ms = 0;
  test_hall0_dn = hall_dn_irq_cnt;
  test_hall0_up = hall_up_irq_cnt;
}

static void testHoldReset() {
  test_hold_sum = 0.0f;
  test_hold_err_sum = 0.0f;
  test_hold_n = 0;
  test_pulse_peak = esc_pulse_us;
  test_pulse_sat = false;
}

static void testNotePulse() {
  if (esc_pulse_us > test_pulse_peak) test_pulse_peak = esc_pulse_us;
  if (esc_pulse_us >= (ESC_PULSE_MAX_US - 2)) test_pulse_sat = true;
}

static float mapRpmMaxOf(const ThrottleMap& m) {
  if (!m.valid || m.n < 1) return 0.0f;
  return m.rpm[m.n - 1];
}

/** 低优路径打印：TUNE 摘要（mean_err/u/ff/pulse + 一句建议）。 */
static void tuneEmitSummary(const char* tag) {
  const TuneStats& t = tuneStats();
  char hint[96];
  int code = tuneHint(hint, sizeof(hint));
  if (t.n < 1) {
    hostPrintf("# %s n=0 (no closed samples; TUNE REC ON or run CLOSED/TEST)\n",
               tag ? tag : "TUNE");
    return;
  }
  float inv = 1.0f / (float)t.n;
  hostPrintf(
      "# %s n=%lu mean_err=%.1f mean_u=%.1f mean_ff=%.0f mean_pulse=%.0f "
      "mean_rpm=%.0f mean_tgt=%.0f i_sat=%lu/%lu map_max=%.0f hint=%s code=%d\n",
      tag ? tag : "TUNE", (unsigned long)t.n, (double)(t.sum_err * inv),
      (double)(t.sum_u * inv), (double)(t.sum_ff * inv), (double)(t.sum_pulse * inv),
      (double)(t.sum_rpm * inv), (double)(t.sum_tgt * inv), (unsigned long)t.i_sat_n,
      (unsigned long)t.n, (double)t.map_rpm_max, hint, code);
}

static void testEmitDone() {
  hostPrintf(
      "# TEST DONE name=%s ok=%d enc_rpm=%.0f target=%.0f pulse=%u err=%.0f "
      "hall_d=%d sat=%d reason=%s\n",
      test_name, test_ok ? 1 : 0, (double)test_enc_rpm, (double)test_set_rpm,
      (unsigned)test_pulse_peak, (double)test_err_mean, test_hall_d,
      test_pulse_sat ? 1 : 0, test_reason);
  tuneEmitSummary("TEST LOG");
  if (test_ok) {
    hostPrintf("# TEST OK name=%s enc_rpm=%.0f\n", test_name, (double)test_enc_rpm);
  } else {
    hostPrintf("# TEST FAIL name=%s reason=%s\n", test_name, test_reason);
  }
}

static void testFinish(bool ok, const char* reason) {
  if (!test_active && !test_finish_pending) return;
  test_ok = ok;
  strncpy(test_reason, reason, sizeof(test_reason) - 1);
  test_reason[sizeof(test_reason) - 1] = '\0';
  test_active = false;
  test_finish_pending = false;
  test_phase = TEST_PH_DONE;
  // 停油（不走 doStop 以免多余 ACK 淹没摘要；仍清运行态）
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  clearPid();
  open_pwm_hold = false;
  if (run_state == RUN_RUNNING) run_state = RUN_IDLE;
  applyEscPulse(ESC_PULSE_MIN_US);
  testEmitDone();
  if (tune_owned_by_test) {
    tuneRecSet(false);
    tune_owned_by_test = false;
  }
}

static void testAbortQuiet() {
  // ESTOP 已停油：只收尾 TEST 摘要
  if (!test_active) return;
  test_ok = false;
  strncpy(test_reason, "estop", sizeof(test_reason) - 1);
  test_reason[sizeof(test_reason) - 1] = '\0';
  test_active = false;
  test_phase = TEST_PH_DONE;
  test_enc_rpm = fabsf(rpm_signed_stable);
  test_err_mean = fabsf(test_enc_rpm - test_set_rpm);
  test_hall_d = (int)((hall_dn_irq_cnt - test_hall0_dn) + (hall_up_irq_cnt - test_hall0_up));
  test_pulse_peak = esc_pulse_us;
  testEmitDone();
  if (tune_owned_by_test) {
    tuneRecSet(false);
    tune_owned_by_test = false;
  }
}

static bool testEvalSpin(float* mean_out, float* med_out, int* hall_d_out, bool* hall_ok_out) {
  uint8_t n = test_spin_n;
  float mean = 0.0f;
  if (n > 0) {
    float s = 0.0f;
    for (uint8_t i = 0; i < n; ++i) s += test_spin_buf[i];
    mean = s / (float)n;
  }
  float tmp[TEST_SPIN_MAX_SAMP];
  for (uint8_t i = 0; i < n; ++i) tmp[i] = test_spin_buf[i];
  float med = testMedianInplace(tmp, n);
  float level = (med > 1.0f) ? med : mean;
  float frac = (n > 0) ? ((float)test_spin_above / (float)n) : 0.0f;
  bool enc_ok = (n >= 10) && (level > TEST_RPM_SPIN_MIN) && (frac >= TEST_SPIN_FRAC_MIN);
  int ddn = (int)(hall_dn_irq_cnt - test_hall0_dn);
  int dup = (int)(hall_up_irq_cnt - test_hall0_up);
  int hd = ddn + dup;
  bool hall_ok = (hd >= 1);
  if (mean_out) *mean_out = mean;
  if (med_out) *med_out = med;
  if (hall_d_out) *hall_d_out = hd;
  if (hall_ok_out) *hall_ok_out = hall_ok;
  return enc_ok;
}

static void testArmClosed(float rpm) {
  if (run_state == RUN_ESTOP) run_state = RUN_IDLE;
  ctrl_mode = MODE_CLOSED;
  soft_enable = true;
  soft_rate_up_rpm_s = TEST_SOFT_RATE_UP;
  soft_rate_down_rpm_s = TEST_SOFT_RATE_DOWN;
  soft_rate_rpm_s = TEST_SOFT_RATE_UP;
  target_rpm_max = TEST_RPMMAX;
  open_pwm_hold = false;
  learn_active = false;
  measure_active = false;
  target_cmd = rpm;
  test_set_rpm = rpm;
  if (!soft_enable) target_ramped = target_cmd;
  clearPid();
  run_state = RUN_RUNNING;
  last_host_ms = millis();
  host_seen = true;
  hostPrintf("# TEST ARM name=%s rpm=%.0f mode=CLOSED soft_up=%.0f\n",
             test_name, (double)rpm, (double)TEST_SOFT_RATE_UP);
}

static bool testStartByName(const char* name) {
  if (test_active) {
    hostPrintln("# ERR TEST busy; TEST STOP first");
    return false;
  }
  TestKind k = TEST_KIND_NONE;
  if (strcasecmp(name, "smoke2000") == 0) k = TEST_KIND_SMOKE2000;
  else if (strcasecmp(name, "step7500") == 0) k = TEST_KIND_STEP7500;
  else if (strcasecmp(name, "idle") == 0) k = TEST_KIND_IDLE;
  else {
    hostPrintln("# ERR TEST START smoke2000|step7500|idle");
    return false;
  }
  test_kind = k;
  strncpy(test_name, name, sizeof(test_name) - 1);
  test_name[sizeof(test_name) - 1] = '\0';
  test_reason[0] = '\0';
  test_ok = false;
  test_enc_rpm = 0.0f;
  test_err_mean = 0.0f;
  test_hall_d = 0;
  test_pulse_peak = 0;
  test_pulse_sat = false;
  test_finish_pending = false;
  test_active = true;
  test_phase_t0 = millis();
  // TEST 段内自动 RAM 记录（不依赖主机保活）；结束打 # TEST LOG
  tuneSetMapRpmMax(mapRpmMaxOf(activeMap()));
  if (!tuneRecOn()) {
    tuneRecSet(true);
    tune_owned_by_test = true;
  } else {
    tuneRecReset();
    tune_owned_by_test = false;
  }
  if (k == TEST_KIND_IDLE) {
    test_set_rpm = 0.0f;
    test_err_frac = 0.0f;
    test_hold_ms = TEST_IDLE_WINDOW_MS;
    testHoldReset();
    test_phase = TEST_PH_HOLD;  // 直接采静止窗
    hostPrintf("# TEST START name=idle hold_ms=%lu\n", (unsigned long)TEST_IDLE_WINDOW_MS);
  } else if (k == TEST_KIND_SMOKE2000) {
    test_err_frac = TEST_SMOKE_ERR_FRAC;
    test_hold_ms = TEST_SMOKE_HOLD_MS;
    test_phase = TEST_PH_ARM;
    hostPrintf("# TEST START name=smoke2000 target=%.0f hold_ms=%lu\n",
               (double)TEST_SMOKE_RPM, (unsigned long)TEST_SMOKE_HOLD_MS);
  } else {
    test_err_frac = TEST_STEP_ERR_FRAC;
    test_hold_ms = TEST_STEP_HOLD_MS;
    test_phase = TEST_PH_ARM;
    hostPrintf("# TEST START name=step7500 gate=%.0f then=%.0f hold_ms=%lu\n",
               (double)TEST_SMOKE_RPM, (double)TEST_STEP_RPM, (unsigned long)TEST_STEP_HOLD_MS);
  }
  return true;
}

static void testStopCmd() {
  if (!test_active) {
    hostPrintln("# ACK TEST STOP (idle)");
    return;
  }
  test_enc_rpm = fabsf(rpm_signed_stable);
  test_err_mean = fabsf(test_enc_rpm - test_set_rpm);
  test_hall_d = (int)((hall_dn_irq_cnt - test_hall0_dn) + (hall_up_irq_cnt - test_hall0_up));
  test_pulse_peak = esc_pulse_us;
  testFinish(false, "host_stop");
}

static void testStatusCmd() {
  const char* ph = "?";
  switch (test_phase) {
    case TEST_PH_IDLE: ph = "idle"; break;
    case TEST_PH_ARM: ph = "arm"; break;
    case TEST_PH_RAMP: ph = "ramp"; break;
    case TEST_PH_SPIN: ph = "spin"; break;
    case TEST_PH_HOLD: ph = "hold"; break;
    case TEST_PH_STEP_RAMP: ph = "step_ramp"; break;
    case TEST_PH_STEP_SPIN: ph = "step_spin"; break;
    case TEST_PH_STEP_HOLD: ph = "step_hold"; break;
    case TEST_PH_STOPPING: ph = "stopping"; break;
    case TEST_PH_DONE: ph = "done"; break;
  }
  hostPrintf(
      "# TEST active=%d name=%s phase=%s target=%.0f enc=%.0f pulse=%u ok=%d reason=%s\n",
      test_active ? 1 : 0, test_name[0] ? test_name : "-", ph, (double)test_set_rpm,
      (double)fabsf(rpm_signed_stable), (unsigned)esc_pulse_us, test_ok ? 1 : 0,
      test_reason[0] ? test_reason : "-");
}

/** 400Hz 控制任务内调用：状态切换才 hostPrintf（异步环）。 */
static void testTick(float rpm_meas) {
  if (!test_active) return;
  uint32_t now = millis();
  float rpm = fabsf(rpm_meas);
  testNotePulse();

  switch (test_phase) {
    case TEST_PH_ARM:
      if (test_kind == TEST_KIND_SMOKE2000 || test_kind == TEST_KIND_STEP7500) {
        testArmClosed(TEST_SMOKE_RPM);
        test_phase = TEST_PH_RAMP;
        test_phase_t0 = now;
      }
      break;

    case TEST_PH_RAMP:
      if (fabsf(target_ramped - test_set_rpm) <= TEST_RAMP_NEAR_RPM ||
          (now - test_phase_t0) >= TEST_RAMP_TIMEOUT_MS) {
        testSpinReset();
        test_phase = TEST_PH_SPIN;
        test_phase_t0 = now;
        hostPrintf("# TEST SPIN begin target=%.0f\n", (double)test_set_rpm);
      }
      break;

    case TEST_PH_SPIN:
    case TEST_PH_STEP_SPIN: {
      if (test_spin_last_ms == 0 || (now - test_spin_last_ms) >= TEST_SPIN_SAMPLE_MS) {
        test_spin_last_ms = now;
        if (test_spin_n < TEST_SPIN_MAX_SAMP) {
          test_spin_buf[test_spin_n++] = rpm;
          if (rpm > TEST_RPM_SPIN_MIN) test_spin_above++;
        }
      }
      if ((now - test_phase_t0) < TEST_SPIN_WINDOW_MS) break;
      float mean = 0.0f, med = 0.0f;
      int hd = 0;
      bool hall_ok = false;
      bool enc_ok = testEvalSpin(&mean, &med, &hd, &hall_ok);
      test_enc_rpm = (med > 1.0f) ? med : mean;
      test_hall_d = hd;
      test_err_mean = fabsf(test_enc_rpm - test_set_rpm);
      hostPrintf("# TEST SPIN enc_ok=%d hall_ok=%d enc=%.0f hall_d=%d\n",
                 enc_ok ? 1 : 0, hall_ok ? 1 : 0, (double)test_enc_rpm, hd);
      if (!enc_ok) {
        testFinish(false, "no_spin");
        break;
      }
      if (test_phase == TEST_PH_SPIN && test_kind == TEST_KIND_STEP7500) {
        // smoke 门闩通过 → 升 7500
        target_cmd = TEST_STEP_RPM;
        test_set_rpm = TEST_STEP_RPM;
        test_phase = TEST_PH_STEP_RAMP;
        test_phase_t0 = now;
        hostPrintf("# TEST GATE ok -> step %.0f\n", (double)TEST_STEP_RPM);
      } else if (test_phase == TEST_PH_STEP_SPIN) {
        testHoldReset();
        test_phase = TEST_PH_STEP_HOLD;
        test_phase_t0 = now;
        hostPrintf("# TEST HOLD begin target=%.0f ms=%lu\n",
                   (double)test_set_rpm, (unsigned long)test_hold_ms);
      } else {
        // smoke2000：进入保持
        testHoldReset();
        test_phase = TEST_PH_HOLD;
        test_phase_t0 = now;
        hostPrintf("# TEST HOLD begin target=%.0f ms=%lu\n",
                   (double)test_set_rpm, (unsigned long)test_hold_ms);
      }
      break;
    }

    case TEST_PH_STEP_RAMP:
      if (fabsf(target_ramped - test_set_rpm) <= TEST_RAMP_NEAR_RPM ||
          (now - test_phase_t0) >= TEST_RAMP_TIMEOUT_MS) {
        testSpinReset();
        test_phase = TEST_PH_STEP_SPIN;
        test_phase_t0 = now;
        hostPrintf("# TEST SPIN begin target=%.0f\n", (double)test_set_rpm);
      }
      break;

    case TEST_PH_HOLD:
    case TEST_PH_STEP_HOLD: {
      if (test_kind == TEST_KIND_IDLE) {
        // 静止确认：不给油
        if (run_state == RUN_RUNNING) {
          target_cmd = 0.0f;
          target_ramped = 0.0f;
          applyEscPulse(ESC_PULSE_MIN_US);
          run_state = RUN_IDLE;
        }
      }
      test_hold_sum += rpm;
      test_hold_err_sum += fabsf(rpm - test_set_rpm);
      test_hold_n++;
      if ((now - test_phase_t0) < test_hold_ms) break;
      float mean = (test_hold_n > 0) ? (test_hold_sum / (float)test_hold_n) : 0.0f;
      float errm = (test_hold_n > 0) ? (test_hold_err_sum / (float)test_hold_n) : 0.0f;
      test_enc_rpm = mean;
      test_err_mean = errm;
      test_hall_d = (int)((hall_dn_irq_cnt - test_hall0_dn) + (hall_up_irq_cnt - test_hall0_up));
      if (test_kind == TEST_KIND_IDLE) {
        bool ok = (mean <= TEST_IDLE_RPM_MAX);
        testFinish(ok, ok ? "idle_ok" : "not_idle");
        break;
      }
      float lim = test_set_rpm * test_err_frac;
      if (lim < 150.0f) lim = 150.0f;
      bool ok = (fabsf(mean - test_set_rpm) <= lim) || (errm <= lim);
      // pulse 饱和仅记入 sat=，不单独 FAIL
      if (run_state == RUN_ESTOP) ok = false;
      testFinish(ok, ok ? "hold_ok" : "hold_err");
      break;
    }

    case TEST_PH_STOPPING:
    case TEST_PH_DONE:
    case TEST_PH_IDLE:
    default:
      break;
  }
}

void doEstop(const char* reason) {
  run_state = RUN_ESTOP;
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  clearPid();
  learn_active = false;
  measure_active = false;
  open_pwm_hold = false;
  move_active = false;
  holdph_active = false;
  sense_learn = false;
  esccal_high = false;
  if (ctrl_mode == MODE_LEARN || ctrl_mode == MODE_MEASURE) ctrl_mode = MODE_OPEN;
  applyEscPulse(ESC_PULSE_MIN_US);
  if (test_active) {
    test_finish_pending = true;
    testAbortQuiet();
  }
  hostPrintf("# ESTOP %s pulse=%u\n", reason, (unsigned)esc_pulse_us);
}

void doStop(bool immediate) {
  if (test_active) {
    test_enc_rpm = fabsf(rpm_signed_stable);
    test_err_mean = fabsf(test_enc_rpm - test_set_rpm);
    test_hall_d = (int)((hall_dn_irq_cnt - test_hall0_dn) + (hall_up_irq_cnt - test_hall0_up));
    test_pulse_peak = esc_pulse_us;
    testFinish(false, "host_stop");
    return;
  }
  target_cmd = 0.0f;
  if (immediate || !soft_enable) {
    target_ramped = 0.0f;
    applyEscPulse(ESC_PULSE_MIN_US);
  }
  clearPid();
  run_state = RUN_IDLE;
  learn_active = false;
  measure_active = false;
  open_pwm_hold = false;
  move_active = false;
  holdph_active = false;
  sense_learn = false;
  if (ctrl_mode == MODE_LEARN || ctrl_mode == MODE_MEASURE) ctrl_mode = MODE_OPEN;
  hostPrintf("# ACK STOP soft=%d pulse=%u\n", soft_enable && !immediate, (unsigned)esc_pulse_us);
}

void doStart() {
  if (run_state == RUN_ESTOP) {
    hostPrintln("# ERR clear ESTOP first with STOP");
    return;
  }
  if (ctrl_mode == MODE_SAFE) {
    hostPrintln("# ERR MODE SAFE; set MODE OPEN|CLOSED");
    return;
  }
  if (ctrl_mode == MODE_LEARN || ctrl_mode == MODE_MEASURE) {
    hostPrintln("# ERR cannot START in LEARN/MEASURE");
    return;
  }
  run_state = RUN_RUNNING;
  if (!soft_enable) target_ramped = target_cmd;
  clearPid();
  last_host_ms = millis();
  host_seen = true;
  hostPrintf("# ACK START target=%.1f mode=%s\n", (double)target_cmd, modeName());
}

void updateSoftRamp(float dt) {
  if (!soft_enable) {
    target_ramped = target_cmd;
    soft_cmd_accel_rpm_s = 0.0f;
    return;
  }
  float err = target_cmd - target_ramped;
  // 升/降用不同斜率：空载惯量下减速常需更慢的指令斜坡（备忘 S1）
  float rate = (err >= 0.0f) ? soft_rate_up_rpm_s : soft_rate_down_rpm_s;
  float max_step = rate * dt;
  if (err > max_step) {
    target_ramped += max_step;
    soft_cmd_accel_rpm_s = rate;  // 正在加速
  } else if (err < -max_step) {
    target_ramped -= max_step;
    soft_cmd_accel_rpm_s = -rate;  // 正在减速（负加速度）
  } else {
    target_ramped = target_cmd;
    soft_cmd_accel_rpm_s = 0.0f;
  }
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

  // 分区 PID：按目标转速插值 kp/ki/kd；关闭或无有效表时回退全局 kp/ki/kd
  float kp_use = kp, ki_use = ki, kd_use = kd;
  if (gain_schedule_on && activeGainMap().valid) {
    float gkp, gki, gkd;
    if (interpGain(rpm_tgt, &gkp, &gki, &gkd)) {
      kp_use = gkp;
      ki_use = gki;
      kd_use = gkd;
    }
  }

  // S3：按指令加/减速调整增益与积分限幅（用 soft_cmd_accel，非噪声实测 α）
  float i_lim = adg_i_lim_accel;
  const bool decelerating = (soft_cmd_accel_rpm_s < -1.0f);
  const bool accelerating = (soft_cmd_accel_rpm_s > 1.0f);
  if (adg_on) {
    if (adg_dual) {
      if (decelerating) {
        kp_use = adg_kp_decel;
        ki_use = adg_ki_decel;
        i_lim = adg_i_lim_decel;
      } else if (accelerating) {
        kp_use = adg_kp_accel;
        ki_use = adg_ki_accel;
        i_lim = adg_i_lim_accel;
      }
    } else if (decelerating) {
      kp_use *= adg_kp_decel_scale;
      ki_use *= adg_ki_decel_scale;
      i_lim = adg_i_lim_decel;
    }
  } else {
    // ADG OFF：硬限；7500 实测 u≈60≈ki*800+kp*err 触顶（次因）。治本仍是扩展前馈。
    i_lim = 1200.0f;
  }
  if (i_lim < 50.0f) i_lim = 50.0f;
  if (i_lim > 1200.0f) i_lim = 1200.0f;

  pid_i += err * dt;
  if (pid_i > i_lim) pid_i = i_lim;
  if (pid_i < -i_lim) pid_i = -i_lim;
  float derr = (dt > 1e-4f) ? ((err - pid_prev_err) / dt) : 0.0f;
  pid_prev_err = err;
  float u = kp_use * err + ki_use * pid_i + kd_use * derr;
  // S2 加速度前馈：用指令斜坡加速度（非实测 α），单位 ka=us/(rpm/s)
  float uff = ka_us_per_rpms * soft_cmd_accel_rpm_s;
  float u_tot = u + uff;
  long pulse = (long)ff + (long)(u_tot + ((u_tot >= 0) ? 0.5f : -0.5f));
  if (pulse < ESC_PULSE_MIN_US) pulse = ESC_PULSE_MIN_US;
  if (pulse > ESC_PULSE_MAX_US) pulse = ESC_PULSE_MAX_US;
  uint16_t pout = (uint16_t)pulse;
  g_closed_err = err;
  g_closed_ff = (float)ff;
  g_closed_u = u_tot;
  // RAM 记录：不依赖主机保活；实时路径禁打印
  if (tuneRecOn()) {
    tuneRecPush(err, (float)ff, u_tot, pout, meas, rpm_tgt, pid_i, i_lim);
  }
  return pout;
}

void learnBegin() {
  if (run_state == RUN_RUNNING) {
    hostPrintln("# ERR STOP before LEARN");
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
  learn_mode_ramp = false;
  learn_pulse = ESC_PULSE_MIN_US;
  learn_idx = 0;
  learn_step_t0 = millis();
  mapClear(activeMap());
  activeMap().pulse[0] = ESC_PULSE_MIN_US;
  activeMap().rpm[0] = 0.0f;
  activeMap().n = 1;
  applyEscPulse(ESC_PULSE_MIN_US);
  float rpm_stop = target_rpm_max * LEARN_RPM_STOP_RATIO;
  hostPrintf(
      "# ACK LEARN START profile=%s max_us=%u rpm_stop=%.0f rpm_limit=%.0f step=%uus settle=%lums\n",
      profileName(), (unsigned)learn_max_us, (double)rpm_stop, (double)target_rpm_max,
      (unsigned)LEARN_STEP_US, (unsigned long)LEARN_SETTLE_MS);
  hostPrintln("# Learn stops only when: RPM>=rpm_stop OR pulse>=2000 OR map full");
  hostPrintf("# LEARN LOG begin t_ms=%lu pulse=%u\n",
                (unsigned long)millis(), (unsigned)learn_pulse);
}

/** LEARN RAMP：开环脉宽连续斜坡上升（非台阶），比 LEARN START 更快扫完全程。 */
void learnBeginRamp() {
  if (run_state == RUN_RUNNING) {
    hostPrintln("# ERR STOP before LEARN");
    return;
  }
  learn_max_us = ESC_PULSE_MAX_US;
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  clearPid();
  run_state = RUN_IDLE;
  ctrl_mode = MODE_LEARN;
  learn_active = true;
  learn_mode_ramp = true;
  learn_pulse = ESC_PULSE_MIN_US;
  learn_ramp_pulse_f = (float)ESC_PULSE_MIN_US;
  learn_ramp_last_sample_ms = millis();
  learn_ramp_last_pulse = ESC_PULSE_MIN_US;
  learn_idx = 0;
  learn_step_t0 = millis();
  mapClear(activeMap());
  activeMap().pulse[0] = ESC_PULSE_MIN_US;
  activeMap().rpm[0] = 0.0f;
  activeMap().n = 1;
  applyEscPulse(ESC_PULSE_MIN_US);
  float rpm_stop = target_rpm_max * LEARN_RPM_STOP_RATIO;
  hostPrintf(
      "# ACK LEARN RAMP profile=%s max_us=%u rpm_stop=%.0f rpm_limit=%.0f rate=%.0fus/s\n",
      profileName(), (unsigned)learn_max_us, (double)rpm_stop, (double)target_rpm_max,
      (double)RAMP_US_PER_S);
  hostPrintln("# Learn stops only when: RPM>=rpm_stop OR pulse>=2000 OR map full");
  hostPrintf("# LEARN LOG begin t_ms=%lu pulse=%u\n",
                (unsigned long)millis(), (unsigned)learn_pulse);
}

void learnAbort() {
  learn_active = false;
  learn_mode_ramp = false;
  ctrl_mode = MODE_OPEN;
  applyEscPulse(ESC_PULSE_MIN_US);
  target_cmd = 0;
  target_ramped = 0;
  run_state = RUN_IDLE;
  hostPrintln("# LEARN ABORT");
}

void suggestPidFromMap() {
  ThrottleMap& m = activeMap();
  if (!m.valid || m.n < 2) {
    hostPrintln("# SUGGEST PID need >=2 points");
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
  hostPrintf("# SUGGEST gain=%.3f rpm/us  PID kp=%.4f ki=%.4f kd=0\n",
                (double)gain, (double)kp_s, (double)ki_s);
  hostPrintf("# HINT apply: PID %.4f %.4f 0\n", (double)kp_s, (double)ki_s);
}

/**
 * 由前馈图逐段局部斜率生成分区 PID 表（GainMap），类似 EFI 分区 MAP。
 * 每段 Δpulse>0 且 Δrpm>0 时：gain=drpm/dpulse，kp=clamp(0.35/gain,0.02,0.4)，
 * ki=clamp(kp*2.5,0.05,1.5)，断点=段中点转速。另加 rpm=0 柔和增益作首点。
 */
void suggestPidZonesFromMap() {
  ThrottleMap& m = activeMap();
  GainMap& g = activeGainMap();
  if (!m.valid || m.n < 2) {
    hostPrintln("# SUGGEST GAINMAP need >=2 map points");
    return;
  }
  gainMapClear(g);
  g.rpm[0] = 0.0f;
  g.kp[0] = 0.05f;
  g.ki[0] = 0.10f;
  g.kd[0] = 0.0f;
  g.n = 1;

  for (int i = 0; i < m.n - 1 && g.n < GAIN_MAX_POINTS; ++i) {
    float dpulse = (float)(m.pulse[i + 1] - m.pulse[i]);
    float drpm = m.rpm[i + 1] - m.rpm[i];
    if (dpulse <= 0.0f || drpm <= 1.0f) continue;  // 跳过无效/倒退段
    float gain = drpm / dpulse;
    if (gain < 0.05f) gain = 0.05f;
    float kp_s = 0.35f / gain;
    float ki_s = kp_s * 2.5f;
    if (kp_s < 0.02f) kp_s = 0.02f;
    if (kp_s > 0.4f) kp_s = 0.4f;
    if (ki_s < 0.05f) ki_s = 0.05f;
    if (ki_s > 1.5f) ki_s = 1.5f;
    float rpm_bp = 0.5f * (m.rpm[i] + m.rpm[i + 1]);  // 断点=段中点转速
    int idx = g.n++;
    g.rpm[idx] = rpm_bp;
    g.kp[idx] = kp_s;
    g.ki[idx] = ki_s;
    g.kd[idx] = 0.0f;
  }

  if (g.n < 2) {
    hostPrintln("# SUGGEST GAINMAP no valid rising segments in map");
    gainMapClear(g);
    return;
  }
  g.valid = true;
  saveGainMapToNvs(profile_id);
  gain_schedule_on = true;
  prefs.putBool("gainon", true);
  hostPrintf("# SUGGEST GAINMAP n=%d profile=%s\n", g.n, profileName());
  for (int i = 0; i < g.n; ++i) {
    hostPrintf("# GAIN point rpm=%.0f kp=%.4f ki=%.4f kd=0\n",
                  (double)g.rpm[i], (double)g.kp[i], (double)g.ki[i]);
  }
  hostPrintln("# HINT apply: GAIN ON (auto-enabled)");
}

void dumpActiveMap() {
  ThrottleMap& m = activeMap();
  hostPrintf("# MAP BEGIN profile=%s points=%d valid=%d\n",
                profileName(), m.n, m.valid ? 1 : 0);
  for (int i = 0; i < m.n; ++i) {
    hostPrintf("# MAP point i=%d pulse=%u rpm=%.1f\n",
                  i, (unsigned)m.pulse[i], (double)m.rpm[i]);
  }
  hostPrintln("# MAP END");
}

void measureBegin() {
  if (run_state == RUN_RUNNING) {
    hostPrintln("# ERR STOP before MEASURE");
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
  hostPrintln("# MEASURE START — use PWM / MEASURE +50|-50 / HOLD / AUTO / SAVE");
  hostPrintln("# OBSERVE: increase pulse, watch rpm on telemetry");
}

void measureAbort() {
  measure_active = false;
  learn_active = false;
  open_pwm_hold = false;
  ctrl_mode = MODE_OPEN;
  measure_pulse = ESC_PULSE_MIN_US;
  applyEscPulse(ESC_PULSE_MIN_US);
  hostPrintln("# MEASURE ABORT");
}

void measureSetPulse(long us) {
  if (us < ESC_PULSE_MIN_US) us = ESC_PULSE_MIN_US;
  if (us > ESC_PULSE_MAX_US) us = ESC_PULSE_MAX_US;
  measure_pulse = (uint16_t)us;
  // 学习中只由 learnTick 出油门，避免与手动 PWM 打架
  if (ctrl_mode == MODE_MEASURE && !learn_active) {
    applyEscPulse(measure_pulse);
  }
  hostPrintf("# MEASURE pulse=%u\n", (unsigned)measure_pulse);
}

void measureHold(float rpm_meas) {
  ThrottleMap& m = activeMap();
  if (m.n >= MAP_MAX_POINTS) {
    hostPrintln("# ERR map full");
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
  hostPrintf("# HOLD pulse=%u rpm=%.1f points=%d\n",
                (unsigned)measure_pulse, (double)rpm_meas, m.n);
}

void measureSave() {
  ThrottleMap& m = activeMap();
  m.valid = (m.n >= 2);
  if (!m.valid) {
    hostPrintln("# ERR MEASURE SAVE need >=2 HOLD points");
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
  hostPrintf("# MEASURE SAVE profile=%s points=%d -> MODE CLOSED\n",
                profileName(), m.n);
}

void measureClear() {
  mapClear(activeMap());
  hostPrintln("# MEASURE CLEAR");
}

void escCalHigh() {
  // 标准油门行程校准：先输出最高油门，再给电调上电
  esccal_high = true;
  measure_active = false;
  learn_active = false;
  run_state = RUN_IDLE;
  ctrl_mode = MODE_SAFE;
  applyEscPulse(ESC_PULSE_MAX_US);
  hostPrintf("# ACK ESCCAL HIGH pulse=%u — hold max; power ESC battery now\n",
                (unsigned)esc_pulse_us);
  hostPrintln("# Next: after beeps, send ESCCAL LOW");
}

void escCalLow() {
  esccal_high = false;
  applyEscPulse(ESC_PULSE_MIN_US);
  hostPrintf("# ACK ESCCAL LOW pulse=%u — wait long beep, then ESCCAL DONE\n",
                (unsigned)esc_pulse_us);
}

void escCalDone() {
  esccal_high = false;
  applyEscPulse(ESC_PULSE_MIN_US);
  ctrl_mode = MODE_OPEN;
  hostPrintf("# ACK ESCCAL DONE pulse=%u mode=OPEN\n", (unsigned)esc_pulse_us);
}

/** LEARN START（台阶）与 LEARN RAMP（斜坡）结束时共用：存图/建议 PID/建议分区 PID/回 CLOSED。 */
void finishLearn(const char* stop_reason) {
  activeMap().valid = (activeMap().n >= 2);
  saveMapToNvs(profile_id);
  dumpActiveMap();
  suggestPidFromMap();
  suggestPidZonesFromMap();
  learn_active = false;
  learn_mode_ramp = false;
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
  hostPrintf(
      "# ACK LEARN DONE profile=%s points=%d rpm_max=%.1f pulse_max=%u "
      "rpm_limit=%.0f cover=%.1f%% reason=%s full_throttle=%d can_raise=%d -> MODE CLOSED\n",
      profileName(), activeMap().n, (double)rpm_hi, (unsigned)pulse_hi,
      (double)target_rpm_max, (double)cover, stop_reason, at_full_throttle, can_raise);
  hostPrintf(
      "# LEARN RESULT rpm_peak=%.1f pulse_peak=%u rpm_limit=%.0f reason=%s\n",
      (double)rpm_hi, (unsigned)pulse_hi, (double)target_rpm_max, stop_reason);
  if (at_full_throttle) {
    hostPrintf(
        "# EVAL: ESC full throttle %uus → measured peak RPM=%.1f (limit=%.0f, cover=%.1f%%)\n",
        (unsigned)ESC_PULSE_MAX_US, (double)rpm_hi, (double)target_rpm_max, (double)cover);
  } else if (strcmp(stop_reason, "rpm_cap") == 0) {
    hostPrintf(
        "# EVAL: reached rpm_limit~%.0f at pulse=%u — do not raise throttle further\n",
        (double)target_rpm_max, (unsigned)pulse_hi);
  } else if (can_raise) {
    hostPrintf(
        "# EVAL: rpm_peak=%.0f < limit=%.0f and pulse<%u — unexpected early stop\n",
        (double)rpm_hi, (double)target_rpm_max, (unsigned)ESC_PULSE_MAX_US);
  }
  hostPrintln("# NEXT: apply PID, low RPM closed-loop test, then raise setpoint gradually");
}

/** LEARN START 的台阶学习节拍：升脉宽→停留→记点→判停。 */
void learnTickStep(float rpm_meas) {
  uint32_t now = millis();
  applyEscPulse(learn_pulse);

  uint32_t elapsed = now - learn_step_t0;
  // 停留期间周期性回报，避免 UI 以为卡住
  static uint32_t last_hb = 0;
  if ((now - last_hb) >= 500) {
    last_hb = now;
    uint32_t remain = (elapsed < LEARN_SETTLE_MS) ? (LEARN_SETTLE_MS - elapsed) : 0;
    hostPrintf("# LEARN settle pulse=%u rpm=%.1f remain_ms=%lu\n",
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

  hostPrintf("# ACK LEARN point pulse=%u rpm=%.1f n=%d next_in=%lums\n",
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
    finishLearn(stop_reason);
    return;
  }

  learn_pulse = (uint16_t)(learn_pulse + LEARN_STEP_US);
  if (learn_pulse > learn_max_us) learn_pulse = learn_max_us;
  learn_idx++;
  learn_step_t0 = now;
  hostPrintf("# LEARN next pulse=%u\n", (unsigned)learn_pulse);
}

/**
 * LEARN RAMP 的斜坡学习节拍：脉宽按 RAMP_US_PER_S 连续爬升（无台阶停留），
 * 每 ~100ms 采一次样，脉宽较上次记录点增量够大才存入前馈图（防止点数超限）。
 */
void learnTickRamp(float rpm_meas, float dt) {
  uint32_t now = millis();

  learn_ramp_pulse_f += RAMP_US_PER_S * dt;
  long new_pulse = (long)learn_ramp_pulse_f;
  if (new_pulse < ESC_PULSE_MIN_US) new_pulse = ESC_PULSE_MIN_US;
  if (new_pulse > ESC_PULSE_MAX_US) new_pulse = ESC_PULSE_MAX_US;
  learn_pulse = (uint16_t)new_pulse;
  applyEscPulse(learn_pulse);

  float rpm_abs = fabsf(rpm_meas);

  // 心跳回报，避免 UI 以为卡住
  static uint32_t last_hb = 0;
  if ((now - last_hb) >= 500) {
    last_hb = now;
    hostPrintf("# LEARN RAMP settle pulse=%u rpm=%.1f n=%d\n",
                  (unsigned)learn_pulse, (double)rpm_abs, activeMap().n);
  }

  if ((now - learn_ramp_last_sample_ms) >= RAMP_SAMPLE_MS) {
    learn_ramp_last_sample_ms = now;
    ThrottleMap& m = activeMap();
    if (m.n < MAP_MAX_POINTS &&
        learn_pulse >= (uint16_t)(learn_ramp_last_pulse + RAMP_SAMPLE_PULSE_STEP)) {
      int i = m.n;
      m.pulse[i] = learn_pulse;
      m.rpm[i] = rpm_abs;
      m.n++;
      learn_ramp_last_pulse = learn_pulse;
      hostPrintf("# LEARN RAMP sample pulse=%u rpm=%.1f n=%d\n",
                    (unsigned)learn_pulse, (double)rpm_abs, m.n);
    }
  }

  float rpm_stop = target_rpm_max * LEARN_RPM_STOP_RATIO;
  const char* stop_reason = nullptr;
  if (rpm_abs >= rpm_stop) {
    stop_reason = "rpm_cap";
  } else if (learn_pulse >= ESC_PULSE_MAX_US) {
    stop_reason = "pulse_cap";
  } else if (activeMap().n >= MAP_MAX_POINTS) {
    stop_reason = "map_full";
  }

  if (stop_reason != nullptr) {
    finishLearn(stop_reason);
  }
}

void learnTick(float rpm_meas, float dt) {
  if (!learn_active) return;
  if (learn_mode_ramp) {
    learnTickRamp(rpm_meas, dt);
  } else {
    learnTickStep(rpm_meas);
  }
}

void phaseMoveFinish(const char* why) {
  move_active = false;
  sense_learn = false;
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  clearPid();
  run_state = RUN_IDLE;
  applyEscPulse(ESC_PULSE_MIN_US);
  hostPrintf("# ACK MOVE DONE reason=%s accum=%.2f target=%.2f\n",
                why, (double)move_accum_deg, (double)move_target_deg);
}

void holdPhaseFinish(const char* why) {
  float disc = discEstFromMotor();
  uint32_t elapsed = millis() - holdph_t0_ms;
  uint32_t brake_age =
      (holdph_brake_ms > 0) ? (millis() - holdph_brake_ms) : 0;
  uint32_t prof = holdph_profile_ms;
  uint32_t soft = holdph_soft_ms;
  float rpm0 = holdph_rpm_start;
  holdph_active = false;
  sense_learn = false;
  move_active = false;
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  soft_cmd_accel_rpm_s = 0.0f;
  holdph_pulse = ESC_PULSE_MIN_US;
  holdph_state = HOLDPH_ST_APPROACH;
  holdph_profile_ms = 0;
  holdph_soft_ms = 0;
  holdph_soft_done = false;
  clearPid();
  run_state = RUN_IDLE;
  applyEscPulse(ESC_PULSE_MIN_US);
  hostPrintf(
      "# ACK HOLD PHASE DONE reason=%s target=%.1f disc=%.2f lead=%.1f "
      "hall_confirm=%d pulse=%u elapsed_ms=%lu brake_to_stop_ms=%lu "
      "profile_ms=%lu soft_ms=%lu rpm0=%.0f\n",
      why, (double)holdph_target_deg,
      (double)(isnan(disc) ? -1.0f : disc), (double)holdph_lead_deg,
      holdph_hall_seen ? 1 : 0, (unsigned)esc_pulse_us,
      (unsigned long)elapsed, (unsigned long)brake_age,
      (unsigned long)prof, (unsigned long)soft, (double)rpm0);
}

/** 有效提前量（盘°）：基础角 + ω·T + k·RPM；单向接近时在「目标前」这么多度切油 */
static float holdPhaseEffectiveLead(float disc_rate_deg_s, float motor_rpm_abs) {
  float lead = holdph_lead_deg;
  if (disc_rate_deg_s > 0.0f && holdph_lead_ms > 0.0f) {
    lead += disc_rate_deg_s * (holdph_lead_ms * 0.001f);
  }
  lead += holdph_lead_rpm_k * motor_rpm_abs;
  if (lead < 2.0f) lead = 2.0f;
  if (lead > 80.0f) lead = 80.0f;
  return lead;
}

/** 接近段：剩余盘角越大油门越高；近提前区再降速；脉宽夹 holdph_max_pulse */
static void holdPhaseApplyCrawl(float remain_disc_deg) {
  float rpm = holdph_crawl_rpm;
  if (remain_disc_deg < 50.0f) rpm = fminf(rpm, 100.0f);
  if (remain_disc_deg < 25.0f) rpm = fminf(rpm, 70.0f);
  if (remain_disc_deg < 15.0f) rpm = fminf(rpm, 55.0f);
  if (rpm < 40.0f) rpm = 40.0f;
  target_cmd = rpm;
  target_ramped = rpm;
  soft_cmd_accel_rpm_s = 0.0f;
  uint16_t p = computePulseCrawl(rpm);
  if (p > holdph_max_pulse) p = holdph_max_pulse;
  if (p < CRAWL_PULSE_MIN) p = CRAWL_PULSE_MIN;
  holdph_pulse = p;
}

/** 软降段：闭环线性 RPM 指令 start→crawl；不夹 max_pulse */
static void holdPhaseApplySoftDown(uint32_t elapsed_ms, float motor_rpm_abs) {
  float u = 1.0f;
  if (holdph_soft_ms > 0) {
    u = (float)elapsed_ms / (float)holdph_soft_ms;
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
  }
  float rpm_cmd =
      holdph_rpm_start + (holdph_crawl_rpm - holdph_rpm_start) * u;
  if (rpm_cmd < holdph_crawl_rpm) rpm_cmd = holdph_crawl_rpm;
  float dt = 1.0f / (float)CTRL_HZ;
  float rate = 0.0f;
  if (holdph_soft_ms > 0) {
    rate = (holdph_crawl_rpm - holdph_rpm_start) * 1000.0f /
           (float)holdph_soft_ms;  // 负=减速
  }
  soft_cmd_accel_rpm_s = rate;
  target_cmd = rpm_cmd;
  target_ramped = rpm_cmd;
  ctrl_mode = MODE_CLOSED;
  holdph_pulse = computePulseClosed(rpm_cmd, motor_rpm_abs, dt);
}

static void holdPhaseEnterBrake(const char* why) {
  holdph_state = HOLDPH_ST_BRAKE;
  holdph_brake_ms = millis();
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  soft_cmd_accel_rpm_s = 0.0f;
  holdph_pulse = ESC_PULSE_MIN_US;
  applyEscPulse(ESC_PULSE_MIN_US);
  hostPrintf(
      "# HOLD PHASE BRAKE reason=%s remain_was=%.2f elapsed_ms=%lu rpm=%.0f\n",
      why, (double)(isnan(holdph_prev_remain) ? -1.0f : holdph_prev_remain),
      (unsigned long)(millis() - holdph_t0_ms), (double)fabsf(rpm_signed_stable));
}

/** stop_ms=0：旧行为立即爬行；stop_ms>0：先闭环软降再爬行+提前制动 */
bool holdPhaseBegin(float target_disc_deg, float rpm, uint32_t stop_ms) {
  if (run_state == RUN_ESTOP) {
    hostPrintln("# ERR clear ESTOP first");
    return false;
  }
  if (ctrl_mode == MODE_LEARN || ctrl_mode == MODE_MEASURE) {
    hostPrintln("# ERR abort LEARN/MEASURE before HOLD PHASE");
    return false;
  }
  if (test_active) {
    hostPrintln("# ERR abort TEST before HOLD PHASE");
    return false;
  }
  if (isnan(last_deg)) {
    hostPrintln("# ERR no angle yet");
    return false;
  }
  // 提前制动依赖编码器盘相位；需至少一次霍尔绝对复位（GPIO4→0° / GPIO5→180° / HALL CAL）
  if (!flap_calibrated) {
    hostPrintln(
        "# ERR HOLD PHASE needs Hall latch (pass GPIO4=0 / GPIO5=180 / HALL CAL) — "
        "encoder disc phase = wrap(ref + sense*dmotor/gear)");
    return false;
  }

  target_disc_deg = wrap360(target_disc_deg);
  if (rpm < 40.0f) rpm = 40.0f;
  if (rpm > 400.0f) rpm = 400.0f;
  if (rpm > target_rpm_max) rpm = target_rpm_max;

  holdph_target_deg = target_disc_deg;
  holdph_crawl_rpm = rpm;
  holdph_t0_ms = millis();
  holdph_brake_ms = 0;
  holdph_prev_disc = NAN;
  holdph_prev_remain = NAN;
  holdph_hall_seen = false;
  holdph_state = HOLDPH_ST_APPROACH;
  holdph_dn0 = hall_dn_irq_cnt;
  holdph_up0 = hall_up_irq_cnt;
  holdph_rpm_start = fabsf(rpm_signed_stable);
  if (holdph_rpm_start < rpm) holdph_rpm_start = rpm;

  // 定时软降：默认先试 ~2s；软降占 profile 前 ~75%，余量给爬行+提前制动
  if (stop_ms > 0) {
    if (stop_ms < 800) stop_ms = 800;
    if (stop_ms > 15000) stop_ms = 15000;
    holdph_profile_ms = stop_ms;
    holdph_soft_ms = (stop_ms * 3u) / 4u;  // 75%
    if (holdph_soft_ms < 500) holdph_soft_ms = 500;
    holdph_soft_done = false;
  } else {
    holdph_profile_ms = 0;
    holdph_soft_ms = 0;
    holdph_soft_done = true;  // 无软降，直接允许 lead 制动
  }

  float disc = discEstFromMotor();
  if (!isnan(disc)) {
    float remain = discRemainFwd(disc, target_disc_deg);
    if (remain <= holdph_tol_deg && holdph_rpm_start < 80.0f) {
      hostPrintf("# ACK HOLD PHASE already at target=%.1f disc=%.2f\n",
                 (double)target_disc_deg, (double)disc);
      holdph_active = false;
      applyEscPulse(ESC_PULSE_MIN_US);
      run_state = RUN_IDLE;
      return true;
    }
  }

  // 超时：有 profile 时 = profile + 余量；否则按爬行估一盘
  if (holdph_profile_ms > 0) {
    holdph_timeout_ms = holdph_profile_ms + 10000;
    if (holdph_timeout_ms < 12000) holdph_timeout_ms = 12000;
  } else {
    float motor_rpm = fmaxf(rpm, 40.0f);
    float disc_rpm = motor_rpm / fmaxf(GEAR_RATIO, 1.0f);
    uint32_t est_ms = (uint32_t)((360.0f / fmaxf(disc_rpm, 0.1f)) * (60.0f / 360.0f) *
                                 1000.0f * 2.5f);
    if (est_ms < 8000) est_ms = 8000;
    if (est_ms > 45000) est_ms = 45000;
    holdph_timeout_ms = est_ms;
  }

  move_active = false;
  sense_learn = false;
  learn_active = false;
  measure_active = false;
  stopat_on = false;
  holdph_active = true;
  clearPid();
  run_state = RUN_RUNNING;

  if (holdph_soft_ms > 0 && holdph_rpm_start > holdph_crawl_rpm + 50.0f) {
    ctrl_mode = MODE_CLOSED;
    holdPhaseApplySoftDown(0, holdph_rpm_start);
  } else {
    holdph_soft_done = true;
    holdph_soft_ms = 0;
    ctrl_mode = MODE_OPEN;
    holdPhaseApplyCrawl(180.0f);
  }
  applyEscPulse(holdph_pulse);

  const bool t0 = fabsf(wrap180(target_disc_deg)) <= 0.5f;
  const bool t180 = fabsf(wrap180(target_disc_deg - 180.0f)) <= 0.5f;
  const char* marker = t0 ? "GPIO4/0deg" : (t180 ? "GPIO5/180deg" : "synth/enc");
  hostPrintf(
      "# ACK HOLD PHASE target=%.1f marker=%s crawl_rpm=%.0f rpm0=%.0f "
      "profile_ms=%lu soft_ms=%lu max_pulse=%u lead_deg=%.1f lead_ms=%.0f "
      "lead_k=%.3f timeout_ms=%lu tol=%.1f via=%s "
      "(soft-down then early-brake BEFORE hall; uni ESC)\n",
      (double)holdph_target_deg, marker, (double)holdph_crawl_rpm,
      (double)holdph_rpm_start, (unsigned long)holdph_profile_ms,
      (unsigned long)holdph_soft_ms, (unsigned)holdph_max_pulse,
      (double)holdph_lead_deg, (double)holdph_lead_ms,
      (double)holdph_lead_rpm_k, (unsigned long)holdph_timeout_ms,
      (double)holdph_tol_deg, esc_sense > 0 ? "CW" : "CCW");
  return true;
}

void holdPhaseTick(float ddeg_motor, float motor_rpm_abs) {
  if (!holdph_active) return;

  uint32_t elapsed = millis() - holdph_t0_ms;
  if (elapsed > holdph_timeout_ms) {
    hostPrintf("# ERR HOLD PHASE timeout target=%.1f state=%u — ESTOP\n",
               (double)holdph_target_deg, (unsigned)holdph_state);
    doEstop("hold_phase_timeout");
    return;
  }

  float disc = discEstFromMotor();
  if (isnan(disc)) {
    if (holdph_state == HOLDPH_ST_APPROACH) {
      if (!holdph_soft_done && holdph_soft_ms > 0)
        holdPhaseApplySoftDown(elapsed, motor_rpm_abs);
      else
        holdPhaseApplyCrawl(90.0f);
    } else {
      holdph_pulse = ESC_PULSE_MIN_US;
    }
    return;
  }

  float remain = discRemainFwd(disc, holdph_target_deg);
  float disc_rate = fabsf(ddeg_motor) * (float)CTRL_HZ / fmaxf(GEAR_RATIO, 1.0f);
  float lead = holdPhaseEffectiveLead(disc_rate, motor_rpm_abs);

  // 软降未完成：多圈经过目标时不因霍尔/lead 切油（否则高速误刹）
  // 禁止仅凭瞬时低转速提前结束（测速毛刺会误触发，随后 lead 在高惯量下切油）
  if (!holdph_soft_done && holdph_soft_ms > 0 &&
      holdph_state == HOLDPH_ST_APPROACH) {
    bool soft_time_up = (elapsed >= holdph_soft_ms);
    bool cmd_low = (target_ramped <= holdph_crawl_rpm + 50.0f);
    bool meas_low = (motor_rpm_abs <= holdph_crawl_rpm + 120.0f);
    bool soft_early_ok =
        (elapsed >= (holdph_soft_ms * 2u) / 3u) && cmd_low && meas_low;
    if (soft_time_up || soft_early_ok) {
      holdph_soft_done = true;
      ctrl_mode = MODE_OPEN;
      soft_cmd_accel_rpm_s = 0.0f;
      hostPrintf("# HOLD PHASE SOFTDONE elapsed_ms=%lu rpm=%.0f remain=%.1f\n",
                 (unsigned long)elapsed, (double)motor_rpm_abs, (double)remain);
    } else {
      holdph_prev_remain = remain;
      holdph_prev_disc = disc;
      // 仅计霍尔沿作诊断，不制动
      const bool want0 = fabsf(wrap180(holdph_target_deg)) <= holdph_tol_deg;
      const bool want180 =
          fabsf(wrap180(holdph_target_deg - 180.0f)) <= holdph_tol_deg;
      if (want0 && hall_dn_irq_cnt > holdph_dn0) {
        holdph_dn0 = hall_dn_irq_cnt;  // 吃掉沿，软降后再确认
      }
      if (want180 && hall_up_irq_cnt > holdph_up0) {
        holdph_up0 = hall_up_irq_cnt;
      }
      holdPhaseApplySoftDown(elapsed, motor_rpm_abs);
      return;
    }
  }

  // 软降刚结束但转速仍高：继续闭环压到爬行区，暂不 lead 制动
  if (holdph_soft_done && holdph_profile_ms > 0 &&
      holdph_state == HOLDPH_ST_APPROACH &&
      motor_rpm_abs > holdph_crawl_rpm + 250.0f) {
    holdph_prev_remain = remain;
    holdph_prev_disc = disc;
    const bool want0 = fabsf(wrap180(holdph_target_deg)) <= holdph_tol_deg;
    const bool want180 =
        fabsf(wrap180(holdph_target_deg - 180.0f)) <= holdph_tol_deg;
    if (want0 && hall_dn_irq_cnt > holdph_dn0) holdph_dn0 = hall_dn_irq_cnt;
    if (want180 && hall_up_irq_cnt > holdph_up0) holdph_up0 = hall_up_irq_cnt;
    // 继续压转速到爬行
    float dt = 1.0f / (float)CTRL_HZ;
    soft_cmd_accel_rpm_s = -soft_rate_down_rpm_s;
    target_cmd = holdph_crawl_rpm;
    target_ramped = holdph_crawl_rpm;
    ctrl_mode = MODE_CLOSED;
    holdph_pulse = computePulseClosed(holdph_crawl_rpm, motor_rpm_abs, dt);
    return;
  }

  // 霍尔仅作确认/诊断：不在此沿上才切油（切油必须发生在 remain<=lead）
  const bool want0 = fabsf(wrap180(holdph_target_deg)) <= holdph_tol_deg;
  const bool want180 = fabsf(wrap180(holdph_target_deg - 180.0f)) <= holdph_tol_deg;
  if (want0 && hall_dn_irq_cnt > holdph_dn0) {
    holdph_hall_seen = true;
    holdph_dn0 = hall_dn_irq_cnt;
    if (holdph_state == HOLDPH_ST_APPROACH) {
      // 霍尔已到而尚未提前制动 → 提前量不足，紧急切油
      holdPhaseEnterBrake("hall_dn_before_lead");
    }
  }
  if (want180 && hall_up_irq_cnt > holdph_up0) {
    holdph_hall_seen = true;
    holdph_up0 = hall_up_irq_cnt;
    if (holdph_state == HOLDPH_ST_APPROACH) {
      holdPhaseEnterBrake("hall_up_before_lead");
    }
  }

  if (holdph_state == HOLDPH_ST_APPROACH) {
    // 预测制动：剩余盘角进入提前窗 → 立刻 1000µs（不等霍尔）
    if (remain <= lead) {
      holdph_prev_remain = remain;
      holdPhaseEnterBrake("lead");
    } else {
      holdph_prev_remain = remain;
      holdph_prev_disc = disc;
      holdPhaseApplyCrawl(remain);
    }
    return;
  }

  // BRAKE / SETTLE：保持最低油门，等惯量耗尽
  holdph_pulse = ESC_PULSE_MIN_US;
  target_cmd = 0.0f;
  target_ramped = 0.0f;
  soft_cmd_accel_rpm_s = 0.0f;

  float err = fabsf(wrap180(disc - holdph_target_deg));
  bool near = err <= holdph_tol_deg;
  // 单向越过目标后停稳也算成功（提前量略大时停在目标前一点点则靠 near）
  bool past = false;
  if (!isnan(holdph_prev_disc)) {
    float prev_remain = discRemainFwd(holdph_prev_disc, holdph_target_deg);
    past = (prev_remain > 0.5f) && (remain < 0.5f) && (err <= holdph_tol_deg * 2.0f);
  }
  holdph_prev_disc = disc;
  holdph_prev_remain = remain;

  bool slow = motor_rpm_abs <= holdph_stop_rpm;
  uint32_t since_brake = millis() - holdph_brake_ms;

  if (holdph_state == HOLDPH_ST_BRAKE) {
    if (slow || since_brake >= 400) {
      holdph_state = HOLDPH_ST_SETTLE;
    }
    return;
  }

  // SETTLE
  if (slow && (near || past || holdph_hall_seen)) {
    const char* why = holdph_hall_seen ? "settle_hall" : (past ? "settle_past" : "settle_near");
    holdPhaseFinish(why);
    return;
  }
  if (slow && since_brake >= 1500) {
    // 已停转但相位偏差大：仍停车，避免空转；报告 miss
    if (err <= holdph_tol_deg * 3.0f) {
      holdPhaseFinish("settle_slow");
    } else {
      hostPrintf("# ERR HOLD PHASE miss disc=%.2f target=%.1f err=%.1f — ESTOP\n",
                 (double)disc, (double)holdph_target_deg, (double)err);
      doEstop("hold_phase_miss");
    }
    return;
  }
  if (since_brake >= 4000 && !slow) {
    hostPrintf("# ERR HOLD PHASE coast spin disc=%.2f rpm=%.0f — ESTOP\n",
               (double)disc, (double)motor_rpm_abs);
    doEstop("hold_phase_coast");
  }
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
    hostPrintln("# ERR no angle yet");
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
    hostPrintf("# ACK MOVE skip already at target rel=%.2f (user wanted Δ=%.2f)\n",
                  (double)rel, (double)user_delta_cw);
    return false;
  }
  hostPrintf(
      "# UNI userΔcw=%.2f -> target=%.2f travel=%.2f via=%s (esc_sense=%s)\n",
      (double)user_delta_cw, (double)target, (double)travel,
      esc_sense > 0 ? "CW" : "CCW", esc_sense > 0 ? "+encoder" : "-encoder");
  return phaseMoveBegin(esc_sense, travel, rpm);
}

bool phaseGotoRel(float target_rel, float rpm) {
  if (isnan(last_deg)) {
    hostPrintln("# ERR no angle yet");
    return false;
  }
  target_rel = wrap360(target_rel);
  float rel = phaseRelFromAbs(last_deg);
  float travel = uniTravelDeg(rel, target_rel);
  if (travel < 0.5f) {
    hostPrintf("# ACK GOTO already at %.2f (rel=%.2f)\n", (double)target_rel, (double)rel);
    return false;
  }
  hostPrintf("# GOTO target=%.2f from=%.2f travel=%.2f via=%s\n",
                (double)target_rel, (double)rel, (double)travel,
                esc_sense > 0 ? "CW" : "CCW");
  return phaseMoveBegin(esc_sense, travel, rpm);
}

bool phaseMoveBegin(int dir, float deg, float rpm) {
  if (run_state == RUN_ESTOP) {
    hostPrintln("# ERR clear ESTOP first");
    return false;
  }
  if (ctrl_mode == MODE_LEARN || ctrl_mode == MODE_MEASURE) {
    hostPrintln("# ERR abort LEARN/MEASURE before MOVE");
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
  holdph_active = false;
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
  hostPrintf("# ACK MOVE via=%s travel=%.2f rpm=%.1f esc_sense=%d pulse~%u\n",
                move_dir > 0 ? "CW" : "CCW", (double)deg, (double)rpm, esc_sense,
                (unsigned)esc_pulse_us);
  return true;
}

void senseLearnBegin(float rpm_or_pulse) {
  if (run_state == RUN_ESTOP) {
    hostPrintln("# ERR clear ESTOP first");
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
  hostPrintf("# ACK SENSE AUTO pulse=%uus for %lums (fixed throttle, no soft)\n",
                (unsigned)sense_pulse_us, (unsigned long)SENSE_MS);
}

void phaseTick(float abs_deg, float ddeg) {
  float rel = phaseRelFromAbs(abs_deg);

  if (sense_learn) {
    applyEscPulse(sense_pulse_us);  // 辨识期间强制固定脉宽
    sense_accum += ddeg;
    if ((millis() - sense_t0) >= SENSE_MS) {
      if (fabsf(sense_accum) < 3.0f) {
        hostPrintf(
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
      hostPrintf("# ACK SENSE=%d (%s when throttle) accum=%.2f pulse=%u saved\n",
                    esc_sense, esc_sense > 0 ? "CW/encoder+" : "CCW/encoder-",
                    (double)sense_accum, (unsigned)sense_pulse_us);
    }
    return;
  }

  if (holdph_active) {
    holdPhaseTick(ddeg, fabsf(rpm_signed_stable));
    return;
  }

  if (move_active) {
    move_accum_deg += ddeg;
    bool wrong = (move_dir > 0 && ddeg < -0.05f) || (move_dir < 0 && ddeg > 0.05f);
    if (wrong) {
      move_wrong_ms += (CTRL_PERIOD_US / 1000);
      if (move_wrong_ms > 800) {
        // 自动纠正电调方向并重试剩余角度
        float remain = move_target_deg - fabsf(move_accum_deg);
        if (remain < 0.5f) remain = move_target_deg;
        esc_sense = -esc_sense;
        saveEscSenseToNvs();
        hostPrintf("# WARN flipped esc_sense -> %d, retry remain=%.2f\n",
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
        hostPrintf("# ACK STOPAT hit target=%.2f rel=%.2f\n",
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
  // 主机超时：默认 1.5s；FLIGHT / 板内 TEST / LEARN / HOLD PHASE / HOST OFF(ms=0) 禁用
  if (host_timeout_ms > 0 && !flight_mode && !test_active && host_seen &&
      run_state == RUN_RUNNING && !learn_active && !holdph_active) {
    if ((millis() - last_host_ms) > host_timeout_ms) {
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
    learnTick(rpm_meas, dt);
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

  if (holdph_active) {
    applyEscPulse(holdph_pulse);
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
  } else if (open_pwm_hold) {
    // 主机 PWM 探点：保持 esc_pulse_us，不被 target_ramped 覆盖
    pulse = esc_pulse_us;
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
    hostPrintln("# ACK PING");
    return;
  }
  if (strcasecmp(line, "ENC?") == 0 || strcasecmp(line, "ENC") == 0) {
    EncShared snap;
    encGetSnapshot(&snap);
    // ENC? = 采集/使用层：enc_sample_hz≈2000（采了2k、用2k求速）
    hostPrintf(
        "# ENC collect_hz=%lu use_hz=%lu meas_hz=%.1f cpu%%=%.2f loop_hz=%.0f window=%d "
        "spi_hz=%lu nyquist_rpm=%lu counts=%lld raw=%u rpm=%.1f ef=%u agc=%u seq=%lu "
        "(采集+使用同层全用@2k; 输出见CTRL?)\n",
        (unsigned long)ENC_SAMPLE_HZ, (unsigned long)ENC_SAMPLE_HZ,
        (double)enc_sample_hz_meas, (double)enc_cpu_pct, (double)enc_loop_hz, ENC_VEL_WINDOW,
        (unsigned long)ENC_SPI_HZ, (unsigned long)(ENC_SAMPLE_HZ * 30UL), (long long)snap.counts,
        (unsigned)snap.raw, (double)snap.rpm_signed, (unsigned)snap.ef, (unsigned)snap.agc,
        (unsigned long)snap.seq);
    return;
  }
  if (strcasecmp(line, "CTRL?") == 0 || strcasecmp(line, "CTRL") == 0) {
    // CTRL? = 输出层：ctrl_out_hz≈400（读最新RPM→PID→ESC）；禁止叫采集/使用
    hostPrintf(
        "# CTRL out_hz=%lu meas_out_hz=%.1f period_us=%lu esc_hz=%lu usb_telem_hz=%lu "
        "flight=%d test=%d ble=%d host_timeout=%s "
        "(输出400; 采集/使用见ENC? 数据流:采集2k→使用2k→输出400)\n",
        (unsigned long)CTRL_HZ, (double)ctrl_out_hz_meas, (unsigned long)CTRL_PERIOD_US,
        (unsigned long)esc_pwm_hz, (unsigned long)USB_TELEM_HZ, flight_mode ? 1 : 0,
        test_active ? 1 : 0, bleIsEnabled() ? 1 : 0, hostTimeoutStatusStr());
    return;
  }
  if (strcasecmp(line, "TELEM?") == 0 || strcasecmp(line, "TELEM") == 0) {
    // usb_telem_hz = 每秒完整 CSV 帧数（1帧=1行全字段），非「帧内算速次数」
    hostPrintf(
        "# TELEM usb_telem=%s usb_telem_hz=%lu (frames/s, 1 frame=1 full CSV line, period=%lums) "
        "ble_telem_hz=%u async_log_drop=%lu "
        "(collect/use=%lu out=%lu; telem only copies latest snapshot; TELEM ON|OFF)\n",
        g_usb_telem_on ? "ON" : "OFF", (unsigned long)USB_TELEM_HZ,
        (unsigned long)(1000UL / USB_TELEM_HZ), (unsigned)bleTelemHz(),
        (unsigned long)hostAsyncLogDropped(), (unsigned long)ENC_SAMPLE_HZ,
        (unsigned long)CTRL_HZ);
    return;
  }
  if (strcasecmp(line, "TELEM ON") == 0 || strcasecmp(line, "TELEM 1") == 0) {
    g_usb_telem_on = true;
    hostPrintln("# ACK TELEM ON (USB CSV @20Hz)");
    return;
  }
  if (strcasecmp(line, "TELEM OFF") == 0 || strcasecmp(line, "TELEM 0") == 0) {
    g_usb_telem_on = false;
    hostPrintln("# ACK TELEM OFF (USB CSV paused; LOAD?/cmds still work)");
    return;
  }

  if (strcasecmp(line, "LOAD?") == 0 || strcasecmp(line, "LOAD") == 0) {
    // 一次打印负载快照：采集回调 / 控制周期 / loop 粗闲 / async drop / 工况开关
    uint32_t ring_ov = 0;
#if USE_FIXED_POINT
    ring_ov = enc_ring_overruns(&g_enc_ring);
#endif
    hostPrintf(
        "# LOAD enc_busy_avg_us=%.1f enc_busy_max_us=%lu enc_cpu%%=%.2f meas_hz=%.1f "
        "ctrl_expect_us=%lu ctrl_period_avg_us=%.1f ctrl_period_max_us=%lu "
        "ctrl_busy_avg_us=%.1f ctrl_busy_max_us=%lu ctrl_overrun=%lu meas_out_hz=%.1f "
        "loop_hz=%.0f async_log_drop=%lu "
        "fixed=%d usb_telem=%s ble=%d flight=%d enc_ring_overrun=%lu "
        "(LOAD RESET clears max/overrun/drop)\n",
        (double)enc_busy_avg_us, (unsigned long)enc_busy_max_us, (double)enc_cpu_pct,
        (double)enc_sample_hz_meas, (unsigned long)CTRL_PERIOD_US, (double)ctrl_period_avg_us,
        (unsigned long)ctrl_period_max_us, (double)ctrl_busy_avg_us,
        (unsigned long)ctrl_busy_max_us, (unsigned long)ctrl_overrun_cnt,
        (double)ctrl_out_hz_meas, (double)loop_hz_meas, (unsigned long)hostAsyncLogDropped(),
        USE_FIXED_POINT, g_usb_telem_on ? "ON" : "OFF", bleIsEnabled() ? 1 : 0,
        flight_mode ? 1 : 0, (unsigned long)ring_ov);
    return;
  }
  if (strcasecmp(line, "LOAD RESET") == 0) {
    loadStatsReset();
    hostPrintln("# ACK LOAD RESET (max/overrun/async_log_drop cleared)");
    return;
  }

  if (strcasecmp(line, "FIXED?") == 0 || strcasecmp(line, "FIXED") == 0) {
#if USE_FIXED_POINT
    hostPrintf("# FIXED use_fixed_point=1 enc_ring_cap=%u count=%lu overrun=%lu (Q16 backup path)\n",
               (unsigned)ENC_RING_CAP, (unsigned long)enc_ring_count(&g_enc_ring),
               (unsigned long)enc_ring_overruns(&g_enc_ring));
#else
    hostPrintf("# FIXED use_fixed_point=0 shadow_q16=%d (float main; enable with USE_FIXED_POINT=1)\n",
               Q16_SHADOW_COMPARE);
#endif
    return;
  }
  if (strcasecmp(line, "Q16CMP?") == 0 || strcasecmp(line, "Q16CMP") == 0) {
#if Q16_SHADOW_COMPARE
    uint32_t n;
    double sum;
    float mx, lf, lq;
    portENTER_CRITICAL(&g_q16cmp_mux);
    n = q16cmp_n;
    sum = q16cmp_sum_abs;
    mx = q16cmp_max_abs;
    lf = q16cmp_last_f;
    lq = q16cmp_last_q;
    portEXIT_CRITICAL(&g_q16cmp_mux);
    double mean = (n > 0) ? (sum / (double)n) : 0.0;
    hostPrintf(
        "# Q16CMP shadow=1 n=%lu mean_abs=%.6f max_abs=%.6f rpm_f=%.3f rpm_q=%.3f "
        "(same-sample float vs Q16; NOT dual-flash tracking)\n",
        (unsigned long)n, mean, (double)mx, (double)lf, (double)lq);
#else
    hostPrintln("# Q16CMP shadow=0 (compile with Q16_SHADOW_COMPARE=1 on float path)");
#endif
    return;
  }
  if (strcasecmp(line, "Q16CMP RESET") == 0) {
#if Q16_SHADOW_COMPARE
    // 只清统计，不清影子 EMA（避免与浮点状态失步造成假尖刺）
    q16cmpStatsReset();
    hostPrintln("# ACK Q16CMP RESET (stats only; shadow EMA kept)");
#else
    hostPrintln("# ACK Q16CMP RESET (shadow disabled)");
#endif
    return;
  }
  if (strcasecmp(line, "Q16CMP FULL RESET") == 0) {
#if Q16_SHADOW_COMPARE
    q16cmpFullReset();
    hostPrintln("# ACK Q16CMP FULL RESET (stats+shadow EMA)");
#else
    hostPrintln("# ACK Q16CMP FULL RESET (shadow disabled)");
#endif
    return;
  }
  if (strcasecmp(line, "CORE?") == 0 || strcasecmp(line, "CORE") == 0) {
    hostPrintf("# CORE ctrl=%lu enc_cb=%lu loop=%lu (ctrl pin Core1; enc_cb 随调度)\n",
               (unsigned long)g_ctrl_core_id, (unsigned long)g_enc_cb_core_id,
               (unsigned long)g_loop_core_id);
    return;
  }
  if (strcasecmp(line, "BLE?") == 0) {
    hostPrintf("# BLE enabled=%d conn=%d rate=%u lite=%d name=%s flight=%d wifi=off\n",
               bleIsEnabled() ? 1 : 0, bleConnected() ? 1 : 0, (unsigned)bleTelemHz(),
               bleLiteOn() ? 1 : 0, BLE_DEVICE_NAME, flight_mode ? 1 : 0);
    return;
  }
  if (strcasecmp(line, "BLE ON") == 0 || strcasecmp(line, "BLE 1") == 0) {
    bleSetEnabled(true);
    hostPrintf("# ACK BLE ON enabled=%d\n", bleIsEnabled() ? 1 : 0);
    return;
  }
  if (strcasecmp(line, "BLE OFF") == 0 || strcasecmp(line, "BLE 0") == 0) {
    bleSetEnabled(false);
    hostPrintf("# ACK BLE OFF enabled=%d\n", bleIsEnabled() ? 1 : 0);
    return;
  }
  if (strcasecmp(line, "FLIGHT?") == 0) {
    hostPrintf("# FLIGHT=%s ble=%s host_timeout=%s wifi=off mode=%s\n",
               flight_mode ? "ON" : "OFF", bleIsEnabled() ? "on" : "off",
               hostTimeoutStatusStr(), flight_mode ? "FLIGHT" : "TEST");
    return;
  }
  if (strcasecmp(line, "FLIGHT ON") == 0) {
    flight_mode = true;
    bleSetEnabled(false);
    hostPrintln("# ACK FLIGHT ON ble=off host_timeout=disabled wifi=off");
    return;
  }
  if (strcasecmp(line, "FLIGHT OFF") == 0) {
    flight_mode = false;
    hostPrintf("# ACK FLIGHT OFF host_timeout=%s (BLE unchanged; use BLE ON to re-enable)\n",
               hostTimeoutStatusStr());
    return;
  }
  if (strcasecmp(line, "HOST?") == 0 || strcasecmp(line, "HOST") == 0) {
    hostPrintf("# HOST timeout=%s cfg_ms=%lu default_ms=%lu flight=%d test=%d learn=%d\n",
               hostTimeoutStatusStr(), (unsigned long)host_timeout_ms,
               (unsigned long)HOST_TIMEOUT_DEFAULT_MS, flight_mode ? 1 : 0,
               test_active ? 1 : 0, learn_active ? 1 : 0);
    return;
  }
  if (strcasecmp(line, "HOST OFF") == 0 || strcasecmp(line, "HOST TO OFF") == 0) {
    host_timeout_ms = 0;
    hostPrintln("# ACK HOST OFF timeout=disabled (bench)");
    return;
  }
  if (strcasecmp(line, "HOST ON") == 0) {
    host_timeout_ms = HOST_TIMEOUT_DEFAULT_MS;
    hostPrintf("# ACK HOST ON timeout=%.3fs\n",
               (double)HOST_TIMEOUT_DEFAULT_MS / 1000.0);
    return;
  }
  if (strncasecmp(line, "HOST TO", 7) == 0) {
    char* p = line + 7;
    while (*p == ' ' || *p == '\t' || *p == '=' || *p == ':') ++p;
    if (*p == '\0') {
      hostPrintln("# ERR HOST TO <sec>|OFF");
      return;
    }
    if (strcasecmp(p, "OFF") == 0) {
      host_timeout_ms = 0;
      hostPrintln("# ACK HOST OFF timeout=disabled (bench)");
      return;
    }
    float sec = (float)atof(p);
    if (!(sec >= 0.0f) || sec > 120.0f) {
      hostPrintln("# ERR HOST TO <sec> range 0..120 (0=OFF)");
      return;
    }
    if (sec == 0.0f) {
      host_timeout_ms = 0;
      hostPrintln("# ACK HOST OFF timeout=disabled (bench)");
      return;
    }
    host_timeout_ms = (uint32_t)(sec * 1000.0f + 0.5f);
    if (host_timeout_ms < 200) host_timeout_ms = 200;  // 过短易误触发
    hostPrintf("# ACK HOST TO timeout=%.3fs (%lu ms)\n",
               (double)host_timeout_ms / 1000.0, (unsigned long)host_timeout_ms);
    return;
  }
  if (strcasecmp(line, "TUNE?") == 0 || strcasecmp(line, "TUNE") == 0) {
    tuneSetMapRpmMax(mapRpmMaxOf(activeMap()));
    hostPrintf("# TUNE rec=%s ring=%u last_err=%.1f last_ff=%.0f last_u=%.1f last_pulse=%u "
               "last_rpm=%.0f last_tgt=%.0f i=%.0f/%.0f\n",
               tuneRecOn() ? "ON" : "OFF", (unsigned)tuneRingCount(),
               (double)tuneStats().last_err, (double)tuneStats().last_ff,
               (double)tuneStats().last_u, (unsigned)tuneStats().last_pulse,
               (double)tuneStats().last_rpm, (double)tuneStats().last_tgt,
               (double)tuneStats().last_pid_i, (double)tuneStats().last_i_lim);
    tuneEmitSummary("TUNE");
    return;
  }
  if (strcasecmp(line, "TUNE REC ON") == 0 || strcasecmp(line, "TUNE ON") == 0) {
    tune_owned_by_test = false;
    tuneSetMapRpmMax(mapRpmMaxOf(activeMap()));
    tuneRecSet(true);
    hostPrintln("# ACK TUNE REC ON (RAM ring; no print in ctrl path)");
    return;
  }
  if (strcasecmp(line, "TUNE REC OFF") == 0 || strcasecmp(line, "TUNE OFF") == 0) {
    tune_owned_by_test = false;
    tuneRecSet(false);
    hostPrintln("# ACK TUNE REC OFF");
    return;
  }
  if (strcasecmp(line, "TUNE DUMP") == 0) {
    uint16_t nring = tuneRingCount();
    hostPrintf("# TUNE DUMP begin n=%u (sparse every 10th of ring)\n", (unsigned)nring);
    for (uint16_t i = 0; i < nring; i += 10) {
      TuneSample s;
      if (!tuneRingAt(i, &s)) break;
      hostPrintf("# TUNE S i=%u err=%.1f ff=%.0f u=%.1f pulse=%u rpm=%.0f tgt=%.0f iacc=%.0f sat=%u\n",
                 (unsigned)i, (double)s.err, (double)s.ff, (double)s.u, (unsigned)s.pulse,
                 (double)s.rpm, (double)s.target, (double)s.pid_i, (unsigned)s.i_sat);
    }
    hostPrintln("# TUNE DUMP end");
    return;
  }
  if (strncasecmp(line, "BLE RATE", 8) == 0) {
    char* p = line + 8;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    int hz = atoi(p);
    bleSetRate((uint16_t)hz);
    hostPrintf("# ACK BLE RATE=%u Hz (USB telem %lu Hz; ctrl_out %lu Hz)\n",
               (unsigned)bleTelemHz(), (unsigned long)USB_TELEM_HZ, (unsigned long)CTRL_HZ);
    return;
  }
  if (strncasecmp(line, "BLE LITE", 8) == 0) {
    char* p = line + 8;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    if (strcasecmp(p, "ON") == 0 || strcasecmp(p, "1") == 0) bleSetLite(true);
    else if (strcasecmp(p, "OFF") == 0 || strcasecmp(p, "0") == 0) bleSetLite(false);
    else {
      hostPrintln("# ERR BLE LITE ON|OFF");
      return;
    }
    hostPrintf("# ACK BLE LITE=%s\n", bleLiteOn() ? "ON" : "OFF");
    return;
  }
  if (strcasecmp(line, "TEST?") == 0 || strcasecmp(line, "TEST") == 0) {
    testStatusCmd();
    return;
  }
  if (strcasecmp(line, "TEST STOP") == 0) {
    testStopCmd();
    return;
  }
  if (strncasecmp(line, "TEST START", 10) == 0) {
    char* p = line + 10;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == '\0') {
      hostPrintln("# ERR TEST START smoke2000|step7500|idle");
      return;
    }
    testStartByName(p);
    return;
  }
  if (strcasecmp(line, "START") == 0) {
    doStart();
    return;
  }
  if (strcasecmp(line, "STOP SOFT") == 0) {
    if (run_state == RUN_ESTOP) {
      run_state = RUN_IDLE;
      hostPrintln("# ACK ESTOP cleared");
    }
    doStop(false);
    return;
  }
  if (strcasecmp(line, "STOP") == 0 || strcasecmp(line, "STOP NOW") == 0) {
    if (run_state == RUN_ESTOP) {
      run_state = RUN_IDLE;
      hostPrintln("# ACK ESTOP cleared");
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
    hostPrintf("# ACK RPMMAX=%.0f (learn stops ~%.0f, overspeed=%.0f)\n",
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
    open_pwm_hold = false;
    target_cmd = rpm;
    hostPrintf("# ACK RPM=%.1f (max=%.0f)\n", (double)target_cmd, (double)target_rpm_max);
    return;
  }
  if (strcasecmp(line, "SOFT?") == 0) {
    hostPrintf("# SOFT %s up=%.1f down=%.1f rpm/s ramped=%.1f cmd=%.1f accel=%.1f ka=%.4f\n",
                  soft_enable ? "ON" : "OFF",
                  (double)soft_rate_up_rpm_s, (double)soft_rate_down_rpm_s,
                  (double)target_ramped, (double)target_cmd,
                  (double)soft_cmd_accel_rpm_s, (double)ka_us_per_rpms);
    return;
  }
  if (strcasecmp(line, "KA?") == 0) {
    hostPrintf("# KA=%.4f us/(rpm/s)  (dPulse=KA*cmd_accel)\n", (double)ka_us_per_rpms);
    return;
  }
  if (strncasecmp(line, "KA", 2) == 0 && (line[2] == ' ' || line[2] == '=' || line[2] == ':')) {
    char* p = line + 2;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    float v = strtof(p, nullptr);
    if (v < 0.0f) v = 0.0f;
    if (v > 2.0f) v = 2.0f;  // 防止过大脉宽冲击
    ka_us_per_rpms = v;
    prefs.putFloat("ka", ka_us_per_rpms);
    hostPrintf("# ACK KA=%.4f us/(rpm/s)\n", (double)ka_us_per_rpms);
    return;
  }
  if (strcasecmp(line, "ADG?") == 0 || strcasecmp(line, "ADG") == 0) {
    hostPrintf(
        "# ADG %s dual=%s ki_scale=%.3f kp_scale=%.3f ilim_dec=%.0f ilim_acc=%.0f "
        "accel_kp=%.4f accel_ki=%.4f decel_kp=%.4f decel_ki=%.4f\n",
        adg_on ? "ON" : "OFF", adg_dual ? "ON" : "OFF",
        (double)adg_ki_decel_scale, (double)adg_kp_decel_scale,
        (double)adg_i_lim_decel, (double)adg_i_lim_accel,
        (double)adg_kp_accel, (double)adg_ki_accel,
        (double)adg_kp_decel, (double)adg_ki_decel);
    return;
  }
  if (strcasecmp(line, "ADG ON") == 0) {
    adg_on = true;
    hostPrintln("# ACK ADG ON");
    return;
  }
  if (strcasecmp(line, "ADG OFF") == 0) {
    adg_on = false;
    hostPrintln("# ACK ADG OFF");
    return;
  }
  if (strcasecmp(line, "ADG DUAL ON") == 0) {
    adg_dual = true;
    adg_on = true;
    hostPrintln("# ACK ADG DUAL ON (also ADG ON)");
    return;
  }
  if (strcasecmp(line, "ADG DUAL OFF") == 0) {
    adg_dual = false;
    hostPrintln("# ACK ADG DUAL OFF (scale mode)");
    return;
  }
  if (strncasecmp(line, "ADG KI_SCALE", 12) == 0) {
    float v = strtof(line + 12, nullptr);
    if (v < 0.0f) v = 0.0f;
    if (v > 2.0f) v = 2.0f;
    adg_ki_decel_scale = v;
    hostPrintf("# ACK ADG KI_SCALE=%.3f\n", (double)adg_ki_decel_scale);
    return;
  }
  if (strncasecmp(line, "ADG KP_SCALE", 12) == 0) {
    float v = strtof(line + 12, nullptr);
    if (v < 0.0f) v = 0.0f;
    if (v > 2.0f) v = 2.0f;
    adg_kp_decel_scale = v;
    hostPrintf("# ACK ADG KP_SCALE=%.3f\n", (double)adg_kp_decel_scale);
    return;
  }
  if (strncasecmp(line, "ADG ILIM", 8) == 0) {
    // ADG ILIM <decel> [accel]
    char* p = line + 8;
    float d = strtof(p, &p);
    float a = strtof(p, &p);
    if (d < 50.0f) d = 50.0f;
    if (d > 1200.0f) d = 1200.0f;
    adg_i_lim_decel = d;
    if (a >= 50.0f && a <= 1200.0f) adg_i_lim_accel = a;
    hostPrintf("# ACK ADG ILIM decel=%.0f accel=%.0f\n",
                  (double)adg_i_lim_decel, (double)adg_i_lim_accel);
    return;
  }
  if (strncasecmp(line, "ADG ACCEL", 9) == 0) {
    char* p = line + 9;
    float a = strtof(p, &p);
    float b = strtof(p, &p);
    if (a < 0.0f) a = 0.0f;
    if (b < 0.0f) b = 0.0f;
    adg_kp_accel = a;
    adg_ki_accel = b;
    hostPrintf("# ACK ADG ACCEL kp=%.4f ki=%.4f\n",
                  (double)adg_kp_accel, (double)adg_ki_accel);
    return;
  }
  if (strncasecmp(line, "ADG DECEL", 9) == 0) {
    char* p = line + 9;
    float a = strtof(p, &p);
    float b = strtof(p, &p);
    if (a < 0.0f) a = 0.0f;
    if (b < 0.0f) b = 0.0f;
    adg_kp_decel = a;
    adg_ki_decel = b;
    hostPrintf("# ACK ADG DECEL kp=%.4f ki=%.4f\n",
                  (double)adg_kp_decel, (double)adg_ki_decel);
    return;
  }
  if (strcasecmp(line, "ADG SAVE") == 0) {
    prefs.putBool("adg_on", adg_on);
    prefs.putBool("adg_dual", adg_dual);
    prefs.putFloat("adg_kis", adg_ki_decel_scale);
    prefs.putFloat("adg_kps", adg_kp_decel_scale);
    prefs.putFloat("adg_ild", adg_i_lim_decel);
    prefs.putFloat("adg_ila", adg_i_lim_accel);
    prefs.putFloat("adg_kpa", adg_kp_accel);
    prefs.putFloat("adg_kia", adg_ki_accel);
    prefs.putFloat("adg_kpd", adg_kp_decel);
    prefs.putFloat("adg_kid", adg_ki_decel);
    hostPrintln("# ACK ADG SAVE");
    return;
  }
  if (strncasecmp(line, "SOFT RATE UP", 12) == 0) {
    float r = strtof(line + 12, nullptr);
    if (r < 50.0f) r = 50.0f;
    if (r > 5000.0f) r = 5000.0f;
    soft_rate_up_rpm_s = r;
    soft_rate_rpm_s = r;
    hostPrintf("# ACK SOFT RATE UP=%.1f\n", (double)soft_rate_up_rpm_s);
    return;
  }
  if (strncasecmp(line, "SOFT RATE DOWN", 14) == 0) {
    float r = strtof(line + 14, nullptr);
    if (r < 50.0f) r = 50.0f;
    if (r > 5000.0f) r = 5000.0f;
    soft_rate_down_rpm_s = r;
    hostPrintf("# ACK SOFT RATE DOWN=%.1f\n", (double)soft_rate_down_rpm_s);
    return;
  }
  if (strncasecmp(line, "SOFT RATE", 9) == 0) {
    float r = strtof(line + 9, nullptr);
    if (r < 50.0f) r = 50.0f;
    if (r > 5000.0f) r = 5000.0f;
    soft_rate_up_rpm_s = r;
    soft_rate_down_rpm_s = r;
    soft_rate_rpm_s = r;
    hostPrintf("# ACK SOFT RATE=%.1f (up=down)\n", (double)r);
    return;
  }
  if (strcasecmp(line, "SOFT ON") == 0 || strcasecmp(line, "SOFT=ON") == 0) {
    soft_enable = true;
    hostPrintln("# ACK SOFT ON");
    return;
  }
  if (strcasecmp(line, "SOFT OFF") == 0 || strcasecmp(line, "SOFT=OFF") == 0) {
    soft_enable = false;
    hostPrintln("# ACK SOFT OFF");
    return;
  }
  if (strncasecmp(line, "SOFT", 4) == 0 && (line[4] == '\0' || line[4] == ' ' || line[4] == '=' || line[4] == ':')) {
    char* p = line + 4;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    soft_enable = (strncasecmp(p, "ON", 2) == 0);
    hostPrintf("# ACK SOFT %s\n", soft_enable ? "ON" : "OFF");
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
        hostPrintf("# ACK MODE %s (aborted LEARN)\n", modeName());
        return;
      }
      hostPrintln("# ERR LEARN active — use LEARN ABORT or STOP first");
      return;
    }
    if (strncasecmp(p, "OPEN", 4) == 0) ctrl_mode = MODE_OPEN;
    else if (strncasecmp(p, "CLOSED", 6) == 0) {
      open_pwm_hold = false;
      ctrl_mode = MODE_CLOSED;
    } else if (strncasecmp(p, "MEASURE", 7) == 0) {
      measureBegin();
      return;
    } else if (strncasecmp(p, "LEARN", 5) == 0) {
      hostPrintln("# ERR use LEARN START|RAMP or MEASURE AUTO|RAMP");
      return;
    } else if (strncasecmp(p, "SAFE", 4) == 0) {
      ctrl_mode = MODE_SAFE;
      doStop(true);
    } else {
      hostPrintln("# ERR MODE");
      return;
    }
    measure_active = false;
    learn_active = false;
    clearPid();
    hostPrintf("# ACK MODE %s\n", modeName());
    return;
  }
  if (strncasecmp(line, "PROFILE", 7) == 0) {
    char* p = line + 7;
    while (*p == ' ') ++p;
    if (strncasecmp(p, "flap", 4) == 0) profile_id = PROF_FLAP;
    else profile_id = PROF_NOLOAD;
    hostPrintf("# ACK PROFILE %s valid=%d points=%d\n",
                  profileName(), activeMap().valid, activeMap().n);
    return;
  }
  if (strcasecmp(line, "HALL?") == 0 || strcasecmp(line, "HALL") == 0) {
    hallPrintStatus();
    return;
  }
  if (strcasecmp(line, "HALL CAL") == 0) {
    hallCalibrateNow();
    hostPrintf(
        "# ACK HALL CAL disc_ref=0 at motor_unwrap=%.3f counts=%lld "
        "gear=%d/%d=%.6f (physical 0°=GPIO4; then enc-only until next Hall)\n",
        (double)disc_ref_motor_unwrap, (long long)disc_ref_motor_counts,
        (int)GEAR_RATIO_NUM, (int)GEAR_RATIO_DEN, (double)GEAR_RATIO);
    return;
  }
  if (strncasecmp(line, "HALL ACTIVE", 11) == 0) {
    char* p = line + 11;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    if (strcasecmp(p, "LOW") == 0) {
      hall_active_low = true;
      hallAttachIrqs();
      hostPrintln("# ACK HALL ACTIVE LOW (IRQ ANYEDGE+SW enter-low)");
      return;
    }
    if (strcasecmp(p, "HIGH") == 0) {
      hall_active_low = false;
      hallAttachIrqs();
      hostPrintln("# ACK HALL ACTIVE HIGH (IRQ ANYEDGE+SW enter-high)");
      return;
    }
    hostPrintln("# ERR HALL ACTIVE LOW|HIGH");
    return;
  }

  if (strcasecmp(line, "PHASE ZERO") == 0 || strcasecmp(line, "PHASE0") == 0) {
    if (isnan(last_deg)) {
      hostPrintln("# ERR no angle yet");
      return;
    }
    phase_zero_deg = last_deg;
    savePhaseZeroToNvs();
    stopat_prev_rel = NAN;
    hostPrintf("# ACK PHASE ZERO abs=%.3f -> rel=0  (saved)\n", (double)phase_zero_deg);
    return;
  }
  if (strcasecmp(line, "PHASE LEAD?") == 0 || strcasecmp(line, "PHASE LEAD") == 0) {
    hostPrintf(
        "# ACK PHASE LEAD deg=%.2f ms=%.0f k=%.4f offset=%.1f  "
        "(eff=lead_deg + w_disc*ms/1000 + k*|rpm|; NVS hplead/hpleadt/hpleadk/hphoff)\n",
        (double)holdph_lead_deg, (double)holdph_lead_ms, (double)holdph_lead_rpm_k,
        (double)disc_phase_offset_deg);
    return;
  }
  if (strncasecmp(line, "PHASE LEAD SAVE", 15) == 0) {
    saveHoldPhaseLeadToNvs();
    hostPrintf("# ACK PHASE LEAD SAVE deg=%.2f ms=%.0f k=%.4f offset=%.1f\n",
               (double)holdph_lead_deg, (double)holdph_lead_ms,
               (double)holdph_lead_rpm_k, (double)disc_phase_offset_deg);
    return;
  }
  if (strncasecmp(line, "PHASE OFFSET", 12) == 0) {
    char* p = line + 12;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    if (*p == '\0' || strcasecmp(p, "?") == 0) {
      hostPrintf("# ACK PHASE OFFSET deg=%.1f (NVS hphoff; wrap into disc_est)\n",
                 (double)disc_phase_offset_deg);
      return;
    }
    if (strcasecmp(p, "SAVE") == 0) {
      prefs.putFloat("hphoff", disc_phase_offset_deg);
      hostPrintf("# ACK PHASE OFFSET SAVE deg=%.1f\n", (double)disc_phase_offset_deg);
      return;
    }
    float off = strtof(p, &p);
    disc_phase_offset_deg = wrap360(off);
    hostPrintf("# ACK PHASE OFFSET deg=%.1f (RAM; PHASE OFFSET SAVE or PHASE LEAD SAVE → NVS)\n",
               (double)disc_phase_offset_deg);
    return;
  }
  if (strncasecmp(line, "PHASE LEAD ", 11) == 0) {
    char* p = line + 11;
    while (*p == ' ') ++p;
    if (*p == '\0') {
      hostPrintln("# ERR PHASE LEAD <deg> [ms] [k] | PHASE LEAD SAVE | PHASE LEAD?");
      return;
    }
    float d = strtof(p, &p);
    while (*p == ' ') ++p;
    float ms = (*p != '\0') ? strtof(p, &p) : holdph_lead_ms;
    while (*p == ' ') ++p;
    float k = (*p != '\0') ? strtof(p, &p) : holdph_lead_rpm_k;
    if (d < 1.0f) d = 1.0f;
    if (d > 60.0f) d = 60.0f;
    if (ms < 0.0f) ms = 0.0f;
    if (ms > 500.0f) ms = 500.0f;
    if (k < 0.0f) k = 0.0f;
    if (k > 0.2f) k = 0.2f;
    holdph_lead_deg = d;
    holdph_lead_ms = ms;
    holdph_lead_rpm_k = k;
    hostPrintf("# ACK PHASE LEAD deg=%.2f ms=%.0f k=%.4f (RAM; PHASE LEAD SAVE → NVS)\n",
               (double)holdph_lead_deg, (double)holdph_lead_ms,
               (double)holdph_lead_rpm_k);
    return;
  }
  if (strncasecmp(line, "PHASE HOLD ", 11) == 0 || strncasecmp(line, "HOLD PHASE ", 11) == 0) {
    char* p = line + 11;
    while (*p == ' ') ++p;
    float tgt = strtof(p, &p);
    while (*p == ' ') ++p;
    float rpm = holdph_crawl_rpm;
    uint32_t stop_ms = 0;
    if (*p != '\0') {
      rpm = strtof(p, &p);
      while (*p == ' ') ++p;
      if (*p != '\0') stop_ms = (uint32_t)strtoul(p, nullptr, 10);
    }
    holdPhaseBegin(tgt, rpm, stop_ms);
    return;
  }
  // 简写：PHASE 0|90|180 [rpm] [stop_ms]（扑翼盘相位；勿与电机 PHASE ZERO 混淆）
  if (strncasecmp(line, "PHASE ", 6) == 0) {
    char* p = line + 6;
    while (*p == ' ') ++p;
    if ((*p >= '0' && *p <= '9') || *p == '+' || *p == '-') {
      float tgt = strtof(p, &p);
      while (*p == ' ') ++p;
      float rpm = holdph_crawl_rpm;
      uint32_t stop_ms = 0;
      if (*p != '\0') {
        rpm = strtof(p, &p);
        while (*p == ' ') ++p;
        if (*p != '\0') stop_ms = (uint32_t)strtoul(p, nullptr, 10);
      }
      holdPhaseBegin(tgt, rpm, stop_ms);
      return;
    }
  }
  if (strcasecmp(line, "PHASE?") == 0 || strcasecmp(line, "PHASE") == 0) {
    float absd = isnan(last_deg) ? 0.0f : last_deg;
    float rel = phaseRelFromAbs(absd);
    float disc = discEstFromMotor();
    hostPrintf(
        "# ACK PHASE motor_abs=%.3f motor_rel=%.3f zero=%.3f esc_sense=%d "
        "disc_deg=%.2f disc_ref=%.1f offset=%.1f cal=%d holdph=%d state=%u target=%.1f "
        "lead=%.1f/%.0fms gear=%d/%d stopat=%s/%.1f move=%d "
        "(disc=wrap(ref+sense*dmotor/gear+offset); Hall latch, enc continuous)\n",
        (double)absd, (double)rel, (double)phase_zero_deg, esc_sense,
        (double)(isnan(disc) ? -1.0f : disc), (double)disc_ref_deg,
        (double)disc_phase_offset_deg, flap_calibrated ? 1 : 0,
        holdph_active ? 1 : 0, (unsigned)holdph_state, (double)holdph_target_deg,
        (double)holdph_lead_deg, (double)holdph_lead_ms,
        (int)GEAR_RATIO_NUM, (int)GEAR_RATIO_DEN,
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
      hostPrintln("# ERR SENSE AUTO|+|−|CW|CCW");
      return;
    }
    saveEscSenseToNvs();
    hostPrintf("# ACK SENSE=%d (%s) saved\n", esc_sense,
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
      hostPrintln("# ERR MOVE CW|CCW <deg> [rpm]  (uni ESC auto-converts path)");
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
      hostPrintln("# ACK STOPAT OFF");
      return;
    }
    float deg = strtof(p, nullptr);
    stopat_deg = wrap360(deg);
    stopat_on = true;
    stopat_prev_rel = NAN;
    hostPrintf("# ACK STOPAT ON deg=%.2f tol=%.1f via=%s\n",
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
  if (strcasecmp(line, "LEARN RAMP") == 0) {
    // 开环斜坡学习：脉宽连续上升，比台阶更快扫完全程
    if (ctrl_mode != MODE_MEASURE) measureBegin();
    learnBeginRamp();
    return;
  }
  if (strncasecmp(line, "LEARN MAXUS", 11) == 0) {
    // 学习油门上限固定为电调最大；忽略更低值（避免再被设成 1600）
    learn_max_us = ESC_PULSE_MAX_US;
    hostPrintf("# ACK LEARN MAXUS=%u (fixed to ESC max; ignore lower)\n",
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
      hostPrintln("# ERR enter MEASURE first");
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
  if (strcasecmp(line, "MEASURE RAMP") == 0) {
    if (ctrl_mode != MODE_MEASURE) measureBegin();
    learnBeginRamp();
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
    hostPrintf("# PID kp=%.4f ki=%.4f kd=%.4f adapt=%d\n",
                  (double)kp, (double)ki, (double)kd, adapt_on ? 1 : 0);
    return;
  }
  if (strcasecmp(line, "PID SAVE") == 0) {
    savePidToNvs();
    hostPrintln("# ACK PID SAVE");
    return;
  }
  if (strcasecmp(line, "GAIN ON") == 0) {
    gain_schedule_on = true;
    prefs.putBool("gainon", true);
    hostPrintln("# ACK GAIN ON");
    return;
  }
  if (strcasecmp(line, "GAIN OFF") == 0) {
    gain_schedule_on = false;
    prefs.putBool("gainon", false);
    hostPrintln("# ACK GAIN OFF");
    return;
  }
  if (strcasecmp(line, "GAIN?") == 0 || strcasecmp(line, "GAIN") == 0) {
    dumpGainMap();
    return;
  }
  if (strcasecmp(line, "GAIN SAVE") == 0) {
    saveGainMapToNvs(profile_id);
    hostPrintln("# ACK GAIN SAVE");
    return;
  }
  if (strncasecmp(line, "PID", 3) == 0) {
    char* p = line + 3;
    float a = strtof(p, &p);
    float b = strtof(p, &p);
    float c = strtof(p, &p);
    kp = a; ki = b; kd = c;
    clearPid();
    hostPrintf("# ACK PID kp=%.4f ki=%.4f kd=%.4f\n", (double)kp, (double)ki, (double)kd);
    return;
  }
  if (strncasecmp(line, "ADAPT", 5) == 0) {
    char* p = line + 5;
    while (*p == ' ') ++p;
    adapt_on = (strncasecmp(p, "ON", 2) == 0);
    hostPrintf("# ACK ADAPT %s\n", adapt_on ? "ON" : "OFF");
    return;
  }
  if (strncasecmp(line, "FREQ", 4) == 0) {
    char* p = line + 4;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    if (*p == '\0' || strcasecmp(p, "?") == 0) {
      hostPrintf("# FREQ %lu Hz period=%luus proto=%s\n",
                    (unsigned long)esc_pwm_hz, (unsigned long)esc_period_us,
                    esc_proto == ESC_PROTO_DSHOT ? "DSHOT" : "PWM");
      return;
    }
    if (esc_proto != ESC_PROTO_PWM) {
      hostPrintln("# ERR FREQ only in PROTO PWM (use DSHOTRATE for DShot)");
      return;
    }
    long hz = strtol(p, nullptr, 10);
    if (hz < 50 || hz > (long)ESC_PWM_HZ_MAX) {
      hostPrintf("# ERR FREQ range 50..%lu\n", (unsigned long)ESC_PWM_HZ_MAX);
      return;
    }
    bool ok = setEscPwmFreq((uint32_t)hz);
    hostPrintf("# ACK FREQ %lu ok=%d period=%luus pulse=%u\n",
                  (unsigned long)esc_pwm_hz, ok ? 1 : 0,
                  (unsigned long)esc_period_us, (unsigned)esc_pulse_us);
    return;
  }
  if (strcasecmp(line, "PROTO?") == 0 || strcasecmp(line, "PROTO") == 0) {
    if (esc_proto == ESC_PROTO_DSHOT) {
      hostPrintf("# PROTO DSHOT rate=%u thr=%u\n",
                    (unsigned)g_dshot.rate, (unsigned)g_dshot.throttle);
    } else {
      hostPrintf("# PROTO PWM freq=%lu pulse=%u\n",
                    (unsigned long)esc_pwm_hz, (unsigned)esc_pulse_us);
    }
    return;
  }
  if (strncasecmp(line, "PROTO", 5) == 0) {
    char* p = line + 5;
    while (*p == ' ') ++p;
    if (strncasecmp(p, "PWM", 3) == 0) {
      bool ok = setEscProtoPwm();
      hostPrintf("# ACK PROTO PWM ok=%d freq=%lu\n",
                    ok ? 1 : 0, (unsigned long)esc_pwm_hz);
      return;
    }
    if (strncasecmp(p, "DSHOT", 5) == 0) {
      // PROTO DSHOT [150|300|600]
      char* q = p + 5;
      while (*q == ' ') ++q;
      DshotRate rate = DSHOT300;
      if (*q) {
        long r = strtol(q, nullptr, 10);
        if (r == 150) rate = DSHOT150;
        else if (r == 300) rate = DSHOT300;
        else if (r == 600) rate = DSHOT600;
        else {
          hostPrintln("# ERR PROTO DSHOT rate 150|300|600");
          return;
        }
      } else if (g_dshot.rate == DSHOT150 || g_dshot.rate == DSHOT300 ||
                 g_dshot.rate == DSHOT600) {
        rate = g_dshot.rate;
      }
      bool ok = setEscProtoDshot(rate);
      hostPrintf("# ACK PROTO DSHOT ok=%d rate=%u\n",
                    ok ? 1 : 0, (unsigned)g_dshot.rate);
      return;
    }
    hostPrintln("# ERR PROTO PWM|DSHOT [150|300|600]");
    return;
  }
  if (strcasecmp(line, "DSHOTRATE?") == 0) {
    hostPrintf("# DSHOTRATE %u proto=%s\n",
                  (unsigned)g_dshot.rate,
                  esc_proto == ESC_PROTO_DSHOT ? "DSHOT" : "PWM");
    return;
  }
  if (strncasecmp(line, "DSHOTRATE", 9) == 0) {
    char* p = line + 9;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    long r = strtol(p, nullptr, 10);
    DshotRate rate;
    if (r == 150) rate = DSHOT150;
    else if (r == 300) rate = DSHOT300;
    else if (r == 600) rate = DSHOT600;
    else {
      hostPrintln("# ERR DSHOTRATE 150|300|600");
      return;
    }
    if (esc_proto != ESC_PROTO_DSHOT) {
      bool ok = setEscProtoDshot(rate);
      hostPrintf("# ACK DSHOTRATE %u ok=%d (entered DSHOT)\n",
                    (unsigned)rate, ok ? 1 : 0);
      return;
    }
    bool ok = dshotSetRate(g_dshot, rate);
    dshotSetThrottle(g_dshot, 0, false);
    esc_pulse_us = ESC_PULSE_MIN_US;
    hostPrintf("# ACK DSHOTRATE %u ok=%d\n", (unsigned)g_dshot.rate, ok ? 1 : 0);
    return;
  }
  if (strncasecmp(line, "DSHOT", 5) == 0) {
    // DSHOT <0..2047> 直接油门；勿与 DSHOTRATE 混淆（已先匹配 RATE）
    char* p = line + 5;
    while (*p == ' ' || *p == '=' || *p == ':') ++p;
    if (*p == '\0' || strcasecmp(p, "?") == 0) {
      hostPrintf("# DSHOT thr=%u rate=%u proto=%s\n",
                    (unsigned)g_dshot.throttle, (unsigned)g_dshot.rate,
                    esc_proto == ESC_PROTO_DSHOT ? "DSHOT" : "PWM");
      return;
    }
    long v = strtol(p, nullptr, 10);
    if (v < 0 || v > 2047) {
      hostPrintln("# ERR DSHOT range 0..2047");
      return;
    }
    if (esc_proto != ESC_PROTO_DSHOT) {
      if (!setEscProtoDshot(g_dshot.rate == DSHOT_OFF ? DSHOT300 : g_dshot.rate)) {
        hostPrintln("# ERR enter DSHOT failed");
        return;
      }
    }
    open_pwm_hold = true;
    run_state = RUN_RUNNING;
    ctrl_mode = MODE_OPEN;
    dshotSetThrottle(g_dshot, (uint16_t)v, false);
    // 同步显示用脉宽（逆映射近似）
    if (v <= 0) esc_pulse_us = ESC_PULSE_MIN_US;
    else {
      float t = ((float)v - 48.0f) / (2047.0f - 48.0f);
      if (t < 0) t = 0;
      if (t > 1) t = 1;
      esc_pulse_us = (uint16_t)(ESC_PULSE_MIN_US +
                                t * (ESC_PULSE_MAX_US - ESC_PULSE_MIN_US) + 0.5f);
    }
    hostPrintf("# ACK DSHOT=%ld pulse~%u\n", v, (unsigned)esc_pulse_us);
    return;
  }
  if (strncasecmp(line, "PWM", 3) == 0) {
    long us = strtol(line + 3, nullptr, 10);
    if (us < ESC_PULSE_MIN_US || us > ESC_PULSE_MAX_US) {
      hostPrintln("# ERR PWM range");
      return;
    }
    // 自动学习期间禁止主机 PWM 抢控（UI 滑条被遥测带动时会误发）
    if (ctrl_mode == MODE_LEARN || learn_active) {
      hostPrintln("# ERR ignore PWM during LEARN");
      return;
    }
    if (ctrl_mode == MODE_MEASURE) {
      measureSetPulse(us);
      return;
    }
    open_pwm_hold = true;
    run_state = RUN_RUNNING;
    ctrl_mode = MODE_OPEN;
    applyEscPulse((uint16_t)us);
    hostPrintf("# ACK PWM=%ld\n", us);
    return;
  }
  hostPrintf("# ERR unknown: %s\n", line);
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
  // BLE NUS RX → 同一指令解析（远程主机）
  char ble_line[96];
  while (bleTakeRxLine(ble_line, sizeof(ble_line))) {
    handleCommandLine(ble_line);
  }
}

// ---------------- 输出任务 @400Hz（esp_timer → 信号量 → Core1）----------------
// 只读 EncShared 最新 RPM；禁止在此做 SPI。采集/使用仍在 encSampleCb @2k。
static void ctrlTimerCb(void* /*arg*/) {
  // ESP_TIMER_TASK 上下文（非硬 ISR）→ 普通 Give
  if (g_ctrl_sem) xSemaphoreGive(g_ctrl_sem);
}

static void motorControlTask(void* /*arg*/) {
  g_ctrl_core_id = (uint32_t)xPortGetCoreID();
  static bool have_last_counts = false;
  static int64_t last_ctrl_counts = 0;
  static uint32_t ctrl_stat_ms = 0;
  static uint32_t ctrl_stat_ticks = 0;

  for (;;) {
    if (g_ctrl_sem) xSemaphoreTake(g_ctrl_sem, portMAX_DELAY);
    else vTaskDelay(1);

    uint32_t t0 = micros();
    // 控制周期：期望 CTRL_PERIOD_US(2500)；实测间隔与 overrun
    if (g_ctrl_last_wake_us != 0) {
      uint32_t period = t0 - g_ctrl_last_wake_us;
      g_ctrl_period_acc_us += period;
      g_ctrl_period_n++;
      portENTER_CRITICAL(&g_load_mux);
      if (period > ctrl_period_max_us) ctrl_period_max_us = period;
      if (period > (CTRL_PERIOD_US + CTRL_OVERRUN_SLACK_US)) ctrl_overrun_cnt++;
      portEXIT_CRITICAL(&g_load_mux);
    }
    g_ctrl_last_wake_us = t0;

    uint32_t now = millis();
    EncShared snap;
    encGetSnapshot(&snap);

    uint16_t raw = snap.raw;
    float abs_deg = (float)raw * ENC_DEG_PER_COUNT;
    float deg = phaseRelFromAbs(abs_deg);

    motor_unwrapped_counts = snap.counts;
    motor_unwrapped_deg = (float)snap.counts * ENC_DEG_PER_COUNT;
    rpm_signed_stable = snap.rpm_signed;
    rpm_filt = snap.rpm_signed;
    float rpm_mag = fabsf(snap.rpm_signed);

    float ddeg = 0.0f;
    if (have_last_counts) {
      ddeg = (float)(snap.counts - last_ctrl_counts) * ENC_DEG_PER_COUNT;
    }
    last_ctrl_counts = snap.counts;
    have_last_counts = true;

    float dt = (float)CTRL_PERIOD_US * 1.0e-6f;
    if (last_ms != 0 && now > last_ms) {
      float d = (now - last_ms) * 0.001f;
      if (d > 0.0005f && d < 0.05f) dt = d;
    }
    if (snap.seq > 0) last_deg = abs_deg;
    last_ms = now;

    phaseTick(abs_deg, ddeg);
    hallPoll(now, deg, abs_deg, raw);
    controlTick(rpm_mag, dt);  // 输出：读使用层最新 |rpm| → PID → ESC
    testTick(rpm_mag);         // 板内 TEST（状态切换才打印）

    uint32_t busy = micros() - t0;
    g_ctrl_busy_acc_us += busy;
    g_ctrl_busy_n++;
    portENTER_CRITICAL(&g_load_mux);
    if (busy > ctrl_busy_max_us) ctrl_busy_max_us = busy;
    portEXIT_CRITICAL(&g_load_mux);

    g_ctrl_tick_cnt++;
    ctrl_stat_ticks++;
    if (ctrl_stat_ms == 0) {
      ctrl_stat_ms = now;
      ctrl_stat_ticks = 0;
    } else if ((now - ctrl_stat_ms) >= 500) {
      uint32_t dms = now - ctrl_stat_ms;
      ctrl_out_hz_meas = (float)ctrl_stat_ticks * 1000.0f / (float)dms;
      if (g_ctrl_period_n > 0) {
        ctrl_period_avg_us = (float)g_ctrl_period_acc_us / (float)g_ctrl_period_n;
      }
      if (g_ctrl_busy_n > 0) {
        ctrl_busy_avg_us = (float)g_ctrl_busy_acc_us / (float)g_ctrl_busy_n;
      }
      g_ctrl_period_acc_us = 0;
      g_ctrl_period_n = 0;
      g_ctrl_busy_acc_us = 0;
      g_ctrl_busy_n = 0;
      ctrl_stat_ms = now;
      ctrl_stat_ticks = 0;
    }
  }
}

static void controlOutputBegin() {
  g_ctrl_sem = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(motorControlTask, "motorCtrl", 6144, nullptr, 5, &g_ctrl_task, 1);
  hostForbidPrintFromTask(g_ctrl_task);  // 控制任务内 hostPrintf → 异步环
  const esp_timer_create_args_t args = {
      .callback = &ctrlTimerCb,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "ctrl_out",
      .skip_unhandled_events = true,
  };
  if (esp_timer_create(&args, &g_ctrl_timer) == ESP_OK) {
    esp_timer_start_periodic(g_ctrl_timer, CTRL_PERIOD_US);
  }
}

// ---------------- 低优遥测任务（Core0 prio=1；勿堵 2k/400 实时路径）----------------
static TaskHandle_t g_telem_task = nullptr;
static uint32_t g_telem_iters = 0;

static void telemTask(void* /*arg*/) {
  uint32_t next_telem_ms = 0;
  uint32_t stat_prev_ms = 0;
  uint32_t stat_prev_seq = 0;
  uint32_t stat_prev_loop = 0;
  for (;;) {
    hostDrainAsyncLog();

    uint32_t now = millis();
    // 负载统计窗：即使 TELEM OFF 也跑（便于对比负担）
    EncShared snap;
    encGetSnapshot(&snap);
    if (stat_prev_ms == 0) {
      stat_prev_ms = now;
      stat_prev_seq = snap.seq;
      stat_prev_loop = g_loop_iters;
      g_telem_iters = 0;
    } else if ((now - stat_prev_ms) >= 500) {
      uint32_t dms = now - stat_prev_ms;
      enc_sample_hz_meas = (float)(snap.seq - stat_prev_seq) * 1000.0f / (float)dms;
      enc_loop_hz = (float)g_telem_iters * 1000.0f / (float)dms;  // 实际 USB 帧率粗估
      uint32_t loop_now = g_loop_iters;
      loop_hz_meas = (float)(loop_now - stat_prev_loop) * 1000.0f / (float)dms;
      uint32_t bacc, bcnt, bmx;
      encTakeBusy(&bacc, &bcnt, &bmx);
      if (bcnt > 0) {
        float avg_us = (float)bacc / (float)bcnt;
        enc_busy_avg_us = avg_us;
        enc_cpu_pct = avg_us * enc_sample_hz_meas / 10000.0f;
      }
      enc_busy_max_window_us = bmx;
      portENTER_CRITICAL(&g_load_mux);
      if (bmx > enc_busy_max_us) enc_busy_max_us = bmx;
      portEXIT_CRITICAL(&g_load_mux);
      stat_prev_ms = now;
      stat_prev_seq = snap.seq;
      stat_prev_loop = loop_now;
      g_telem_iters = 0;
    }

    if ((int32_t)(now - next_telem_ms) < 0) {
      vTaskDelay(1);
      continue;
    }
    next_telem_ms = now + (1000UL / USB_TELEM_HZ);

    if (!g_usb_telem_on) {
      // 仍跑 BLE lite（若开）与统计；仅暂停 USB CSV
      float oh = isnan(out_hz_meas) ? -1.0f : out_hz_meas;
      float gm = isnan(gear_ratio_meas) ? -1.0f : gear_ratio_meas;
      bleEmitTelemLite(now, rpm_signed_stable, esc_pulse_us, target_ramped, (int)ctrl_mode,
                       (int)run_state, oh, gm);
      vTaskDelay(1);
      continue;
    }

    uint16_t raw = snap.raw;
    float abs_deg = (float)raw * ENC_DEG_PER_COUNT;
    float deg = phaseRelFromAbs(abs_deg);
    float rad = deg * (TWO_PI_F / 360.0f);

    g_telem_iters++;

    int mode_i = (int)ctrl_mode;
    int run_i = (int)run_state;
    int prof_i = (int)profile_id;
    float disc_est = discEstFromMotor();
    float hd_m = isnan(last_hd_motor_rel) ? -1.0f : last_hd_motor_rel;
    float hu_m = isnan(last_hu_motor_rel) ? -1.0f : last_hu_motor_rel;
    float disc_out = isnan(disc_est) ? -1.0f : disc_est;
    float oh = isnan(out_hz_meas) ? -1.0f : out_hz_meas;
    float gm = isnan(gear_ratio_meas) ? -1.0f : gear_ratio_meas;

    char line[360];
    int n = snprintf(
        line, sizeof(line),
        "%lu,%u,%.3f,%.6f,%.2f,%u,%u,%u,%u,%u,%.1f,%d,%.4f,%.4f,%.4f,%d,%d,%u,%u,%.3f,%.3f,%.3f,%.5f,%.5f,%.1f,%.1f,%.0f,%.1f\n",
        (unsigned long)now, (unsigned)raw, deg, rad, rpm_signed_stable, (unsigned)snap.ef,
        (unsigned)snap.agc, (unsigned)snap.mag_low, (unsigned)snap.mag_high,
        (unsigned)esc_pulse_us, (double)target_ramped, mode_i, (double)kp, (double)ki, (double)kd,
        prof_i, run_i, (unsigned)hall_dn_lvl, (unsigned)hall_up_lvl, (double)disc_out,
        (double)hd_m, (double)hu_m, (double)oh, (double)gm, (double)enc_sample_hz_meas,
        (double)g_closed_err, (double)g_closed_ff, (double)g_closed_u);
    if (n > 0) {
      if (n >= (int)sizeof(line)) n = (int)sizeof(line) - 1;
      Serial.write((const uint8_t*)line, (size_t)n);
    }
    bleEmitTelemLite(now, rpm_signed_stable, esc_pulse_us, target_ramped, mode_i, run_i, oh, gm);
    (void)prof_i;
  }
}

static void telemBegin() {
  xTaskCreatePinnedToCore(telemTask, "telem", 6144, nullptr, 1, &g_telem_task, 0);
}

// ---------------- setup / loop ----------------
void setup() {
  setupEscPwmMinFirst();

  hallSetup();

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  Serial.begin(SERIAL_BAUD);
  delay(200);
#if FEATURE_BLE_DEFAULT
  bleBegin();
  delay(50);
#else
  hostPrintln("# BLE skipped (FEATURE_BLE_DEFAULT=0)");
#endif

  prefs.begin("escctl", false);
  loadMapsFromNvs();
  loadGainMapsFromNvs();

  spi->begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  spi->beginTransaction(SPISettings(ENC_SPI_HZ, MSBFIRST, SPI_MODE1));
  spiFrame(buildReadCmd(REG_ERRFL));
  spiFrame(buildReadCmd(REG_NOP));
  // 采集+使用 @2k；输出任务只读快照；遥测低优异步
  encoderBegin();
  controlOutputBegin();
  telemBegin();

  hostPrintln("# AS5047P + ESC Ready (ESP32-S3)");
  hostPrintf("# mode=%s ble=%s wifi=off\n",
             flight_mode ? "FLIGHT" : "TEST", bleIsEnabled() ? "on" : "off");
  hostPrintln("# 数据流: 采集2k → 使用2k(求速全用) → 输出400(最新RPM→ESC)");
  hostPrintln("# HARD: no Serial/hostPrintf in 2k encSampleCb or 400Hz motorCtrl (async log + telemTask)");
  hostPrintf("# USE_FIXED_POINT=%d (0=float main; 1=Q16/ring backup; see README_Q16_BACKUP.md)\n", USE_FIXED_POINT);
  hostPrintf("# collect/use_hz=%lu out_hz=%lu esc_pwm_hz=%lu usb_telem_hz=%lu (frames/s) baud=%lu pin=%d\n",
             (unsigned long)ENC_SAMPLE_HZ, (unsigned long)CTRL_HZ, (unsigned long)esc_pwm_hz,
             (unsigned long)USB_TELEM_HZ, (unsigned long)SERIAL_BAUD, PIN_ESC_PWM);
  hostPrintf("# 采集+使用: esp_timer @%luHz; 输出: Core1 @%luHz; USB telemTask %luHz Serial.write\n",
             (unsigned long)ENC_SAMPLE_HZ, (unsigned long)CTRL_HZ, (unsigned long)USB_TELEM_HZ);
  hostPrintf("# BLE name=%s enabled=%d rate=%u (BLE/FLIGHT/CORE/TELEM?)\n",
             BLE_DEVICE_NAME, bleIsEnabled() ? 1 : 0, (unsigned)BLE_TELEM_HZ_DEFAULT);
  hostPrintf("# GAIN schedule=%s points=%d | rpm_max=%.0f | ESC min=%uus profile=%s\n",
             gain_schedule_on ? "ON" : "OFF", activeGainMap().n, (double)target_rpm_max,
             (unsigned)ESC_PULSE_MIN_US, profileName());
  hostPrintln("# cmds: PING ENC? CTRL? TELEM? LOAD? CORE? FIXED? BLE|FLIGHT|HOST|TUNE|TEST START|STOP|?");
  hostPrintln("# TEST: smoke2000|step7500|idle (onboard; host only START/STOP; no keepalive)");
}

void loop() {
  g_loop_core_id = (uint32_t)xPortGetCoreID();
  g_loop_iters++;
  pollSerialCommands();
  hostDrainAsyncLog();  // 命令侧旁路再排一拍；主遥测在 telemTask
  vTaskDelay(1);
}
