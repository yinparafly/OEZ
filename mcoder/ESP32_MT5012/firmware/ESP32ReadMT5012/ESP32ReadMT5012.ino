/************************************************
 * ESP32-S3 读取 TLE5012B（市面常称 5012 / MT5012）
 *
 * 依据 Infineon TLE5012B Datasheet Rev.2.1：
 *   - 3 线 SSC（SPI 兼容）：SCK / CSQ / DATA（半双工）
 *   - 时序：MSB first；DATA 上升沿输出、下降沿采样 → SPI Mode1
 *   - 读角命令字 0x8021：RW=1, Lock=0000, UPD=0, ADDR=0x02(AVAL), ND=1
 *   - 帧：COMMAND + DATA + SAFETY；t_wr_delay≥130ns；t_CSoff≥600ns
 *   - 15-bit 绝对角，分辨率 360°/32768 ≈ 0.010986°
 *   - 供电 VDD 3.0~5.5V；t_Pon typ 57ms
 *
 * 控制用输出：固定 1600 Hz 过采样 → 4 点短窗平均 → 400 Hz 输出
 *
 * 接线（与 AS5047P 引脚对齐，见 README）：
 *   CS=GPIO10, SCLK=GPIO12, MOSI=GPIO11, MISO=GPIO13
 *   模块仅 1 根 DATA → ESP32 MOSI 与 MISO 在模块侧短接至 DATA
 ************************************************/
#include <SPI.h>
#include <math.h>

static const int PIN_CS   = 10;
static const int PIN_SCLK = 12;
static const int PIN_MISO = 13;
static const int PIN_MOSI = 11;

// ---- 固定：1600 Hz 过采样 → 400 Hz 控制输出 ----
#define CTRL_HZ     400
#define OVERSAMPLE  4
#define MAX_JUMP_DEG 45.0f
#define SERIAL_BAUD 921600

static const uint32_t SAMPLE_HZ = (uint32_t)CTRL_HZ * (uint32_t)OVERSAMPLE;  // 1600
static const uint32_t SAMPLE_US = 1000000UL / SAMPLE_HZ;                     // 625
static const uint32_t SPI_HZ    = 500000;  // 杜邦线建议 ≤1MHz；手册 push-pull 最高 8Mbit/s

static const uint16_t CMD_READ_ANGLE = 0x8021;
static const uint16_t CMD_READ_STAT  = 0x8001;

static const float TWO_PI_F = 6.28318530717958647692f;
static const float DEG_PER_LSB = 360.0f / 32768.0f;

SPIClass* spi = &SPI;

float last_raw_deg = NAN;
float last_out_deg = NAN;
uint32_t last_out_us = 0;
float rpm_filt = 0.0f;

struct AngleSample {
  uint16_t raw15;
  uint16_t safety;
  bool angle_valid;
  bool sys_ok;
  bool if_ok;
  bool no_reset;
};

uint16_t spiTransfer16(uint16_t data) {
  return spi->transfer16(data);
}

AngleSample readAVAL() {
  AngleSample s = {};

  digitalWrite(PIN_CS, LOW);
  spiTransfer16(CMD_READ_ANGLE);

  pinMode(PIN_MOSI, INPUT);
  delayMicroseconds(1);

  s.raw15 = spiTransfer16(0xFFFF) & 0x7FFF;
  s.safety = spiTransfer16(0xFFFF);

  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_MOSI, OUTPUT);
  digitalWrite(PIN_MOSI, HIGH);
  delayMicroseconds(1);

  s.no_reset    = (s.safety >> 15) & 1;
  s.sys_ok      = (s.safety >> 14) & 1;
  s.if_ok       = (s.safety >> 13) & 1;
  s.angle_valid = (s.safety >> 12) & 1;
  return s;
}

float unwrapDeltaDeg(float cur, float prev) {
  float d = cur - prev;
  if (d > 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

float wrap360(float deg) {
  deg = fmodf(deg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  return deg;
}

void setup() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_MOSI, OUTPUT);
  digitalWrite(PIN_MOSI, HIGH);

  Serial.begin(SERIAL_BAUD);
  delay(80);

  spi->begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE1));

  digitalWrite(PIN_CS, LOW);
  spiTransfer16(CMD_READ_STAT);
  pinMode(PIN_MOSI, INPUT);
  delayMicroseconds(1);
  (void)spiTransfer16(0xFFFF);
  (void)spiTransfer16(0xFFFF);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_MOSI, OUTPUT);
  digitalWrite(PIN_MOSI, HIGH);
  delayMicroseconds(1);

  Serial.println("# TLE5012B Ready (ESP32-S3) SSC Mode1 half-duplex");
  Serial.printf("# sample_hz=%lu ctrl_hz=%u oversample=%u sample_us=%lu baud=%lu\n",
                (unsigned long)SAMPLE_HZ,
                (unsigned)CTRL_HZ,
                (unsigned)OVERSAMPLE,
                (unsigned long)SAMPLE_US,
                (unsigned long)SERIAL_BAUD);
  Serial.println("# format: t_ms,raw,deg,rad,rpm,ef,agc,magL,magH,rate_hz");
  Serial.println("# note: raw=15bit AVAL; ef=!(sys&if); agc=N/A(-1); magL=!angle_valid; magH=0");
  Serial.println("# note: deg/rad/rpm 为 1600Hz 过采样短窗滤波后的 400Hz 控制用值");
}

void loop() {
  static uint32_t next_us = 0;
  static float win[OVERSAMPLE];
  static uint8_t win_n = 0;
  static uint16_t last_raw15 = 0;
  static uint8_t last_ef = 0;
  static uint8_t last_magL = 0;

  uint32_t now_us = micros();
  if ((int32_t)(now_us - next_us) < 0) return;
  next_us += SAMPLE_US;
  if ((int32_t)(now_us - next_us) > (int32_t)(SAMPLE_US * 4)) {
    next_us = now_us + SAMPLE_US;
  }

  AngleSample s = readAVAL();
  float deg_inst = s.raw15 * DEG_PER_LSB;

  float deg_use = deg_inst;
  if (!isnan(last_raw_deg)) {
    float d = unwrapDeltaDeg(deg_inst, last_raw_deg);
    if (fabsf(d) > MAX_JUMP_DEG) {
      deg_use = last_raw_deg;
    }
  }
  last_raw_deg = deg_use;

  last_raw15 = s.raw15;
  last_ef = (s.sys_ok && s.if_ok) ? 0 : 1;
  last_magL = s.angle_valid ? 0 : 1;

  win[win_n++] = deg_use;
  if (win_n < OVERSAMPLE) return;

  win_n = 0;
  float base = win[0];
  float sum = base;
  for (uint8_t i = 1; i < OVERSAMPLE; i++) {
    sum += base + unwrapDeltaDeg(win[i], base);
  }
  float deg = wrap360(sum / (float)OVERSAMPLE);
  float rad = deg * (TWO_PI_F / 360.0f);
  uint16_t raw_out = (uint16_t)lroundf(deg * 32768.0f / 360.0f) & 0x7FFF;

  uint32_t now_ms = millis();
  float rpm = 0.0f;
  if (!isnan(last_out_deg) && now_us > last_out_us) {
    float dt = (now_us - last_out_us) * 1e-6f;
    if (dt > 0.0002f) {
      float ddeg = unwrapDeltaDeg(deg, last_out_deg);
      float rpm_inst = (ddeg / 360.0f) / dt * 60.0f;
      rpm_filt = 0.7f * rpm_filt + 0.3f * rpm_inst;
      rpm = rpm_filt;
    }
  }
  last_out_deg = deg;
  last_out_us = now_us;

  Serial.printf("%lu,%u,%.3f,%.6f,%.2f,%u,%d,%u,%u,%u\n",
                (unsigned long)now_ms,
                (unsigned)raw_out,
                deg,
                rad,
                rpm,
                (unsigned)last_ef,
                -1,
                (unsigned)last_magL,
                0u,
                (unsigned)CTRL_HZ);
}
