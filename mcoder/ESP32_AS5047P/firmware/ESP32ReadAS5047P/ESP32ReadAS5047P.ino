/************************************************
 * ESP32-S3 读取 AS5047P 磁编码器 (SPI)
 * 依据 ams AS5047P Datasheet [v1-01]：
 *   - SPI Mode1 (CPOL=0, CPHA=1), MSB first, ≤10MHz
 *   - 16-bit 命令帧：PARC(偶校验) + R/W + ADDR[13:0]
 *   - 流水线读（Figure 15）：本帧 MISO = 上一命令的数据
 *   - 读 ANGLECOM (0x3FFF) = DAEC 补偿后的 14-bit 角度
 *   - CSn 帧间隔 ≥350ns；上电后等待 t_pon ≥10ms
 *
 * 控制用输出：默认 1600 Hz 过采样 → 短窗平均 → 400 Hz 输出
 * 供电见 README：对接 ESP32 请用 3.3V。
 ************************************************/
#include <SPI.h>
#include <math.h>

// ---- 引脚（可按板子修改）----
static const int PIN_CS   = 10;
static const int PIN_SCLK = 12;
static const int PIN_MISO = 13;
static const int PIN_MOSI = 11;

// ---- 固定：1600 Hz 过采样 → 400 Hz 控制输出 ----
#define CTRL_HZ     400
#define OVERSAMPLE  4   // 必须为 4：内部 1600 Hz，输出 400 Hz

static const uint32_t SAMPLE_HZ = (uint32_t)CTRL_HZ * (uint32_t)OVERSAMPLE;
static const uint32_t SAMPLE_US = 1000000UL / SAMPLE_HZ;

// 单次过采样步长上允许的最大跳变（度）。超则剔除，沿用上次。
// 45°@1600Hz ≈ 12000 RPM，覆盖常见电机；噪声尖峰通常更大。
#define MAX_JUMP_DEG 45.0f

// 串口：400 Hz 行输出需高于 115200（约 18~24 KB/s）
#define SERIAL_BAUD 921600

// 寄存器地址 (14-bit ADDR)
static const uint16_t REG_NOP      = 0x0000;
static const uint16_t REG_ERRFL    = 0x0001;
static const uint16_t REG_DIAAGC   = 0x3FFC;
static const uint16_t REG_ANGLECOM = 0x3FFF;  // DAEC 补偿角

static const float TWO_PI_F = 6.28318530717958647692f;

SPIClass* spi = &SPI;

float last_raw_deg = NAN;   // 过采样用：上一有效瞬时角
float last_out_deg = NAN;   // 控制输出用：上一帧滤波角
uint32_t last_out_us = 0;
float rpm_filt = 0.0f;

uint8_t last_agc = 0;
bool last_mag_low = false;
bool last_mag_high = false;

// 须在任何函数之前定义，避免 Arduino 自动插入原型时找不到类型
struct AngleSample {
  uint16_t raw;
  bool ef;
  bool parity_ok;
  uint8_t agc;
  bool mag_low;
  bool mag_high;
};

// 偶校验位：使「PAR + 低15位」中 1 的个数为偶数 → PAR = popcount(low15) & 1
uint16_t evenParityBit(uint16_t value15) {
  uint16_t x = value15 & 0x7FFF;
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

// 构造读命令：PARC | R/W=1 | ADDR[13:0]
uint16_t buildReadCmd(uint16_t addr14) {
  uint16_t cmd = (1u << 14) | (addr14 & 0x3FFF);
  cmd |= (evenParityBit(cmd) << 15);
  return cmd;
}

bool checkEvenParity(uint16_t frame) {
  return evenParityBit(frame) == ((frame >> 15) & 1);
}

uint16_t spiTransfer16(uint16_t data) {
  return spi->transfer16(data);
}

uint16_t spiFrame(uint16_t mosi) {
  digitalWrite(PIN_CS, LOW);
  uint16_t miso = spiTransfer16(mosi);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(1);
  return miso;
}

void parseDiaagc(uint16_t dia) {
  if (!checkEvenParity(dia)) return;
  last_agc = dia & 0xFF;
  last_mag_high = (dia >> 10) & 1;  // MAGH
  last_mag_low  = (dia >> 11) & 1;  // MAGL
}

// 流水线连续读 ANGLECOM；约每 0.5s 插读一次 DIAAGC
AngleSample readAngleCom() {
  static bool primed = false;
  static const uint16_t CMD_ANGLE = buildReadCmd(REG_ANGLECOM);
  static const uint16_t CMD_DIA   = buildReadCmd(REG_DIAAGC);
  static uint16_t diag_div = 0;

  AngleSample s = {};

  if (!primed) {
    spiFrame(CMD_DIA);
    parseDiaagc(spiFrame(CMD_ANGLE));
    primed = true;
  }

  // 过采样后约每 SAMPLE_HZ/2 次刷新诊断（~0.5s）
  if (++diag_div >= (SAMPLE_HZ / 2)) {
    diag_div = 0;
    spiFrame(CMD_DIA);
    parseDiaagc(spiFrame(CMD_ANGLE));
  }

  uint16_t rx = spiFrame(CMD_ANGLE);
  s.parity_ok = checkEvenParity(rx);
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

float wrap360(float deg) {
  deg = fmodf(deg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  return deg;
}

void setup() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  Serial.begin(SERIAL_BAUD);
  delay(20);

  spi->begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));

  spiFrame(buildReadCmd(REG_ERRFL));
  spiFrame(buildReadCmd(REG_NOP));

  Serial.println("# AS5047P Ready (ESP32-S3) per datasheet SPI Mode1");
  Serial.printf("# sample_hz=%lu ctrl_hz=%u oversample=%u sample_us=%lu baud=%lu\n",
                (unsigned long)SAMPLE_HZ,
                (unsigned)CTRL_HZ,
                (unsigned)OVERSAMPLE,
                (unsigned long)SAMPLE_US,
                (unsigned long)SERIAL_BAUD);
  Serial.println("# format: t_ms,raw,deg,rad,rpm,ef,agc,magL,magH,rate_hz");
  Serial.println("# note: deg/rad/rpm 为过采样短窗滤波后的控制用值；rate_hz=ctrl 输出频率");
}

void loop() {
  static uint32_t next_us = 0;
  static float win[OVERSAMPLE];
  static uint8_t win_n = 0;
  static uint16_t last_raw = 0;
  static uint8_t last_ef = 0;
  static uint8_t last_agc_out = 0;
  static uint8_t last_magL = 0;
  static uint8_t last_magH = 0;

  uint32_t now_us = micros();
  if ((int32_t)(now_us - next_us) < 0) return;
  // 追赶调度，避免 loop 抖动导致长期漂移
  next_us += SAMPLE_US;
  if ((int32_t)(now_us - next_us) > (int32_t)(SAMPLE_US * 4)) {
    next_us = now_us + SAMPLE_US;
  }

  AngleSample s = readAngleCom();
  float deg_inst = (s.raw * 360.0f) / 16384.0f;

  // 跳变剔除（unwrap 后 |Δθ| 过大则沿用上次）
  float deg_use = deg_inst;
  if (!isnan(last_raw_deg)) {
    float d = unwrapDeltaDeg(deg_inst, last_raw_deg);
    if (fabsf(d) > MAX_JUMP_DEG) {
      deg_use = last_raw_deg;
    }
  }
  last_raw_deg = deg_use;

  last_raw = s.raw;
  last_ef = s.ef ? 1 : 0;
  last_agc_out = s.agc;
  last_magL = s.mag_low ? 1 : 0;
  last_magH = s.mag_high ? 1 : 0;

  win[win_n++] = deg_use;
  if (win_n < OVERSAMPLE) return;

  // ---- 满窗：短窗滑动平均 → 400 Hz 控制帧 ----
  win_n = 0;
  float sum = 0.0f;
  // 相对第一点 unwrap 再平均，避免跨 0° 时算术平均出错
  float base = win[0];
  sum = base;
  for (uint8_t i = 1; i < OVERSAMPLE; i++) {
    sum += base + unwrapDeltaDeg(win[i], base);
  }
  float deg = wrap360(sum / (float)OVERSAMPLE);
  float rad = deg * (TWO_PI_F / 360.0f);
  // 输出用 raw：由滤波角反推（UI 兼容）
  uint16_t raw_out = (uint16_t)lroundf(deg * 16384.0f / 360.0f) & 0x3FFF;

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

  Serial.printf("%lu,%u,%.3f,%.6f,%.2f,%u,%u,%u,%u,%u\n",
                (unsigned long)now_ms,
                (unsigned)raw_out,
                deg,
                rad,
                rpm,
                (unsigned)last_ef,
                (unsigned)last_agc_out,
                (unsigned)last_magL,
                (unsigned)last_magH,
                (unsigned)CTRL_HZ);
}
