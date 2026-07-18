/************************************************
 * ESP32-S3 读取 AS5047P 磁编码器 (SPI)
 * 依据 ams AS5047P Datasheet [v1-01]：
 *   - SPI Mode1 (CPOL=0, CPHA=1), MSB first, ≤10MHz
 *   - 16-bit 命令帧：PARC(偶校验) + R/W + ADDR[13:0]
 *   - 流水线读（Figure 15）：本帧 MISO = 上一命令的数据
 *   - 读 ANGLECOM (0x3FFF) = DAEC 补偿后的 14-bit 角度
 *   - CSn 帧间隔 ≥350ns；上电后等待 t_pon ≥10ms
 *
 * 供电见 README：模块原理图为 5V 接法；对接 ESP32 请用
 * 3.3V 模式（VDD 与 VDD3V3 短接）或加电平转换。
 ************************************************/
#include <SPI.h>
#include <math.h>

// ---- 引脚（可按板子修改）----
static const int PIN_CS   = 10;
static const int PIN_SCLK = 12;
static const int PIN_MISO = 13;
static const int PIN_MOSI = 11;

// 采样周期 (ms) → 100Hz
static const uint32_t SAMPLE_MS = 10;

// 寄存器地址 (14-bit ADDR)
static const uint16_t REG_NOP      = 0x0000;
static const uint16_t REG_ERRFL    = 0x0001;
static const uint16_t REG_DIAAGC   = 0x3FFC;
static const uint16_t REG_ANGLECOM = 0x3FFF;  // DAEC 补偿角

static const float TWO_PI_F = 6.28318530717958647692f;

SPIClass* spi = &SPI;

float last_deg = NAN;
uint32_t last_ms = 0;
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
// ANGLECOM→0xFFFF（与样本程序一致）
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

// 一次 16-bit SPI 帧（手册：CSn 下降沿开始，上升沿结束并执行命令）
uint16_t spiFrame(uint16_t mosi) {
  digitalWrite(PIN_CS, LOW);
  // t_L ≥ 350ns（GPIO+SPI 启动已满足）
  uint16_t miso = spiTransfer16(mosi);
  digitalWrite(PIN_CS, HIGH);
  // t_CSn ≥ 350ns
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
    // 灌入管道：先读 DIAAGC，再对齐到 ANGLECOM
    spiFrame(CMD_DIA);
    parseDiaagc(spiFrame(CMD_ANGLE));  // MISO = DIAAGC
    primed = true;
  }

  // 每隔约 50 次采样刷新磁场诊断（不打断主角度流水线过多）
  if (++diag_div >= 50) {
    diag_div = 0;
    spiFrame(CMD_DIA);                 // 丢掉一帧旧 ANGLE
    parseDiaagc(spiFrame(CMD_ANGLE));  // MISO = DIAAGC
  }

  uint16_t rx = spiFrame(CMD_ANGLE);   // MISO = ANGLECOM
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

void setup() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  Serial.begin(115200);
  // t_pon ≥ 10ms（手册：first valid angular data）
  delay(20);

  // ≤10MHz；杜邦线建议 1~2MHz
  spi->begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));

  // 读 ERRFL 清错误标志（读后自动清零）
  spiFrame(buildReadCmd(REG_ERRFL));
  spiFrame(buildReadCmd(REG_NOP));

  Serial.println("# AS5047P Ready (ESP32-S3) per datasheet SPI Mode1");
  Serial.println("# format: t_ms,raw,deg,rad,rpm,ef,agc,magL,magH");
}

void loop() {
  static uint32_t next_ms = 0;
  uint32_t now = millis();
  if ((int32_t)(now - next_ms) < 0) return;
  next_ms = now + SAMPLE_MS;

  AngleSample s = readAngleCom();
  float deg = (s.raw * 360.0f) / 16384.0f;
  float rad = (s.raw * TWO_PI_F) / 16384.0f;

  float rpm = 0.0f;
  if (!isnan(last_deg) && now > last_ms) {
    float dt = (now - last_ms) * 0.001f;
    if (dt > 0.0005f) {
      float ddeg = unwrapDeltaDeg(deg, last_deg);
      float rpm_inst = (ddeg / 360.0f) / dt * 60.0f;
      rpm_filt = 0.7f * rpm_filt + 0.3f * rpm_inst;
      rpm = rpm_filt;
    }
  }
  last_deg = deg;
  last_ms = now;

  Serial.printf("%lu,%u,%.3f,%.6f,%.2f,%u,%u,%u,%u\n",
                (unsigned long)now,
                (unsigned)s.raw,
                deg,
                rad,
                rpm,
                (unsigned)s.ef,
                (unsigned)s.agc,
                (unsigned)s.mag_low,
                (unsigned)s.mag_high);
}
