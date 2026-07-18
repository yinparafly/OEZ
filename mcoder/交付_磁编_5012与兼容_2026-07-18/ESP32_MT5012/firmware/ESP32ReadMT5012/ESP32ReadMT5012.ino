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
 * 接线（与上午 AS5047P 引脚对齐，见 README）：
 *   CS=GPIO10, SCLK=GPIO12, MOSI=GPIO11, MISO=GPIO13
 *   模块仅 1 根 DATA → ESP32 MOSI 与 MISO 在模块侧短接至 DATA
 ************************************************/
#include <SPI.h>
#include <math.h>

static const int PIN_CS   = 10;
static const int PIN_SCLK = 12;
static const int PIN_MISO = 13;
static const int PIN_MOSI = 11;

static const uint32_t SAMPLE_MS = 10;
static const uint32_t SPI_HZ    = 500000;  // 杜邦线建议 ≤1MHz；手册 push-pull 最高 8Mbit/s

// Command Word: Read AVAL (addr 0x02), 1 data word
static const uint16_t CMD_READ_ANGLE = 0x8021;
static const uint16_t CMD_READ_STAT  = 0x8001;  // STAT @ 0x00

static const float TWO_PI_F = 6.28318530717958647692f;
static const float DEG_PER_LSB = 360.0f / 32768.0f;

SPIClass* spi = &SPI;

float last_deg = NAN;
uint32_t last_ms = 0;
float rpm_filt = 0.0f;

struct AngleSample {
  uint16_t raw15;   // 0..32767
  uint16_t safety;
  bool angle_valid; // safety bit12
  bool sys_ok;      // safety bit14
  bool if_ok;       // safety bit13
  bool no_reset;    // safety bit15
};

uint16_t spiTransfer16(uint16_t data) {
  return spi->transfer16(data);
}

// 半双工读：发命令后释放 MOSI，由传感器驱动 DATA（与 Arduino 样本同法）
AngleSample readAVAL() {
  AngleSample s = {};

  digitalWrite(PIN_CS, LOW);
  spiTransfer16(CMD_READ_ANGLE);

  // 切换方向：主机不再驱动 DATA
  pinMode(PIN_MOSI, INPUT);
  delayMicroseconds(1);  // twr_delay ≥ 130ns

  s.raw15 = spiTransfer16(0xFFFF) & 0x7FFF;
  s.safety = spiTransfer16(0xFFFF);

  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_MOSI, OUTPUT);
  digitalWrite(PIN_MOSI, HIGH);
  delayMicroseconds(1);  // tCSoff ≥ 600ns

  // Safety Word：1=正常（除 CRC 外）
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

void setup() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_MOSI, OUTPUT);
  digitalWrite(PIN_MOSI, HIGH);

  Serial.begin(115200);
  // t_Pon typ 57ms；上电后勿写寄存器
  delay(80);

  spi->begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE1));

  // 读一次 STAT 清部分状态闩锁（可选）
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
  Serial.println("# format: t_ms,raw,deg,rad,rpm,ef,agc,magL,magH");
  Serial.println("# note: raw=15bit AVAL; ef=!(sys&if); agc=N/A(-1); magL=!angle_valid; magH=0");
}

void loop() {
  static uint32_t next_ms = 0;
  uint32_t now = millis();
  if ((int32_t)(now - next_ms) < 0) return;
  next_ms = now + SAMPLE_MS;

  AngleSample s = readAVAL();
  float deg = s.raw15 * DEG_PER_LSB;
  float rad = s.raw15 * TWO_PI_F / 32768.0f;

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

  // 与 AS5047P UI 列对齐：ef / agc / magL / magH
  uint8_t ef = (s.sys_ok && s.if_ok) ? 0 : 1;
  int agc = -1;  // TLE5012B 无 AGC 字段
  uint8_t magL = s.angle_valid ? 0 : 1;
  uint8_t magH = 0;

  Serial.printf("%lu,%u,%.3f,%.6f,%.2f,%u,%d,%u,%u\n",
                (unsigned long)now,
                (unsigned)s.raw15,
                deg,
                rad,
                rpm,
                (unsigned)ef,
                agc,
                (unsigned)magL,
                (unsigned)magH);
}
