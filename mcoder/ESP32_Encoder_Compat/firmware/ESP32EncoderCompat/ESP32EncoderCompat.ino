/************************************************
 * ESP32-S3 磁编码器兼容测试固件
 * 支持 AS5047P 与 TLE5012B(5012)，编译宏默认 + 串口运行时切换
 *
 * 控制用输出：固定 1600 Hz 过采样 → 4 点短窗平均 → 400 Hz 输出
 *
 * 切换方式：
 *   1) 编译宏 DEFAULT_SENSOR：0=AS5047P，1=TLE5012B
 *   2) 串口命令（921600）：
 *        S0 或 sensor as5047p
 *        S1 或 sensor tle5012b / sensor 5012
 *        ?  打印当前传感器
 *
 * 引脚默认：CS10 SCLK12 MISO13 MOSI11，VCC=3.3V
 * 协议：t_ms,raw,deg,rad,rpm,ef,agc,magL,magH,rate_hz
 *
 * 注意：两芯片物理接线不同（5012 需 MOSI∥MISO→DATA），换传感器时请改线。
 ************************************************/
#include <math.h>
#include "as5047p_driver.h"
#include "tle5012b_driver.h"

#ifndef DEFAULT_SENSOR
#define DEFAULT_SENSOR 1
#endif

// ---- 固定：1600 Hz 过采样 → 400 Hz 控制输出 ----
#define CTRL_HZ      400
#define OVERSAMPLE   4
#define MAX_JUMP_DEG 45.0f
#define SERIAL_BAUD  921600

static const uint32_t SAMPLE_HZ = (uint32_t)CTRL_HZ * (uint32_t)OVERSAMPLE;
static const uint32_t SAMPLE_US = 1000000UL / SAMPLE_HZ;

static const int PIN_CS   = 10;
static const int PIN_SCLK = 12;
static const int PIN_MISO = 13;
static const int PIN_MOSI = 11;

enum SensorId : uint8_t {
  SENSOR_AS5047P = 0,
  SENSOR_TLE5012B = 1,
};

as5047p::Driver g_as;
tle5012b::Driver g_tle;
SensorId g_sensor = (SensorId)DEFAULT_SENSOR;

float last_raw_deg = NAN;
float last_out_deg = NAN;
uint32_t last_out_us = 0;
float rpm_filt = 0.0f;
String rx_line;

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

const char* sensorName(SensorId id) {
  return (id == SENSOR_TLE5012B) ? "TLE5012B" : "AS5047P";
}

void applySensor(SensorId id, bool announce) {
  g_sensor = id;
  last_raw_deg = NAN;
  last_out_deg = NAN;
  last_out_us = 0;
  rpm_filt = 0.0f;

  if (id == SENSOR_TLE5012B) {
    g_tle.pin_cs = PIN_CS;
    g_tle.pin_sclk = PIN_SCLK;
    g_tle.pin_miso = PIN_MISO;
    g_tle.pin_mosi = PIN_MOSI;
    g_tle.begin();
  } else {
    pinMode(PIN_MOSI, OUTPUT);
    digitalWrite(PIN_MOSI, HIGH);
    g_as.pin_cs = PIN_CS;
    g_as.pin_sclk = PIN_SCLK;
    g_as.pin_miso = PIN_MISO;
    g_as.pin_mosi = PIN_MOSI;
    g_as.begin();
  }

  if (announce) {
    Serial.printf("# SENSOR=%s (%u)\n", sensorName(id), (unsigned)id);
    Serial.printf("# sample_hz=%lu ctrl_hz=%u oversample=%u baud=%lu\n",
                  (unsigned long)SAMPLE_HZ,
                  (unsigned)CTRL_HZ,
                  (unsigned)OVERSAMPLE,
                  (unsigned long)SERIAL_BAUD);
    Serial.println("# format: t_ms,raw,deg,rad,rpm,ef,agc,magL,magH,rate_hz");
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "?" || cmd == "help") {
    Serial.printf("# current SENSOR=%s (%u); cmds: S0/S1, sensor as5047p|tle5012b|5012\n",
                  sensorName(g_sensor), (unsigned)g_sensor);
    return;
  }
  if (cmd == "s0" || cmd == "sensor as5047p" || cmd == "sensor 0") {
    applySensor(SENSOR_AS5047P, true);
    return;
  }
  if (cmd == "s1" || cmd == "sensor tle5012b" || cmd == "sensor 5012" ||
      cmd == "sensor mt5012" || cmd == "sensor 1") {
    applySensor(SENSOR_TLE5012B, true);
    return;
  }
  Serial.printf("# unknown cmd: %s\n", cmd.c_str());
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx_line.length() > 0) {
        handleCommand(rx_line);
        rx_line = "";
      }
    } else {
      rx_line += c;
      if (rx_line.length() > 64) rx_line = "";
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);

  g_as.pin_cs = PIN_CS;
  g_as.pin_sclk = PIN_SCLK;
  g_as.pin_miso = PIN_MISO;
  g_as.pin_mosi = PIN_MOSI;

  g_tle.pin_cs = PIN_CS;
  g_tle.pin_sclk = PIN_SCLK;
  g_tle.pin_miso = PIN_MISO;
  g_tle.pin_mosi = PIN_MOSI;

  Serial.println("# ESP32 Encoder Compat Ready");
  applySensor(g_sensor, true);
}

void loop() {
  pollSerialCommands();

  static uint32_t next_us = 0;
  static float win[OVERSAMPLE];
  static uint8_t win_n = 0;
  static uint16_t last_raw = 0;
  static uint8_t last_ef = 0;
  static int last_agc = -1;
  static uint8_t last_magL = 0;
  static uint8_t last_magH = 0;
  static bool last_is_tle = false;

  uint32_t now_us = micros();
  if ((int32_t)(now_us - next_us) < 0) return;
  next_us += SAMPLE_US;
  if ((int32_t)(now_us - next_us) > (int32_t)(SAMPLE_US * 4)) {
    next_us = now_us + SAMPLE_US;
  }

  uint16_t raw = 0;
  float deg_inst = 0;
  uint8_t ef = 0, magL = 0, magH = 0;
  int agc = -1;
  bool is_tle = (g_sensor == SENSOR_TLE5012B);

  if (is_tle) {
    tle5012b::Sample s = g_tle.readAngle();
    raw = s.raw15;
    deg_inst = g_tle.rawToDeg(raw);
    ef = (s.sys_ok && s.if_ok) ? 0 : 1;
    agc = -1;
    magL = s.angle_valid ? 0 : 1;
    magH = 0;
  } else {
    as5047p::Sample s = g_as.readAngle();
    raw = s.raw;
    deg_inst = g_as.rawToDeg(raw);
    ef = s.ef ? 1 : 0;
    agc = (int)s.agc;
    magL = s.mag_low ? 1 : 0;
    magH = s.mag_high ? 1 : 0;
  }

  // 切换传感器后重置滤波窗
  if (is_tle != last_is_tle) {
    win_n = 0;
    last_raw_deg = NAN;
    last_out_deg = NAN;
    last_is_tle = is_tle;
  }

  float deg_use = deg_inst;
  if (!isnan(last_raw_deg)) {
    float d = unwrapDeltaDeg(deg_inst, last_raw_deg);
    if (fabsf(d) > MAX_JUMP_DEG) {
      deg_use = last_raw_deg;
    }
  }
  last_raw_deg = deg_use;

  last_raw = raw;
  last_ef = ef;
  last_agc = agc;
  last_magL = magL;
  last_magH = magH;

  win[win_n++] = deg_use;
  if (win_n < OVERSAMPLE) return;

  win_n = 0;
  float base = win[0];
  float sum = base;
  for (uint8_t i = 1; i < OVERSAMPLE; i++) {
    sum += base + unwrapDeltaDeg(win[i], base);
  }
  float deg = wrap360(sum / (float)OVERSAMPLE);
  float rad = deg * (6.28318530717958647692f / 360.0f);

  uint16_t raw_out;
  if (is_tle) {
    raw_out = (uint16_t)lroundf(deg * 32768.0f / 360.0f) & 0x7FFF;
  } else {
    raw_out = (uint16_t)lroundf(deg * 16384.0f / 360.0f) & 0x3FFF;
  }

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
                last_agc,
                (unsigned)last_magL,
                (unsigned)last_magH,
                (unsigned)CTRL_HZ);
}
