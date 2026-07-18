/************************************************
 * ESP32-S3 磁编码器兼容测试固件
 * 支持 AS5047P 与 TLE5012B(5012)，编译宏默认 + 串口运行时切换
 *
 * 切换方式：
 *   1) 编译宏 DEFAULT_SENSOR：0=AS5047P，1=TLE5012B
 *   2) 串口命令（115200）：
 *        S0 或 sensor as5047p
 *        S1 或 sensor tle5012b / sensor 5012
 *        ?  打印当前传感器
 *
 * 引脚默认：CS10 SCLK12 MISO13 MOSI11，VCC=3.3V
 * 协议：t_ms,raw,deg,rad,rpm,ef,agc,magL,magH
 *
 * 注意：两芯片物理接线不同（5012 需 MOSI∥MISO→DATA），换传感器时请改线。
 ************************************************/
#include <math.h>
#include "as5047p_driver.h"
#include "tle5012b_driver.h"

// ---- 编译期默认传感器：0=AS5047P，1=TLE5012B ----
#ifndef DEFAULT_SENSOR
#define DEFAULT_SENSOR 1
#endif

static const int PIN_CS   = 10;
static const int PIN_SCLK = 12;
static const int PIN_MISO = 13;
static const int PIN_MOSI = 11;
static const uint32_t SAMPLE_MS = 10;

enum SensorId : uint8_t {
  SENSOR_AS5047P = 0,
  SENSOR_TLE5012B = 1,
};

as5047p::Driver g_as;
tle5012b::Driver g_tle;
SensorId g_sensor = (SensorId)DEFAULT_SENSOR;

float last_deg = NAN;
uint32_t last_ms = 0;
float rpm_filt = 0.0f;
String rx_line;

float unwrapDeltaDeg(float cur, float prev) {
  float d = cur - prev;
  if (d > 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

const char* sensorName(SensorId id) {
  return (id == SENSOR_TLE5012B) ? "TLE5012B" : "AS5047P";
}

void applySensor(SensorId id, bool announce) {
  g_sensor = id;
  last_deg = NAN;
  last_ms = 0;
  rpm_filt = 0.0f;

  // 重新 begin：两驱动共用同一组 SPI 引脚
  if (id == SENSOR_TLE5012B) {
    g_tle.pin_cs = PIN_CS;
    g_tle.pin_sclk = PIN_SCLK;
    g_tle.pin_miso = PIN_MISO;
    g_tle.pin_mosi = PIN_MOSI;
    g_tle.begin();
  } else {
    // 恢复 MOSI 输出（5012 半双工可能把 MOSI 留在 INPUT）
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
    Serial.println("# format: t_ms,raw,deg,rad,rpm,ef,agc,magL,magH");
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
  Serial.begin(115200);
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

  static uint32_t next_ms = 0;
  uint32_t now = millis();
  if ((int32_t)(now - next_ms) < 0) return;
  next_ms = now + SAMPLE_MS;

  uint16_t raw = 0;
  float deg = 0, rad = 0;
  uint8_t ef = 0, magL = 0, magH = 0;
  int agc = -1;

  if (g_sensor == SENSOR_TLE5012B) {
    tle5012b::Sample s = g_tle.readAngle();
    raw = s.raw15;
    deg = g_tle.rawToDeg(raw);
    rad = g_tle.rawToRad(raw);
    ef = (s.sys_ok && s.if_ok) ? 0 : 1;
    agc = -1;
    magL = s.angle_valid ? 0 : 1;
    magH = 0;
  } else {
    as5047p::Sample s = g_as.readAngle();
    raw = s.raw;
    deg = g_as.rawToDeg(raw);
    rad = g_as.rawToRad(raw);
    ef = s.ef ? 1 : 0;
    agc = (int)s.agc;
    magL = s.mag_low ? 1 : 0;
    magH = s.mag_high ? 1 : 0;
  }

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

  Serial.printf("%lu,%u,%.3f,%.6f,%.2f,%u,%d,%u,%u\n",
                (unsigned long)now,
                (unsigned)raw,
                deg,
                rad,
                rpm,
                (unsigned)ef,
                agc,
                (unsigned)magL,
                (unsigned)magH);
}
