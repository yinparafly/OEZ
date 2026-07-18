#pragma once
/************************************************
 * TLE5012B (5012/MT5012) SSC 半双工驱动
 * 依据 Infineon TLE5012B Datasheet Rev.2.1
 ************************************************/
#include <Arduino.h>
#include <SPI.h>

namespace tle5012b {

static const uint16_t CMD_READ_ANGLE = 0x8021;  // AVAL @ 0x02, ND=1
static const uint16_t CMD_READ_STAT  = 0x8001;

struct Sample {
  uint16_t raw15;
  uint16_t safety;
  bool angle_valid;
  bool sys_ok;
  bool if_ok;
};

class Driver {
 public:
  int pin_cs = 10;
  int pin_sclk = 12;
  int pin_miso = 13;
  int pin_mosi = 11;
  uint32_t spi_hz = 500000;
  SPIClass* spi = &SPI;

  void begin() {
    pinMode(pin_cs, OUTPUT);
    digitalWrite(pin_cs, HIGH);
    pinMode(pin_mosi, OUTPUT);
    digitalWrite(pin_mosi, HIGH);
    delay(80);  // t_Pon typ 57ms
    spi->begin(pin_sclk, pin_miso, pin_mosi, -1);
    spi->beginTransaction(SPISettings(spi_hz, MSBFIRST, SPI_MODE1));

    digitalWrite(pin_cs, LOW);
    spi->transfer16(CMD_READ_STAT);
    pinMode(pin_mosi, INPUT);
    delayMicroseconds(1);
    (void)spi->transfer16(0xFFFF);
    (void)spi->transfer16(0xFFFF);
    digitalWrite(pin_cs, HIGH);
    pinMode(pin_mosi, OUTPUT);
    digitalWrite(pin_mosi, HIGH);
    delayMicroseconds(1);
  }

  Sample readAngle() {
    Sample s = {};
    digitalWrite(pin_cs, LOW);
    spi->transfer16(CMD_READ_ANGLE);
    pinMode(pin_mosi, INPUT);
    delayMicroseconds(1);
    s.raw15 = spi->transfer16(0xFFFF) & 0x7FFF;
    s.safety = spi->transfer16(0xFFFF);
    digitalWrite(pin_cs, HIGH);
    pinMode(pin_mosi, OUTPUT);
    digitalWrite(pin_mosi, HIGH);
    delayMicroseconds(1);

    s.sys_ok = (s.safety >> 14) & 1;
    s.if_ok = (s.safety >> 13) & 1;
    s.angle_valid = (s.safety >> 12) & 1;
    return s;
  }

  float rawToDeg(uint16_t raw15) const {
    return (raw15 * 360.0f) / 32768.0f;
  }

  float rawToRad(uint16_t raw15) const {
    return (raw15 * 6.28318530717958647692f) / 32768.0f;
  }
};

}  // namespace tle5012b
