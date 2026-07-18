#pragma once
/************************************************
 * AS5047P SPI 驱动（自上午 ESP32_AS5047P 抽取，独立副本）
 * 不修改原工程。
 ************************************************/
#include <Arduino.h>
#include <SPI.h>

namespace as5047p {

static const uint16_t REG_NOP      = 0x0000;
static const uint16_t REG_ERRFL    = 0x0001;
static const uint16_t REG_DIAAGC   = 0x3FFC;
static const uint16_t REG_ANGLECOM = 0x3FFF;

struct Sample {
  uint16_t raw;
  bool ef;
  bool parity_ok;
  uint8_t agc;
  bool mag_low;
  bool mag_high;
};

inline uint16_t evenParityBit(uint16_t value15) {
  uint16_t x = value15 & 0x7FFF;
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

inline uint16_t buildReadCmd(uint16_t addr14) {
  uint16_t cmd = (1u << 14) | (addr14 & 0x3FFF);
  cmd |= (evenParityBit(cmd) << 15);
  return cmd;
}

inline bool checkEvenParity(uint16_t frame) {
  return evenParityBit(frame) == ((frame >> 15) & 1);
}

class Driver {
 public:
  int pin_cs = 10;
  int pin_sclk = 12;
  int pin_miso = 13;
  int pin_mosi = 11;
  uint32_t spi_hz = 2000000;
  SPIClass* spi = &SPI;

  uint8_t last_agc = 0;
  bool last_mag_low = false;
  bool last_mag_high = false;
  bool primed = false;
  uint16_t diag_div = 0;

  void begin() {
    pinMode(pin_cs, OUTPUT);
    digitalWrite(pin_cs, HIGH);
    delay(20);
    spi->begin(pin_sclk, pin_miso, pin_mosi, -1);
    spi->beginTransaction(SPISettings(spi_hz, MSBFIRST, SPI_MODE1));
    spiFrame(buildReadCmd(REG_ERRFL));
    spiFrame(buildReadCmd(REG_NOP));
    primed = false;
    diag_div = 0;
  }

  uint16_t spiFrame(uint16_t mosi) {
    digitalWrite(pin_cs, LOW);
    uint16_t miso = spi->transfer16(mosi);
    digitalWrite(pin_cs, HIGH);
    delayMicroseconds(1);
    return miso;
  }

  void parseDiaagc(uint16_t dia) {
    if (!checkEvenParity(dia)) return;
    last_agc = dia & 0xFF;
    last_mag_high = (dia >> 10) & 1;
    last_mag_low = (dia >> 11) & 1;
  }

  Sample readAngle() {
    static const uint16_t CMD_ANGLE = buildReadCmd(REG_ANGLECOM);
    static const uint16_t CMD_DIA = buildReadCmd(REG_DIAAGC);
    Sample s = {};

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
    s.parity_ok = checkEvenParity(rx);
    s.ef = (rx >> 14) & 1;
    s.raw = rx & 0x3FFF;
    s.agc = last_agc;
    s.mag_low = last_mag_low;
    s.mag_high = last_mag_high;
    return s;
  }

  float rawToDeg(uint16_t raw14) const {
    return (raw14 * 360.0f) / 16384.0f;
  }

  float rawToRad(uint16_t raw14) const {
    return (raw14 * 6.28318530717958647692f) / 16384.0f;
  }
};

}  // namespace as5047p
