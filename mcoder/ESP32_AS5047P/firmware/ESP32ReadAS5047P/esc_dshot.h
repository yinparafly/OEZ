/************************************************
 * ESP32-S3 DShot TX via RMT (IDF5)：DShot150/300/600
 * 与模拟 PWM 共用 GPIO；切换前须 ledcDetach。
 ************************************************/
#pragma once

#include <Arduino.h>
#include <driver/rmt_tx.h>
#include <esp_err.h>

enum DshotRate : uint16_t {
  DSHOT_OFF = 0,
  DSHOT150 = 150,
  DSHOT300 = 300,
  DSHOT600 = 600,
};

struct EscDshot {
  rmt_channel_handle_t chan = nullptr;
  rmt_encoder_handle_t encoder = nullptr;
  gpio_num_t gpio = GPIO_NUM_NC;
  DshotRate rate = DSHOT300;
  uint16_t throttle = 0;  // 0=停转；48..2047=油门
  bool ready = false;
  uint16_t last_packet = 0xFFFF;
};

static inline uint16_t dshotEncodePacket(uint16_t value, bool telemetry) {
  uint16_t packet = (uint16_t)((value << 1) | (telemetry ? 1u : 0u));
  uint16_t csum = 0;
  uint16_t csum_data = packet;
  for (int i = 0; i < 3; i++) {
    csum ^= csum_data;
    csum_data >>= 4;
  }
  csum &= 0xF;
  return (uint16_t)((packet << 4) | csum);
}

static inline void dshotBitTimings(DshotRate rate, uint16_t* t0h, uint16_t* t0l,
                                   uint16_t* t1h, uint16_t* t1l) {
  // RMT 40MHz → 1 tick = 25ns；DShot600 bit≈1.67us
  uint32_t scale = 1;
  if (rate == DSHOT300) scale = 2;
  else if (rate == DSHOT150) scale = 4;
  *t0h = (uint16_t)(25 * scale);
  *t0l = (uint16_t)(42 * scale);
  *t1h = (uint16_t)(50 * scale);
  *t1l = (uint16_t)(17 * scale);
}

static inline void dshotEnd(EscDshot& d) {
  if (d.chan) {
    rmt_disable(d.chan);
    rmt_del_channel(d.chan);
    d.chan = nullptr;
  }
  if (d.encoder) {
    rmt_del_encoder(d.encoder);
    d.encoder = nullptr;
  }
  d.ready = false;
  d.throttle = 0;
  d.last_packet = 0xFFFF;
  if (d.gpio != GPIO_NUM_NC) {
    pinMode((int)d.gpio, OUTPUT);
    digitalWrite((int)d.gpio, LOW);
  }
}

static inline void dshotStopTx(EscDshot& d) {
  if (!d.chan) return;
  rmt_disable(d.chan);
  rmt_enable(d.chan);
}

static inline bool dshotBegin(EscDshot& d, int pin, DshotRate rate) {
  dshotEnd(d);
  if (rate != DSHOT150 && rate != DSHOT300 && rate != DSHOT600) rate = DSHOT300;
  d.gpio = (gpio_num_t)pin;
  d.rate = rate;
  d.throttle = 0;
  d.last_packet = 0xFFFF;

  rmt_tx_channel_config_t tx_cfg = {};
  tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_cfg.gpio_num = d.gpio;
  tx_cfg.mem_block_symbols = 64;
  tx_cfg.resolution_hz = 40 * 1000 * 1000;
  tx_cfg.trans_queue_depth = 4;
  esp_err_t err = rmt_new_tx_channel(&tx_cfg, &d.chan);
  if (err != ESP_OK) {
    d.chan = nullptr;
    return false;
  }

  uint16_t t0h, t0l, t1h, t1l;
  dshotBitTimings(rate, &t0h, &t0l, &t1h, &t1l);
  rmt_bytes_encoder_config_t bytes_cfg = {};
  bytes_cfg.bit0.level0 = 1;
  bytes_cfg.bit0.duration0 = t0h;
  bytes_cfg.bit0.level1 = 0;
  bytes_cfg.bit0.duration1 = t0l;
  bytes_cfg.bit1.level0 = 1;
  bytes_cfg.bit1.duration0 = t1h;
  bytes_cfg.bit1.level1 = 0;
  bytes_cfg.bit1.duration1 = t1l;
  bytes_cfg.flags.msb_first = 1;
  err = rmt_new_bytes_encoder(&bytes_cfg, &d.encoder);
  if (err != ESP_OK) {
    rmt_del_channel(d.chan);
    d.chan = nullptr;
    return false;
  }

  if (rmt_enable(d.chan) != ESP_OK) {
    rmt_del_encoder(d.encoder);
    rmt_del_channel(d.chan);
    d.encoder = nullptr;
    d.chan = nullptr;
    return false;
  }
  d.ready = true;
  return true;
}

static inline bool dshotTransmit(EscDshot& d, uint16_t packet) {
  if (!d.ready || !d.chan || !d.encoder) return false;
  uint8_t bytes[2] = {
      (uint8_t)((packet >> 8) & 0xFF),
      (uint8_t)(packet & 0xFF),
  };
  dshotStopTx(d);
  rmt_transmit_config_t tx = {};
  tx.loop_count = -1;
  if (rmt_transmit(d.chan, d.encoder, bytes, sizeof(bytes), &tx) != ESP_OK) return false;
  d.last_packet = packet;
  return true;
}

static inline bool dshotSetThrottle(EscDshot& d, uint16_t value, bool telemetry = false) {
  if (value > 2047) value = 2047;
  d.throttle = value;
  uint16_t packet = dshotEncodePacket(value, telemetry);
  if (packet == d.last_packet && d.ready) return true;
  return dshotTransmit(d, packet);
}

static inline bool dshotSetRate(EscDshot& d, DshotRate rate) {
  if (d.gpio == GPIO_NUM_NC) return false;
  int pin = (int)d.gpio;
  uint16_t thr = 0;  // 换速率必须先停转
  if (!dshotBegin(d, pin, rate)) return false;
  return dshotSetThrottle(d, thr, false);
}

static inline uint16_t pulseUsToDshot(uint16_t pulse_us, uint16_t min_us, uint16_t max_us) {
  if (pulse_us <= min_us) return 0;
  if (pulse_us >= max_us) return 2047;
  float t = (float)(pulse_us - min_us) / (float)(max_us - min_us);
  return (uint16_t)(48.0f + t * (2047.0f - 48.0f) + 0.5f);
}
