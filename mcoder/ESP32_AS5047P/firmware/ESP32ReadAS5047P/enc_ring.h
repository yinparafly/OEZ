/************************************************
 * Encoder sample ring (backup / stage D)
 * Single-producer (encSampleCb) / single-consumer (ctrl)
 * Overwrite-oldest on overrun; never block writer.
 * Realtime path: no Serial.print / hostPrintf
 ************************************************/
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ENC_RING_CAP
#define ENC_RING_CAP 32
#endif

typedef struct {
  uint32_t t_us;
  int64_t counts;
} enc_ring_sample_t;

typedef struct {
  enc_ring_sample_t buf[ENC_RING_CAP];
  volatile uint32_t wr;
  volatile uint32_t rd;
  volatile uint32_t overrun;
} enc_ring_t;

static inline void enc_ring_init(enc_ring_t* r) {
  if (!r) return;
  r->wr = 0;
  r->rd = 0;
  r->overrun = 0;
}

static inline void enc_ring_push(enc_ring_t* r, uint32_t t_us, int64_t counts) {
  if (!r) return;
  uint32_t w = r->wr;
  uint32_t next = w + 1u;
  if (next - r->rd >= ENC_RING_CAP) {
    r->rd = next - ENC_RING_CAP;
    r->overrun++;
  }
  uint32_t slot = w % ENC_RING_CAP;
  r->buf[slot].t_us = t_us;
  r->buf[slot].counts = counts;
  r->wr = next;
}

static inline uint32_t enc_ring_count(const enc_ring_t* r) {
  if (!r) return 0;
  return r->wr - r->rd;
}

static inline uint32_t enc_ring_peek_n(const enc_ring_t* r, enc_ring_sample_t* out, uint32_t n) {
  if (!r || !out || n == 0) return 0;
  uint32_t avail = enc_ring_count(r);
  if (avail == 0) return 0;
  if (n > avail) n = avail;
  if (n > ENC_RING_CAP) n = ENC_RING_CAP;
  uint32_t start = r->wr - n;
  for (uint32_t i = 0; i < n; i++) {
    out[i] = r->buf[(start + i) % ENC_RING_CAP];
  }
  return n;
}

static inline void enc_ring_consume_all(enc_ring_t* r) {
  if (!r) return;
  r->rd = r->wr;
}

static inline uint32_t enc_ring_overruns(const enc_ring_t* r) {
  return r ? r->overrun : 0;
}

#ifdef __cplusplus
}
#endif