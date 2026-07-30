#include "snap_bin.h"

#include <Arduino.h>
#include <string.h>
#include <stdio.h>

#include "ble_nus.h"
#include "esp_heap_caps.h"

/** v2：payload 存 counts，转速由 PC/手机算；本地 live RPM 仍在 .ino */
static const uint32_t SNAP_MAGIC = 0xAB1C0002UL;
static const size_t SNAP_CAP = 5000;   // 400 回溯 + 2s@2kHz≈4000 + 余量
static const size_t RING_CAP = 1200;   // 触发前临时环缓（≥400）

typedef struct __attribute__((packed)) {
  uint32_t t_us;
  int64_t counts;    // PCNT 累计 counts（原始量）
  uint32_t index_n;
} SnapPoint;  // 16 bytes

static SnapPoint* g_snap = nullptr;
static size_t g_snap_n = 0;
static volatile bool g_snap_rec = false;
static volatile bool g_snap_done_edge = false;
static uint32_t g_snap_t0_us = 0;
static uint32_t g_snap_dur_us = 2000000UL;
static uint32_t g_snap_hz = 2000;
static int32_t g_snap_steps = 4000;
static int64_t g_snap_last_c = 0;
static bool g_snap_base_ok = false;

// 武装环缓
static SnapPoint* g_ring = nullptr;
static size_t g_ring_w = 0;
static size_t g_ring_n = 0;
static volatile bool g_armed_ring = false;
static int64_t g_ring_last_c = 0;
static bool g_ring_base_ok = false;
static uint32_t g_ring_t0_us = 0;

static bool g_alive_running = false;
static uint8_t g_alive_i = 0;
static uint32_t g_alive_next_ms = 0;
static bool g_dump_ready = true;
static volatile bool g_archive_edge = false;
/** false=RAM 有有效数据时禁止覆盖；仅 MONITOR START 置 true */
static bool g_snap_allow_overwrite = true;

static void storePoint(SnapPoint* dst, size_t* n, size_t cap, uint32_t t_us, int64_t counts,
                       uint32_t index_n, bool ring_mode) {
  SnapPoint p;
  p.t_us = t_us;
  p.counts = counts;
  p.index_n = index_n;
  if (ring_mode) {
    dst[g_ring_w] = p;
    g_ring_w = (g_ring_w + 1) % RING_CAP;
    if (g_ring_n < RING_CAP) g_ring_n++;
  } else {
    if (*n < cap) dst[(*n)++] = p;
  }
}

static void pushPoint(SnapPoint* dst, size_t* n, size_t cap, int64_t* last_c, bool* base_ok,
                      uint32_t* t0_us, int64_t counts, uint32_t index_n, uint32_t now_us,
                      bool ring_mode) {
  if (!dst || !n) return;
  if (!*base_ok) {
    *last_c = counts;
    *base_ok = true;
    *t0_us = now_us;
    storePoint(dst, n, cap, 0, counts, index_n, ring_mode);
    return;
  }
  *last_c = counts;
  uint32_t elapsed = now_us - *t0_us;
  storePoint(dst, n, cap, elapsed, counts, index_n, ring_mode);
}

bool snapAlloc() {
  if (!g_snap) {
    g_snap = (SnapPoint*)heap_caps_malloc(SNAP_CAP * sizeof(SnapPoint), MALLOC_CAP_INTERNAL);
    if (!g_snap) g_snap = (SnapPoint*)malloc(SNAP_CAP * sizeof(SnapPoint));
  }
  if (!g_ring) {
    g_ring = (SnapPoint*)heap_caps_malloc(RING_CAP * sizeof(SnapPoint), MALLOC_CAP_INTERNAL);
    if (!g_ring) g_ring = (SnapPoint*)malloc(RING_CAP * sizeof(SnapPoint));
  }
  // 注意：不要在此清 g_snap_n —— 否则 MONITOR 再武装会误清空已记 RAM
  return g_snap != nullptr && g_ring != nullptr;
}

void snapFree() {
  if (g_snap) {
    free(g_snap);
    g_snap = nullptr;
  }
  if (g_ring) {
    free(g_ring);
    g_ring = nullptr;
  }
  g_snap_n = 0;
  g_ring_n = 0;
  g_snap_rec = false;
  g_armed_ring = false;
}

void snapSetStepsPerRev(int32_t steps) {
  if (steps > 0) g_snap_steps = steps;
}

int32_t snapStepsPerRev() { return g_snap_steps; }

uint32_t snapFileMagic() { return SNAP_MAGIC; }

size_t snapPointSize() { return sizeof(SnapPoint); }

bool snapArmBegin() {
  if (!snapAlloc()) return false;
  if (g_snap_rec) return false;
  g_armed_ring = true;
  g_ring_w = 0;
  g_ring_n = 0;
  g_ring_base_ok = false;
  // 仅 MONITOR START（经此函数）清空上一拍 RAM；其它路径不得抹掉
  g_snap_n = 0;
  g_dump_ready = false;
  g_snap_allow_overwrite = true;
  g_alive_running = false;
  g_snap_done_edge = false;
  return true;
}

void snapArmEnd() { g_armed_ring = false; }

bool snapIsArmed() { return g_armed_ring; }

bool snapRecStart(uint32_t duration_ms, uint32_t sample_hz) {
  if (!snapAlloc()) return false;
  if (g_snap_rec) return false;
  if (duration_ms < 100) duration_ms = 100;
  if (duration_ms > 3000) duration_ms = 3000;
  if (sample_hz < 200) sample_hz = 200;
  if (sample_hz > 5000) sample_hz = 5000;
  g_armed_ring = false;
  g_snap_n = 0;
  g_snap_hz = sample_hz;
  g_snap_dur_us = duration_ms * 1000UL;
  g_snap_t0_us = micros();
  g_snap_base_ok = false;
  g_alive_running = false;
  g_dump_ready = false;
  g_snap_allow_overwrite = false;
  g_snap_rec = true;
  return true;
}

bool snapTriggerFromRing(uint16_t backtrack_n, uint32_t duration_ms) {
  if (!g_snap || !g_ring) return false;
  if (g_snap_rec) return false;
  if (duration_ms < 100) duration_ms = 100;
  if (duration_ms > 3000) duration_ms = 3000;
  if (backtrack_n > RING_CAP) backtrack_n = (uint16_t)RING_CAP;

  size_t n_bt = g_ring_n;
  if (n_bt > backtrack_n) n_bt = backtrack_n;

  // 空环缓触发会把已有 RAM 抹成 0 — 拒绝（手机掉线重连误触发时尤其危险）
  if (n_bt == 0) {
    return false;
  }
  if (g_snap_n > 0 && !g_snap_allow_overwrite) {
    return false;
  }

  g_snap_n = 0;
  if (n_bt > 0) {
    size_t start = (g_ring_w + RING_CAP - n_bt) % RING_CAP;
    uint32_t t0 = g_ring[start].t_us;
    for (size_t i = 0; i < n_bt && g_snap_n < SNAP_CAP; ++i) {
      SnapPoint p = g_ring[(start + i) % RING_CAP];
      p.t_us = p.t_us - t0;  // 相对回溯起点
      g_snap[g_snap_n++] = p;
    }
  }

  g_armed_ring = false;
  g_snap_hz = 2000;
  g_snap_dur_us = duration_ms * 1000UL;
  g_snap_t0_us = micros();
  g_snap_base_ok = false;  // 正式段继续追加 counts
  g_snap_done_edge = false;
  g_dump_ready = false;
  g_alive_running = false;
  g_snap_allow_overwrite = false;
  g_snap_rec = true;
  return true;
}

bool snapIsRecording() { return g_snap_rec; }

uint16_t snapCount() { return (uint16_t)g_snap_n; }

uint32_t snapHz() { return g_snap_hz; }

uint16_t snapRingCount() { return (uint16_t)g_ring_n; }

bool snapDumpReady() { return g_dump_ready && !g_snap_rec && !g_alive_running; }

bool snapOnSampleCounts(int64_t counts, uint32_t index_n, uint32_t now_us) {
  if (!g_snap && !g_ring) return false;

  // 武装环缓（尚未触发正式记录）
  if (g_armed_ring && !g_snap_rec) {
    if (!g_ring) return false;
    pushPoint(g_ring, &g_ring_n, RING_CAP, &g_ring_last_c, &g_ring_base_ok, &g_ring_t0_us, counts,
              index_n, now_us, true);
    return true;
  }

  if (!g_snap_rec || !g_snap) return false;

  if (!g_snap_base_ok) {
    g_snap_last_c = counts;
    g_snap_base_ok = true;
    g_snap_t0_us = now_us;
    // 若已有回溯段，正式点从末尾时间接着写；否则从 0
    uint32_t t_base = 0;
    if (g_snap_n > 0) {
      t_base = g_snap[g_snap_n - 1].t_us + (1000000UL / g_snap_hz);
    }
    if (g_snap_n < SNAP_CAP) {
      storePoint(g_snap, &g_snap_n, SNAP_CAP, t_base, counts, index_n, false);
    }
    return true;
  }

  uint32_t elapsed = now_us - g_snap_t0_us;
  if (elapsed > g_snap_dur_us || g_snap_n >= SNAP_CAP) {
    g_snap_rec = false;
    g_snap_done_edge = true;
    return true;
  }

  g_snap_last_c = counts;

  uint32_t t_base = 0;
  if (g_snap_n > 0) {
    t_base = g_snap[g_snap_n - 1].t_us + (1000000UL / g_snap_hz);
  } else {
    t_base = elapsed;
  }

  if (g_snap_n < SNAP_CAP) {
    storePoint(g_snap, &g_snap_n, SNAP_CAP, t_base, counts, index_n, false);
  }
  return true;
}

void snapPollDone() {
  if (g_snap_done_edge) {
    g_snap_done_edge = false;
    Serial.printf("# SNAP DONE n=%u hz=%lu bytes=%u — RAM full; ALIVE then SD archive\n",
                  (unsigned)g_snap_n, (unsigned long)g_snap_hz,
                  (unsigned)(g_snap_n * sizeof(SnapPoint)));
    if (bleConnected()) {
      char s[72];
      snprintf(s, sizeof(s), "# SNAP DONE n=%u\n", (unsigned)g_snap_n);
      bleSendLine(s);
    }
    g_dump_ready = false;
    g_alive_running = true;
    g_alive_i = 0;
    g_alive_next_ms = millis() + 1000;
  }

  if (!g_alive_running) return;
  uint32_t now = millis();
  if ((int32_t)(now - g_alive_next_ms) < 0) return;
  g_alive_i++;
  Serial.printf("# ALIVE %u n=%u\n", (unsigned)g_alive_i, (unsigned)g_snap_n);
  if (bleConnected()) {
    char s[48];
    snprintf(s, sizeof(s), "# ALIVE %u n=%u\n", (unsigned)g_alive_i, (unsigned)g_snap_n);
    bleSendLine(s);
  }
  if (g_alive_i >= 2) {
    g_alive_running = false;
    g_dump_ready = true;
    g_archive_edge = true;
    Serial.printf("# SNAP DUMP READY n=%u — auto SD SAVE (RAM→SD); optional DUMP BIN\n",
                  (unsigned)g_snap_n);
    if (bleConnected()) {
      char s[64];
      snprintf(s, sizeof(s), "# SNAP DUMP READY n=%u\n", (unsigned)g_snap_n);
      bleSendLine(s);
    }
  } else {
    g_alive_next_ms = now + 1000;
  }
}

bool snapTakeArchiveEdge() {
  if (!g_archive_edge) return false;
  g_archive_edge = false;
  return true;
}

void snapPrintStatus() {
  Serial.printf("# SNAP n=%u ring=%u hz=%lu ready=%d rec=%d arm=%d alive=%d bytes=%u magic=v2\n",
                (unsigned)g_snap_n, (unsigned)g_ring_n, (unsigned long)g_snap_hz,
                snapDumpReady() ? 1 : 0, g_snap_rec ? 1 : 0, g_armed_ring ? 1 : 0,
                g_alive_running ? 1 : 0, (unsigned)(g_snap_n * sizeof(SnapPoint)));
}

void snapDiagLine(char* out, size_t out_sz) {
  if (!out || out_sz < 8) return;
  snprintf(out, out_sz,
           "# DIAG snap=%u ring=%u rec=%d arm_ring=%d ready=%d alive=%d\n",
           (unsigned)g_snap_n, (unsigned)g_ring_n, g_snap_rec ? 1 : 0, g_armed_ring ? 1 : 0,
           snapDumpReady() ? 1 : 0, g_alive_running ? 1 : 0);
}

const uint8_t* snapDataBytes() { return (const uint8_t*)g_snap; }

size_t snapDataLen() { return g_snap ? (g_snap_n * sizeof(SnapPoint)) : 0; }

uint32_t snapCrc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

bool snapDumpBinary() {
  if (!g_snap) {
    Serial.println("# SNAP empty (no buffer)");
    return false;
  }
  if (!snapDumpReady()) {
    Serial.println("# DUMP BIN wait (ALIVE not done — wait SNAP DUMP READY)");
    return false;
  }
  static_assert(sizeof(SnapPoint) == 16, "SnapPoint must be 16 bytes (v2 counts)");
  uint16_t n = (uint16_t)g_snap_n;
  uint16_t hz = (uint16_t)g_snap_hz;
  size_t payload = (size_t)n * sizeof(SnapPoint);
  uint32_t crc = snapCrc32((const uint8_t*)g_snap, payload);

  uint8_t preamble[11];
  for (int i = 0; i < 10; ++i) preamble[i] = 0xAA;
  preamble[10] = 0x55;
  Serial.write(preamble, 11);

  uint32_t magic = SNAP_MAGIC;
  Serial.write((const uint8_t*)&magic, 4);
  Serial.write((const uint8_t*)&n, 2);
  Serial.write((const uint8_t*)&hz, 2);
  Serial.write((const uint8_t*)g_snap, payload);
  Serial.write((const uint8_t*)&crc, 4);
  Serial.printf("\n# BIN END n=%u hz=%u steps=%ld bytes=%u crc=0x%08lX\n", (unsigned)n,
                (unsigned)hz, (long)g_snap_steps, (unsigned)(8 + payload + 4),
                (unsigned long)crc);
  return true;
}

bool snapDumpHex() {
  if (!g_snap || g_snap_n == 0) {
    Serial.println("# SNAP empty");
    return false;
  }
  uint16_t n = (uint16_t)((g_snap_n > 40) ? 40 : g_snap_n);
  Serial.printf("# HEX BEGIN n=%u show=%u (t_us,counts,index_n) magic=v2\n", (unsigned)g_snap_n,
                (unsigned)n);
  for (uint16_t i = 0; i < n; ++i) {
    const SnapPoint& p = g_snap[i];
    Serial.printf("# %u %lu %lld %lu\n", (unsigned)i, (unsigned long)p.t_us,
                  (long long)p.counts, (unsigned long)p.index_n);
  }
  Serial.println("# HEX END");
  return true;
}
