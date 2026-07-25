/************************************************
 * 手拧长采：定时回调只写 DRAM 环，loop 刷入 PSRAM
 * （定时器上下文写 PSRAM 会导致 n 一直为 0）
 ************************************************/
#include "cap_long.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  int16_t rpm_x10;
  int32_t dcounts;
  uint16_t dindex;
} CapSample;

static CapSample* g_cap = nullptr;
static size_t g_cap_cap = 0;
static volatile size_t g_cap_n = 0;
static volatile bool g_cap_run = false;
static uint32_t g_cap_t0 = 0;
static uint32_t g_cap_end_ms = 0;
static int64_t g_cap_c0 = 0;
static uint32_t g_cap_i0 = 0;
static bool g_cap_base_set = false;
static portMUX_TYPE g_cap_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_cap_done_evt = false;
static volatile uint32_t g_cap_isr_hits = 0;
static volatile uint32_t g_cap_ring_drop = 0;

// DRAM 环形缓冲（定时器可写）
static const size_t RING_N = 512;
static CapSample g_ring[RING_N];
static volatile uint16_t g_ring_w = 0;
static volatile uint16_t g_ring_r = 0;

static int16_t rpmToX10Cap(float rpm) {
  long v = lroundf(rpm * 10.0f);
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

bool capAlloc() {
  if (g_cap) return true;
  size_t free_b = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  Serial.printf("# CAPTURE alloc: spiram_free=%u\n", (unsigned)free_b);

  const size_t try_n[] = {80000, 40000, 20000, 10000};
  for (size_t i = 0; i < sizeof(try_n) / sizeof(try_n[0]); ++i) {
    size_t n = try_n[i];
    size_t bytes = n * sizeof(CapSample);
    if (free_b > 0 && bytes + 65536 > free_b) continue;
    void* p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p) {
      g_cap = (CapSample*)p;
      g_cap_cap = n;
      g_cap_n = 0;
      Serial.printf("# CAPTURE ok samples=%u (≈%us@2kHz) bytes=%u\n", (unsigned)n,
                    (unsigned)(n / 2000), (unsigned)bytes);
      return true;
    }
  }
  if (free_b > 128000) {
    size_t n = (free_b - 65536) / sizeof(CapSample);
    if (n > 10000) {
      void* p = heap_caps_malloc(n * sizeof(CapSample), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (p) {
        g_cap = (CapSample*)p;
        g_cap_cap = n;
        g_cap_n = 0;
        Serial.printf("# CAPTURE ok(fit) samples=%u bytes=%u\n", (unsigned)n,
                      (unsigned)(n * sizeof(CapSample)));
        return true;
      }
    }
  }
  Serial.printf("# CAPTURE fail: spiram_free=%u\n", (unsigned)free_b);
  return false;
}

void capFree() {
  if (g_cap) {
    heap_caps_free(g_cap);
    g_cap = nullptr;
  }
  g_cap_cap = 0;
  g_cap_n = 0;
  g_cap_run = false;
}

bool capStart(uint32_t duration_ms) {
  if (!g_cap && !capAlloc()) return false;
  uint32_t max_ms = (uint32_t)((g_cap_cap * 1000UL) / 2000UL);
  if (max_ms < 1000) max_ms = 1000;
  if (duration_ms < 5000) duration_ms = 5000;  // 允许短测；UI 建议 30~100
  if (duration_ms > 100000) duration_ms = 100000;
  if (duration_ms > max_ms) duration_ms = max_ms;

  portENTER_CRITICAL(&g_cap_mux);
  g_cap_n = 0;
  g_ring_w = 0;
  g_ring_r = 0;
  g_cap_t0 = millis();
  g_cap_end_ms = g_cap_t0 + duration_ms;
  g_cap_c0 = 0;
  g_cap_i0 = 0;
  g_cap_base_set = false;
  g_cap_done_evt = false;
  g_cap_isr_hits = 0;
  g_cap_ring_drop = 0;
  g_cap_run = true;
  portEXIT_CRITICAL(&g_cap_mux);
  return true;
}

void capStop() {
  if (g_cap_run) g_cap_done_evt = true;
  g_cap_run = false;
}

bool capTakeDoneEdge() {
  if (!g_cap_done_evt) return false;
  g_cap_done_evt = false;
  return true;
}

bool capIsRunning() { return g_cap_run; }

uint32_t capRemainMs() {
  if (!g_cap_run) return 0;
  uint32_t now = millis();
  if (now >= g_cap_end_ms) return 0;
  return g_cap_end_ms - now;
}

size_t capCount() { return g_cap_n; }
size_t capCapacity() { return g_cap_cap; }

uint32_t capDurationMs() {
  if (!g_cap || g_cap_n == 0) return 0;
  return g_cap[g_cap_n - 1].t_ms;
}

uint32_t capIsrHits() { return g_cap_isr_hits; }
uint32_t capRingDrop() { return g_cap_ring_drop; }

/** 定时器：只写入内部 RAM 环，禁止碰 PSRAM */
void capPushIsr(uint32_t t_ms, float rpm, int64_t counts, uint32_t index_n) {
  if (!g_cap_run) return;
  g_cap_isr_hits++;
  if (t_ms >= g_cap_end_ms) {
    g_cap_run = false;
    g_cap_done_evt = true;
    return;
  }

  CapSample s;
  s.t_ms = t_ms;  // 绝对 ms，loop 里再减 t0
  s.rpm_x10 = rpmToX10Cap(rpm);
  // counts/index 先原样塞进临时字段；loop 转成相对
  if (counts > 2147483647LL) counts = 2147483647LL;
  if (counts < -2147483648LL) counts = -2147483648LL;
  s.dcounts = (int32_t)counts;
  s.dindex = (index_n > 65535) ? 65535 : (uint16_t)index_n;

  uint16_t w = g_ring_w;
  uint16_t next = (uint16_t)((w + 1) % RING_N);
  if (next == g_ring_r) {
    g_cap_ring_drop++;
    return;
  }
  g_ring[w] = s;
  g_ring_w = next;
}

/** 主循环：DRAM 环 → PSRAM */
void capPoll() {
  if (!g_cap) return;

  while (g_ring_r != g_ring_w) {
    CapSample raw = g_ring[g_ring_r];
    g_ring_r = (uint16_t)((g_ring_r + 1) % RING_N);

    if (!g_cap_base_set) {
      g_cap_c0 = raw.dcounts;
      g_cap_i0 = raw.dindex;
      g_cap_base_set = true;
    }
    if (g_cap_n >= g_cap_cap) {
      g_cap_run = false;
      g_cap_done_evt = true;
      break;
    }
    CapSample s;
    s.t_ms = (raw.t_ms >= g_cap_t0) ? (raw.t_ms - g_cap_t0) : 0;
    s.rpm_x10 = raw.rpm_x10;
    s.dcounts = (int32_t)((int64_t)raw.dcounts - g_cap_c0);
    uint32_t di = (raw.dindex >= g_cap_i0) ? (raw.dindex - g_cap_i0) : 0;
    s.dindex = (di > 65535) ? 65535 : (uint16_t)di;
    g_cap[g_cap_n] = s;
    g_cap_n++;
  }

  // 时间到且环已空 → 结束
  if (g_cap_run && millis() >= g_cap_end_ms && g_ring_r == g_ring_w) {
    g_cap_run = false;
    g_cap_done_evt = true;
  }
}

static volatile bool g_cap_usb_dumping = false;
static size_t g_dump_src_i = 0;   // 源下标（步进 stride）
static size_t g_dump_n = 0;
static size_t g_dump_out = 0;     // 已发出行数
static size_t g_dump_stride = 10;
static bool g_dump_active = false;
static bool g_dump_wait_ack = false;
static uint32_t g_dump_wait_since = 0;
static const size_t CAP_DUMP_CHUNK = 80;       // 每块行数（ACK 前）
static const uint32_t CAP_DUMP_ACK_TIMEOUT_MS = 400;  // 无 NEXT 时自续，防卡死

bool capIsDumping() { return g_cap_usb_dumping; }

static void capDumpFinish(bool empty_ok) {
  if (empty_ok) {
    Serial.println("CAP END 0");
  } else {
    Serial.printf("CAP END %u src=%u stride=%u\n", (unsigned)g_dump_out, (unsigned)g_dump_n,
                  (unsigned)g_dump_stride);
  }
  g_dump_active = false;
  g_dump_wait_ack = false;
  g_cap_usb_dumping = false;
}

/** 发一块（最多 CAP_DUMP_CHUNK 行）；返回 true=还有数据 */
static bool capDumpSendChunk() {
  if (!g_dump_active || !g_cap) return false;
  if (g_dump_src_i >= g_dump_n) {
    capDumpFinish(false);
    return false;
  }

  char line[64];
  size_t emitted = 0;
  while (g_dump_src_i < g_dump_n && emitted < CAP_DUMP_CHUNK) {
    CapSample s = g_cap[g_dump_src_i];
    int n = snprintf(line, sizeof(line), "C,%lu,%.1f,%ld,%u\n", (unsigned long)s.t_ms,
                     s.rpm_x10 / 10.0f, (long)s.dcounts, (unsigned)s.dindex);
    if (n > 0) {
      Serial.write((const uint8_t*)line, (size_t)n);
    }
    g_dump_src_i += g_dump_stride;
    g_dump_out++;
    emitted++;
    if ((emitted & 0x07) == 0) yield();
  }
  Serial.flush();

  if (g_dump_src_i >= g_dump_n) {
    capDumpFinish(false);
    return false;
  }
  Serial.printf("# CAP CHUNK out=%u src=%u/%u need=NEXT\n", (unsigned)g_dump_out,
                (unsigned)g_dump_src_i, (unsigned)g_dump_n);
  g_dump_wait_ack = true;
  g_dump_wait_since = millis();
  return true;
}

void capDumpNext() {
  if (!g_dump_active) {
    Serial.println("# CAP NEXT ignored (no active dump)");
    return;
  }
  g_dump_wait_ack = false;
}

void capDumpUsb(uint16_t stride) {
  capPoll();
  if (g_dump_active) {
    Serial.println("# CAP DUMP busy — wait CAP END or reboot");
    return;
  }
  if (stride < 1) stride = 1;
  if (stride > 50) stride = 50;

  g_cap_usb_dumping = true;
  g_dump_n = g_cap_n;
  g_dump_src_i = 0;
  g_dump_out = 0;
  g_dump_stride = stride;
  g_dump_wait_ack = false;

  size_t approx = (g_dump_n + stride - 1) / stride;
  Serial.printf(
      "# CAP DUMP n=%u dur_ms=%lu stride=%u out≈%u chunk=%u isr=%lu drop=%lu\n",
      (unsigned)g_dump_n, (unsigned long)capDurationMs(), (unsigned)stride, (unsigned)approx,
      (unsigned)CAP_DUMP_CHUNK, (unsigned long)g_cap_isr_hits, (unsigned long)g_cap_ring_drop);

  if (!g_cap || g_dump_n == 0) {
    Serial.println("# CAP empty");
    capDumpFinish(true);
    return;
  }
  Serial.println("# CAP FMT C,t_ms,rpm,dcounts,dindex");
  Serial.println("# CAP DUMP v5 ACK chunks — PC must CAPTURE NEXT each chunk");
  g_dump_active = true;
  // 首块在 capDumpPoll 发出，避免在命令处理里堵死 USB
}

void capDumpPoll() {
  if (!g_dump_active) return;
  if (g_dump_wait_ack) {
    if ((millis() - g_dump_wait_since) < CAP_DUMP_ACK_TIMEOUT_MS) return;
    // 超时自续（串口监视器无 NEXT 时也能慢慢拉完）
    g_dump_wait_ack = false;
  }
  (void)capDumpSendChunk();
}
