#include "snap_bin.h"
#include "eqep_abi.h"
#include "speed_est.h"
#include <string.h>

static SnapPoint g_snap[SNAP_CAP];
unsigned int g_snap_n = 0;
static SnapPoint g_ring[RING_CAP];
static unsigned int g_ring_w = 0;
static unsigned int g_ring_n = 0;
static uint32_t  g_ring_t0 = 0;
static uint32_t  g_snap_base = 0;   /* 回溯段首点的 ring 相对时间（时间基准统一） */
static int64_t   g_ring_last_c = 0;
static bool      g_ring_ok = false;
bool      g_recording = false;
bool      g_alive = false;
static bool      g_armed  = false;
static uint32_t  g_alive_ms = 0;
static bool      g_staging = false;
static uint32_t  g_staging_t0 = 0;
static uint32_t  g_cap_ms  = 2000;
snap_done_fn snap_done_cb = 0;

static void push_ring(uint32_t t_us, int64_t counts, uint32_t idx, int32_t erpm) {
    SnapPoint *p = &g_ring[g_ring_w];
    p->t_us=t_us; p->counts_lo=(uint32_t)counts; p->counts_hi=(int32_t)(counts>>32); p->index_n=idx;
    p->event_rpm=erpm;
    g_ring_w = (g_ring_w + 1) % RING_CAP;
    if (g_ring_n < RING_CAP) g_ring_n++;
}
static void push_snap(uint32_t t_us, int64_t counts, uint32_t idx, int32_t erpm) {
    if (g_snap_n < SNAP_CAP) {
        SnapPoint *p = &g_snap[g_snap_n++];
        p->t_us=t_us; p->counts_lo=(uint32_t)counts; p->counts_hi=(int32_t)(counts>>32); p->index_n=idx;
        p->event_rpm=erpm;
    }
}
static int64_t pt_counts(const SnapPoint *p) {
    return ((int64_t)p->counts_hi << 32) | (uint32_t)p->counts_lo;
}
static void backtrack(void) {
    /* 临界区：ring 由 UTO ISR push、本函数在 main 循环消费，
       抽稀后点间隔大（div×17µs），backtrack 400 点覆盖数十 ms，
       与 ISR push 竞争会造成时间戳乱序（flush 兜底被迫 clamp → 同戳双点 → PC 尖刺）。
       DINT 期间 QPOSCNT 硬件继续计数，恢复后 ISR 差分一次性处理，不丢位置。 */
    unsigned int n = (g_ring_n < BACKTRACK_N) ? g_ring_n : BACKTRACK_N;
    unsigned int start = (g_ring_w + RING_CAP - n) % RING_CAP;
    uint32_t base = 0; unsigned int i;
    DINT;
    for (i = 0; i < n; i++) {
        SnapPoint *p = &g_ring[(start + i) % RING_CAP];
        if (i == 0) { base = p->t_us; g_snap_base = base; }
        push_snap(p->t_us - base, pt_counts(p), p->index_n, p->event_rpm);
    }
    g_ring_n = 0;  /* consumed the ring */
    EINT;
}
static void flush_ring(void) {
    while (g_snap_n < SNAP_CAP && g_ring_n > 0) {
        unsigned int rd = (g_ring_w + RING_CAP - g_ring_n) % RING_CAP;
        SnapPoint *p = &g_ring[rd];
        /* ring 点已是相对 g_ring_t0 的相对时间（push_ring 时算好）。
           与回溯段同一基准 g_snap_base 压缩，保证时间单调连续；
           绝不能再减 g_ring_t0（u32 回绕 → 巨大时间戳 → 曲线错乱）。 */
        uint32_t t = p->t_us - g_snap_base;
        /* 单调性兜底：任何乱序进来都钳到 prev+1，绝不产生等戳双点
           （等戳 → PC 差分 dt=0 → 尖刺，见联调记录 2026-08-03）。 */
        if (g_snap_n > 0 && t <= g_snap[g_snap_n-1].t_us)
            t = g_snap[g_snap_n-1].t_us + 1u;
        push_snap(t, pt_counts(p), p->index_n, p->event_rpm);
        g_ring_n--;
    }
}

bool snap_alloc(void) {
    g_snap_n=0; g_ring_w=0; g_ring_n=0; g_ring_ok=false; g_snap_base=0;
    g_recording=false; g_alive=false; g_staging=false;
    return true;
}
void snap_arm(void)   { g_armed = true; }
void snap_disarm(void){ g_armed = false; }
int  snap_armed(void) { return g_armed || g_staging || g_recording || g_alive; }

void snap_on_event(uint32_t t_us, int64_t counts, uint32_t idx, int32_t event_rpm) {
    if (!g_ring_ok) { g_ring_t0=t_us; g_ring_last_c=counts; g_ring_ok=true; }
    uint32_t t = t_us - g_ring_t0;
    push_ring(t, counts, idx, event_rpm);
    g_ring_last_c = counts;
    if (g_recording) flush_ring();
}

void snap_poll(uint32_t rpm) {
    uint32_t ms = spd_abs_ms();
    /* Armed ring: fill ring continuously, detect trigger */
    if (!g_recording && !g_alive && g_armed) {
        if (rpm > 10) {
            if (!g_staging) { g_staging = true; g_staging_t0 = ms; }
            /* Confirm trigger: index advance OR 200ms OR ring≥120 OR rpm-drop+ring≥40 */
            int trig = 0;
            uint32_t rn = g_ring_n;
            if (rn >= 120) trig = 1;
            if (ms - g_staging_t0 >= 200) trig = 1;
            if (rpm < 5 && rn >= 40) trig = 1;
            if (trig) {
                backtrack();          /* copy last N from ring to snap */
                g_recording = true;
                g_staging   = false;
                g_alive_ms  = ms;     /* tracking for recording timeout */
                flush_ring();         /* drain remaining ring into snap */
            }
        } else if (g_staging) {
            /* RPM dropped below threshold — cancel staging */
            g_staging = false;
        }
    }
    /* Recording: keep flushing ring, check stop */
    if (g_recording) {
        flush_ring();
        if (g_snap_n >= SNAP_CAP || (ms - g_alive_ms) >= g_cap_ms) {
            g_recording = false;
            g_alive = true;
            g_alive_ms = ms;
        }
    }
    /* ALIVE: wait 2s for user to retrieve data */
    if (g_alive && !g_recording) {
        if (ms - g_alive_ms >= SNAP_ALIVE_MS) {
            g_alive = false;
            g_armed = false;   /* done → disarm */
            if (snap_done_cb && g_snap_n > 0)
                snap_done_cb((uint32_t)g_snap_n, (uint32_t)((uint32_t)g_snap_n * 20UL));
        }
    }
}

bool snap_done(void)      { return !g_recording && !g_alive && g_snap_n > 0; }
bool snap_dump_ready(void){ return snap_done(); }
bool snap_full(void)      { return g_snap_n >= SNAP_CAP; }
uint32_t snap_data_bytes(void){ return (uint32_t)((uint32_t)g_snap_n * 20UL); }
const unsigned char *snap_data(void) { return (const unsigned char *)g_snap; }

/* C28x 内存 word 寻址：uchar* 步进=16 位。逐 word 拆分导出字节流（协议小端序） */
void snap_dump_stream(snap_byte_fn out) {
    const uint16_t *w = (const uint16_t *)g_snap;
    uint32_t i, nw = g_snap_n * 10u;   /* 每点 10 words = 20 字节 */
    for (i = 0; i < nw; i++) {
        uint16_t wd = w[i];
        out((unsigned char)(wd & 0xFFu));
        out((unsigned char)(wd >> 8));
    }
}
void snap_set_ms(uint32_t ms) { g_cap_ms = ms; }
void snap_force_done(void) { g_recording=false; g_alive=true; g_alive_ms=0; }
uint32_t snap_remain_ms(void) {
    if (!g_recording) return 0;
    uint32_t el = spd_abs_ms() - g_alive_ms;
    return (el < g_cap_ms) ? (g_cap_ms - el) : 0;
}

uint32_t snap_crc32(const unsigned char *buf, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFu; uint32_t i, j;
    for (i=0;i<len;i++) { crc^=buf[i]; for(j=0;j<8;j++) crc=(crc>>1)^((crc&1)?0xEDB88320u:0); }
    return ~crc;
}
