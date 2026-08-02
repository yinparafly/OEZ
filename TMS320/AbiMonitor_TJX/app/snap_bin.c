#include "snap_bin.h"
#include "eqep_abi.h"
#include "speed_est.h"
#include <string.h>

#define SNAP_MAGIC 0xAB1C0002UL

static SnapPoint g_snap[SNAP_CAP];
unsigned int g_snap_n = 0;
static SnapPoint g_ring[RING_CAP];
static unsigned int g_ring_w = 0;
static unsigned int g_ring_n = 0;
static uint32_t  g_ring_t0 = 0;
static int64_t   g_ring_last_c = 0;
static bool      g_ring_ok = false;
bool      g_recording = false;
bool      g_alive = false;
static uint32_t  g_alive_ms = 0;
static bool      g_staging = false;
static uint32_t  g_staging_t0 = 0;
static uint32_t  g_cap_ms  = 2000;

/* ---- internal helpers ---- */
static void push_ring(uint32_t t_us, int64_t counts, uint32_t idx)
{
    SnapPoint *p = &g_ring[g_ring_w];
    p->t_us    = t_us;
    p->counts  = counts;
    p->index_n = idx;
    g_ring_w = (g_ring_w + 1) % RING_CAP;
    if (g_ring_n < RING_CAP) g_ring_n++;
}

static void push_snap(uint32_t t_us, int64_t counts, uint32_t idx)
{
    if (g_snap_n < SNAP_CAP) {
        SnapPoint *p = &g_snap[g_snap_n++];
        p->t_us    = t_us;
        p->counts  = counts;
        p->index_n = idx;
    }
}

static void backtrack(void)
{
    unsigned int n = (g_ring_n < BACKTRACK_N) ? g_ring_n : BACKTRACK_N;
    unsigned int start = (g_ring_w + RING_CAP - n) % RING_CAP;
    uint32_t base_t = 0;
    unsigned int i;
    for (i = 0; i < n; i++) {
        SnapPoint *p = &g_ring[(start + i) % RING_CAP];
        if (i == 0) base_t = p->t_us;
        push_snap(p->t_us - base_t, p->counts, p->index_n);
    }
}

/* ---- Public API ---- */
bool snap_alloc(void) {
    g_snap_n    = 0;
    g_ring_w    = 0;
    g_ring_n    = 0;
    g_ring_ok   = false;
    g_recording = false;
    g_alive     = false;
    g_staging   = false;
    return true;
}

int snap_armed(void) {
    return g_staging || g_recording || g_alive;
}

void snap_on_event(uint32_t t_us, int64_t counts, uint32_t idx)
{
    if (!g_ring_ok) {
        g_ring_t0     = t_us;
        g_ring_last_c = counts;
        g_ring_ok     = true;
    }
    uint32_t t = t_us - g_ring_t0;
    push_ring(t, counts, idx);
    g_ring_last_c = counts;
}

void snap_poll(uint32_t rpm)
{
    uint32_t ms = spd_abs_ms();

    if (!g_recording && !g_alive && !g_staging && rpm > 10) {
        g_staging = true;
        g_staging_t0 = ms;
    }
    if (g_staging && !g_recording) {
        uint32_t idx_trig = g_ring_n > 0 ? 1 : 0;
        uint32_t time_trig = (ms - g_staging_t0 >= 200) ? 1 : 0;
        uint32_t ring_trig = (g_ring_n >= 120) ? 1 : 0;
        uint32_t rpm_drop = (rpm < 5 && g_ring_n >= 40) ? 1 : 0;
        if (idx_trig || time_trig || ring_trig || rpm_drop) {
            backtrack();
            g_recording = true;
            g_staging   = false;
        }
    }
    if (g_recording) {
        uint32_t pts = g_snap_n;
        if (pts >= SNAP_CAP || (ms - (g_staging_t0 + 200)) >= g_cap_ms) {
            g_recording = false;
            g_alive = true;
            g_alive_ms = ms;
        }
    }
    if (g_alive) {
        if (ms - g_alive_ms >= SNAP_ALIVE_MS) {
            g_alive = false;
        }
    }
}

bool snap_done(void)      { return !g_recording && !g_alive && g_snap_n > 0; }
bool snap_dump_ready(void) { return snap_done(); }
bool snap_full(void)       { return g_snap_n >= SNAP_CAP; }

uint32_t snap_data_bytes(void) { return (uint32_t)(g_snap_n * 16U); }

const unsigned char *snap_data(void) { return (const unsigned char *)g_snap; }

void snap_set_ms(uint32_t ms)  { g_cap_ms = ms; }

void snap_force_done(void)     { g_recording = false; g_alive = true; g_alive_ms = 0; }

/* ---- CRC32 ---- */
uint32_t snap_crc32(const unsigned char *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        for (j = 0; j < 8; j++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0);
    }
    return ~crc;
}
