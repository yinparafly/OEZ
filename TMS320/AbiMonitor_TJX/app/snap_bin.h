#ifndef SNAP_BIN_H
#define SNAP_BIN_H

#include <stdint.h>
#include <stdbool.h>

#define SNAP_CAP      600
#define RING_CAP      300
#define BACKTRACK_N   100
#define SNAP_STEPS    4000

typedef struct {
    uint32_t t_us;
    int64_t  counts;
    uint32_t index_n;
} SnapPoint;

#define SNAP_ALIVE_MS  2000U

bool snap_alloc(void);
void snap_free(void);
void snap_arm(void);
void snap_disarm(void);
int  snap_armed(void);

/* Called from 2kHz UTO ISR — push point to ring buffer */
void snap_on_event(uint32_t t_us, int64_t counts, uint32_t index_n);

/* Tick-driven state machine — call from main loop */
void snap_poll(uint32_t rpm);
bool snap_done(void);
bool snap_dump_ready(void);
bool snap_full(void);

/* Data dump */
uint32_t snap_data_bytes(void);
const unsigned char *snap_data(void);
void     snap_set_ms(uint32_t ms);
void     snap_force_done(void);
uint32_t snap_remain_ms(void);

/* CLI debugging */
extern unsigned int g_snap_n;
extern bool      g_recording;
extern bool      g_alive;

/* CRC32 */
uint32_t snap_crc32(const unsigned char *buf, uint32_t len);

#endif
