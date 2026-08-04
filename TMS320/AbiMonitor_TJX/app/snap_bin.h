#ifndef SNAP_BIN_H
#define SNAP_BIN_H

#include <stdint.h>
#include <stdbool.h>

#define SNAP_CAP      4400
#define RING_CAP      600
#define BACKTRACK_N   400
#define SNAP_STEPS    4000

/* ESP32 v3 20B 点格式（紧凑布局，全 4B 成员保证无 padding）：
   t_us u32 + counts i64(lo,hi) + index_n u32 + event_rpm i32 — 内存序 = 协议 LE <IqIi */
typedef struct {
    uint32_t t_us;
    uint32_t counts_lo;   /* counts 低 32 位 */
    int32_t  counts_hi;   /* counts 高 32 位（符号位） */
    uint32_t index_n;
    int32_t  event_rpm;   /* 记录点瞬时 rpm（ISR 内 dpos*15000/dt_us），
                              正=正转 负=反转；臂环预触发段为 0 */
} SnapPoint;
typedef char snap_pt_size_check[(sizeof(SnapPoint) == 10U) ? 1 : -1];  /* 10 words = 20B @word-寻址 C28x */

#define SNAP_ALIVE_MS  2000U

bool snap_alloc(void);
void snap_free(void);
void snap_arm(void);
void snap_disarm(void);
int  snap_armed(void);
typedef void (*snap_done_fn)(uint32_t n, uint32_t bytes);
extern snap_done_fn snap_done_cb;

/* Called from 2kHz UTO ISR — push point to ring buffer */
void snap_on_event(uint32_t t_us, int64_t counts, uint32_t index_n, int32_t event_rpm);

/* Tick-driven state machine — call from main loop */
void snap_poll(uint32_t rpm);
bool snap_done(void);
bool snap_dump_ready(void);
bool snap_full(void);

/* Data dump */
uint32_t snap_data_bytes(void);
const unsigned char *snap_data(void);
/* C28x word 寻址：逐 word 拆分导出字节流（协议小端序），out 回调每字节调用 */
typedef void (*snap_byte_fn)(unsigned char b);
void     snap_dump_stream(snap_byte_fn out);
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
