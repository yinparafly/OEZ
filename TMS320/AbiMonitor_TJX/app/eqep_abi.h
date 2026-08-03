#ifndef EQEP_ABI_H
#define EQEP_ABI_H

#include <stdint.h>

#define ABI_GEAR_4X 1
#define ABI_GEAR_1X 4

void abi_init(void);
int64_t  abi_counts(void);
uint32_t abi_index_n(void);
void     abi_reset_index(void);
uint64_t abi_now_us(void);
int      abi_gear(void);
void     abi_set_gear(int gear);
uint32_t abi_last_period_us(void);
uint32_t abi_missed_events(void);
uint32_t abi_event_count(void);
uint32_t abi_pcm_dbg(void);   /* uto counter when PCM disabled */
uint32_t abi_qdc_dbg(void);
uint32_t abi_iel_dbg(void);

#endif
