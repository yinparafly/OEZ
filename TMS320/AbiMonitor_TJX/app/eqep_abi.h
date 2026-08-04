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

/* ---- 动态 UTO 周期扩展（spec v3.1 §4.3a-B1/C1） ---- */

/*
 * 返回最新有效电机侧 rpm，int32 有符号（spec §4.3a-C1）：
 *   正 = 正转，负 = 反转，0 = 停转或超时清零。
 * 该值是 UTO 事件差分（Δcounts/Δt）的瞬时结果；dc=0（无新边沿）时经
 * 外推衰减状态机输出（保持 3 周期 → 线性衰减 4~10 → 清零 ≥10）。
 * 调用方（speed_est/main/cli）只读此值，不复制衰减逻辑。
 */
int32_t abi_get_latest_rpm(void);

/* 连续 dc==0 的 UTO 周期计数（CLI ABI? 调试用） */
uint32_t abi_get_uto_idle_evt(void);

/* 当前实际写入 QUPRD 的周期值（CLI ABI? 调试用） */
uint32_t abi_get_current_qupr(void);

/* ---- 记录层事件抽稀（spec v3.1 §记录分级） ----
 * 返回 snap 记录当前档位 div：每 div 个【真实事件】记 1 点。
 *   1=全记  2=隔1记1  4=隔3记1  8=隔7记1  16=隔15记1
 * eQEP 始终 4x 计数，counts 永远真实——div 只影响点密度，
 * PC 端 counts 差分公式不变；div 本身由 rpm 自动分档（见 eqep_abi.c）。
 */
uint32_t abi_snap_div(void);

/* 最近一次【真实事件】(has_event) 时刻（µs 低 32 位），
   main 看门狗：PWM 活动但 0.5s 无事件 → 停机（安全）。 */
uint32_t abi_last_event_us(void);

#ifdef DEBUG
/* Task 5 探针：最近 UTO ISR 实际开销（CPU 周期数）——DEBUG 构建生效 */
uint32_t abi_isr_dbg_max_cycles(void);
uint32_t abi_isr_dbg_sum_cycles(void);
uint32_t abi_isr_dbg_count(void);
#endif

/* ---- 软模拟倍频（spec v3.1 §调试辅助） ----
 * 真实电机开环 PWM 只能到 ~5555rpm（物理限制）。软模拟把 ISR 差分
 * dpos / counts 放大 K 倍，等效呈现 K×rpm 的高速工况，用于验证
 * UTO 动态周期 + 事件流记录管线在"1 万转"级事件率下的行为。
 *
 * 重要：
 *  - 进入 SIM 后 rpm/qupr/counts 均为【假值】（等效模拟值）；
 *  - 圈数校验：counts ÷ (4000×K) 才等于真实物理圈数；
 *  - 一键还原真相：SIM OFF（设 K=1）。
 */
void     abi_sim_set(uint32_t multiplier);   /* K≥2 启用软模拟；1=还原 */
uint32_t abi_sim_get(void);                  /* 当前倍数，1=未启用 */

#endif
