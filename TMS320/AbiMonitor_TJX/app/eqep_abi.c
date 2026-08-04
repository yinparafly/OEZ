#include "eqep_abi.h"
#include "snap_bin.h"
#include "board.h"
#include "driverlib.h"
#include "../module_driver/bsp_motor_hallencoder.h"

/* ---- Clock ---- */
volatile uint64_t g_us64 = 0;
volatile uint32_t g_us_tick_snap = 0;
#define TIMER_PERIOD 150000u

/* ---- 动态 UTO 周期常量（spec v3.1 §4.2/§4.3 钉死） ----
   QUPRD 以 SYSCLK(150MHz) 计数。QUPR = N_MIN × 2_250_000 / rpm（4000 步/圈）。 */
#define QUPR_MIN        2500u     /* ≈60kHz 夹顶，锚定 9000rpm+ */
#define QUPR_MAX        375000u   /* ≈2.5ms → 400Hz，锚定扑翼油门刷新率 */
#define N_MIN           10u       /* 每 UTO 周期目标边沿步数（8~16 可调） */
#define EQEP_CLK_COEFF  2250000ul /* N_MIN×60×150MHz/4000 */
#define QUPR_INIT       22500u    /* 6.7kHz @1000rpm 中速档，上电加载 */

/* 迟滞：相邻事件 rpm 相对差 < 12.5% 不改 QUPR（定点，无浮点） */
#define HYST_NUM        125
#define HYST_DEN        1000

/* 外推衰减：dc=0 保持 3 周期，第 4~10 线性衰减，≥10 清零 */
#define HOLD_PERIODS    3
#define DECAY_END_PERIODS 10

/* 回退触发条件（朋友评审阻塞项 A，仅注释，不实现）：
   Task 5 实测 UTO ISR 均值 > 0.6µs（约 90 周期），或 60kHz 夹顶段占空比 > 10%
   → 改 1kHz 延迟写：1kHz ISR 算 QUPR，UTO ISR 只采样计数。 */

void abi_clock_tick(void) {
    g_us64 += 1000;
    g_us_tick_snap = HWREG(myCPUTIMER0_BASE + CPUTIMER_O_TIM);   /* 递减计数在 TIM（TCR 是控制位） */
}

uint64_t abi_now_us(void) {
    uint64_t base;
    uint32_t cnt, elap;
    base = g_us64;
    cnt  = HWREG(myCPUTIMER0_BASE + CPUTIMER_O_TIM);
    if (cnt <= g_us_tick_snap)
        elap = g_us_tick_snap - cnt;
    else
        elap = g_us_tick_snap + TIMER_PERIOD - cnt;
    return base + (uint64_t)(elap / 150u);
}

/* ---- State ---- */
static int64_t  g_counts    = 0;
static uint32_t g_index     = 0;
static uint32_t g_missed    = 0;
static uint32_t g_period    = 0;
static uint32_t g_ev_cnt    = 0;
static uint32_t g_uto_db    = 0;
static uint32_t g_qdc_db    = 0;
static uint32_t g_iel_db    = 0;
static uint32_t g_prev_pos  = 0;
static uint32_t g_prev_us   = 0;
static uint32_t g_last_ix_cnt = 0xFFFFFFFFu;

/* ---- 动态 UTO 周期 / 外推衰减状态（spec §4.3a-B1：归属本文件） ---- */
static volatile int32_t  s_latest_rpm      = 0;
static volatile uint32_t s_current_qupr    = QUPR_INIT;
static int32_t           s_last_rpm_for_hyst = 0;  /* 上次实际写 QUPR 时对应 rpm */
static uint32_t          s_uto_idle_evt    = 0;
static uint8_t           s_zero_dc_cnt     = 0;
static int32_t           s_held_rpm        = 0;

/* 软模拟倍频（SIM）：K=1 还原真实；K≥2 时 ISR 差分 dpos 放大 K 倍，
   counts/rpm/事件率均为【等效假值】，仅用于验证高速记录管线。
   一键还原：SIM OFF（K=1）。圈数校验须除以 K。 */
static volatile uint32_t s_sim_k = 1;

/* 最近一次【真实事件】(has_event) 的时刻（µs，非静止），
   供 main 看门狗判断"给了 PWM 但 0.5s 无编码器信号 → 停机"（见 main）。 */
static volatile uint32_t s_last_event_us = 0;

/* ---- 记录层事件抽稀（spec v3.1 §记录分级，软件抽稀方案） ----
   eQEP 始终 4x 计数（counts 永远真实），只在 snap 记录层按转速分档：
   每 div 个【真实事件】（has_event，dc≠0）记 1 点。
   静止事件（dc=0，无新边沿）不参与计数、不记录 —— 只记录事件。
   低速 div=1 全记；高速 div 增大，保证 4400 点覆盖 0.5~1.0s 窗口。
   分档（电机侧 rpm 绝对值，无迟滞首版，边界后期可按实测曲线微调）：
     <500   → div=1（全记）
     500~1500 → div=2
     1500~4000 → div=4
     4000~8000 → div=8
     >=8000  → div=16
   后期修改提示：边界与 div 均为纯参数，调整只影响记录密度，
   不影响测速精度（测速走全事件差分，与记录抽稀无关）。 */
static uint16_t s_snap_div      = 1;   /* 当前档位 div */
static uint16_t s_snap_evt_cnt  = 0;   /* 距上次记录已过的事件数 */

static uint16_t calc_snap_div(int32_t rpm)
{
    uint32_t a = (rpm < 0) ? (uint32_t)(-rpm) : (uint32_t)rpm;
    if (a < 500u)      return 1;
    else if (a < 1500u) return 2;
    else if (a < 4000u) return 4;
    else if (a < 8000u) return 8;
    else                return 16;
}

#ifdef DEBUG
static volatile uint32_t s_isr_dbg_max   = 0;
static volatile uint32_t s_isr_dbg_sum   = 0;
static volatile uint32_t s_isr_dbg_count = 0;
#endif

/* ---- 动态 UTO 周期：计算 + 钳位 + 迟滞 + 写 QUPRD ----
   rpm<1 视为静止 → 强制 QUPR_MAX（400Hz 保底反馈链路）。 */
static void abi_set_uto_period(int32_t rpm)
{
    uint32_t qupr;

    if (rpm == 0) {
        if (s_current_qupr == QUPR_MAX)   /* 已停在 MAX：免重复写寄存器 */
            return;
        qupr = QUPR_MAX;
    } else {
        uint32_t abs_rpm = (uint32_t)((rpm < 0) ? -rpm : rpm);
        uint64_t num = (uint64_t)N_MIN * (uint64_t)EQEP_CLK_COEFF;
        qupr = (uint32_t)(num / abs_rpm);
        if (qupr < QUPR_MIN) qupr = QUPR_MIN;
        if (qupr > QUPR_MAX) qupr = QUPR_MAX;
    }

    /* 迟滞：相对差 |Δrpm|×1000 < |last_rpm|×125 → 不写寄存器 */
    if (s_last_rpm_for_hyst != 0) {
        int32_t drpm = rpm - s_last_rpm_for_hyst;
        uint32_t adrpm = (drpm < 0) ? (uint32_t)(-drpm) : (uint32_t)drpm;
        uint32_t alast = (s_last_rpm_for_hyst < 0) ?
                         (uint32_t)(-s_last_rpm_for_hyst) : (uint32_t)s_last_rpm_for_hyst;
        if ((uint64_t)adrpm * HYST_DEN < (uint64_t)alast * HYST_NUM)
            return;
    }

    EQEP_loadUnitTimer(Module_EQEP_BASE, qupr);
    s_current_qupr      = qupr;
    s_last_rpm_for_hyst = rpm;
}

/* ---- 外推衰减状态机（spec §4.3：保持 3 / 衰减 4~10 / 清零 ≥10） ---- */
static void abi_update_rpm_hold_decay(int32_t event_rpm, bool has_new_event)
{
    if (has_new_event) {
        s_latest_rpm  = event_rpm;
        s_held_rpm    = event_rpm;
        s_zero_dc_cnt = 0;
        return;
    }

    s_uto_idle_evt++;
    s_zero_dc_cnt++;

    if (s_zero_dc_cnt <= HOLD_PERIODS) {
        s_latest_rpm = s_held_rpm;                       /* 保持阶段 */
    } else if (s_zero_dc_cnt <= DECAY_END_PERIODS) {
        /* 线性衰减：remain = DECAY_END - cnt + 1，total = DECAY_END - HOLD */
        uint32_t remain = DECAY_END_PERIODS - s_zero_dc_cnt + 1u;
        uint32_t total  = DECAY_END_PERIODS - HOLD_PERIODS;
        s_latest_rpm = (int32_t)(((int64_t)s_held_rpm * remain) / total);
    } else {
        s_latest_rpm = 0;
        s_held_rpm   = 0;
    }
}

/* ---- Public API ---- */
void abi_init(void)
{
    g_counts  = 0;
    g_index   = 0;
    g_missed  = 0;
    g_period  = 0;
    g_ev_cnt  = 0;
    g_uto_db  = 0;
    g_qdc_db  = 0;
    g_iel_db  = 0;
    g_prev_us = 0;

    /* Enable UTO, keep IEL. Disable PCM and QDC interrupts.
       Also clear QPOSCTL — SysConfig writes 0xFFFF (corrupted by cycles=0). */
    EQEP_disableInterrupt(Module_EQEP_BASE,
        EQEP_INT_POS_COMP_MATCH | EQEP_INT_DIR_CHANGE);
    EQEP_enableInterrupt(Module_EQEP_BASE, EQEP_INT_UNIT_TIME_OUT);
    EQEP_loadUnitTimer(Module_EQEP_BASE, QUPR_INIT);
    EQEP_enableUnitTimer(Module_EQEP_BASE, QUPR_INIT);
    EQEP_setLatchMode(Module_EQEP_BASE,
        EQEP_LATCH_CNT_READ_BY_CPU | EQEP_LATCH_UNIT_TIME_OUT);

    /* Fix corrupted QPOSCTL: disable PCM compare + clear PCSPW garbage */
    EALLOW;
    HWREGH(Module_EQEP_BASE + EQEP_O_QPOSCTL) = 0x0000;
    EDIS;

    /* Disable index reset: keep QPOSCNT monotonic for proper delta calc */
    EQEP_disableInterrupt(Module_EQEP_BASE, EQEP_INT_DIR_CHANGE);
    EQEP_setPositionCounterConfig(Module_EQEP_BASE, EQEP_POSITION_RESET_MAX_POS, 0xFFFFFFFEu);

    g_prev_pos = HWREG(Module_EQEP_BASE + EQEP_O_QPOSCNT);

    s_latest_rpm      = 0;
    s_current_qupr    = QUPR_INIT;
    s_last_rpm_for_hyst = 0;
    s_uto_idle_evt    = 0;
    s_zero_dc_cnt     = 0;
    s_held_rpm        = 0;
    s_last_event_us   = (uint32_t)(abi_now_us() & 0xFFFFFFFFuL);   /* 看门狗从"现在"起算 */
#ifdef DEBUG
    s_isr_dbg_max   = 0;
    s_isr_dbg_sum   = 0;
    s_isr_dbg_count = 0;
#endif
}

int64_t  abi_counts(void)        { return g_counts; }
uint32_t abi_index_n(void)       { return g_index; }
void     abi_reset_index(void)   { DINT; g_index = 0; EINT; }
int      abi_gear(void)          { return ABI_GEAR_4X; }
void     abi_set_gear(int g)     { (void)g; }
uint32_t abi_last_period_us(void){ return g_period; }
uint32_t abi_missed_events(void) { return g_missed; }
uint32_t abi_event_count(void)   { return g_ev_cnt; }
uint32_t abi_pcm_dbg(void)       { return g_uto_db; }
uint32_t abi_qdc_dbg(void)       { return g_qdc_db; }
uint32_t abi_iel_dbg(void)       { return g_iel_db; }

/* ---- 动态 UTO 周期扩展对外接口（spec §4.3a-B1/C1） ---- */
int32_t  abi_get_latest_rpm(void) { return s_latest_rpm; }
uint32_t abi_get_uto_idle_evt(void){ return s_uto_idle_evt; }
uint32_t abi_get_current_qupr(void){ return s_current_qupr; }
uint32_t abi_snap_div(void)       { return s_snap_div; }
uint32_t abi_last_event_us(void)  { return s_last_event_us; }   /* 真实事件时刻 */

/* ---- Task 4 遥测换算：QUPR(150MHz tick) ↔ µs/Hz/nominal rpm ----
   QUPR 以 SYSCLK(150MHz) 计数：uto_us = QUPR/150，freq_hz = 150MHz/QUPR，
   nom_rpm = N_MIN×2_250_000/QUPR（当前周期档对应的名义转速）。 */
uint32_t abi_uto_period_us(void) { return s_current_qupr / 150u; }
uint32_t abi_uto_freq_hz(void)   { return 150000000u / s_current_qupr; }
uint32_t abi_nom_rpm(void)       { return (uint32_t)((uint64_t)N_MIN * EQEP_CLK_COEFF / s_current_qupr); }

/* ---- SIM 软模拟倍频接口（spec v3.1 §调试辅助） ----
   K=1 → 还原真实信号（默认值，出厂/实机必为 1）。
   K≥2 → ISR 把 dpos/counts 放大 K 倍，rpm/qupr/事件率均为
          【等效假值】，用于在电机物理上限（~5555rpm）之外
          验证 UTO 动态周期 + 事件流记录管线在"1 万转"级
          事件率下的正确性（含圈数校验：counts÷4000÷K=真实圈数）。
   后期修改提示：此接口只控制 dpos 放大点，无其他副作用；
   如需改放大语义（如按圈数而非步数），只动 ISR 内 s_sim_k 判定块。 */
void abi_sim_set(uint32_t multiplier)
{
    DINT;                         /* 防与 ISR 读 s_sim_k 竞争 */
    s_sim_k = (multiplier >= 1u) ? multiplier : 1u;
    EINT;
}
uint32_t abi_sim_get(void)
{
    return s_sim_k;               /* 1 = 未启用（真实数据） */
}

/* ---- EQEP ISR ---- */
__interrupt void INT_Module_EQEP_ISR(void)
{
    uint16_t flg = HWREGH(Module_EQEP_BASE + EQEP_O_QFLG);
#ifdef DEBUG
    uint32_t isr_t0 = HWREG(myCPUTIMER0_BASE + CPUTIMER_O_TIM);
#endif

    if (flg & EQEP_INT_UNIT_TIME_OUT)
    {
        uint32_t pos    = HWREG(Module_EQEP_BASE + EQEP_O_QPOSCNT);
        uint32_t us_now = (uint32_t)(abi_now_us() & 0xFFFFFFFFuL);
        int32_t  dpos   = (int32_t)(pos - g_prev_pos);
        uint32_t dt_us  = us_now - g_prev_us;          /* 每次 UTO 都算（A1） */

        /* SIM 软模拟倍频：读真实 dpos 后立即放大，后续 counts/missed/
           event_rpm 全部基于放大后的假 dpos（等效模拟 K×rpm 工况）。
           s_sim_k==1 时保持真实值，SIM OFF 一键还原。 */
        if (s_sim_k > 1u) {
            int64_t sim = (int64_t)dpos * (int64_t)s_sim_k;
            dpos = (int32_t)((sim > INT32_MAX)  ? INT32_MAX : sim);
            dpos = (int32_t)((sim < INT32_MIN) ? INT32_MIN : sim);
        }

        /* A1: last 每次更新 */
        g_prev_pos = pos;
        g_prev_us  = us_now;

        if (dpos > 0)
            g_counts += (int64_t)dpos;
        else if (dpos < 0)
            g_counts += (int64_t)dpos;  /* handle wrap */
        g_missed += (uint32_t)((dpos > 0 ? (uint32_t)dpos : (uint32_t)(-dpos)) / 4000u);

        /* 事件差分 rpm：rpm = Δc×60e6/(Δt_us×4000) = Δc×15000/Δt_us */
        bool  has_event = (dpos != 0);
        int32_t event_rpm = 0;
        if (has_event && dt_us != 0u)
            event_rpm = (int32_t)(((int64_t)dpos * 15000) / dt_us);

if (has_event) {
            s_last_event_us = us_now;          /* 真实事件时刻（看门狗用） */
            /* 记录层事件抽稀：只对【真实事件】计数，每 div 个事件记 1 点。
               静止事件（has_event==0）不进来，天然"只记录事件"。
               低速 div=1 全记；高速稀疏记，4400 点覆盖 0.5~1.0s。 */
            s_snap_div     = calc_snap_div(event_rpm);   /* 档位随转速更新 */
            s_snap_evt_cnt++;
            if (s_snap_evt_cnt >= s_snap_div) {
                s_snap_evt_cnt = 0;
                snap_on_event(us_now, g_counts, g_index, event_rpm); /* 每 div 事件记 1 点 */
            }
            if (dt_us != 0u)                           /* UTO 周期测量（上轮→本轮） */
                g_period = dt_us;
        }
        g_ev_cnt++;
        g_uto_db++;

        abi_update_rpm_hold_decay(event_rpm, has_event);
        abi_set_uto_period(abi_get_latest_rpm());      /* 每次都调（B1） */

        HWREGH(Module_EQEP_BASE + EQEP_O_QCLR) = (uint16_t)EQEP_INT_UNIT_TIME_OUT;
    }

    if (flg & EQEP_INT_DIR_CHANGE) {
        g_qdc_db++;
        HWREGH(Module_EQEP_BASE + EQEP_O_QCLR) = (uint16_t)EQEP_INT_DIR_CHANGE;
    }

    if (flg & EQEP_INT_INDEX_EVNT_LATCH) {
        g_index++;
        { uint32_t cnt = Encoder_Count;
          if (g_last_ix_cnt != 0xFFFFFFFFu) {
              uint32_t ppr = cnt - g_last_ix_cnt;
              if (ppr != 0) {
                  if (Encoder_PulsePerRev == 0)
                      Encoder_PulsePerRev = ppr;
                  else
                      Encoder_PulsePerRev = (Encoder_PulsePerRev * 7u + ppr) / 8u;
              }
          }
          g_last_ix_cnt = cnt;
          Encoder_Index_Count++; }
        g_iel_db++;
        HWREGH(Module_EQEP_BASE + EQEP_O_QCLR) = (uint16_t)EQEP_INT_INDEX_EVNT_LATCH;
    }

    flg = HWREGH(Module_EQEP_BASE + EQEP_O_QFLG);
    if (flg) HWREGH(Module_EQEP_BASE + EQEP_O_QCLR) = flg;
    Interrupt_clearACKGroup(INT_Module_EQEP_INTERRUPT_ACK_GROUP);

#ifdef DEBUG
    /* Task 5 探针：CPUTIMER0 递减，delta = (t0 - t1 + period) % period */
    {
        uint32_t isr_t1 = HWREG(myCPUTIMER0_BASE + CPUTIMER_O_TIM);
        uint32_t cyc = (isr_t0 >= isr_t1) ? (isr_t0 - isr_t1)
                                          : (isr_t0 + TIMER_PERIOD - isr_t1);
        if (cyc > s_isr_dbg_max) s_isr_dbg_max = cyc;
        s_isr_dbg_sum   += cyc;
        s_isr_dbg_count++;
    }
#endif
}

/* ---- DEBUG 探针读取（Task 5 用，仅 DEBUG 构建） ---- */
#ifdef DEBUG
uint32_t abi_isr_dbg_max_cycles(void)   { return s_isr_dbg_max; }
uint32_t abi_isr_dbg_sum_cycles(void)   { return s_isr_dbg_sum; }
uint32_t abi_isr_dbg_count(void)        { return s_isr_dbg_count; }
#endif
