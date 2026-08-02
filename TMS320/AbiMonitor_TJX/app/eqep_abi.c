#include "eqep_abi.h"
#include "board.h"
#include "driverlib.h"
#include "../module_driver/bsp_motor_hallencoder.h"

/* ---- Clock ---- */
volatile uint64_t g_us64 = 0;
volatile uint32_t g_us_tick_snap = 0;
#define TIMER_PERIOD 150000u
#define UTO_PERIOD    75000u  /* 500us = 2kHz */

void abi_clock_tick(void) {
    g_us64 += 1000;
    g_us_tick_snap = HWREG(myCPUTIMER0_BASE + CPUTIMER_O_TCR);
}

uint64_t abi_now_us(void) {
    uint64_t base;
    uint32_t cnt, elap;
    base = g_us64;
    cnt  = HWREG(myCPUTIMER0_BASE + CPUTIMER_O_TCR);
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

    /* Enable UTO, keep IEL. Disable PCM and QDC interrupts */
    EQEP_disableInterrupt(Module_EQEP_BASE,
        EQEP_INT_POS_COMP_MATCH | EQEP_INT_DIR_CHANGE);
    EQEP_enableInterrupt(Module_EQEP_BASE, EQEP_INT_UNIT_TIME_OUT);
    EQEP_enableUnitTimer(Module_EQEP_BASE, UTO_PERIOD);
    EQEP_setLatchMode(Module_EQEP_BASE,
        EQEP_LATCH_CNT_READ_BY_CPU | EQEP_LATCH_UNIT_TIME_OUT);

    /* Disable index reset: keep QPOSCNT monotonic for proper delta calc */
    EQEP_setPositionCounterConfig(Module_EQEP_BASE, EQEP_POSITION_RESET_MAX_POS, 0xFFFFFFFEu);

    g_prev_pos = HWREG(Module_EQEP_BASE + EQEP_O_QPOSCNT);
}

int64_t  abi_counts(void)        { return g_counts; }
uint32_t abi_index_n(void)       { return g_index; }
int      abi_gear(void)          { return ABI_GEAR_4X; }
void     abi_set_gear(int g)     { (void)g; }
uint32_t abi_last_period_us(void){ return g_period; }
uint32_t abi_missed_events(void) { return g_missed; }
uint32_t abi_event_count(void)   { return g_ev_cnt; }
uint32_t abi_pcm_dbg(void)       { return g_uto_db; }
uint32_t abi_qdc_dbg(void)       { return g_qdc_db; }
uint32_t abi_iel_dbg(void)       { return g_iel_db; }

/* ---- EQEP ISR ---- */
__interrupt void INT_Module_EQEP_ISR(void)
{
    uint16_t flg = HWREGH(Module_EQEP_BASE + EQEP_O_QFLG);

    if (flg & EQEP_INT_UNIT_TIME_OUT)
    {
        uint32_t pos = HWREG(Module_EQEP_BASE + EQEP_O_QPOSCNT);
        int32_t  d   = (int32_t)(pos - g_prev_pos);
        g_prev_pos   = pos;
        if (d > 0)
            g_counts += (int64_t)d;
        else if (d < 0)
            g_counts += (int64_t)d;  /* handle wrap */
        g_missed += (uint32_t)((d > 0 ? (uint32_t)d : (uint32_t)(-d)) / 4000u);

        { uint32_t us_now = (uint32_t)(abi_now_us() & 0xFFFFFFFFuL);
          if (g_prev_us > 0 && us_now > g_prev_us)
              g_period = us_now - g_prev_us;
          g_prev_us = us_now; }
        g_ev_cnt++;
        g_uto_db++;
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
}
