/**
  ******************************************************************************
  * @file    abi.c
  * @brief   AS5047P ABI 编码器捕获（Task 2）
  *          - TIM2 编码器 4X（PA5=A, PA1=B, AF1），32 位 CNT
  *          - EXTI4（PA4）= Index，中断计数
  *          - TIM5 UTO 自适应定时器（32 位，60MHz 刻度）：每周期 ~N_MIN 步，
  *            动态 ARR（QUPR 夹顶 + 12.5% 迟滞），ISR 内累计 64 位硬件时间戳
  *          - 抽稀档位：按 rpm 查 app_config 档位表，切换档位重置 sub
  *          - 测速：事件差分 dpos×60e6/(dt_us×4000) rpm
  * 时钟链：HSE25 → SYSCLK480 → HCLK240 → PCLK1 120 → TIM5 = 2×PCLK1 = 240MHz
  *          PSC = 2×PCLK1/60MHz - 1 = 3（运行时推导，锁死 60MHz 刻度）
  ******************************************************************************
  */
#include "abi.h"
#include "app_config.h"
#include "stm32h7xx_hal.h"

/*------------------------------------------ 内部状态 -------------------------*/

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim5;

/* 64 位刻度计数（60MHz → 每刻度 1/60µs），TIM5 ISR 内累加，纯硬件零漂移 */
static volatile uint64_t g_tick64;

static volatile int32_t  g_rpm;      /* 事件差分测速（有符号） */
static volatile uint32_t g_idx_cnt;  /* Index 圈数 */

static uint8_t g_icf_cur = 0xFF;     /* 当前生效 IC 滤波值（0xFF=未初始化） */

/* Index→Index EMA 校准时基准：1 圈实测 4X 步数，EMA 平滑 */
static uint32_t g_ema_prev_cnt;
static volatile uint32_t g_ema_steps;   /* 0=未校准 */

/* 方向修正：cfg_pol=0（默认）A/B 反相接法 → 计数反向，取反对齐正向；
 * cfg_pol=1 正相接法 → 直接采用 CNT。ISR 与主循环共用。 */
static uint32_t SignCnt(uint32_t raw)
{
    return cfg_pol ? raw : (0u - raw);
}

/*------------------------------------------ TIM2 编码器 4X --------------------*/

static void Tim2_Encoder_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = { 0 };
    gpio.Pin       = GPIO_PIN_5 | GPIO_PIN_1;      /* PA5=A, PA1=B */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFFFFFu;   /* 32 位计数 */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef enc = { 0 };
    enc.EncoderMode = TIM_ENCODERMODE_TI12;       /* 4X */
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;  /* TI1 → IC1 */
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;  /* TI2 → IC2 */
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    /* 初始输入滤波：取动态表低速档（最强滤波档，ISR 首拍按转速接管） */
    enc.IC1Filter   = Config_GetFltIcf(0) & 0x0F;
    enc.IC2Filter   = Config_GetFltIcf(0) & 0x0F;
    if (HAL_TIM_Encoder_Init(&htim2, &enc) != HAL_OK) {
        while (1) { }
    }
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);            /* 清零计数起点（上电 CNT 是垃圾值） */
    g_icf_cur = Config_GetFltIcf(0) & 0x0F;
}

/*------------------------------------------ EXTI4 Index ----------------------*/

static void Exti4_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = { 0 };
    gpio.Pin  = GPIO_PIN_4;                        /* PA4 = Index */
    gpio.Mode = GPIO_MODE_IT_RISING;               /* 只上升沿：每转 +1（RISING_FALLING 会每转 +2） */
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);        /* 低于 UTO(TIM5) */
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void EXTI4_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
        Snap_OnIndex();                 /* state 无关回调（记录打到 Index 帧） */
        uint32_t cnt = SignCnt(TIM2->CNT);   /* 用事件时刻 counts 校准 */
        if (g_ema_prev_cnt) {
            uint32_t d = cnt - g_ema_prev_cnt;
            if (d > 0 && d < 100000u) {
                if (!g_ema_steps) g_ema_steps = d;          /* 首圈直接取 */
                else g_ema_steps = (uint32_t)(((uint64_t)g_ema_steps * 7 + d) / 8);
            }
        }
        g_ema_prev_cnt = cnt;
        g_idx_cnt++;
    }
}

/*------------------------------------------ TIM5 UTO -------------------------*/

static void Tim5_Uto_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();

    htim5.Instance               = TIM5;
    /* PSC 由实际时钟推导，锁死 60MHz 刻度：2×PCLK1 / 60MHz - 1 */
    htim5.Init.Prescaler         = (uint32_t)((2uL * HAL_RCC_GetPCLK1Freq()) / 60000000uL) - 1uL;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = UTO_QUPR_MIN_60M;   /* 初始 60kHz 夹顶 */
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        while (1) { }
    }

    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    HAL_TIM_Base_Start_IT(&htim5);
}

void TIM5_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) == RESET) return;
    __HAL_TIM_CLEAR_IT(&htim5, TIM_FLAG_UPDATE);

    /* 1) 64 位刻度累计（零漂移）：累计"本周期实际生效的 ARR（影子值）"而非预装载值。
     *    ARPE=1：写 ARR 进预装载，经一个周期才锁存到影子。本周期实际时长 = 上次
     *    写入的 target（/未变化时的当前值），提前量高于读 ARR 会偏一个周期 →
     *    UTO 自适应 + LOG 差分对 ARR 跳变敏感（尖刺来源之一），这里修正贴合实际。 */
    static uint32_t g_arr_used = UTO_QUPR_MIN_60M;
    g_tick64 += g_arr_used;
    uint32_t us_now = (uint32_t)(g_tick64 / 60uL);

    /* 2) 事件读取：A/B 相序可能使 TIM2 反向计数，SignCnt 取反保证正向=递增 */
    uint32_t cnt = SignCnt(TIM2->CNT);
    static uint32_t prev_cnt;
    int32_t dpos = (int32_t)(cnt - prev_cnt);
    prev_cnt = cnt;

/* 3) 测速：相邻事件差分 + 原始刻度（1/60µs 分辨率）+ 动态 EMA。
     *    不用 µs 取整（g_tick64/60 丢精度 → 高速量化锯齿），直接用 60MHz 刻度。
     *    不用滑窗（固定 8 事件跨度因 UTO ARR 阶梯跳变而在时间上不均匀）。
     *    动态 EMA：低速 α=20% 快速跟踪，高速 α=4% 强平滑；最小步进 ±1 防卡零。
     *    测速参数可调（FILT 命令保留）：dpos 拒限 / 静止归零。 */
    static uint64_t prev_tick64;
    uint64_t dt_tick = g_tick64 - prev_tick64;
    prev_tick64 = g_tick64;
    if (dt_tick) {
        int64_t rpm_inst = 0;
        static uint8_t zero_cnt;
        int32_t dlim = (int32_t)Config_GetFltDpos() * N_MIN;    /* 可调 dpos 上限 */
        if (dpos && dpos > -dlim && dpos < dlim) {
            rpm_inst = (int64_t)dpos * 60000000LL * 60LL / (int64_t)dt_tick /
                       (int64_t)Abi_GetStepsPerRev();
            if (rpm_inst > 60000)  rpm_inst = 60000;   /* 合理性钳制 */
            if (rpm_inst < -60000) rpm_inst = -60000;
            zero_cnt = 0;
        } else if (!dpos) {
            if (++zero_cnt >= Config_GetFltZero()) { g_rpm = 0; zero_cnt = 0; }
        }
        if (rpm_inst) {
            /* 归零判据：|rpm|<10 连续 N 周期视为噪音归零 */
            if (rpm_inst > -10 && rpm_inst < 10) {
                if (++zero_cnt >= Config_GetFltZero()) { g_rpm = 0; zero_cnt = 0; }
            } else {
                zero_cnt = 0;
            }
            /* 动态 EMA：低速 α=20%（快速跟踪），中速 10%，高速 4%（强平滑）。
             * 电机惯性大，高速大幅平滑不丢物理精度；最小步进 ±1 防卡零。 */
            uint32_t ra = (uint32_t)(rpm_inst > 0 ? rpm_inst : -rpm_inst);
            uint32_t alpha = (ra < 2000u) ? 200u : (ra < 4000u) ? 100u : 40u;
            int64_t step = rpm_inst - (int64_t)g_rpm;
            if (step) {
                int64_t e = step * (int64_t)alpha / 1000;
                if (e == 0) e = (step > 0) ? 1 : -1;   /* 最小步进 ±1 */
                g_rpm = (int32_t)((int64_t)g_rpm + e);
            }
            if (g_rpm > -1 && g_rpm < 1) g_rpm = 0;
        }
    }

    /* 4) 每事件记录（不抽稀，SD 卡支持大容量） */
    Snap_OnEvent(cnt, g_idx_cnt, us_now, (uint32_t)(g_tick64 % 60uL));

    /* 5) 动态周期：保持每周期 ~N_MIN 步，夹顶 + 迟滞防抖 */
    uint32_t target = (uint32_t)((uint64_t)htim5.Instance->ARR * N_MIN /
                                 (dpos ? (uint32_t)(dpos < 0 ? -dpos : dpos) : N_MIN));
    if (target < UTO_QUPR_MIN_60M) target = UTO_QUPR_MIN_60M;
    if (target > UTO_QUPR_MAX_60M) target = UTO_QUPR_MAX_60M;
    /* ARPE=1：写 ARR 进预装载，经一个周期后事件才锁存到影子（计数器用它计时）。
     * 因此下一次 ISR 的真实步长不是 target，而是「本周期锁存时预装载里的旧值」，
     * 即 ISR 顶部读到的 arr（未写前的旧 ARR）。所以 g_arr_used 保存 arr 而不是 target。
     * 迟滞避免 target 微抖动→每次改写 ARR（否则锁存延迟会持续错位）。 */
    uint32_t arr = htim5.Instance->ARR;   /* 锁存后的旧 ARR（本周期实际时长） */
    __HAL_TIM_SET_AUTORELOAD(&htim5, target);   /* 下一周期生效 */
    g_arr_used = arr;                    /* 记录本周期真实步长，供下个 ISR 的 tick64 累加 */
    prev_cnt = cnt;
}

/*------------------------------------------ 对外接口 -------------------------*/

void Abi_Init(void)
{
    Tim2_Encoder_Init();
    Exti4_Init();
    Tim5_Uto_Init();
}

int32_t Abi_GetRpm(void)      { return (int32_t)g_rpm; }
uint32_t Abi_GetIndexCnt(void){ return g_idx_cnt; }
uint32_t Abi_GetUsNow(void)   { return (uint32_t)(g_tick64 / 60uL); }
uint32_t Abi_GetCnt(void)     { return SignCnt(TIM2->CNT); }   /* 与 UTO ISR 同向修正 */

/* 测速一圈步数：持久化配置 cfg_steps_per_rev（默认 4000），恒 >=1 防除零 */
uint32_t Abi_GetStepsPerRev(void)
{
    return (cfg_steps_per_rev && cfg_steps_per_rev < 200000u) ? cfg_steps_per_rev : STEPS_PER_REV;
}

/* Index 校准 EMA 当前值（0=尚未拿到首圈差分） */
uint32_t Abi_GetCalib(void)   { return g_ema_steps; }

/* CAL SET：把 EMA 步骤写入持久化配置并清零重校 */
void Abi_SetCalibSteps(uint32_t steps)
{
    Config_SetStepsPerRev(steps);
    g_ema_steps = 0;
    g_ema_prev_cnt = 0;
}

/* 查档位表：|rpm| < bnd[i] → div[i]，否则末档 */
uint8_t Abi_GetDiv(void)
{
    int32_t rpm = Abi_GetRpm();
    if (rpm < 0) rpm = -rpm;
    for (uint8_t i = 0; i + 1 < cfg_gear_n; i++) {
        if ((uint32_t)rpm < cfg_gear_bnd[i]) return cfg_gear_div[i];
    }
    return cfg_gear_div[cfg_gear_n - 1];
}

/* Task 8 动态 IC 输入滤波：读-改-写 TIM2 CCMR1（IC1F bit[7:4]、IC2F bit[15:12]）。
 * HAL 编码器模式 IC1Filter<<4 / IC2Filter<<12（见 stm32h7xx_hal_tim.c:3108），
 * 文档里写 bit[3:0]/bit[11:8] 是错误的，这里按 HAL 位偏移写。不停 TIM2。 */
void Abi_SetInputFilter(uint8_t val)
{
    if (val > 0x0F) val = 0x0F;
    TIM2->CCMR1 = (TIM2->CCMR1 & ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F)) |
                  ((uint32_t)val << 4U) | ((uint32_t)val << 12U);
}

uint8_t Abi_GetInputFilter(void)
{
    return (uint8_t)(((TIM2->CCMR1 & TIM_CCMR1_IC1F) >> 4U) & 0x0F);
}

/* Task 2 空实现（weak）：Task 3 snap_bin.c 强定义覆盖 */
#if defined(__GNUC__)
__attribute__((weak))
#endif
void Snap_OnEvent(uint32_t cnt, uint32_t idx, uint32_t us_now, uint32_t sub_us)
{
    (void)cnt; (void)idx; (void)us_now; (void)sub_us;
}

#if defined(__GNUC__)
__attribute__((weak))
#endif
void Snap_OnIndex(void)
{
}
