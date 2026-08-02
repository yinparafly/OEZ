/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 文档网站：wiki.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 嘉立创社区问答：https://www.jlc-bbs.com/lckfb
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 */

#include "bsp_motor_hallencoder.h"

volatile uint32_t Encoder_Count = 0;         // 编码器计数，用于用户使用。
volatile uint32_t Encoder_Last_Count = 0;    // 存储上一次采样时的编码器位置值
volatile int32_t  Motor_dir = 0;             // 电机旋转方向（正转/反转）
volatile uint32_t Encoder_PulsePerRev = 0;   // 每转脉冲数（由 INDEX 事件校准）
volatile uint32_t Encoder_Index_Count = 0;   // INDEX 事件计数
volatile uint32_t Encoder_Last_Index_Count = 0; // 已弃用，INDEX 校准移至 app/eqep_abi.c
/*
volatile uint32_t Encoder_Last_Index_Count = 0; // 上一次 INDEX 时的累计计数
*/
bool Encoder_InitDone   = false;


void Encoder_Init(void)
{
    Encoder_Last_Count = EQEP_getPosition(Module_EQEP_BASE);
    Encoder_InitDone   = false;
}

/******************************************************************
 * 函 数 名 称：Motor_Init
 * 函 数 说 明：EPWM1A 输出 50Hz RC 电调信号（1ms 停 / 2ms 全速）
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：SYSCLK=150MHz，TBCLK=150M/64=2.34375MHz
               TBPRD=46875 -> 50Hz；CMPA: 2343(1ms) ~ 4687(2ms)
******************************************************************/
void Motor_Init(void)
{
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_64, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM1_BASE, 46875);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0);
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE,
        EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 2343);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);
}

/******************************************************************
 * 函 数 名 称：Motor_Set_PWM
 * 函 数 说 明：左右电机速度控制
 * 函 数 形 参：Mode: 1左转 2右转 0停止
               Speed: 速度
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：PWM值范围0-9999
******************************************************************/
void Motor_Set_PWM(uint8_t Mode, uint16_t Speed)
{
    uint32_t cmp = 2343u;   // 默认 1ms = 停
    if ((Mode != 0) && (Speed > 0)) {
        cmp = 2343u + (uint32_t)Speed * 2344u / 9999u;
        if (cmp > 4687) {
            cmp = 4687;
        }
    }
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, cmp);
}

/******************************************************************
 * 函 数 名 称：Get_Encoder_Dir
 * 函 数 说 明：获取方向
 * 函 数 形 参：
 * 函 数 返 回：
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
int32_t Get_Encoder_Dir(void)
{
    return Motor_dir;
}

/******************************************************************
 * 函 数 名 称：Get_Encoder_Value
 * 函 数 说 明：获取编码器的值
 * 函 数 形 参：
 * 函 数 返 回：
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
uint32_t Get_Encoder_Value(void)
{ 
    return Encoder_Count;
}

//
// 由 1kHz CPU Timer 周期调用，实时采样 EQEP 位置并累加
// INDEX 校准在 INT_Module_EQEP_ISR 中完成（事件同步，不丢事件）
//
void Encoder_Periodic_Update(void)
{
    uint32_t newCount = EQEP_getPosition(Module_EQEP_BASE);
    int32_t  newDir   = EQEP_getDirection(Module_EQEP_BASE);

    if (!Encoder_InitDone) {
        Encoder_Last_Count = newCount;
        Encoder_InitDone   = true;
        return;
    }

    int32_t diff = (int32_t)(newCount - Encoder_Last_Count);
    Encoder_Last_Count = newCount;
    Encoder_Count     += (uint32_t)ABS(diff);
    Motor_dir          = newDir;
}

//
// EQEP 事件 ISR：已移至 app/eqep_abi.c（PCM 逐边沿事件内核）
// 原 INDEX 校准逻辑已合并到 eqep_abi.c 中
//
/*
__interrupt void INT_Module_EQEP_ISR(void)
{
    uint32_t intStatus = EQEP_getInterruptStatus(Module_EQEP_BASE);

    if (intStatus & EQEP_INT_INDEX_EVNT_LATCH) {
        uint32_t cnt = Encoder_Count;
        if (Encoder_Index_Count != 0xFFFFFFFFU) {
            uint32_t ppr = cnt - Encoder_Last_Index_Count;
            if (ppr != 0) {
                if (Encoder_PulsePerRev == 0) {
                    Encoder_PulsePerRev = ppr;
                } else {
                    Encoder_PulsePerRev = (Encoder_PulsePerRev * 7u + ppr) / 8u;
                }
            }
        }
        Encoder_Last_Index_Count = cnt;
        Encoder_Index_Count++;
    }

    EQEP_clearInterruptStatus(Module_EQEP_BASE,
        EQEP_INT_POS_COMP_MATCH|EQEP_INT_DIR_CHANGE|EQEP_INT_INDEX_EVNT_LATCH|EQEP_INT_GLOBAL);
    Interrupt_clearACKGroup(INT_Module_EQEP_INTERRUPT_ACK_GROUP);
}
*/
