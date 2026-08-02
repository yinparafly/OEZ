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
volatile uint32_t Encoder_Last_Count = 0;    // 存储上一次中断时的编码器位置值
volatile int32_t  Motor_dir = 0;             // 电机旋转方向（正转/反转）
bool Encoder_InitDone   = false;


void Encoder_Init(void)
{
    Encoder_Last_Count = EQEP_getPositionLatch(Module_EQEP_BASE);
    Encoder_InitDone   = false;
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
    if(Mode == 1){
        EPWM_setCounterCompareValue(Module_PWM_M_BASE, EPWM_COUNTER_COMPARE_A, 0);
        EPWM_setCounterCompareValue(Module_PWM_P_BASE, EPWM_COUNTER_COMPARE_A, Speed);
    }else if(Mode == 2){
        EPWM_setCounterCompareValue(Module_PWM_M_BASE, EPWM_COUNTER_COMPARE_A, Speed);
        EPWM_setCounterCompareValue(Module_PWM_P_BASE, EPWM_COUNTER_COMPARE_A, 0);
    }else if (Mode == 0) {
        EPWM_setCounterCompareValue(Module_PWM_M_BASE, EPWM_COUNTER_COMPARE_A, 0);
        EPWM_setCounterCompareValue(Module_PWM_P_BASE, EPWM_COUNTER_COMPARE_A, 0);
    }else {
        EPWM_setCounterCompareValue(Module_PWM_M_BASE, EPWM_COUNTER_COMPARE_A, 0);
        EPWM_setCounterCompareValue(Module_PWM_P_BASE, EPWM_COUNTER_COMPARE_A, 0);
    }
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
// 每周期触发一次中断
//
__interrupt void INT_Module_EQEP_ISR(void)
{
    uint32_t newCount = EQEP_getPositionLatch(Module_EQEP_BASE);
    int32_t newDir = EQEP_getDirection(Module_EQEP_BASE);

    if (!Encoder_InitDone) {
        Encoder_Last_Count = newCount;
        Encoder_InitDone   = true;
        EQEP_clearInterruptStatus(Module_EQEP_BASE,EQEP_INT_UNIT_TIME_OUT|EQEP_INT_GLOBAL);
        Interrupt_clearACKGroup(INT_Module_EQEP_INTERRUPT_ACK_GROUP);
        return;
    }

    int32_t diff = (int32_t)(newCount - Encoder_Last_Count);
    uint32_t delta = (uint32_t)ABS(diff);

    Encoder_Count      = delta;
    Encoder_Last_Count = newCount;
    Motor_dir          = newDir;

   EQEP_clearInterruptStatus(Module_EQEP_BASE,EQEP_INT_UNIT_TIME_OUT|EQEP_INT_GLOBAL);
   Interrupt_clearACKGroup(INT_Module_EQEP_INTERRUPT_ACK_GROUP);
}
