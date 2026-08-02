/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 文档网站：wiki.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 嘉立创社区问答：https://www.jlc-bbs.com/lckfb
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 */

#ifndef	__BSP_MOTOR_HALLENCODER_H__
#define __BSP_MOTOR_HALLENCODER_H__

#include "tjx_init.h"

// 获得绝对值
#define ABS(a)      (a>0 ? a:(-a))

extern volatile uint32_t Encoder_Count;
extern volatile int32_t  Motor_dir;
extern volatile uint32_t Encoder_PulsePerRev;
extern volatile uint32_t Encoder_Index_Count;

void Encoder_Init(void);
void Encoder_Periodic_Update(void);
void Motor_Init(void);
int32_t Get_Encoder_Dir(void);
uint32_t Get_Encoder_Value(void);
void Motor_Set_PWM(uint8_t Mode, uint16_t Speed);

#endif

