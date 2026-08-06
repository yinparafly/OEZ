/***
	*************************************************************************************************
	*	@file  	main.c
	*	@version V0.1
	*	@date    2026-08-06
	*	@brief   AS5047P ABI 编码器事件流监控/记录器（Task 1 骨架）
	*************************************************************************************************
	*	时钟链：HSE25M -> PLL 480M -> AHB/2 = 240M -> APB1/2 = 120M
	*	USART1: PA9 TX / PA10 RX，921600,8N1，中断收
	*	LED:    PC13（低电平点亮）
	*************************************************************************************************
***/

#include "main.h"
#include "led.h"
#include "usart.h"
#include "app_cli.h"
#include "app_config.h"
#include "app_pwm.h"
#include "abi.h"
#include "snap_bin.h"

void SystemClock_Config(void);

int main(void)
{
	SCB_EnableICache();
	SCB_EnableDCache();
	HAL_Init();
	SystemClock_Config();

	LED_Init();			// LED 初始化
	Usart_Init();		// 串口 921600
	Config_Load();		// 读取 Flash 配置（失效则重置默认）
	Pm_Init();			// 测试 PWM（TIM3_CH2/PA7，占空比 0）
	Abi_Init();			// ABI 编码器：TIM2 4X + EXTI4 Index + TIM5 UTO
	Snap_Init();		// Task 3 事件记录管线
	Cli_Init();			// 打印 banner

	while (1)
	{
		Pm_Tick();		// PWM 油门渐变逼近目标（电调安全斜坡）
		Cli_Poll();		// 轮询处理命令行
	}
}

/**
  * @brief  System Clock Configuration
  *         SYSCLK = 480MHz, HCLK = 240MHz, APB1/APB2/APB3/APB4 = 120MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
