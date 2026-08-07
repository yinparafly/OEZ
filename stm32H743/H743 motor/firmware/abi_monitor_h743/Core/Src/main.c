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
#include "sd_card.h"
#include "flash_save.h"

/* Task 5：DONE 上升沿自动保存（每次触发只保存一次） */
static uint8_t g_done_prev;

void SystemClock_Config(void);

int main(void)
{
	/* 注意：不使能 DCache——SDMMC 内部 DMA 直访内存，DCache 会导致
	 * 读写缓冲不一致（读回陈旧 cache/写时 DMA 取脏数据）。数据量小，无需 cache。 */
	SCB_EnableICache();
	HAL_Init();
	SystemClock_Config();

	LED_Init();			// LED 初始化
	Usart_Init();		// 串口 921600
	Config_Load();		// 读取 Flash 配置（失效则重置默认）
	Pm_Init();			// 测试 PWM（TIM3_CH2/PA7，占空比 0）
	Abi_Init();			// ABI 编码器：TIM2 4X + EXTI4 Index + TIM5 UTO
	Snap_Init();		// Task 3 事件记录管线
	Cli_Init();			// 打印 banner
	SD_Init();			// Task 4 SD 卡（SDMMC1，失败不阻塞）
	Sd_Mount();			// 卡已插则挂载（无卡约 5s 超时后继续）
	g_done_prev = 0;

	while (1)
	{
		Pm_Tick();		// PWM 油门渐变逼近目标（电调安全斜坡）
		Cli_Poll();		// 轮询处理命令行

		/*
		 * Task 5 自动保存（芯片优先，SD 备份在后）：
		 * DONE 上升沿（每次触发完成）-> 先写 Flash（无卡也能存，掉电不丢），
		 * 再写 SD（卡已挂载时）。卡未挂载时跳过 SD（SD INIT 后可手动补录）。
		 * DONE 状态保持，兼容 SNAP 轮询验证。
		 */
		uint8_t done = Snap_IsReady();
		if (done && !g_done_prev) {      // DONE 上升沿：本次触发刚完成
			Flash_SaveSnap();		 // 芯片保存（内部打印 ok/fail）
			if (SD_CardMounted())
				Sd_SaveSnap();		 // SD 备份（内部打印 ok/fail）
		}
		g_done_prev = done;
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
