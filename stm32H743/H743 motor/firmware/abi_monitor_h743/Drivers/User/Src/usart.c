/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART1 串口驱动（中断收 + 轮询发），PA9=TX / PA10=RX
  * @note    波特率固定 CFG_UART_BAUD=921600。超时校验可选择开（默认关）。
  ******************************************************************************
  */
#include "usart.h"
#include "app_config.h"
#include "app_cli.h"
#include "stm32h7xx_hal.h"

#define RX_FIFO_SIZE   256

static volatile uint8_t  rx_fifo[RX_FIFO_SIZE];
static volatile uint16_t rx_head, rx_tail;

static UART_HandleTypeDef huart1;

static void Usart_MspInit(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = { 0 };
    gpio.Pin   = GPIO_PIN_9 | GPIO_PIN_10;     /* PA9=TX, PA10=RX */
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void Usart_Init(void)
{
    rx_head = rx_tail = 0;

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = CFG_UART_BAUD;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    Usart_MspInit();

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        while (1) { }
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void Usart_PutChar(char c)
{
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET) {}
    huart1.Instance->TDR = c;
}

void Usart_Print(const char *s)
{
    while (*s) { Usart_PutChar(*s++); }
}

void Usart_Printc(char c) { Usart_PutChar(c); }

void Usart_Write(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) Usart_PutChar((char)buf[i]);
}

uint8_t Usart_GetChar(char *c)
{
    if (rx_head == rx_tail) return 0;
    *c = (char)rx_fifo[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) % RX_FIFO_SIZE);
    return 1;
}

void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        uint16_t next = (uint16_t)((rx_head + 1) % RX_FIFO_SIZE);
        char c = (char)(huart1.Instance->RDR);
        if (next != rx_tail) {           /* 未满则入队 */
            rx_fifo[rx_head] = (uint8_t)c;
            rx_head = next;
        }
        Cli_OnChar(c);
    }
}