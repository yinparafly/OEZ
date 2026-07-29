#include "usart_cmd.h"
#include "main.h"

static volatile bool start_flag = false;

static volatile bool test_pending = false;
static volatile bool verify_pending = false;
static uint8_t test_port = 0;
static uint16_t test_pin_mask = 0;
static uint8_t cmd_buf[4];
static uint8_t cmd_len = 0;

void USART1_Init(uint32_t baud) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

bool Cmd_GetStartFlag(void) { return start_flag; }
void Cmd_ClearStartFlag(void) { start_flag = false; }

bool Cmd_GetPinTest(uint8_t* port, uint16_t* pin_mask) {
    if (!test_pending) return false;
    *port = test_port;
    *pin_mask = test_pin_mask;
    test_pending = false;
    return true;
}

bool Cmd_GetVerify(uint8_t* port, uint16_t* pin_mask) {
    if (!verify_pending) return false;
    *port = test_port;
    *pin_mask = test_pin_mask;
    verify_pending = false;
    return true;
}

void Cmd_ProcessChar(uint8_t c) {
    if (c == CMD_START_CHAR) {
        start_flag = true;
        return;
    }
    if (c == '1') { GREEN_LED_ON(); return; }
    if (c == '2') { GREEN_LED_OFF(); return; }
    if (c == '3') { RED_LED_ON(); return; }
    if (c == '4') { RED_LED_OFF(); return; }
    if (c == 'g') { GREEN_LED_TOGGLE(); return; }
    if (c == 'r') { RED_LED_TOGGLE(); return; }

    if (c == 'T' || c == 'V') {
        cmd_len = 1;
        cmd_buf[0] = c;
        test_pending = false;
        verify_pending = false;
        return;
    }

    if (cmd_len > 0 && (cmd_buf[0] == 'T' || cmd_buf[0] == 'V')) {
        cmd_buf[cmd_len++] = c;
        if (cmd_len >= 4) {
            test_port = cmd_buf[1];
            test_pin_mask = ((uint16_t)cmd_buf[2] << 8) | cmd_buf[3];
            if (cmd_buf[0] == 'T') test_pending = true;
            else verify_pending = true;
            cmd_len = 0;
        }
    }
}
