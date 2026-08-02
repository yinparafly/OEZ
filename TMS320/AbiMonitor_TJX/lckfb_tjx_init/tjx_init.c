#include "tjx_init.h"

/* Delay Function */
void delay_us(int us) { DEVICE_DELAY_US(1); }
void delay_1us(int us) { DEVICE_DELAY_US(1); }
void delay_ms(int ms) { while(ms--){DEVICE_DELAY_US(1000);} }
void delay_1ms(int ms) { while(ms--){DEVICE_DELAY_US(1000);} }
void delay_s(int s) { while(s--){delay_ms(1000);} }


static void SCI_Send_1char(char dat)
{
    SCI_writeCharArray(SCIA_BASE, (uint16_t *)&dat, 1);
}
static void SCI_Send_String(char * string)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while( *string!=0 && string!=0 )
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        SCI_Send_1char(*string++);
    }
}

int lc_printf(char* format, ...)
{
    va_list args;
    va_start(args, format);

    char buffer[255] = {0};
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // 发送数据
    SCI_Send_String(buffer);

    return len;
}

