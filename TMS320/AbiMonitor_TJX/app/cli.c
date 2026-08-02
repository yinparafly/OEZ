#include "cli.h"
#include "sd_fatfs.h"
#include "eqep_abi.h"
#include "speed_est.h"
#include "fatfs/diskio.h"
#include "board.h"
#include "driverlib.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define CLI_SCI_BASE Debug_Serial_BASE

static void cli_putc(char c)
{
    if (c == '\n')
    {
        uint16_t cr = (uint16_t)'\r';
        SCI_writeCharArray(CLI_SCI_BASE, &cr, 1);
    }
    uint16_t dat = (uint16_t)(unsigned char)c;
    SCI_writeCharArray(CLI_SCI_BASE, &dat, 1);
}

void cli_put_raw(const char *s)
{
    while (*s)
        cli_putc(*s++);
}

void cli_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cli_put_raw(buf);
}

// ---- RX 环形缓冲区（ISR 写入，主循环读取） ----
#define CLI_RX_BUF_SIZE 128
static volatile char  cli_rx_buf[CLI_RX_BUF_SIZE];
static volatile uint8_t cli_rx_head;   // ISR 写入位置
static volatile uint8_t cli_rx_tail;   // cli_task 读取位置

// 命令行组装缓冲区
static char     cmd_buf[64];
static uint16_t cmd_len;

void cli_init(void)
{
    cli_rx_head = 0;
    cli_rx_tail = 0;
    cmd_len = 0;
    // SCI RX 中断由 board.c 的 Board_init() 配置（SCI_INT_RXRDY_BRKDT）
    // ISR 负责读取字符 → cli_push_rx() → 环形缓冲区
    // cli_task() 轮询消费缓冲区（无需关中断，环形缓冲区单生产者/单消费者安全）
}

// ISR 调用：推入一个字符到环形缓冲区
void cli_push_rx(char c)
{
    uint8_t next = (uint8_t)(cli_rx_head + 1U);
    if (next >= CLI_RX_BUF_SIZE)
        next = 0;
    // 缓冲区满则丢弃（不阻塞 ISR）
    if (next == cli_rx_tail)
        return;
    cli_rx_buf[cli_rx_head] = c;
    cli_rx_head = next;
}

// 从环形缓冲区取一个字符，返回 0 表示空
static int cli_pop_rx(void)
{
    if (cli_rx_tail == cli_rx_head)
        return -1;
    char c = cli_rx_buf[cli_rx_tail];
    uint8_t next = (uint8_t)(cli_rx_tail + 1U);
    if (next >= CLI_RX_BUF_SIZE)
        next = 0;
    cli_rx_tail = next;
    return (int)(unsigned char)c;
}

static void cli_handle(char *line)
{
    if (strcmp(line, "PING") == 0)
    {
        cli_put_raw("# PONG\n");
    }
    else if (strcmp(line, "FW?") == 0)
    {
        cli_put_raw("# FW 0.1.0-PCMREBUILD\n");
    }
    else if (strcmp(line, "SD?") == 0)
    {
        DSTATUS st = disk_status(0);
        extern volatile uint8_t sd_dbg_step, sd_dbg_r1;
        cli_printf("# SD %s step=%u r1=0x%02X\n",
                   (st & STA_NOINIT) ? "NOINIT" : "READY",
                   (unsigned)sd_dbg_step, (unsigned)sd_dbg_r1);
    }
    else if (strcmp(line, "SD INIT") == 0)
    {
        if (sd_fatfs_init() == 0)
            cli_put_raw("# SD OK\n");
        else
            cli_printf("# SD FAIL %s\n", sd_fatfs_last_err());
    }
    else if (strcmp(line, "SD TEST") == 0)
    {
        if (sd_fatfs_test() == 0)
            cli_put_raw("# SDTEST OK\n");
        else
            cli_printf("# SDTEST FAIL %s\n", sd_fatfs_last_err());
    }
    else if (strcmp(line, "SPI LB") == 0)
    {
        extern void sd_spi_loopback_test(void);
        extern volatile uint8_t sd_loopback_ok;
        sd_spi_loopback_test();
        cli_printf("# SPI LB %s\n", sd_loopback_ok ? "OK" : "FAIL");
    }
    else if (strcmp(line, "QPW") == 0)
    {
        /* Write 0x0043 to QPOSCTL and read back */
        volatile uint16_t *p = (volatile uint16_t *)(Module_EQEP_BASE + EQEP_O_QPOSCTL);
        *p = 0x0043;
        cli_printf("# QPW -> 0x%04X\n", (unsigned)*p);
    }
    else if (strcmp(line, "QSTS") == 0)
    {
        uint16_t s = HWREGH(Module_EQEP_BASE + EQEP_O_QEPSTS);
        cli_printf("# QEPSTS=0x%04X\n", (unsigned)s);
    }
    else if (strcmp(line, "QCTL") == 0)
    {
        uint16_t c = HWREGH(Module_EQEP_BASE + EQEP_O_QPOSCTL);
        cli_printf("# QPOSCTL=0x%04X\n", (unsigned)c);
    }
    else if (strcmp(line, "QCMP") == 0)
    {
        uint32_t c = HWREG(Module_EQEP_BASE + EQEP_O_QPOSCMP);
        uint32_t p = HWREG(Module_EQEP_BASE + EQEP_O_QPOSCNT);
        cli_printf("# QCMP=%lu QPOS=%lu\n", (unsigned long)c, (unsigned long)p);
    }
    else if (strcmp(line, "QEINT") == 0)
    {
        uint16_t e = HWREGH(Module_EQEP_BASE + EQEP_O_QEINT);
        cli_printf("# QEINT=0x%04X\n", (unsigned)e);
    }
    else if (strcmp(line, "QFLG") == 0)
    {
        uint16_t f = HWREGH(Module_EQEP_BASE + EQEP_O_QFLG);
        cli_printf("# QFLG=0x%04X\n", (unsigned)f);
    }
    else if (strcmp(line, "SPD?") == 0)
    {
        cli_printf("# RPM=%lu gear=%d ms=%lu\n", (unsigned long)spd_rpm(), spd_gear(), (unsigned long)spd_abs_ms());
    }
    else if (strcmp(line, "ABI?") == 0)
    {
        DINT;
        int64_t cnt = abi_counts();
        EINT;
        cli_printf("# ABI cnt=%lld idx=%lu miss=%lu pcm=%lu qdc=%lu iel=%lu g=%d per=%lu\n",
                   cnt, (unsigned long)abi_index_n(),
                   (unsigned long)abi_missed_events(),
                   (unsigned long)abi_pcm_dbg(),
                   (unsigned long)abi_qdc_dbg(),
                   (unsigned long)abi_iel_dbg(),
                   (int)abi_gear(),
                   (unsigned long)abi_last_period_us());
    }
    else if (strcmp(line, "HELP") == 0)
    {
        cli_put_raw("# CMDS: PING FW? SD? SD INIT SD TEST SPI LB ABI? HELP\n");
    }
    else
    {
        cli_put_raw("# UNKNOWN\n");
    }
}

void cli_task(void)
{
    int c;
    // 从环形缓冲区逐字符消费（可以一次处理多个字符）
    while ((c = cli_pop_rx()) >= 0)
    {
        if (c == '\n' || c == '\r')
        {
            if (cmd_len > 0)
            {
                cmd_buf[cmd_len] = '\0';
                cli_handle(cmd_buf);
                cmd_len = 0;
            }
        }
        else if (cmd_len < sizeof(cmd_buf) - 1)
        {
            cmd_buf[cmd_len++] = (char)c;
        }
    }
}
