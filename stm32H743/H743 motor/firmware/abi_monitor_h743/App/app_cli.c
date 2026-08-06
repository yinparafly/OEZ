/**
  ******************************************************************************
  * @file    app_cli.c
  * @brief   串口命令行：HELP / ID / CFG SHOW / CFG SAVE / CFG RESET / CFG GEAR
  * @note    上游 usart.c 收满一行（收到 \r\n）调用 Cli_OnLine()，
  *          Cli_Poll() 每次主循环轮询处理行缓冲。
  ******************************************************************************
  */
#include "app_cli.h"
#include "app_config.h"
#include "app_pwm.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>

#define CLI_LINE_MAX   80       /* 一行最大长度 */
#define CLI_ARG_MAX    9        /* 参数最大个数 */

static char     cli_buf[CLI_LINE_MAX + 2];
static uint16_t cli_len;

/* 分隔出空格/tab 分隔的 token；返回个数 */
static uint8_t Cli_Split(char *line, char *argv[], uint8_t max)
{
    uint8_t n = 0;
    char *p = line;
    while (*p && n < max) {
        while (*p == ' ' || *p == '\t') *p++ = 0;
        if (!*p) break;
        argv[n++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
    }
    return n;
}

/* 发送一串字符 */
static void put(const char *s) { Usart_Print(s); }

static void put_u32(uint32_t v)
{
    char t[12];
    t[11] = 0;
    char *p = t + 11;
    if (v == 0) *--p = '0';
    while (v) { *--p = (char)('0' + (v % 10)); v /= 10; }
    Usart_Print(p);
}

static void cli_help(void)
{
    put("HELP              - 本帮助\r\n");
    put("ID                - 版本信息\r\n");
    put("CFG SHOW          - 显示档位表\r\n");
    put("CFG SAVE          - 保存到 Flash\r\n");
    put("CFG RESET         - 恢复默认档位\r\n");
    put("CFG GEAR n d1..dn b1..b(n-1)\r\n");
    put("                  - 设置 n 档抽稀（5files_max=5, d=1/2/4/8/16, b 逐增）\r\n");
    put("PWM 0..1000      - 测试电机占空比（‰，0 停机）\r\n");
}

static void cli_id(void)
{
    put(APP_VERSION_STR "\r\n");
    put("baud  = "); put_u32(CFG_UART_BAUD); put("\r\n");
}

static void cli_cfg_show(void)
{
    put("gear_n = "); put_u32(Config_GetGearN()); put("\r\n");
    for (uint8_t i = 0; i < Config_GetGearN(); i++) {
        put("  div["); put_u32(i); put("] = "); put_u32(Config_GetGearDiv(i));
        if (i < Config_GetGearN() - 1) {
            put("   bnd["); put_u32(i); put("] = "); put_u32(Config_GetGearBnd(i));
        } else {
            put("   (末档)");
        }
        put("\r\n");
    }
}

static void cli_cfg(uint8_t n, char *argv[])
{
    if (n < 2) { cli_cfg_show(); return; }

    if (strcmp(argv[1], "SHOW") == 0)        { cli_cfg_show(); return; }
    if (strcmp(argv[1], "SAVE") == 0)        { Config_Save();  put("saved\r\n"); return; }
    if (strcmp(argv[1], "RESET") == 0)       { Config_Reset(); put("reset\r\n"); cli_cfg_show(); return; }

    if (strcmp(argv[1], "GEAR") == 0) {
        uint8_t  div[CFG_GEAR_MAX] = { 0 };       /* d1..dn */
        uint32_t bnd[CFG_GEAR_MAX - 1] = { 0 };   /* b1..b(n-1) */
        if (n < 4) { put("用法: CFG GEAR n d1..dn b1..b(n-1)\r\n"); return; }
        uint32_t gn = strtoul(argv[2], 0, 0);
        if (gn < CFG_GEAR_MIN || gn > CFG_GEAR_MAX) {
            put("档数需 "); put_u32(CFG_GEAR_MIN); put(".."); put_u32(CFG_GEAR_MAX); put("\r\n");
            return;
        }
        if (n != 3 + gn + (gn - 1)) { put("参数个数不符\r\n"); return; }
        for (uint8_t i = 0; i < gn; i++)          div[i] = (uint8_t)strtoul(argv[3 + i], 0, 0);
        for (uint8_t i = 0; i < gn - 1; i++)      bnd[i] = strtoul(argv[3 + gn + i], 0, 0);
        if (!Config_CheckGear(gn, div, bnd)) { put("非法档位参数\r\n"); return; }
        Config_SetGear(gn, div, bnd);
        put("gear set ok\r\n");
        cli_cfg_show();
        return;
    }

    cli_cfg_show();
}

static void cli_pwm(uint8_t n, char *argv[])
{
    if (n < 2) { put("用法: PWM 0..1000\r\n"); return; }
    uint32_t d = strtoul(argv[1], 0, 0);
    if (d > 1000) { put("范围 0..1000\r\n"); return; }
    Pm_SetDuty(d);
    put("PWM = "); put_u32(d); put("‰\r\n");
}

/* ---------- 行缓冲处理 ---------- */

static void Cli_Process(const char *line, uint16_t len)
{
    char buf[CLI_LINE_MAX + 2];
    char *argv[CLI_ARG_MAX];
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memcpy(buf, line, len);
    buf[len] = 0;

    put("\r\n");
    uint8_t n = Cli_Split(buf, argv, CLI_ARG_MAX);
    if (n == 0) { put("> "); return; }

    if      (strcmp(argv[0], "HELP") == 0)       cli_help();
    else if (strcmp(argv[0], "ID") == 0)         cli_id();
    else if (strcmp(argv[0], "CFG") == 0)        cli_cfg(n, argv);
    else if (strcmp(argv[0], "PWM") == 0)        cli_pwm(n, argv);
    else {
        put("未知命令: "); put(argv[0]); put(" (输入 HELP)\r\n");
    }
    put("> ");
}

/* ---------- 对外接口 ---------- */

void Cli_Init(void)
{
    cli_len = 0;
    cli_buf[0] = 0;
    put("\r\n" APP_VERSION_STR " (ABI Monitor)\r\n");
    put("输入 HELP 查看命令\r\n> ");
}

/* 接收一个字节：仅在收到 \r 或 \n 时判定一行结束 */
void Cli_OnChar(char c)
{
    if (c == '\r' || c == '\n') {
        if (cli_len) {
            Cli_Process(cli_buf, cli_len);
            cli_len = 0;
        }
        return;
    }
    if (c == 0x08 || c == 0x7F) {                       /* backspace */
        if (cli_len) { cli_len--; }
        return;
    }
    if (cli_len < CLI_LINE_MAX) {
        cli_buf[cli_len++] = c;
    }
}

void Cli_Poll(void)
{
}

/* 供 usart.c / 其余模块复用的小工具 */
void Cli_PrintU32(uint32_t v) { put_u32(v); }