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
#include "abi.h"
#include "snap_bin.h"
#include "sd_card.h"
#include "flash_save.h"
#include "usart.h"
#include "stm32h7xx_hal.h"
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
    put("CFG POL 0|1       - A/B 方向极性（0=计数反向取反，1=正向计入）\r\n");
    put("CAL STEPS         - 显示 Index→Index 实测步数（EMA）\r\n");
    put("CAL SET <n>       - 手动写一圈步数（掉电保存）\r\n");
    put("CAL RESET         - 恢复 4000\r\n");
    put("PWM 0..1000      - 测试电机占空比（‰，0 停机）\r\n");
    put("RPM [ON|OFF]    - 转速（ON 每秒回显一次）\r\n");
    put("CNT              - counts + Index + µs\r\n");
    put("IDX              - Index 圈数\r\n");
    put("ARM              - 预置触发（|rpm|>10 且转过一圈 → 回溯400点+记0.5s）\r\n");
    put("DISARM           - 取消触发预置\r\n");
    put("SD INIT          - 初始化 SD 卡并挂载 FatFS\r\n");
    put("SD SAVE          - 手动把当前 snap 备份为 S<秒>.BIN（不覆盖）\r\n");
    put("SD LS            - 列出卡内文件\r\n");
    put("SD STAT          - 卡信息\r\n");
    put("SD RAW [w] sec  - 裸扇区读写（诊断底层）\r\n");
    put("FLASH            - 芯片保存状态（点数/是否已存）\r\n");
    put("FSAVE            - 把当前 snap 写入芯片 Flash（掉电不丢）\r\n");
    put("DUMP             - 芯片记录按 PC 帧发出（abi_monitor.py 曲线还原）\r\n");
    put("DBG              - GPIO 电平 + TIM2 寄存器\r\n");
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
    if (strcmp(argv[1], "POL") == 0) {
        if (n < 3) { put("用法: CFG POL 0|1\r\n"); return; }
        uint32_t pol = strtoul(argv[2], 0, 0);
        if (pol > 1) { put("0 或 1\r\n"); return; }
        Config_SetPol((uint8_t)pol);
        put("pol = "); put_u32(cfg_pol); put("\r\n");
        return;
    }

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

/* ---------- ABI 状态（Task 2） ---------- */

static uint8_t cli_rpm_mon;   /* RPM ON 时每秒回显 */

static void put_i32(int32_t v)
{
    if (v < 0) { put("-"); v = -v; }
    put_u32((uint32_t)v);
}

static void cli_rpm(uint8_t n, char *argv[])
{
    if (n >= 2 && strcmp(argv[1], "ON") == 0)  { cli_rpm_mon = 1; put("rpm monitor on\r\n"); return; }
    if (n >= 2 && strcmp(argv[1], "OFF") == 0) { cli_rpm_mon = 0; put("rpm monitor off\r\n"); return; }
    put("rpm = "); put_i32(Abi_GetRpm()); put(" rpm, div = "); put_u32(Abi_GetDiv()); put("\r\n");
}

static void cli_cnt(void)
{
    put("cnt = "); put_u32(Abi_GetCnt());
    put(", idx = "); put_u32(Abi_GetIndexCnt());
    put(", us = "); put_u32(Abi_GetUsNow());
    put("\r\n");
}

static void cli_idx(void)
{
    put("idx = "); put_u32(Abi_GetIndexCnt()); put("\r\n");
}

/* CAL：Index→Index 实测步数（Task 6 校准） */
static void cli_cal(uint8_t n, char *argv[])
{
    if (n < 2 || strcmp(argv[1], "STEPS") == 0) {
        put("steps_per_rev = "); put_u32(Abi_GetStepsPerRev());
        put(" (EMA="); put_u32(Abi_GetCalib()); put(")\r\n");
        return;
    }
    if (strcmp(argv[1], "SET") == 0) {
        if (n < 3) { put("用法: CAL SET <n>\r\n"); return; }
        uint32_t steps = strtoul(argv[2], 0, 0);
        if (steps < 1 || steps > 200000u) { put("范围 1..200000\r\n"); return; }
        Abi_SetCalibSteps(steps);
        put("steps set + saved = "); put_u32(Abi_GetStepsPerRev()); put("\r\n");
        return;
    }
    if (strcmp(argv[1], "RESET") == 0) {
        Config_SetStepsPerRev(CFG_STEPS_PER_REV_DEFAULT);
        Abi_SetCalibSteps(CFG_STEPS_PER_REV_DEFAULT);
        put("steps reset = "); put_u32(Abi_GetStepsPerRev()); put("\r\n");
        return;
    }
    put("用法: CAL STEPS|SET <n>|RESET\r\n");
}

/* SNAP：状态/点数（验证自动触发用） */
static void cli_snap(void)
{
    put("snap: state="); put_u32(Snap_State());
    put(" (0=idle 1=arm 2=rec 3=done), count="); put_u32(Snap_Count());
    put(" ready="); put_u32(Snap_IsReady());
    put("\r\n");
}

/* SD 卡（Task 4）：INIT / SAVE / LS / STAT */
static void cli_sd(uint8_t n, char *argv[])
{
if (n < 2 || strcmp(argv[1], "INIT") == 0) {
        uint8_t r = Sd_Mount();
        put("SD init "); put_u32(r);
        put(" (0=ok 1=no-card 2=no-fat)\r\n");
        return;
    }
    if (strcmp(argv[1], "SAVE") == 0) {
        uint8_t e = Sd_SaveSnap();
        put("SD save ret="); put_u32(e);
        put(" (0=ok 1=no-card 2=no-data 3=fail)\r\n");
        return;
    }
    if (strcmp(argv[1], "LS") == 0)   { Sd_Ls();   return; }
    if (strcmp(argv[1], "RAW") == 0)  {
        uint8_t w = (n >= 3 && strcmp(argv[2], "w") == 0) ? 1 : 0;
        uint32_t sec = (n >= 3) ? strtoul(argv[w ? 3 : 2], 0, 0) : 0;
        Sd_Raw(sec, w); return;
    }
    if (strcmp(argv[1], "STAT") == 0) { Sd_Stat(); return; }
    put("用法: SD INIT|SAVE|LS|RAW [sector]|STAT\r\n");
}

/* FLASH（Task 5）：芯片保存 */
static void cli_flash(void)
{
    put("flash: data="); put_u32(Flash_IsData());
    put(" stored="); put_u32(Flash_StoredCount());
    put(" current="); put_u32(Flash_Count());
    put(" saved="); put_u32(Flash_IsSaved());
    put("\r\n");
}

static void cli_fsave(void)
{
    uint8_t e = Flash_SaveSnap();
    put("fsave ret="); put_u32(e);
    put(" (0=ok 1=no-data 2=no-data 3=erase-fail 4=prog-fail)\r\n");
}

static void cli_dump(void)
{
    uint8_t e = Flash_Dump();
    put("\r\ndump ret="); put_u32(e);
    put(" (0=ok 2=no-data 6=tx-fail)\r\n");
}

/* 诊断：GPIO 电平 + TIM2 寄存器（调试用） */
static void cli_dbg(void)
{
    uint32_t idr = GPIOA->IDR;
    put("PA1="); put_u32((idr >> 1) & 1);
    put(" PA4="); put_u32((idr >> 4) & 1);
    put(" PA5="); put_u32((idr >> 5) & 1);
    put("  TIM2: SMCR="); put_u32(TIM2->SMCR);
    put(" CCMR1="); put_u32(TIM2->CCMR1);
    put(" CCER="); put_u32(TIM2->CCER);
    put(" CNT="); put_u32(TIM2->CNT);
    put("\r\n");
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
    else if (strcmp(argv[0], "RPM") == 0)        cli_rpm(n, argv);
    else if (strcmp(argv[0], "CNT") == 0)        cli_cnt();
    else if (strcmp(argv[0], "IDX") == 0)        cli_idx();
    else if (strcmp(argv[0], "CAL") == 0)        cli_cal(n, argv);
    else if (strcmp(argv[0], "ARM") == 0)        { Snap_Arm();    put("armed (等转速过阈+一整圈自动触发)\r\n"); }
    else if (strcmp(argv[0], "DISARM") == 0)     { Snap_Disarm(); put("disarmed\r\n"); }
    else if (strcmp(argv[0], "SNAP") == 0)       cli_snap();
    else if (strcmp(argv[0], "SD") == 0)         cli_sd(n, argv);
    else if (strcmp(argv[0], "FLASH") == 0)      cli_flash();
    else if (strcmp(argv[0], "FSAVE") == 0)      cli_fsave();
    else if (strcmp(argv[0], "DUMP") == 0)       cli_dump();
    else if (strcmp(argv[0], "DBG") == 0)        cli_dbg();
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
    static uint32_t last_ms;
    if (!cli_rpm_mon) { last_ms = 0; return; }
    uint32_t now = HAL_GetTick();
    if (last_ms == 0) last_ms = now;
    if (now - last_ms < 1000) return;
    last_ms = now;
    put("rpm = "); put_i32(Abi_GetRpm());
    put(", cnt = "); put_u32(Abi_GetCnt());
    put(", idx = "); put_u32(Abi_GetIndexCnt());
    put(", div = "); put_u32(Abi_GetDiv());
    put("\r\n");
}

/* 供 usart.c / 其余模块复用的小工具 */
void Cli_PrintU32(uint32_t v) { put_u32(v); }