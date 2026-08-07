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
    put("MONITOR START    - PC UI 兼容：武装 + 每秒 L, 帧\r\n");
    put("MONITOR STOP     - PC UI 兼容：解除武装\r\n");
    put("LOG DUMP [USB]   - PC UI 兼容：snap 按 D, 文本行回传\r\n");
    put("SNAP? / ABI?     - PC UI 兼容：状态 / 版本\r\n");
    put("SD INIT          - 初始化 SD 卡并挂载 FatFS\r\n");
    put("SD SAVE          - 手动把当前 snap 备份为 S<秒>.BIN（不覆盖）\r\n");
    put("SD LS            - 列出卡内文件\r\n");
    put("SD STAT          - 卡信息\r\n");
    put("SD RAW [w] sec  - 裸扇区读写（诊断底层）\r\n");
    put("FLASH            - 芯片保存状态（点数/是否已存）\r\n");
    put("FSAVE            - 把当前 snap 写入芯片 Flash（掉电不丢）\r\n");
    put("DUMP             - 芯片记录按 PC 帧发出（abi_monitor.py 曲线还原）\r\n");
    put("DBG              - GPIO 电平 + TIM2 寄存器\r\n");
    put("FILT SHOW        - 动态滤波参数\r\n");
    put("FILT EMA <10..255>    - EMA α‰（默认 150）\r\n");
    put("FILT DPOS <2..20>     - dpos 超限倍数 ×N_MIN（默认 6）\r\n");
    put("FILT ZERO <1..60>     - 静止归零周期数（默认 10）\r\n");
    put("FILT RATE <0..100000> - 爬坡斜率限幅 rpm/s（0=自动实测学习，默认 0）\r\n");
    put("FILT ICF <i> <v>      - 动态表第 i 档滤波值 0..15（i=0..4）\r\n");
    put("FILT BND <i> <rpm>    - 动态表第 i 边界 rpm（i=0..3，严格递增）\r\n");
    put("FILT RANGE <lo> <hi>  - 人工范围钳制 0..15（lo<=hi）\r\n");
    put("FILT RESET      - 恢复全自动默认表\r\n");
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

static uint8_t pc_mon;        /* PC UI 监控模式：推送 L, 帧 */
static uint8_t pc_done_flag;  /* 记录完成已广播（防重发） */

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

/* MONITOR（PC UI 兼容）：START=ARM+推送 L, 帧；STOP=DISARM */
static void cli_monitor(uint8_t n, char *argv[])
{
    if (n >= 2 && strcmp(argv[1], "START") == 0) {
        Snap_Arm();
        pc_mon = 1;
        pc_done_flag = 0;
        put("monitor start (armed)\r\n");
        return;
    }
    if (n >= 2 && strcmp(argv[1], "STOP") == 0) {
        Snap_Disarm();
        pc_mon = 0;
        put("monitor stop (disarmed)\r\n");
        return;
    }
    put("用法: MONITOR START|STOP\r\n");
}

/* LOG DUMP（PC UI 兼容）：snap 点按 D, 文本行回传 + D END（UI 自动拉取路径）
 * 测速对齐 ESP32 VEL_WINDOW：记录点多为抽稀事件（div≥1），裸相邻点差分会把
 * UTO ARR 跳变/div 切换的非均匀间隔放大成尖刺（朋友分析②）。改为 LOG_WIN 点
 * 窗口差分 rmp[i]=(c[i]-c[i-W])*60e6/((t[i]-t[i-W])*steps)，与固件/PC 多拍窗一致。 */
#define LOG_DUMP_WIN 96

static void cli_log_dump(void)
{
    uint32_t n = Snap_Count();
    if (n == 0) { put("D END 0\r\n"); return; }
    const SnapPt *p = Snap_Points();
    uint32_t steps = Abi_GetStepsPerRev();
    for (uint32_t i = 0; i < n; i++) {
        int32_t rpm = 0;
        uint32_t j = (i >= LOG_DUMP_WIN) ? (i - LOG_DUMP_WIN) : 0;
        if (j < i) {
            /* 用亚µs 余数复原全精度时间（60MHz 刻度 → 量化误差 ~0.05%） */
            uint64_t ti = (uint64_t)p[i].t_us * 60uLL + p[i].c_hi;
            uint64_t tj = (uint64_t)p[j].t_us * 60uLL + p[j].c_hi;
            uint64_t dt_tick = ti - tj;
            int32_t dc = (int32_t)(p[i].c_lo - p[j].c_lo);
            if (dt_tick > 0)
                rpm = (int32_t)(((int64_t)dc * 60000000LL * 60LL) / (int64_t)dt_tick / steps);
        }
        int32_t dir = (rpm > 0) ? 1 : ((rpm < 0) ? -1 : 0);
        put("D,"); put_u32(p[i].t_us / 1000u);
        put(",");  put_i32(rpm); put(".0");
        put(",");  put_i32(dir);
        put(",1,"); put_u32(p[i].idx);
        put("\r\n");
    }
    put("D END "); put_u32(n); put("\r\n");
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
    put(" ICF="); put_u32(Abi_GetInputFilter());
    put("\r\n");
}

/* ---------- Task 8 动态滤波 ---------- */

static void cli_filt_show(void)
{
    put("ema    = "); put_u32(Config_GetFltEma()); put("  (10..255 ‰)\r\n");
    put("dpos   = "); put_u32(Config_GetFltDpos()); put("  (2..20 ×N_MIN)\r\n");
    put("zero   = "); put_u32(Config_GetFltZero()); put("  (1..60 周期)\r\n");
    put("rate   = "); put_u32(Config_GetFltRate()); put("  (低速 EMA: 200=20%%  <2000rpm;  中速 100=10%%  <4000rpm;  高速 40=4%%  >=4000rpm)\r\n");
    put("range  = "); put_u32(Config_GetFltIcfMin()); put(".."); put_u32(Config_GetFltIcfMax());
    put(" (IC 钳制范围 0..15)\r\n");
    put("Table(|rpm|<bnd→icf):\r\n");
    for (uint8_t i = 0; i < FLT_ICF_N; i++) {
        put("  ["); put_u32(i); put("] icf="); put_u32(Config_GetFltIcf(i));
        if (i + 1 < FLT_ICF_N) { put("  bnd="); put_u32(Config_GetFltBnd(i)); }
        put("\r\n");
    }
    put("cur_icf(实时生效)="); put_u32(Abi_GetInputFilter()); put("\r\n");
    put("icf 全自动: 按|rpm|查表档位 → 钳位到 range 内动态调整\r\n");
}

static void cli_filt(uint8_t n, char *argv[])
{
    if (n < 2 || strcmp(argv[1], "SHOW") == 0) { cli_filt_show(); return; }

    if (strcmp(argv[1], "RESET") == 0)   { Config_FltReset(); put("filt reset done\r\n"); cli_filt_show(); return; }

    if (n < 4) { put("用法: FILT <EMA|DPOS|ZERO|RATE|ICF|BND|RANGE> <值>\r\n"); return; }

    if (strcmp(argv[1], "EMA") == 0) {
        uint32_t v = strtoul(argv[2], 0, 0);
        if (Config_SetFltEma((uint8_t)v)) { put("范围 10..255\r\n"); return; }
        put("ema set = "); put_u32(cfg_flt_ema0); put("\r\n");
    } else if (strcmp(argv[1], "DPOS") == 0) {
        uint32_t v = strtoul(argv[2], 0, 0);
        if (Config_SetFltDpos((uint8_t)v)) { put("范围 2..20\r\n"); return; }
        put("dpos set = "); put_u32(cfg_flt_dpos); put("\r\n");
    } else if (strcmp(argv[1], "ZERO") == 0) {
        uint32_t v = strtoul(argv[2], 0, 0);
        if (Config_SetFltZero((uint8_t)v)) { put("范围 1..60\r\n"); return; }
        put("zero set = "); put_u32(cfg_flt_zero); put("\r\n");
    } else if (strcmp(argv[1], "RATE") == 0) {
        uint32_t v = strtoul(argv[2], 0, 0);
        if (Config_SetFltRate((uint16_t)v)) { put("范围 0..100000\r\n"); return; }
        put("rate set = "); put_u32(cfg_flt_rate); put("\r\n");
    } else if (strcmp(argv[1], "RANGE") == 0) {
        if (n < 4) { put("用法: FILT RANGE <lo> <hi>\r\n"); return; }
        uint32_t lo = strtoul(argv[2], 0, 0), hi = strtoul(argv[3], 0, 0);
        if (Config_SetFltRange((uint8_t)lo, (uint8_t)hi)) { put("需 lo<=hi, 0..15\r\n"); return; }
        put("range set = "); put_u32(cfg_flt_icf_min); put(".."); put_u32(cfg_flt_icf_max); put("\r\n");
    } else if (strcmp(argv[1], "ICF") == 0) {
        if (n < 5) { put("用法: FILT ICF <i 0..4> <v 0..15>\r\n"); return; }
        uint32_t i = strtoul(argv[2], 0, 0), v = strtoul(argv[3], 0, 0);
        if (Config_SetFltIcfTbl((uint8_t)i, (uint8_t)v)) { put("i 0..4 / v 0..15\r\n"); return; }
        put("icf["); put_u32(i); put("] set = "); put_u32(cfg_flt_icf[i]); put("\r\n");
    } else if (strcmp(argv[1], "BND") == 0) {
        if (n < 5) { put("用法: FILT BND <i 0..3> <rpm>\r\n"); return; }
        uint32_t i = strtoul(argv[2], 0, 0), v = strtoul(argv[3], 0, 0);
        if (Config_SetFltBnd((uint8_t)i, (uint16_t)v)) { put("需严格递增\r\n"); return; }
        put("bnd 已设\r\n");
    } else {
        put("未知 FILT 子命令（FILT SHOW）\r\n");
    }
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
    else if (strcmp(argv[0], "SNAP?") == 0)      cli_snap();
    else if (strcmp(argv[0], "ABI?") == 0)       cli_id();
    else if (strcmp(argv[0], "TIME") == 0)       put("time ok\r\n");
    else if (strcmp(argv[0], "MONITOR") == 0)    cli_monitor(n, argv);
    else if (strcmp(argv[0], "LOG") == 0)        cli_log_dump();
    else if (strcmp(argv[0], "REC") == 0)        { put("rec now (snap arm)\r\n"); Snap_Arm(); }
    else if (strcmp(argv[0], "REVS") == 0)       put("revs ok\r\n");
    else if (strcmp(argv[0], "SD") == 0)         cli_sd(n, argv);
    else if (strcmp(argv[0], "FLASH") == 0)      cli_flash();
    else if (strcmp(argv[0], "FSAVE") == 0)      cli_fsave();
    else if (strcmp(argv[0], "DUMP") == 0)       cli_dump();
    else if (strcmp(argv[0], "DBG") == 0)        cli_dbg();
    else if (strcmp(argv[0], "FILT") == 0)       cli_filt(n, argv);
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

static volatile uint8_t cli_line_ready;   /* ISR→主循环：一行就绪 */
static char  cli_pending[CLI_LINE_MAX + 1];
static volatile uint8_t cli_pending_len;

/* 接收一个字节（来自 USART RX ISR）：仅缓冲，不在此处处理命令（防 ISR 内
 * put() 阻塞 TX 死锁）。一行结束时设 ready 标志，交给主循环 Cli_Poll 处理。 */
void Cli_OnChar(char c)
{
    if (c == '\r' || c == '\n') {
        if (cli_len && !cli_line_ready) {
            cli_buf[cli_len] = '\0';
            for (uint8_t i = 0; i <= cli_len; i++) cli_pending[i] = cli_buf[i];
            cli_pending_len = cli_len;
            cli_line_ready = 1;
            cli_len = 0;
        }
        return;
    }
    if (c == 0x08 || c == 0x7F) {
        if (cli_len) { cli_len--; }
        return;
    }
    if (cli_len < CLI_LINE_MAX) {
        cli_buf[cli_len++] = c;
    }
}

void Cli_Poll(void)
{
    /* ISR→主循环：处理缓冲好的命令（移出 ISR 防止 put() 阻塞 TX 死锁） */
    if (cli_line_ready) {
        cli_line_ready = 0;
        Cli_Process(cli_pending, cli_pending_len);
    }

    static uint32_t last_ms;
    uint32_t now = HAL_GetTick();

    /* PC UI 监控模式（ESP32 对齐）：
     *   ARM   → 10Hz L, 帧实时转速
     *   TRIGGER（触发 REC 瞬间）→ 广播后停止 L 帧，专注记录（防串口拥堵）
     *   DONE  → 广播 # RECORD done，UI 等 10s 后拉 LOG DUMP */
    if (pc_mon) {
        uint8_t st = Snap_State();
        static uint8_t last_st = 0xFF;
        /* 触发边沿：1(armed) → 2(rec) */
        if (last_st != 0xFF && last_st == 1 && st == 2) {
            put("# TRIGGER record started\r\n");
        }
        last_st = st;

        if (st != 2) {                 /* REC 期间停发 L 帧 */
            static uint32_t last_l;
            if (last_l == 0 || now - last_l >= 100) {
                last_l = now;
                int32_t rpm = Abi_GetRpm();
                int32_t dir = (rpm > 0) ? 1 : ((rpm < 0) ? -1 : 0);
                uint8_t armed = (st == 1) ? 1 : 0;
                put("L,"); put_i32(rpm); put(".0,");  /* rpm 带小数：UI 按短帧解析 */
                put_i32(dir);
                put(",");  put_u32(armed);
                put(",0,0,0\r\n");
            }
        }
        /* 记录完成 → 广播供 UI 自动拉取（D, 行） */
        if (!pc_done_flag && st == 3 && Snap_Count() > 0) {
            pc_done_flag = 1;
            put("# RECORD done | n="); put_u32(Snap_Count());
            put(" seg=1 RECORD done\r\n");
        }
    }

    if (!cli_rpm_mon) { last_ms = 0; return; }
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