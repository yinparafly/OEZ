/**
  ******************************************************************************
  * @file    app_config.c
  * @brief   应用配置存取（3/4/5 档抽稀表）+ 内部 Flash 持久化
  * @note    默认 3 档：div 1/2/4，边界 4000/8000rpm（高速自动 div>=2）
  *          Flash 写入需先 HAL_FLASH_Unlock，写后 Lock。
  ******************************************************************************
  */
#include "app_config.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/*------------------------------------------ 全局配置变量 -------------------------*/

uint32_t cfg_baud = CFG_UART_BAUD;
uint8_t  cfg_gear_n  = 3;                                   /* 档数 3/4/5 */
uint8_t  cfg_gear_div[CFG_GEAR_MAX] = {1, 2, 4, 0, 0};      /* div 表 */
uint32_t cfg_gear_bnd[CFG_GEAR_MAX - 1] = {4000, 8000, 0, 0}; /* 边界 rpm */
uint32_t cfg_steps_per_rev = CFG_STEPS_PER_REV_DEFAULT;    /* 实测一圈步数（Index→Index EMA） */
uint8_t  cfg_pol  = CFG_POL_DEFAULT;                        /* A/B 方向极性 */

#define CFG_FLASH_SECTOR    7               /* 0x081E0000 = Bank2 第 8 个扇区（0..7）*/

typedef struct {
    uint32_t magic;                      /* 校验字 */
    uint32_t steps_per_rev;              /* 实测一圈步数 */
    uint8_t  gear_n;
    uint8_t  gear_div[CFG_GEAR_MAX];
    uint8_t  pol;                        /* A/B 方向极性 */
    uint8_t  rsvd;
    uint32_t gear_bnd[CFG_GEAR_MAX - 1];
    uint16_t crc;                        /* CRC16，查表 */
    uint16_t pad;
} AppCfg;

/* CRC16 查表（多项式 0x1021，初值 0xFFFF，输出不变），开机/写前计算 */
static const uint16_t crc16_tab[256] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
    0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
    0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
    0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
    0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
    0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
    0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
    0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
    0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
    0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
    0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
    0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
    0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
    0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
    0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
    0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
    0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
    0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0,
};

static uint16_t Crc16(const uint8_t *p, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) crc = (uint16_t)((crc << 8) ^ crc16_tab[((crc >> 8) ^ *p++) & 0xFF]);
    return crc;
}

/*------------------------------------------ 内部函数 -------------------------------*/

static AppCfg g_cfg;

static void Cfg_Default(AppCfg *c)
{
    memset(c, 0, sizeof(AppCfg));
    c->magic = CFG_MAGIC;
    c->steps_per_rev = CFG_STEPS_PER_REV_DEFAULT;
    c->gear_n = 3;
    c->gear_div[0] = 1; c->gear_div[1] = 2; c->gear_div[2] = 4;
    c->gear_bnd[0] = 4000; c->gear_bnd[1] = 8000;
    c->pol = CFG_POL_DEFAULT;
    c->crc = Crc16((const uint8_t *)c, sizeof(AppCfg) - 4);
}

/*------------------------------------------ 对外接口 -------------------------------*/

void Config_Load(void)
{
    memcpy(&g_cfg, (const void *)CFG_FLASH_BASE, sizeof(AppCfg));
    if (g_cfg.magic != CFG_MAGIC || g_cfg.gear_n < CFG_GEAR_MIN || g_cfg.gear_n > CFG_GEAR_MAX ||
        Crc16((const uint8_t *)&g_cfg, sizeof(AppCfg) - 4) != g_cfg.crc) {
        Cfg_Default(&g_cfg);
        Config_Save();
    }
    cfg_gear_n  = g_cfg.gear_n;
    memcpy(cfg_gear_div, g_cfg.gear_div, sizeof(cfg_gear_div));
    memcpy(cfg_gear_bnd, g_cfg.gear_bnd, sizeof(cfg_gear_bnd));
    cfg_steps_per_rev = g_cfg.steps_per_rev;
    cfg_pol = g_cfg.pol;
}

void Config_Save(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks     = FLASH_BANK_2;
    erase.Sector    = CFG_FLASH_SECTOR;
    erase.NbSectors = 1;
    uint32_t err = 0;
    if (HAL_FLASHEx_Erase(&erase, &err) == HAL_OK) {
        /* H7 按 32B FLASH WORD 编程；配置 36B → 两次对齐写，多余区为 0 */
        static uint64_t buf[8] __attribute__((aligned(32)));    /* 64B 对齐缓冲 */
        memset(buf, 0, sizeof(buf));
        memcpy(buf, &g_cfg, sizeof(AppCfg));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, CFG_FLASH_BASE, (uint32_t)(uintptr_t)buf);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, CFG_FLASH_BASE + 32,
                          (uint32_t)(uintptr_t)((uint8_t *)buf + 32));
    }
    HAL_FLASH_Lock();
}

/* 全局 → g_cfg → Flash（其他 setter 复用，互不覆盖） */
static void Config_Persist(void)
{
    g_cfg.gear_n       = cfg_gear_n;
    memcpy(g_cfg.gear_div, cfg_gear_div, sizeof(cfg_gear_div));
    memcpy(g_cfg.gear_bnd, cfg_gear_bnd, sizeof(cfg_gear_bnd));
    g_cfg.steps_per_rev = cfg_steps_per_rev;
    g_cfg.pol           = cfg_pol;
    g_cfg.crc = Crc16((const uint8_t *)&g_cfg, sizeof(AppCfg) - 4);
    Config_Save();
}

uint8_t Config_GetGearN(void)      { return cfg_gear_n; }
uint8_t Config_GetGearDiv(uint8_t i) { return (i < CFG_GEAR_MAX) ? cfg_gear_div[i] : 1; }
uint32_t Config_GetGearBnd(uint8_t i) { return (i < CFG_GEAR_MAX - 1) ? cfg_gear_bnd[i] : 0; }

/* 校验 CFG GEAR 参数：3/4/5 档、div ∈{1,2,4,8,16}、边界严格递增 */
uint8_t Config_CheckGear(uint8_t n, const uint8_t *div, const uint32_t *bnd)
{
    if (n < CFG_GEAR_MIN || n > CFG_GEAR_MAX) return 0;
    for (uint8_t i = 0; i < n; i++) {
        if (div[i] != 1 && div[i] != 2 && div[i] != 4 && div[i] != 8 && div[i] != 16) return 0;
        if (i < n - 1) {
            if (bnd[i] <= 0) return 0;
            if (i > 0 && bnd[i] <= bnd[i - 1]) return 0;
        }
    }
    return 1;
}

void Config_SetGear(uint8_t n, const uint8_t *div, const uint32_t *bnd)
{
    cfg_gear_n = n;
    memcpy(cfg_gear_div, div, CFG_GEAR_MAX);
    for (uint8_t i = n; i < CFG_GEAR_MAX; i++) cfg_gear_div[i] = 0;
    memcpy(cfg_gear_bnd, bnd, (CFG_GEAR_MAX - 1) * 4);
    for (uint8_t i = n - 1; i < CFG_GEAR_MAX - 1; i++) cfg_gear_bnd[i] = 0;
    Config_Persist();
}

void Config_Reset(void)
{
    cfg_gear_n  = 3;
    cfg_gear_div[0] = 1; cfg_gear_div[1] = 2; cfg_gear_div[2] = 4;
    cfg_gear_div[3] = 0; cfg_gear_div[4] = 0;
    cfg_gear_bnd[0] = 4000; cfg_gear_bnd[1] = 8000;
    cfg_gear_bnd[2] = 0; cfg_gear_bnd[3] = 0;
    Config_Persist();
}

void Config_SetStepsPerRev(uint32_t steps)
{
    cfg_steps_per_rev = CLAMP(steps, 1u, 200000u);
    Config_Persist();
}

void Config_SetPol(uint8_t pol)
{
    cfg_pol = pol ? 1 : 0;
    Config_Persist();
}