/**
  ******************************************************************************
  * @file    sd_card.c
  * @brief   Task 4 SD 卡：SDMMC1 精简驱动 + FatFS（r0.15）+ 自动备份
  *
  * 可靠性优先（用户定）：
  *   - polling 块写，无 DMA / 中断，时序简单、逻辑清晰
  *   - ClockDiv=10 -> PLL1Q 480MHz/(2*10)=24MHz，SD 标准速度限(≤25MHz)内
  *   - 帧 = BIN v2（preamble + <IHH + payload + crc32），PC 端直接解
  *   - 写完 f_sync 落盘；文件名 S 前缀 + 启动秒数(8.3)，FA_CREATE_NEW
  *     检测同名则秒+1 重试，绝不覆盖旧文件
  ******************************************************************************
  */
#include "sd_card.h"
#include "ff.h"
#include "snap_bin.h"
#include "app_cli.h"
#include "usart.h"
#include <string.h>

#define SNAP_HZ        1000u

static const uint8_t SNAP_PREAMBLE[11] = {
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x55
};

SD_HandleTypeDef hsd1;

static FATFS g_fs;
static FIL   g_fil;
static uint8_t g_mounted;

/* ---------- SDMMC1 MSP（覆盖 HAL __weak 实现） ---------- */
void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PC8=D0 PC9=D1 PC10=D2 PC11=D3 PC12=CK, PD2=CMD，AF12 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF12_SDIO1;
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin       = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &gpio);

    __HAL_RCC_SDMMC1_FORCE_RESET();
    __HAL_RCC_SDMMC1_RELEASE_RESET();
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd)
{
    __HAL_RCC_SDMMC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
}

/* ---------- SDMMC 精简驱动（polling） ---------- */

uint8_t SD_Init(void)
{
    if (hsd1.State == HAL_SD_STATE_READY)   /* 已初始化（幂等） */
        return 0;

    hsd1.Instance                 = SDMMC1;
    hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide             = SDMMC_BUS_WIDE_1B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv            = 10;            /* 480MHz/(2*10)=24MHz */

    if (HAL_SD_Init(&hsd1) != HAL_OK)
        return 1;
    if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
        return 1;
    return 0;
}

uint8_t SD_GetCardInfo(HAL_SD_CardInfoTypeDef *info)
{
    return (HAL_SD_GetCardInfo(&hsd1, info) == HAL_OK) ? 0 : 1;
}

uint8_t SD_CardMounted(void) { return g_mounted; }

/* ---------- FatFS 挂载 ---------- */

uint8_t Sd_Mount(void)
{
    FRESULT r = f_mount(&g_fs, "", 1);
    if (r == FR_OK)            { g_mounted = 1; return 0; }
    if (r == FR_NO_FILESYSTEM) { g_mounted = 0; return 2; }
    g_mounted = 0;
    return 1;                                       /* 无卡/超时 */
}

/* ---------- 打印工具 ---------- */
static void sd_puts(const char *s)  { Usart_Print(s); }
static void sd_putu(uint32_t v)     { Cli_PrintU32(v); }

/* 文件名 S<秒7位>.BIN（8.3 短名，秒序可排序） */
static void fmt_name(char *dst, uint32_t s)
{
    dst[0] = 'S';
    for (int i = 6; i >= 1; i--) { dst[i] = (char)('0' + (s % 10)); s /= 10; }
    dst[7] = '.';
    dst[8] = 'B'; dst[9] = 'I'; dst[10] = 'N'; dst[11] = 0;
}

/* ---------- 帧写入：PREAMBLE + <IHH + payload + crc32 LE ---------- */
static uint8_t sd_write_frame(void)
{
    UINT bw = 0;

    uint8_t head[8];
    uint32_t magic = SNAP_MAGIC_V2;
    uint16_t hz = SNAP_HZ, n16 = (uint16_t)Snap_Count();
    memcpy(head,      &magic, 4);
    memcpy(head + 4,  &n16,   2);
    memcpy(head + 6,  &hz,    2);

    if (f_write(&g_fil, SNAP_PREAMBLE, sizeof(SNAP_PREAMBLE), &bw) != FR_OK) return 1;
    if (f_write(&g_fil, head, 8, &bw) != FR_OK) return 1;

    /* payload = snap 点阵直接整块写（16B/点） */
    if (f_write(&g_fil, Snap_Points(), Snap_Count() * sizeof(SnapPt), &bw) != FR_OK) return 1;

    uint32_t crc = Snap_Crc32();
    if (f_write(&g_fil, &crc, 4, &bw) != FR_OK) return 1;

    if (f_sync(&g_fil) != FR_OK) return 1;          /* 落盘防拔卡丢 */

    /* 校验读回：文件长度应为 11+8+n*16+4 */
    uint32_t expect = 11 + 8 + Snap_Count() * 16 + 4;
    uint32_t fsz = (uint32_t)g_fil.obj.objsize;
    if (fsz != expect) return 1;

    if (f_close(&g_fil) != FR_OK) return 1;
    return 0;
}

uint8_t Sd_SaveSnap(void)
{
    if (Snap_Count() == 0) return 2;
    if (!g_mounted) {
        if (Sd_Mount() != 0) return 1;
    }

    uint32_t base = HAL_GetTick() / 1000;
    for (uint32_t i = 0; i < 60; i++) {             /* 秒+1 重试防同名 */
        char name[12];
        fmt_name(name, base + i);
        FRESULT r = f_open(&g_fil, name, FA_CREATE_NEW | FA_WRITE);
        if (r == FR_OK) {
            uint8_t e = sd_write_frame();
            if (e == 0) {
                sd_puts("SD save ok: "); sd_puts(name); sd_puts("\r\n");
            } else {
                sd_puts("SD save fail: "); sd_puts(name); sd_puts("\r\n");
            }
            return e;
        }
        if (r != FR_EXIST) {
            sd_puts("SD open fail FR="); sd_putu((uint32_t)r); sd_puts("\r\n");
            return 3;
        }
    }
    sd_puts("SD name busy\r\n");
    return 3;
}

/* ---------- CLI/调试：LS / STAT ---------- */

uint8_t Sd_Stat(void)
{
    HAL_SD_CardInfoTypeDef info;
    if (SD_GetCardInfo(&info) != 0) { sd_puts("SD card error\r\n"); return 1; }
    sd_puts("SD card: type="); sd_putu(info.CardType);
    sd_puts(" blocks="); sd_putu(info.BlockNbr);
    sd_puts(" blockSize="); sd_putu(info.BlockSize);
    sd_puts(" logBlockNbr="); sd_putu(info.LogBlockNbr);
    sd_puts(" logBlockSize="); sd_putu(info.LogBlockSize);
    sd_puts("\r\n");
    return 0;
}

void Sd_Ls(void)
{
    DIR d;
    FRESULT r = f_opendir(&d, "");
    if (r != FR_OK) { sd_puts("opendir fail FR="); sd_putu((uint32_t)r); sd_puts("\r\n"); return; }
    FILINFO fi;
    while (1) {
        r = f_readdir(&d, &fi);
        if (r != FR_OK) { sd_puts("readdir fail FR="); sd_putu((uint32_t)r); sd_puts("\r\n"); break; }
        if (fi.fname[0] == 0) break;
        sd_puts("  "); sd_puts(fi.fname);
        sd_puts("  sz="); sd_putu(fi.fsize); sd_puts("\r\n");
    }
    f_closedir(&d);
}

/* 底层裸扇区测试（绕过 FatFS）：SD RAW <sector>  读；SD RAW w <sector> 写0xAA再读回 */
void Sd_Raw(uint32_t sector, uint8_t write)
{
    static uint32_t buf[128];   /* 512B */
    if (!write) {
        uint32_t t0 = HAL_GetTick();
        HAL_StatusTypeDef st = HAL_SD_ReadBlocks(&hsd1, (uint8_t *)buf, sector, 1, HAL_MAX_DELAY);
        sd_puts("raw read: st="); sd_putu((uint32_t)st);
        sd_puts(" ms="); sd_putu(HAL_GetTick() - t0);
        if (st == HAL_OK) {
            sd_puts(" d0="); sd_putu(buf[0]);
            sd_puts(" d1="); sd_putu(buf[1]);
            sd_puts(" d2="); sd_putu(buf[2]);
            sd_puts("\r\n");
        } else {
            sd_puts(" fail\r\n");
        }
        return;
    }
    for (uint32_t i = 0; i < 128; i++) buf[i] = 0x5A5A0000u | i;
    uint32_t t0 = HAL_GetTick();
    HAL_StatusTypeDef st = HAL_SD_WriteBlocks(&hsd1, (uint8_t *)buf, sector, 1, HAL_MAX_DELAY);
    sd_puts("raw write: st="); sd_putu((uint32_t)st);
    sd_puts(" ms="); sd_putu(HAL_GetTick() - t0);
    if (st != HAL_OK) { sd_puts(" fail\r\n"); return; }
    uint32_t rd[128];
    st = HAL_SD_ReadBlocks(&hsd1, (uint8_t *)rd, sector, 1, HAL_MAX_DELAY);
    sd_puts(" readback: st="); sd_putu((uint32_t)st);
    if (st == HAL_OK) {
        sd_puts(" d0="); sd_putu(rd[0]); sd_puts(" d1="); sd_putu(rd[1]);
        sd_puts(" d2="); sd_putu(rd[2]);
        sd_puts(" match="); sd_putu((rd[0] == buf[0] && rd[1] == buf[1] && rd[2] == buf[2]) ? 1 : 0);
        sd_puts("\r\n");
    } else {
        sd_puts(" fail\r\n");
    }
}