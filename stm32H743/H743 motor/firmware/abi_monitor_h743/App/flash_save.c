/**
  ******************************************************************************
  * @file    flash_save.c
  * @brief   Task 5 Flash 芯片保存：snap 点阵持久化到内部 Flash（掉电不丢）
  *
  * 芯片保存在前、SD 备份在后（用户定）：
  *   - DONE 后先把 snap 写入内部 Flash（无卡也能存），再备份 SD。
  *   - 布局（Bank2 扇区 0..3，0x08100000 起，共 512KB，>= 484KB SNAP_CAP）：
  *       偏移 0        32B 头 {magic, n, hz, crc32(zlib), pad}
  *       偏移 32       n x SnapPt（16B/点，<IqI 与 PC 工具一致）
  *   - H7 Flash 按 32B FLASHWORD 编程，源缓冲须 32B 对齐（static 对齐数组）。
  *   - 配置区 0x081E0000（Bank2 扇区 7）不受影响。
  *   - Flash_Dump：把芯片内记录按 PC 帧（preamble+<IHH+payload+crc32）
  *     从串口全速发出，等效 SD 文件内容，PC 端 abi_monitor.py 可曲线还原。
  ******************************************************************************
  */
#include "flash_save.h"
#include "snap_bin.h"
#include "app_cli.h"
#include "usart.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define SNAP_FLASH_BASE    0x08100000UL
#define SNAP_FLASH_SECTOR  0                     /* Bank2 扇区编号 0..7（扇7=0x081E0000 为配置区） */
#define SNAP_FLASH_SECTORS 4                     /* 4x128KB = 512KB */

static uint8_t s_flash_saved;                    /* 本次 snap 是否已写入 Flash */

uint32_t Flash_Count(void)   { return Snap_Count(); }
uint8_t  Flash_IsSaved(void) { return s_flash_saved; }

/* 32B 对齐中转缓冲（FLASHWORD 编程源）/ 块发送 */
static __attribute__((aligned(32))) uint8_t s_fw[32];
static uint8_t s_blk[512];

static uint8_t Flash_Erase(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef e;
    e.TypeErase = FLASH_TYPEERASE_SECTORS;
    e.Banks     = FLASH_BANK_2;
    e.Sector    = SNAP_FLASH_SECTOR;
    e.NbSectors = SNAP_FLASH_SECTORS;
    uint32_t err = 0;
    HAL_StatusTypeDef r = HAL_FLASHEx_Erase(&e, &err);
    HAL_FLASH_Lock();
    return (r == HAL_OK && err == 0xFFFFFFFFu) ? 0 : 1;
}

/* 读头字段：magic/n/hz/crc（从 Flash 直读） */
static uint32_t Rd32(uint32_t off)
{
    uint32_t v;
    memcpy(&v, (const void *)(SNAP_FLASH_BASE + off), 4);
    return v;
}

uint8_t Flash_IsData(void)
{
    return (Rd32(0) == SNAP_MAGIC_V2) ? 1 : 0;
}

uint32_t Flash_StoredCount(void)
{
    uint32_t n = Rd32(4);
    return (n <= SNAP_CAP) ? n : 0;
}

uint8_t Flash_SaveSnap(void)
{
    uint32_t n = Snap_Count();
    if (n == 0) return 2;
    if (Flash_Erase() != 0) return 3;

    uint32_t hdr[8] = { 0 };                    /* 32B：magic,n,hz,crc,pad */
    hdr[0] = SNAP_MAGIC_V2;
    hdr[1] = n;
    hdr[2] = SNAP_HZ;
    hdr[3] = Snap_Crc32();

    HAL_FLASH_Unlock();
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, SNAP_FLASH_BASE,
                          (uint32_t)(uintptr_t)hdr) != HAL_OK) {
        HAL_FLASH_Lock();
        return 4;
    }

    /* 点阵：块拷贝到 32B 对齐 s_fw 再编程 */
    const uint8_t *src = (const uint8_t *)Snap_Points();
    uint32_t rem = n * sizeof(SnapPt);
    uint32_t addr = SNAP_FLASH_BASE + 32;
    while (rem) {
        uint32_t k = (rem > 32) ? 32 : rem;
        memcpy(s_fw, src, k);
        if (k < 32) memset(s_fw + k, 0xFF, 32 - k);   /* 末块补 0xFF */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr,
                              (uint32_t)(uintptr_t)s_fw) != HAL_OK) {
            HAL_FLASH_Lock();
            return 4;
        }
        src += k; addr += 32; rem -= k;
    }
    HAL_FLASH_Lock();
    Usart_Print("Flash save ok: n="); Cli_PrintU32(n); Usart_Print("\r\n");
    s_flash_saved = 1;
    return 0;
}

/* DUMP：Flash 内记录按 PC 帧（preamble+<IHH+payload+crc32）全速串口发送 */
uint8_t Flash_Dump(void)
{
    uint32_t n = Flash_StoredCount();
    if (!Flash_IsData() || n == 0) return 2;

    static const uint8_t pre[11] = { 0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x55 };
    Usart_Write(pre, 11);

    uint8_t h[8];
    uint32_t magic = SNAP_MAGIC_V2, n16 = (uint16_t)n, hz = (uint16_t)SNAP_HZ;
    memcpy(h,     &magic, 4);
    memcpy(h + 4, &n16, 2);
    memcpy(h + 6, &hz, 2);
    Usart_Write(h, 8);

    /* payload 从 Flash 直读分块发 */
    uint32_t nbytes = n * sizeof(SnapPt);
    for (uint32_t off = 32; off < 32 + nbytes; ) {
        uint32_t k = (nbytes - (off - 32) > sizeof(s_blk)) ? sizeof(s_blk) : (nbytes - (off - 32));
        memcpy(s_blk, (const void *)(SNAP_FLASH_BASE + off), k);
        off += k;
        Usart_Write(s_blk, k);
    }

    uint32_t crc = Rd32(12);
    Usart_Write((uint8_t *)&crc, 4);
    return 0;
}
