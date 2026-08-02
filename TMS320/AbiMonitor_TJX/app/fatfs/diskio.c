/*-----------------------------------------------------------------------/
/  Low level disk interface (SD Card over SPIB, TMS320F28P550SJ9)        /
/  Port: SPI Mode 0, controller, CS = GPIO6 (software)                   /
/-----------------------------------------------------------------------*/
#include "ff.h"
#include "diskio.h"
#include "driverlib.h"
#include "board.h"

#define SD_LSPCLK_KHZ  37500uL      /* LSPCLK = SYSCLK/4 = 37.5MHz */
#define SD_INIT_KHZ    400uL        /* init clock <= 400kHz */
#define SD_RW_KHZ      9000uL       /* read/write clock ~9MHz (max LSPCLK/4) */

#define SD_CS_HIGH()   GPIO_writePin(6U, 1U)
#define SD_CS_LOW()    GPIO_writePin(6U, 0U)

static volatile DSTATUS sd_stat = STA_NOINIT;
static volatile uint32_t sd_sectors = 0;
volatile uint8_t sd_dbg_step = 0;
volatile uint8_t sd_dbg_r1   = 0xFF;
volatile uint8_t sd_loopback_ok = 0;

static void sd_spi_clock(uint32_t khz)
{
    SPI_disableModule(SD_SPI_BASE);
    SPI_setConfig(SD_SPI_BASE, SD_LSPCLK_KHZ, SPI_PROT_POL0PHA0,
                  SPI_MODE_CONTROLLER, khz, 8U);
    SPI_enableModule(SD_SPI_BASE);
}

static uint8_t sd_xchg(uint8_t dat)
{
    SPI_writeDataNonBlocking(SD_SPI_BASE, (uint16_t)dat << 8U);
    return (uint8_t)(SPI_readDataBlockingNonFIFO(SD_SPI_BASE) & 0xFFU);
}

static void sd_dummy_clocks(uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++)
        sd_xchg(0xFF);
}

static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t r1, i;
    if (cmd & 0x80U)
    {
        SD_CS_LOW();
        sd_dummy_clocks(2);
    }
    sd_xchg((uint8_t)(cmd & 0x7FU) | 0x40U);
    sd_xchg((uint8_t)(arg >> 24));
    sd_xchg((uint8_t)(arg >> 16));
    sd_xchg((uint8_t)(arg >> 8));
    sd_xchg((uint8_t)arg);
    sd_xchg(0x01U);
    for (i = 0; i < 8; i++)
    {
        r1 = sd_xchg(0xFF);
        if (!(r1 & 0x80U))
            return r1;
    }
    return 0xFF;
}

static uint8_t sd_acmd41(uint32_t hcs)
{
    uint8_t r;
    sd_cmd(0x77, 0);
    r = sd_cmd(0x69, hcs);
    return r;
}

/* SPIB loopback test */
void sd_spi_loopback_test(void)
{
    uint8_t sent, recv;
    sd_loopback_ok = 0;

    SPI_disableModule(SD_SPI_BASE);
    SPI_setConfig(SD_SPI_BASE, SD_LSPCLK_KHZ, SPI_PROT_POL0PHA0,
                  SPI_MODE_CONTROLLER, 400U, 8U);
    SPI_enableLoopback(SD_SPI_BASE);
    SPI_enableModule(SD_SPI_BASE);

    for (sent = 0xA5; sent <= 0xAA; sent++)
    {
        SPI_writeDataNonBlocking(SD_SPI_BASE, (uint16_t)sent << 8U);
        recv = (uint8_t)(SPI_readDataBlockingNonFIFO(SD_SPI_BASE) & 0xFFU);
        if (recv != sent)
        {
            SPI_disableModule(SD_SPI_BASE);
            SPI_disableLoopback(SD_SPI_BASE);
            SPI_setConfig(SD_SPI_BASE, SD_LSPCLK_KHZ, SPI_PROT_POL0PHA0,
                          SPI_MODE_CONTROLLER, 400U, 8U);
            SPI_enableModule(SD_SPI_BASE);
            sd_loopback_ok = 0xFF;
            return;
        }
    }
    SPI_disableModule(SD_SPI_BASE);
    SPI_disableLoopback(SD_SPI_BASE);
    SPI_setConfig(SD_SPI_BASE, SD_LSPCLK_KHZ, SPI_PROT_POL0PHA0,
                  SPI_MODE_CONTROLLER, 400U, 8U);
    SPI_enableModule(SD_SPI_BASE);
    sd_loopback_ok = 1;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    uint8_t r1, i;
    uint32_t ocr;
    uint8_t retry;

    if (pdrv != 0)
        return STA_NOINIT;

    sd_spi_clock(SD_INIT_KHZ);

    for (retry = 0; retry < 3; retry++)
    {
        SD_CS_HIGH();
        sd_dummy_clocks(80);

        sd_dbg_step = 0;
        sd_dbg_r1   = 0xFF;

        /* CMD0 enter SPI mode (crc=0x95 for CMD0) */
        SD_CS_LOW();
        sd_dummy_clocks(2);
        sd_xchg(0x40);
        sd_xchg(0x00); sd_xchg(0x00); sd_xchg(0x00); sd_xchg(0x00);
        sd_xchg(0x95);
        for (i = 0; i < 8; i++)
        {
            r1 = sd_xchg(0xFF);
            if (!(r1 & 0x80U))
                break;
        }
        if (r1 > 0x01)
        {
            SD_CS_HIGH();
            sd_dbg_step = 1;
            sd_dbg_r1   = r1;
            continue;
        }

        sd_dbg_step = 2;
        /* CMD8 - voltage check */
        r1 = sd_cmd(0x48, 0x000001AAuL);
        if (r1 > 0x01)
        {
            sd_dbg_step = 6;
            sd_dbg_r1   = r1;
        }
        else
        {
            sd_xchg(0xFF); sd_xchg(0xFF); sd_xchg(0xFF); sd_xchg(0xFF);
        }

        sd_dbg_step = 3;
        /* ACMD41 with HCS=1, retry until ready */
        for (i = 0; i < 200; i++)
        {
            r1 = sd_acmd41(0x40000000uL);
            if (r1 == 0x00)
                break;
        }
        if (r1 > 0x01)
        {
            SD_CS_HIGH();
            sd_dbg_step = 4;
            sd_dbg_r1   = r1;
            continue;
        }

        sd_dbg_step = 5;
        /* CMD58 read OCR */
        r1 = sd_cmd(0x7A, 0);
        ocr = 0;
        for (i = 0; i < 4; i++)
            ocr = (ocr << 8) | sd_xchg(0xFF);

        /* CMD9 read CSD for sector count */
        r1 = sd_cmd(0x49, 0);
        if (r1 == 0x00)
        {
            uint8_t csd[16];
            uint8_t r;
            for (i = 0; i < 17; i++)
            {
                r = sd_xchg(0xFF);
                if (i > 0)
                    csd[i - 1] = r;
            }
            if (ocr & 0x40000000uL)
            {
                sd_sectors = (uint32_t)((((uint32_t)csd[7] & 0x3FU) << 16) |
                                        ((uint32_t)csd[8] << 8) | csd[9]);
                sd_sectors = (sd_sectors + 1) << 10;
            }
            else
            {
                uint32_t c = (((uint32_t)csd[5] & 0x3FU) << 16) |
                             ((uint32_t)csd[6] << 8) | csd[7];
                uint32_t mult = 1U << ((((uint32_t)csd[9] & 0x3U) << 1) |
                                       (((uint32_t)csd[10] >> 7) + 2));
                uint32_t bsz = 1U << ((uint32_t)csd[12] & 0x0FU);
                sd_sectors = (c + 1) * mult * (bsz >> 9);
            }
        }
        SD_CS_HIGH();
        sd_dummy_clocks(8);

        if (ocr & 0x40000000uL)
            sd_stat = 0;
        else
            sd_stat = STA_PROTECT;

        sd_spi_clock(SD_RW_KHZ);
        sd_dbg_step = 0;
        return sd_stat;
    }

    sd_stat = STA_NOINIT;
    return sd_stat;
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0)
        return STA_NOINIT;
    return sd_stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    uint8_t r1, r;
    UINT n;

    if (pdrv != 0 || count == 0)
        return RES_PARERR;

    SD_CS_LOW();
    while (count)
    {
        r1 = sd_cmd(0x51, (uint32_t)sector);
        if (r1 != 0x00)
        {
            SD_CS_HIGH();
            return RES_ERROR;
        }
        {
            uint16_t to = 512;
            do {
                r = sd_xchg(0xFF);
            } while (r == 0xFF && --to);
            if (r != 0xFE)
            {
                SD_CS_HIGH();
                return RES_ERROR;
            }
        }
        for (n = 0; n < 512; n++)
            *buff++ = sd_xchg(0xFF);
        sd_xchg(0xFF); sd_xchg(0xFF);
        SD_CS_HIGH();
        sd_dummy_clocks(8);
        sector++;
        count--;
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    uint8_t r1;
    UINT n;

    if (pdrv != 0 || count == 0)
        return RES_PARERR;
    if (sd_stat & STA_PROTECT)
        return RES_WRPRT;

    SD_CS_LOW();
    while (count)
    {
        r1 = sd_cmd(0x58, (uint32_t)sector);
        if (r1 != 0x00)
        {
            SD_CS_HIGH();
            return RES_ERROR;
        }
        sd_xchg(0xFE);
        for (n = 0; n < 512; n++)
            sd_xchg(*buff++);
        sd_xchg(0xFF); sd_xchg(0xFF);
        {
            uint16_t to = 256;
            do {
                r1 = sd_xchg(0xFF);
            } while ((r1 & 0xFFU) == 0xFFU && --to);
        }
        if ((r1 & 0x05U) != 0x05U)
        {
            SD_CS_HIGH();
            return RES_ERROR;
        }
        while (sd_xchg(0xFF) != 0xFF)
            ;
        SD_CS_HIGH();
        sd_dummy_clocks(8);
        sector++;
        count--;
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0)
        return RES_PARERR;

    switch (cmd)
    {
        case CTRL_SYNC:
            return RES_OK;
        case GET_SECTOR_COUNT:
        {
            DWORD *dp = (DWORD *)buff;
            *dp = sd_sectors;
            return RES_OK;
        }
        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            return RES_OK;
        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}
