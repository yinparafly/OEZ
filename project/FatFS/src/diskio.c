#include "diskio.h"
#include "sd_card.h"

DSTATUS disk_initialize(BYTE drv) {
    if (drv != 0) return STA_NOINIT;
    return SD_Init() ? 0 : STA_NOINIT;
}

DSTATUS disk_status(BYTE drv) {
    if (drv != 0) return STA_NOINIT;
    return 0;
}

DRESULT disk_read(BYTE drv, BYTE* buff, DWORD sector, BYTE count) {
    if (drv != 0 || count == 0) return RES_PARERR;
    for (BYTE i = 0; i < count; i++) {
        if (!SD_ReadSector(sector + i, buff + i * 512))
            return RES_ERROR;
    }
    return RES_OK;
}

#if _READONLY == 0
DRESULT disk_write(BYTE drv, const BYTE* buff, DWORD sector, BYTE count) {
    if (drv != 0 || count == 0) return RES_PARERR;
    for (BYTE i = 0; i < count; i++) {
        if (!SD_WriteSector(sector + i, buff + i * 512))
            return RES_ERROR;
    }
    return RES_OK;
}
#endif

DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void* buff) {
    if (drv != 0) return RES_PARERR;
    switch (ctrl) {
        case CTRL_SYNC: return RES_OK;
        case GET_SECTOR_COUNT: *(DWORD*)buff = 15523840; break;
        case GET_SECTOR_SIZE:  *(WORD*)buff = 512; break;
        case GET_BLOCK_SIZE:   *(DWORD*)buff = 1; break;
        default: return RES_PARERR;
    }
    return RES_OK;
}

DWORD get_fattime(void) { return 0; }