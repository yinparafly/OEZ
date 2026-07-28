#ifndef __SD_CARD_H
#define __SD_CARD_H

#include "stm32f10x.h"
#include <stdbool.h>

#define SD_CS_PORT      GPIOA
#define SD_CS_PIN       GPIO_Pin_4
#define SD_CS_LOW()     GPIO_ResetBits(SD_CS_PORT, SD_CS_PIN)
#define SD_CS_HIGH()    GPIO_SetBits(SD_CS_PORT, SD_CS_PIN)

bool SD_Init(void);
bool SD_ReadSector(uint32_t sector, uint8_t* buffer);
bool SD_WriteSector(uint32_t sector, const uint8_t* buffer);
uint8_t SD_ReadWriteByte(uint8_t data);

#endif