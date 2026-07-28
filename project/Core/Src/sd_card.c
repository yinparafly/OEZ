#include "sd_card.h"
#include <stdbool.h>

#define SPI_LOW_SPEED  128
#define SPI_HIGH_SPEED 4

static uint8_t SD_card_type = 0;

uint8_t SD_ReadWriteByte(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return (uint8_t)SPI1->DR;
}

static void SD_SPI_Init(uint16_t prescaler) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;

    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);

    SD_CS_HIGH();

    SPI_InitTypeDef spi;
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = prescaler;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
}

static uint8_t SD_SendCmdRaw(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t frame[6];
    frame[0] = 0x40 | cmd;
    frame[1] = (uint8_t)(arg >> 24);
    frame[2] = (uint8_t)(arg >> 16);
    frame[3] = (uint8_t)(arg >> 8);
    frame[4] = (uint8_t)arg;
    frame[5] = crc;
    for (int i = 0; i < 6; i++)
        SD_ReadWriteByte(frame[i]);
    uint8_t resp;
    for (int i = 0; i < 10; i++) {
        resp = SD_ReadWriteByte(0xFF);
        if (!(resp & 0x80)) break;
    }
    return resp;
}

static bool SD_WaitReady(void) {
    uint16_t timeout = 500;
    while (SD_ReadWriteByte(0xFF) != 0xFF && timeout) timeout--;
    return timeout != 0;
}

bool SD_Init(void) {
    uint8_t resp;
    uint16_t timeout;

    SD_SPI_Init(SPI_LOW_SPEED);
    SD_CS_HIGH();
    for (int i = 0; i < 10; i++)
        SD_ReadWriteByte(0xFF);
    SD_CS_LOW();

    timeout = 200;
    do {
        resp = SD_SendCmdRaw(0, 0, 0x95);
        timeout--;
    } while (resp != 0x01 && timeout);
    if (timeout == 0) { SD_CS_HIGH(); return false; }

    resp = SD_SendCmdRaw(8, 0x1AA, 0x87);
    if (resp == 0x01) {
        uint8_t buf[4];
        for (int i = 0; i < 4; i++) buf[i] = SD_ReadWriteByte(0xFF);
        if (buf[2] == 0x01 && buf[3] == 0xAA) {
            timeout = 1000;
            do {
                SD_SendCmdRaw(55, 0, 0x01);
                resp = SD_SendCmdRaw(41, 0x40000000, 0x01);
                timeout--;
            } while (resp != 0x00 && timeout);
            if (timeout == 0) { SD_CS_HIGH(); return false; }
            SD_card_type = 6;
        }
    } else {
        timeout = 1000;
        do {
            SD_SendCmdRaw(55, 0, 0x01);
            resp = SD_SendCmdRaw(41, 0, 0x01);
            timeout--;
        } while (resp != 0x00 && timeout);
        if (timeout == 0) { SD_CS_HIGH(); return false; }
        SD_card_type = 2;
    }

    SD_SendCmdRaw(16, 512, 0x01);
    SD_CS_HIGH();
    SD_ReadWriteByte(0xFF);

    SD_SPI_Init(SPI_HIGH_SPEED);
    return true;
}

bool SD_ReadSector(uint32_t sector, uint8_t* buffer) {
    if (!(SD_card_type & 4)) sector *= 512;

    SD_CS_LOW();
    uint8_t resp = SD_SendCmdRaw(17, sector, 0x01);
    if (resp != 0x00) { SD_CS_HIGH(); return false; }

    uint16_t timeout = 500;
    while (SD_ReadWriteByte(0xFF) != 0xFE && timeout) timeout--;
    if (timeout == 0) { SD_CS_HIGH(); return false; }

    for (int i = 0; i < 512; i++)
        buffer[i] = SD_ReadWriteByte(0xFF);
    SD_ReadWriteByte(0xFF);
    SD_ReadWriteByte(0xFF);
    SD_CS_HIGH();
    SD_ReadWriteByte(0xFF);
    return true;
}

bool SD_WriteSector(uint32_t sector, const uint8_t* buffer) {
    if (!(SD_card_type & 4)) sector *= 512;

    SD_CS_LOW();
    if (!SD_WaitReady()) { SD_CS_HIGH(); return false; }

    uint8_t resp = SD_SendCmdRaw(24, sector, 0x01);
    if (resp != 0x00) { SD_CS_HIGH(); return false; }

    SD_ReadWriteByte(0xFE);
    for (int i = 0; i < 512; i++)
        SD_ReadWriteByte(buffer[i]);
    SD_ReadWriteByte(0xFF);
    SD_ReadWriteByte(0xFF);

    resp = SD_ReadWriteByte(0xFF);
    SD_CS_HIGH();
    SD_ReadWriteByte(0xFF);
    if ((resp & 0x1F) != 0x05) return false;
    return true;
}