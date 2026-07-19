/************************************************
 * ESP32 BLE UART（Nordic UART）— 电机+磁编码器台架
 ************************************************/
#pragma once

#include <Arduino.h>
#include <stdint.h>

#define BLE_DEVICE_NAME "OEZ-RPM"
#define BLE_TELEM_HZ_DEFAULT 20
#define BLE_TELEM_HZ_MIN 5
#define BLE_TELEM_HZ_MAX 50

void bleBegin();
bool bleConnected();
bool bleTakeRxLine(char* out, size_t out_sz);
void bleEmitTelemLite(uint32_t t_ms, float rpm, uint16_t pulse, float target, int mode, int run);
void bleSetRate(uint16_t hz);
void bleSetLite(bool on);
uint16_t bleTelemHz();
bool bleLiteOn();
void bleSendLine(const char* s);
void bleSendRaw(const char* data, size_t n);

void hostPrintln(const char* s);
void hostPrintf(const char* fmt, ...);
