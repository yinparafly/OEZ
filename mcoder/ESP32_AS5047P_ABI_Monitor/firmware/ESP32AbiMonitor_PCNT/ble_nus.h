/************************************************
 * Nordic UART BLE — ABI 转速监控专用（独立工程）
 ************************************************/
#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#ifndef BLE_DEVICE_NAME
#define BLE_DEVICE_NAME "OEZ-ABI"
#endif

#ifndef BLE_TELEM_HZ_DEFAULT
#define BLE_TELEM_HZ_DEFAULT 10
#endif

void bleBegin();
bool bleConnected();
/** 本次 loop 检测到「刚连上」边沿（供 SESSION/对时）。 */
bool bleTakeConnectEdge();
bool bleTakeRxLine(char* out, size_t out_sz);
void bleSendLine(const char* s);
void bleSendRaw(const char* data, size_t n);
void bleSetRate(uint16_t hz);
uint16_t bleTelemHz();
/** STAGING/RECORD 期间静音 host*→BLE，避免一点监控就灌包卡死 PC/手机。 */
void bleMuteHost(bool mute);
bool bleHostMuted();
/** 只打串口，不灌 BLE（PING 等心跳用，避免冲掉遥测）。 */
void hostSerialPrintln(const char* s);
void hostPrintln(const char* s);
void hostPrintf(const char* fmt, ...);
