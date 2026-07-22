/************************************************
 * ESP32 BLE UART（Nordic UART）— 电机+磁编码器台架
 *
 * 硬约束：2kHz 采集回调 / 400Hz 控制任务内禁止 Serial/hostPrintf 阻塞。
 * hostPrintf/hostPrintln 若在实时路径被调用 → 入异步日志环，由 hostDrainAsyncLog 排出。
 ************************************************/
#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLE_DEVICE_NAME "OEZ-RPM"
#define BLE_TELEM_HZ_DEFAULT 20
#define BLE_TELEM_HZ_MIN 5
#define BLE_TELEM_HZ_MAX 50

void bleBegin();
/** 运行时开关：OFF 停 advertising 并短路 NUS TX/RX；ON 恢复广播（已 init 则不再 deinit）。 */
void bleSetEnabled(bool on);
void bleEnd();
bool bleIsEnabled();
bool bleConnected();
bool bleTakeRxLine(char* out, size_t out_sz);
void bleEmitTelemLite(uint32_t t_ms, float rpm, uint16_t pulse, float target, int mode, int run,
                      float out_hz_meas, float gear_meas);
void bleSetRate(uint16_t hz);
void bleSetLite(bool on);
uint16_t bleTelemHz();
bool bleLiteOn();
void bleSendLine(const char* s);
void bleSendRaw(const char* data, size_t n);

/** 登记禁止直接打印的任务（电机控制任务）。 */
void hostForbidPrintFromTask(TaskHandle_t t);
/** 采集回调进出：标记实时路径，hostPrintf 只入环。 */
void hostEnterRealtimeCb();
void hostExitRealtimeCb();
/** 低优路径（loop）排出异步日志；每拍可多行。 */
void hostDrainAsyncLog();
uint32_t hostAsyncLogDropped();
/** 清零 async_log_drop 计数（LOAD RESET）。 */
void hostAsyncLogDropReset();

void hostPrintln(const char* s);
void hostPrintf(const char* fmt, ...);
