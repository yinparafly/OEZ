/************************************************
 * SPI microSD（与 ABI 15/16/17 错开）
 *   CS=10  SCK=12  MOSI=11  MISO=13  VCC=3.3V
 ************************************************/
#pragma once

#include <stddef.h>
#include <stdint.h>

bool sdBegin();
bool sdReady();
void sdPrintStatus();

/** 写读校验 /oez_sd_test.txt */
bool sdSelfTest();

/**
 * 把 SNAP 缓冲写成 /snap_XXXX.bin
 * payload = 原始 SnapPoint 字节（调用方给指针+长度）
 * 返回写出路径提示（串口打印）
 */
bool sdWriteBinary(const char* path, const uint8_t* data, size_t len);

/** 写小文本（时间戳 sidecar 等） */
bool sdWriteText(const char* path, const char* text);

/**
 * 找根目录最新 /snap_*.bin（按文件名字典序，墙钟命名可近似最新）。
 * path_out 需 ≥64；成功返回 true。
 */
bool sdFindLatestSnap(char* path_out, size_t path_sz, uint32_t* size_out);

/**
 * 整文件读入堆（调用方 free）。传 BLE 前先读完再关文件，避免 SPI/BLE 交错。
 */
bool sdReadEntire(const char* path, uint8_t** out, size_t* out_len);

/** 列出根目录若干文件 */
void sdListRoot(uint8_t max_files = 20);
