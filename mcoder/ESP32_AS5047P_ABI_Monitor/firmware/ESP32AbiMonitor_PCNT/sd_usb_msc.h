/************************************************
 * Native USB → 把 SPI SD 挂成只读 U 盘（MSC）
 * 串口仍走 CH343(COM)；读文件用板载 USB-C/OTG 口。
 *
 * 编译需 USB Mode = USB-OTG (TinyUSB)，见 flash_com6.bat
 ************************************************/
#pragma once

#include <stdint.h>

/** 本固件是否编进了 MSC（hwcdc 下为 false） */
bool sdMscAvailable();

bool sdMscIsOn();

/**
 * 开启 U 盘：停止本机对 SD 的文件操作，PC 可拷 snap_*.bin
 * 返回 false=无卡/不支持/已开
 */
bool sdMscOn();

/** 关闭 U 盘，恢复固件写 SD（REC/SD SAVE） */
bool sdMscOff();

void sdMscPrintStatus();
