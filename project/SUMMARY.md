# STM32F103 ABZ Encoder Data Recorder - 项目总结

## 文件位置

| 项目 | 路径 |
|------|------|
| 项目根目录 | `D:\oezcon\project\` |
| 固件源代码 | `D:\oezcon\project\Core\Src\` |
| 头文件 | `D:\oezcon\project\Core\Inc\` |
| 链接脚本 | `D:\oezcon\project\linker.ld` |
| Makefile | `D:\oezcon\project\Makefile` |
| 烧录脚本 | `D:\oezcon\project\flash.bat` |
| 固件 HEX | `D:\oezcon\project\firmware.hex` |
| PC UI | `D:\oezcon\project\pc_ui.py` |

## 引脚分配

| 引脚 | 功能 | 方向 |
|------|------|------|
| PA0 | TIM2_CH1 - 编码器 A | 输入 |
| PA1 | TIM2_CH2 - 编码器 B | 输入 |
| PB0 | 绿灯 (Green LED) | 推挽输出 |
| PB1 | 红灯 (Red LED) | 推挽输出 |
| PB10 | 编码器 Z 信号 (EXTI10 上升沿) | 浮空输入 |
| PB11 | 启动监控 (短接到 GND 开始) | 上拉输入 |
| PC13 | 板载状态 LED | 推挽输出 |
| PA9 | USART1 TX (115200) | 推挽输出 |
| PA10 | USART1 RX (115200) | 浮空输入 |
| PA4 | SD 卡 CS | 推挽输出 |
| PA5 | SPI1 SCK | 推挽输出 |
| PA6 | SPI1 MISO | 浮空输入 |
| PA7 | SPI1 MOSI | 推挽输出 |

## 工作流程

```
上电 → 启动延时(3秒, LED 快闪)
     → 等待开始(PC13 LED 慢闪)
         ├ 短接 PB11 到 GND
         └ PC 发送 'G' 字符(115200 串口)
     → 监控模式(绿灯慢闪, 编码器计数到环形缓冲区)
         ├ RPM > 12 + Z 上升沿 → 触发记录
         └ 短接 PB11 到 GND → 手动触发
     → 触发记录(红灯快闪, 200 预触发 + 3200 后触发采样)
     → SD 卡写入(所有 LED 灭)
     → 完成提示(绿灯快闪2次 + 慢闪1次)
     → 回到等待开始
```

## 技术参数

| 参数 | 值 |
|------|-----|
| MCU | STM32F103C8T6 (72MHz, 64KB Flash, 20KB RAM) |
| 编码器 | TIM2 编码器模式, 32位计数器 |
| 采样率 | 4kHz (TIM3) |
| 触发 | RPM > 12 + Z 上升沿 或 手动 |
| 预触发 | 200 采样 (50ms) |
| 后触发 | 3200 采样 (800ms) |
| 环形缓冲区 | 3584 × uint32_t (14KB) |
| SD 卡 | SPI1 + FatFS R0.08a, FAT32 |
| 文件名 | 0001.CSV ~ 9999.CSV (INDEX.TXT 计数) |
| 看门狗 | IWDG ~1.28s |
| 工具链 | arm-none-eabi-gcc 13.2.1 |

## 编译与烧录

```bat
cd D:\oezcon\project
mingw32-make          // 编译
flash.bat             // 烧录 (需要 ST-Link V2)
```

## SD卡输出 CSV 格式

```csv
index,count
0,12345
1,12347
...
3399,67890
```
