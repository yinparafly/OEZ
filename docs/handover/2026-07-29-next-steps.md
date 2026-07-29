# 后续开发交接文档

## 项目状态

固件已完成全部基础功能，烧录验证通过。剩余工作主要为**功能扩展**和**PC 工具完善**。

## 固件现状

### 引脚分配

| 引脚 | 功能 |
|------|------|
| PA0 | 编码器 A (TIM2_CH1) |
| PA1 | 编码器 B (TIM2_CH2) |
| PB0 | **绿灯** (推挽输出) |
| PB1 | **红灯** (推挽输出) |
| PB10 | Z 信号 (EXTI10 上升沿) |
| PB11 | 启动/触发 (上拉输入，短接GND) |
| PC13 | 板载状态 LED |
| PA4 | SD CS |
| PA5-7 | SPI1 (SCK/MISO/MOSI) |
| PA9 | USART1 TX |
| PA10 | USART1 RX |
| PA13/14 | SWD (烧录/调试) |

### 状态机

```
上电 → STARTUP_DELAY (3s)
     → IDLE (等待开始)
         ├ PB11 短接 GND
         └ 串口 'G'
     → MONITOR (绿灯慢闪, RPM 计算, 缓冲预触发数据)
         ├ RPM>12 + Z 上升沿
         └ PB11 短接 GND (手动触发)
     → TRIGGERED (红灯快闪, 0.8s 后触发采样)
     → WRITING (LED 灭, SD 卡写入)
     → PATTERN_DONE (绿灯快闪2次+慢闪1次)
     → IDLE
```

### PC UI (pc_ui.py)

Python 3 + tkinter + matplotlib + pyserial。三个标签页:
1. **CSV Viewer**: 打开 CSV 显示表格+波形
2. **Serial Monitor**: 串口实时绘图
3. **Control Panel**: LED 测试按钮 + 发送 'G' + 状态显示

## 待开发功能 (按优先级)

### P1: 串口发送编码器数据
- 当前固件只接收串口命令，不发送数据
- 建议: 监控状态下每隔 N 个采样发送一次编码器计数值
- 可在 `Recorder_ISR_Check` 的 MONITOR case 中加入 `USART_SendData`
- PC UI Serial Monitor 标签页已准备好接收

### P2: PC UI 串口发送 'G' 后自动切换标签
- 发送 'G' 后自动切换到 Serial Monitor 标签查看实时数据

### P3: 多次记录合并导出
- 支持将多个 CSV 文件合并为一个
- PC UI 增加导出功能

### P4: SD 卡容量检测
- 启动时检查 SD 卡剩余空间
- 不足时通过 LED 闪烁模式报警

### P5: 参数通过串口配置
- 通过串口命令修改 RPM 触发阈值、采样率等参数
- 当前均为编译时常量 (#define)

### P6: USB 虚拟串口 (CDC)
- 如果使用 STM32F103C8T6 的 USB 外设，可实现 USB CDC 虚拟串口
- 省去外部 USB 转串口模块

### P7: FreeRTOS 移植
- 当前为裸机状态机 (4kHz TIM3 ISR)
- 如果未来需要更复杂的调度，可移植 FreeRTOS

## 已知问题

1. **IWDG 与 Flash 冲突**: 烧录时 IWDG 可能导致算法超时 — 已解决 (使用 `program` 命令绕过)
2. **FatFS 挂载失败处理**: 如果 SD 卡未插入，启动时 `f_mount` 失败，`g_file_index` 默认为 0
   - 目前静默忽略，LED 无提示
3. **PC UI 串口断线重连**: 已实现自动重连选项，但断线时状态指示可能滞后

## 文件索引

| 文件 | 说明 |
|------|------|
| `D:\oezcon\project\Core\Src\main.c` | 主程序, GPIO/定时器/中断初始化 |
| `D:\oezcon\project\Core\Src\recorder.c` | 状态机, LED 控制, RPM 计算, SD 保存 |
| `D:\oezcon\project\Core\Src\encoder.c` | TIM2 编码器驱动 |
| `D:\oezcon\project\Core\Src\usart_cmd.c` | 串口命令处理 |
| `D:\oezcon\project\Core\Src\sd_card.c` | SPI SD 卡驱动 |
| `D:\oezcon\project\Core\Src\stm32f10x_it.c` | 中断处理 |
| `D:\oezcon\project\Core\Src\ring_buffer.c` | 环形缓冲区 |
| `D:\oezcon\project\FatFS\src\diskio.c` | FatFS 底层 SPI 接口 |
| `D:\oezcon\project\pc_ui.py` | PC 上位机 |
| `D:\oezcon\project\flash.bat` | 一键烧录 |
| `D:\oezcon\project\Makefile` | 编译配置 |
| `D:\oezcon\project\linker.ld` | 链接脚本 |
| `D:\oezcon\SUMMARY.md` | 项目总结 |

## 工具链

- **编译器**: `D:\arm-official\arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi\bin\arm-none-eabi-gcc`
- **烧录**: `D:\openocd\xpack-openocd-0.12.0-7\bin\openocd.exe`
- **PC UI**: Python 3.12 + matplotlib 3.11 + pyserial 3.5
- **编译命令**: `cd D:\oezcon\project && mingw32-make`
- **烧录命令**: `cd D:\oezcon\project && flash.bat`
