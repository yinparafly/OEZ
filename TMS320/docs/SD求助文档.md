# SD 卡（SPI 模式）无应答 — 求助

## 硬件环境

| 项 | 值 |
|---|---|
| MCU | TMS320F28P550SJ9（天机星板，128PDT 封装，150MHz） |
| SD 卡 | microSD 成品读卡器模块（自带上下拉），卡为普通 microSD |
| 接口 | SPIB + GPIO 软件片选，SPI Mode 0（POL0PHA0），8bit |
| 原接线 | CLK→GPIO14、PICO(MOSI/DIN)→GPIO30、POCI(MISO/DOUT)→GPIO31、CS→GPIO6 |
| 备用接线 | CLK→GPIO26、PICO→GPIO24、POCI→GPIO25、CS→GPIO6（已改固件待测） |
| 供电 | 模块 VCC 接 3.3V |

## 现象

- `SPI LB`（SPIB 内部环回测试）：**OK**（SPI 外设本身工作正常，时钟/寄存器无问题）
- `SD INIT`：失败。调试变量 `sd_dbg_step=1`、`sd_dbg_r1=0xFF`
  - step=1 = 发送 CMD0（0x40 0x00 0x00 0x00 0x00 0x95）后，8 次轮询读取 R1 全部为 `0xFF`（无有效应答）
  - **MISO 线上没有任何数据回来**（不是应答错误，是根本没有低电平信号）
- `SD TEST`：`open w fail`（SD 未初始化成功，属连带现象）

## 已排查（软件侧，基本可排除）

1. **引脚 mux 已配置**：SysConfig 生成 `board.c` 正确调用 `GPIO_setPinConfig`（SPIB_CLK/PICO/POCI + GPIO6 CS 输出），`Board_init()` 已执行
2. **SPI 读写方向正确**：发送 `data<<8`（左对齐，符合 driverlib 要求）；接收 `&0xFF`（低 8 位有效，符合文档）
3. **CS 时序正确**：CS 拉高时先发 ≥80 dummy 时钟 → CS 拉低 → 发 CMD0 → 轮询 R1
4. **波特率正确**：初始化 400kHz（LSPCLK=37.5MHz 分频计算），读写 9MHz
5. **CMD0 CRC=0x95**（标准值）

## 已排查（硬件侧）

- 模块 VCC 接 3.3V
- MISO 上拉：模块自带（README 曾记录旧模块需外接 10kΩ MISO 上拉，新模块自带，应无此问题）
- 已换第二个读卡器模块，现象相同

## 求助问题

1. CMD0 阶段 MISO 完全无响应（r1 恒 0xFF），最可能是哪些原因？
   - 怀疑：SPI 时钟极性/相位不匹配？卡不支持 SPI 模式？CS 未真正选中？MOSI 信号没送到卡？
2. F28P55 的 SPIB 是否有已知的"时钟极性/相位"坑（比如默认 CPOL/CPHA 与数据手册不同）？
3. 是否有必要先排除 MOSI 是否有输出（用示波器/逻辑分析仪看 CLK/MOSI/CS 波形）？最推荐的第一步硬件测量是什么？
4. 有没有可能 400kHz 初始化时钟对某些卡太慢/太快的兼容性问题？
