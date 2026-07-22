# Q16 / 环缓备份路径（默认关闭）

主线仍为浮点：**采集2k / 使用2k / 输出400 / USB遥测20Hz**。本目录内备份实现不改默认运行路径。

## 文件

| 文件 | 作用 |
|------|------|
| `q16_math.h` | Q16.16 加/乘/除/限幅、EMA、窗差求速、简易 PID |
| `enc_ring.h` | 16/32 点 `(t_us, counts)` 环缓；写覆盖最旧、追尾不阻塞 |
| `ESP32ReadAS5047P.ino` | `#define USE_FIXED_POINT 0`（默认）；为 1 时采集回调走定点求速+EMA 并写入环缓 |

## 如何启用备份

1. 打开 `ESP32ReadAS5047P.ino`，将 `#define USE_FIXED_POINT 0` 改为 `1`（或编译加 `-DUSE_FIXED_POINT=1`）。
2. 仅编译验证（不要 flash / 不要开电机）：
   `arduino-cli compile -b esp32:esp32:esp32s3 mcoder/ESP32_AS5047P/firmware/ESP32ReadAS5047P`
3. 串口可查：`FIXED?` → `use_fixed_point=0|1`、环缓 overrun 计数。
4. 硬件对漂后再考虑删浮点主路；当前 **待硬件验证**。

## 约束

- 实时路径（2k 回调 / 400 输出任务）禁止 `Serial.print` / 阻塞 `hostPrintf`。
- `USE_FIXED_POINT=0` 时环缓/Q16 仍可编译进工程，但采集回调不走定点分支，行为与 A/B 主线一致。