# 版本标签：v40-counts-bin（当前 live）

- 日期：2026-07-26
- 固件：`FW=monitor-v40-counts-bin`
- BIN：`magic=0xAB1C0002`，每点 **16B** = `t_us` + `counts(i64)` + `index_n`
- 转速：PC / Android 用差分窗（默认 32）从 counts 计算；曲线工作室可再平滑/圈毛刺
- ESP32 本地：live 路径仍保留 `VEL_WINDOW`+EMA，**记录中也继续更新**，供将来电机闭环
- 兼容：上位机仍可读旧 `0xAB1C0001`（12B rpm_x10）文件
- Android：`1.9.5-counts-bin`
- 默认正式段：**1 s**（`REC MS` 可改）
- 回退：`backups/v37-keep-shot-2026-07-26/`
