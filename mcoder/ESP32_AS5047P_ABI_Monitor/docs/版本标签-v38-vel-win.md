# 版本标签：v38-vel-win（当前 live）

- 日期：2026-07-26
- 固件：`FW=monitor-v38-vel-win`
- 相对 v37 变更：Snap/环缓转速由「单周期 Δcounts×hz」改为 **`SNAP_VEL_WIN=8`** 多拍 Δcounts/Δt（与 live 类似），减轻低速上升段台阶毛刺；BIN 格式不变。
- Android / PC 协议：仍兼容 v37（无需强制升 APK）。
- 回退：`backups/v37-keep-shot-2026-07-26/` + `docs/版本标签-v37-keep-shot.md`
