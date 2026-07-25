# Android APK 更新说明（1.9.4-share-csv）

与固件 **`FW=monitor-v37-keep-shot`** / PC 蓝牙路径对齐。下午 BLE 踩坑与规则见：  
`docs/工作日志与总结-2026-07-25下午-手机BLE.md`。

## 当前能力

- **开始监控** → 武装后停 BLE 实时转速 → 环缓/短弹射触发 → RAM 记满 → SD 存档  
- 收到 `# BLE PULL READY` → ~200ms → `SNAP?` → **DUMP BIN BLE**（RAM；空则 SD）  
- Dump：**Notify 为主**；约 500ms 无包才救援 READ；payload/`B,seq` 去重  
- **拉取**按钮：手动再拉；拉数/武装中禁用二次「开始监控」  
- 进度条 + 百分比；CRC 失败有提示  
- **发送数据**：只分享 **CSV**（不发 PNG）  
- **打开文件夹**：尽量打开 `下载/AbiRpmMonitor`  
- 导出目录：公共 **`Download/AbiRpmMonitor/`**

## 编译安装

```bat
cd /d E:\OEZCON\mcoder\ESP32_AS5047P_ABI_Monitor\android\AbiRpmMonitor
gradlew.bat assembleDebug
copy /Y app\build\outputs\apk\debug\app-debug.apk E:\OEZCON\mcoder\AbiRpmMonitor-debug.apk
adb install -r E:\OEZCON\mcoder\AbiRpmMonitor-debug.apk
```

或：`build_apk.bat`（若存在）。

要求：手机已开蓝牙；板子已烧 **v37**；同一时间不要 PC 占着 BLE。

## 联调步骤

1. 扫描 → 选 **OEZ-ABI** → 连接  
2. **开始监控** → Toast「请加油」→ 拧电调 / 弹射  
3. Toast：探测到了 → 开始记录 → 记录完毕 → 已存入存储卡  
4. 自动拉数 →「数据已取回」→ 曲线可看  
5. **发送数据** → 系统分享 CSV；**打开文件夹** → `下载/AbiRpmMonitor`

失败时看：`got≈2×exp`（重复包）、`BIN 校验失败`、`MONITOR START count` 是否连涨、`src=RAM|SD`。
