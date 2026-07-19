# 电机 BLE 转速控制（Android）

手机遥控 **ESP32 + AS5047P + 电调** 台架。  
**不是**听声测转速 App（那个在 `../MotorPitchRpm`）。

## 功能

- 显示当前转速（BLE 遥测）
- 启动 / 停止
- 手动设定转速
- 预定按钮：1200 / 2400 / 3600 / 4800（点按后设定并自动 START）

## ESP32

**无需改固件。** 使用现有广播名 `OEZ-RPM` + Nordic UART：

| 方向 | 内容 |
|------|------|
| 手机→板 | `RPM <n>` / `START` / `STOP` / `PING` |
| 板→手机 | `B,t_ms,rpm,pulse,target,mode,run` 或完整 CSV |

## 编译安装

```bat
cd android\MotorBleControl
gradlew.bat assembleDebug
```

APK：`app\build\outputs\apk\debug\app-debug.apk`

手机：打开蓝牙 → 授予附近设备/蓝牙权限 → **扫描** → 选 OEZ-RPM → **连接**。
