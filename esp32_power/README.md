# ESP32 Nora 电源控制模块

## 硬件清单
- ESP32 开发板 (任意型号) x 1
- 5V 继电器模块 x 1 (带光耦隔离)
- 杜邦线若干
- 5V 电源 (可从 Nora 取电)

## 接线图

```
ESP32 GPIO12 ────→ 继电器 IN
ESP32 GND    ────→ 继电器 GND
                  继电器 VCC ← 5V
                  
                  继电器 COM ← 电池正极 (VCC)
                  继电器 NO  → Nora VCC (受控)
                  电池负极 → 共地
```

## 固件烧录

1. 打开 Arduino IDE
2. 安装 ESP32 Board (文件→首选项→附加开发板管理器网址):
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. 工具→开发板→选择 "ESP32 Dev Module"
4. 打开 `nora_power_control/nora_power_control.ino`
5. 修改 WiFi 名称密码:
   ```cpp
   const char* WIFI_SSID = "YOUR_WIFI_SSID";
   const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
   ```
6. 上传固件

## 使用方法

上电后 ESP32 自动连接 WiFi，串口打印 IP 地址。

### 浏览器控制
- `http://<IP>/` — 控制面板 (带按钮)
- `http://<IP>/on` — 上电 Nora
- `http://<IP>/off` — 断电 Nora
- `http://<IP>/reboot` — 重启 Nora (断电2秒→上电)
- `http://<IP>/status` — JSON 状态

### Python 远程重启
```python
import requests

IP = "192.168.1.xxx"  # ESP32 IP

# 重启 Nora
requests.get(f"http://{IP}/reboot")

# 检查状态
status = requests.get(f"http://{IP}/status").json()
print(status)  # {"relay":true,"uptime":120,"reboots":3}
```

### AI-MP 集成
AI-MP v2 可通过 HTTP 调用 ESP32 重启:
```bash
python ai_mp2.py --port COM16 --cmd reboot-remote 192.168.1.xxx
```

## WiFi 模式

- 正常模式：连接你家 WiFi
- 失败自动切换 AP 模式：热点名称 `NoraPower`，密码 `12345678`

## 注意事项

1. **继电器选型**：选择带光耦隔离的模块，避免 ESP32 受电压波动影响
2. **线径**：Nora 最大电流约 2A，继电器选 10A 以上的
3. **共地**：ESP32 和 Nora 必须共地
4. **延迟**：WiFi 延迟约 10-50ms，不影响重启操作
5. **安全性**：HTTP 接口无认证，仅限局域网使用
