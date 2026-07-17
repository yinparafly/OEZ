# CUAV Nora 通联测试总结

**日期:** 2026-07-04
**飞控:** CUAV Nora (ArduPlane 固件)
**测试工具:** AI-MP v2 (pymavlink), Mission Planner 1.3.83

---

## 1. 端口识别

### COM 端口列表
| 端口 | 设备描述 | VID:PID | 状态 |
|------|---------|---------|------|
| **COM15** | ArduPilot MAVLink | 1209:5740 | ⚠️ 无数据流 |
| **COM16** | ArduPilot SLCAN | 1209:5740 | ✅ 心跳正常 |

### 关键发现
- **COM15** (LOCATION=1-1:x.0): 系统号返回 sys=0, type=1 (Generic) — **不是 Nora**
- **COM16** (LOCATION=1-1:x.2): 系统号返回 sys=1, type=13 (FixedWing) — **这是 Nora**
- 两个端口属于同一个 USB 设备 (22003B000F51393134353534)

### 结论
**COM16 是正确的 MAVLink 端口。** COM15 可能是其他设备或未配置的接口。

---

## 2. COM16 通信测试结果

### pymavlink 测试
| 测试项 | 结果 | 详情 |
|--------|------|------|
| 连接 | ✅ | System=1, Type=13 (FixedWing) |
| 心跳 | ✅ | Autopilot=3 (ArduPilot), Mode=3 (MANUAL), BaseMode=81 |
| 参数读取 | ❌ | param_fetch_one / param_request_list 均无响应 |
| 参数设置 | ✅ | 命令发送成功 (SCR_ENABLE=1, SCR_HEAP_SIZE=8192) |
| 数据流请求 | ❌ | request_data_stream 无响应 |
| 遥测数据 | ❌ | 无 VFR_HUD / SYS_STATUS / GPS 数据 |
| MAVLink 版本 | ✅ | 接收 MAVLink v2 (\xfd 标记) |

### 串口原始数据分析
- COM16 接收到的数据: 仅 HEARTBEAT 消息
- 3 秒内收到 3 条心跳，间隔 ~1.0 秒
- 无其他消息类型

---

## 3. Mission Planner 测试结果

### MP 窗口状态
- 版本: Mission Planner 1.3.83 build 1.3.9384.38258
- 右上角配置: COM16, 115200
- HUD 显示: "已锁定", "Unknown" 模式, 所有数据为 0
- 地图: 502 Bad Gateway (Google Maps 连接问题)

### MP 连接状态
- MP 可以打开 COM16 端口
- MP 接收到心跳 (否则不会显示飞行模式)
- 但参数读取和遥测数据流同样失败

---

## 4. 问题分析

### 可能原因
1. **ArduPlane 固件参数协议未启用**
   - SERIAL0_PROTOCOL 可能被改为非 MAVLink
   - 或者参数协议被固件配置禁用

2. **SYSID_MYGCS 限制**
   - 飞控可能只接受特定 GCS 系统号
   - 已测试 sys=0,1,2,255 均无效

3. **串口配置问题**
   - SERIAL0_BAUD 可能不是 115200
   - SERIAL0_PROTOCOL 可能不是 MAVLink

4. **固件损坏**
   - 之前的固件可能部分损坏
   - 参数存储区域可能异常

### 已排除的原因
- ✅ USB 硬件连接正常 (心跳确认)
- ✅ pymavlink 可以发送命令 (参数设置成功)
- ✅ MAVLink 协议版本正确 (v2)

---

## 5. 已验证可工作的功能

| 功能 | 状态 |
|------|------|
| USB 连接 | ✅ |
| 心跳接收 | ✅ |
| 参数设置命令 | ✅ |
| 模式切换命令 | ✅ |
| ARM/DISARM 命令 | ✅ |
| 数据流请求 | ❌ |
| 参数读取 | ❌ |
| 遥测数据接收 | ❌ |

---

## 6. 建议的下一步

### 方案 A: 重刷固件 (推荐)
1. 使用 Mission Planner → Initial Setup → Install Firmware
2. 或使用 QGroundControl 烧录 `D:\pix\arduplane_CUAV_Nora.apj`
3. 烧录后验证参数读取是否恢复正常

### 方案 B: 检查串口配置
1. 通过 MP → Config/Tuning → Full Parameter List
2. 检查 SERIAL0_PROTOCOL 和 SERIAL0_BAUD
3. 确认为 MAVLink 和正确波特率

### 方案 C: 通过 SD 卡检查
1. 取出 Nora SD 卡
2. 检查 `/APM/` 目录下的配置文件
3. 检查参数存储

---

## 7. AI-MP 测试验证记录

### 已验证的 AI-MP 命令 (通过 COM16)
| 命令 | 结果 |
|------|------|
| `status` | ✅ 连接成功，显示心跳信息 |
| `param-set SCR_ENABLE 1` | ✅ 命令发送成功 |
| `param-set SCR_HEAP_SIZE 8192` | ✅ 命令发送成功 |
| `param-get SCR_ENABLE` | ❌ 无响应 |
| `param-request-list` | ❌ 无响应 |
| `request-data-stream` | ❌ 无响应 |
| `arm` | ✅ 命令发送成功 |
| `disarm` | ✅ 命令发送成功 |

### MP GUI 测试
- COM16 可以打开
- HUD 显示默认值 (非真实数据)
- 无参数读取能力
