# ArduPilot SITL TCP/UDP 连接问题分析与解决方案

**日期**: 2026-07-04  
**环境**: Windows 10/11 + WSL2 Ubuntu 22.04 + pymavlink 2.4.49 + ArduPlane SITL

---

## 问题描述

在 Windows 环境下通过 pymavlink 连接 WSL 中运行的 ArduPilot SITL 模拟器时，出现以下两种失败模式：

### 模式一：TCP 连接被拒绝
```
ConnectionRefusedError: [WinError 10061] 由于目标计算机积极拒绝，无法连接
```
SITL 进程启动后立即崩溃退出，端口 5760 无进程监听。

### 模式二：UDP 连接成功但无数据流
```
Heartbeat OK! System=0 Type=1 Mode=UNKNOWN
(后续 0 条数据消息)
```
心跳收到但模拟器不发送遥测数据（空速、姿态、GPS 等）。

---

## 根因分析

### 1. TCP 模式下 SITL 会崩溃

当使用 `--serial0 tcp:5760` 启动 SITL 时：

```
bind port 5760 for SERIAL0
SERIAL0 on TCP port 5760
...
Waiting for internal clock bits to be set (current=0x00)
```

**根因**：ArduPilot SITL 使用 **lock-step（步进锁）仿真模式**。在这种模式下：
- 仿真器每一步都需要外部系统（通常是 MAVProxy）确认 FDM（飞行动力学模型）数据已被消费
- 如果没有外部系统响应，仿真器会在等待时钟信号超时后退出
- TCP 连接是阻塞式的——只有一个客户端能连接，且连接后需要持续交互

**证据**：日志中 `Waiting for internal clock bits to be set (current=0x00)` 来自 `SIM_LP5562.cpp`（LED 驱动仿真），说明外围设备初始化卡住，主循环无法推进。

### 2. UDP 模式下无数据流

使用 `--serial0 udpclient:127.0.0.1:5760` 时：

- SITL 进程保持运行（PID 存活，状态 Sl+）
- pymavlink 能收到心跳（因为心跳由固件主循环直接发出）
- 但空速、姿态、GPS 等遥测数据不发送

**根因**：
- `udpclient` 模式下，SITL 向指定地址发送 UDP 数据包
- 但 pymavlink 的 `udp:` 连接前缀创建的是 **被动 UDP 套接字**（先监听再发送）
- SITL 需要先收到客户端的 UDP 包才能知道回传地址
- 心跳能收到是因为 ArduPilot 固件在主循环中直接发送，不依赖仿真步进
- 但遥测数据（VFR_HUD、ATTITUDE 等）需要仿真步进推进后才产生

### 3. sim_vehicle.py 为什么能工作

`sim_vehicle.py` 启动 SITL 时做了以下关键操作：

```python
# 默认使用 UDP
c.extend(["--serial0", "udpclient:127.0.0.1:" + str(5760+i*10)])
```

然后启动 **MAVProxy** 连接到同一端口：
```python
c.extend(["--master", "tcp:127.0.0.1:" + str(5760 + 10 * i)])
```

MAVProxy 的作用：
1. 作为中间层连接 SITL（通过 UDP）
2. 发送 `REQUEST_DATA_STREAM` 等初始化命令
3. 持续消费 FDM 输出，推动 lock-step 仿真步进
4. 将数据转发给其他客户端（通过 TCP 5762 端口）

---

## 解决方案

### 方案 A：使用 MAVProxy 作为中间层（推荐）

```
AI-MP → MAVProxy (TCP 5762) → SITL (UDP 5760)
```

1. 启动 SITL（UDP 模式）
2. 启动 MAVProxy 连接 SITL
3. pymavlink 连接 MAVProxy 的输出端口

```bash
# WSL 中启动
cd /root/ardupilot
python3 sim_vehicle.py -v ArduPlane --no-mavproxy --out udpclient:127.0.0.1:5760

# 另一个终端启动 MAVProxy
mavproxy.py --master udp:127.0.0.1:5760 --baudrate 115200 --aircraft MySITL
```

然后 pymavlink 连接 MAVProxy 的 TCP 端口 5762。

### 方案 B：修复 pymavlink UDP 连接时序

在 pymavlink 连接 UDP 后，先发送一个 UDP 包让 SITL 知道回传地址：

```python
import socket
# 先发一个空 UDP 包建立回传路径
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b'\x00', ('127.0.0.1', 5760))
sock.close()

# 然后用 pymavlink 连接
master = mavutil.mavlink_connection('udp:127.0.0.1:5760', source_system=255)
```

### 方案 C：使用 TCP 端口 5762（MAVProxy 输出端）

SITL 启动时会自动绑定 SERIAL1 在 TCP 5762。这个端口：
- 不需要 MAVProxy 中间层
- 直接由 SITL 的 TCP 服务器管理
- 可以接受 pymavlink 连接

```python
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762', source_system=255)
```

### 方案 D：使用 sim_vehicle.py 完整启动

```bash
cd /root/ardupilot
python3 sim_vehicle.py -v ArduPlane --out udpclient:127.0.0.1:5760 --no-mavproxy
```

然后 pymavlink 连接 TCP 5762（SITL 的 SERIAL1 输出端口）。

---

## 测试验证

### TCP 直连测试结果
```
启动: arduplane --serial0 tcp:5760
结果: 进程崩溃，日志显示 "Waiting for internal clock bits"
结论: TCP 直连 pymavlink 不可行
```

### UDP 直连测试结果
```
启动: arduplane --serial0 udpclient:127.0.0.1:5760
pymavlink: udp:127.0.0.1:5760
结果: 进程存活，心跳收到，0 条遥测数据
结论: UDP 直连 pymavlink 不完整
```

### 推荐方案
```
使用 sim_vehicle.py 启动 SITL → pymavlink 连接 TCP 5762
```

---

## 关键文件参考

| 文件 | 作用 |
|------|------|
| `Tools/autotest/sim_vehicle.py` | SITL 启动脚本（标准方式） |
| `libraries/SITL/SIM_Aircraft.cpp` | 仿真器主循环，lock-step 逻辑 |
| `libraries/SITL/SIM_Plane.cpp:51` | `lock_step_scheduled = true` |
| `libraries/AP_HAL_SITL/SITL_State.cpp:91` | `_fdm_input_step()` 步进函数 |
| `libraries/SITL/SIM_LP5562.cpp:119` | "clock bits" 消息来源（LED 驱动） |

---

## ArduPilot SITL 架构总结

```
┌─────────────────────────────────────────────┐
│                arduplane                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │ SERIAL0  │  │ SERIAL1  │  │ SERIAL2  │  │
│  │ UDP 5760 │  │ TCP 5762 │  │ TCP 5763 │  │
│  └────┬─────┘  └────┬─────┘  └──────────┘  │
│       │              │                      │
│       └──── FDM (lock-step) ────┘           │
└─────────────────────────────────────────────┘
       │              │
       ▼              ▼
   MAVProxy      pymavlink
   (UDP client)   (TCP client)
       │
       ▼
   AI-MP / Mission Planner
```

**核心约束**：lock-step 模式下，FDM 输出需要被消费才能推进下一步。没有消费者 = 仿真暂停。
