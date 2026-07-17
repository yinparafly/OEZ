# AI-MP 项目完整总结

**日期:** 2026-07-04
**状态:** 开发中

---

## 一、已完成工作

### 1.1 硬件识别与固件
- ✅ COM15 = ArduPilot MAVLink（正确端口）
- ✅ COM16 = ArduPilot SLCAN（控制台）
- ✅ 重刷 ArduPlane 固件 4.6.3.255 到 Nora
- ✅ 固件修复后参数/遥测全部正常

### 1.2 AI-MP v2 (pymavlink 直连)
- 代码: 2576 行
- 功能: 86 项中 72 项完成 (84%)
- 端口: COM15（默认）/ COM16（备用）
- 已验证: 参数读写/航点上传/传感器/EKF/日志/Rally

### 1.3 AI-MAVProxy 新模块
- 代码: ~500 行
- 路径: `D:\oezcon\ai_mavproxy\`
- 已验证: 连接/参数/遥测/飞行控制/ESP32控制

### 1.4 Lua 脚本
- `balloon_drop_v4.lua` — 简化版，遥控器开关触发
- 已拷贝到 SD 卡
- 等待实机测试

### 1.5 测试验证
- ✅ 7/7 自动化测试全部通过
- ✅ 参数读写: SCR_ENABLE=1.0
- ✅ 遥测: Alt/Roll/Pitch/Heading 正常
- ✅ 模式切换: MANUAL→FBWA→MANUAL
- ✅ 固件版本: 4.6.3.255
- ✅ 心跳稳定性: 0.51s 间隔
- ✅ PreArm 警告正常（室内）

---

## 二、经验教训

### 2.1 COM 端口识别
- **COM15 = MAVLink 端口**（sys=1, type=13 FixedWing）
- **COM16 = SLCAN 端口**（sys=1, type=1 FixedWing）
- 固件重刷后端口分配可能变化
- **教训**: 不要假设端口，每次连接前验证 sys/type

### 2.2 pymavlink API
- `param_request_list_send` 需要 `master.mav.` 前缀
- `param_fetch_one` 在 COM15 上正常工作
- `request_data_stream_send` 需要在读取数据前调用
- **教训**: pymavlink API 命名不一致，需要查阅文档

### 2.3 MP 端口冲突
- MP 和 AI-MP 不能同时连接同一个串口
- MP 启动后不会自动连接，需要手动选择端口
- **教训**: 需要在 MP 源码层面集成 CLI，或使用 TCP/UDP 转发

### 2.4 Altitude Angel 插件
- 导致 MP 崩溃（网络连接失败 + socket 缓冲区溢出）
- **教训**: 禁用不必要的第三方插件

### 2.5 Windows 端口管理
- TCP 端口在 TIME_WAIT 状态回收慢
- Socket bind 可能因权限问题失败
- **教训**: 使用 UDP 转发比 TCP 更稳定

### 2.6 SITL 仿真
- SITL 在 WSL 中已编译
- 启动参数: `--model plane --serial1 tcp:0.0.0.0:5760`
- **教训**: SITL 和真机不能同时使用

---

## 三、待完成工作

### P0 - 高优先级

| # | 任务 | 说明 | 依赖 |
|---|------|------|------|
| 1 | 下载 MP 源码 | 通过代理下载 GitHub 源码 | 网络 |
| 2 | MP 添加 CLI 接口 | 在 MP 源码中集成 pymavlink | MP 源码 |
| 3 | Lua 脚本验证 | 在 SITL 中测试 balloon_drop_v4 | SITL |
| 4 | SITL 全自动仿真 | AI-MP 控制 SITL 飞行 | SITL |

### P1 - 中优先级

| # | 任务 | 说明 | 依赖 |
|---|------|------|------|
| 5 | ESP32 固件 | 电源继电器 + GPIO 释放检测 | 硬件 |
| 6 | 舵机连接测试 | HIL 模式下舵机响应 | 舵机 |
| 7 | Balloon Drop 真机测试 | 完整流程验证 | ESP32 + 舵机 |
| 8 | MP 集成 | AI-MP 和 MP 同时工作 | MP 源码 |

### P2 - 低优先级

| # | 任务 | 说明 | 依赖 |
|---|------|------|------|
| 9 | QGC 集成 | 评估 QGroundControl 可用性 | QGC |
| 10 | 日志分析 | DataFlash 日志解析 | Python |
| 11 | 参数对比工具 | 两组参数差异分析 | Python |

---

## 四、文件清单

### 核心代码
```
D:\oezcon\ai_mavproxy\          # AI-MAVProxy 新模块
├── core/connection.py          # 连接管理
├── modules/param_manager.py    # 参数管理
├── modules/flight_control.py   # 飞行控制
├── modules/telemetry.py        # 遥测监控
├── hardware/esp32_control.py   # ESP32 控制
├── cli.py                      # CLI 入口
├── full_auto_test.py           # 全自动测试
├── auto_start.py               # 自动启动
└── test_report.txt             # 测试报告

D:\oezcon\ai_mp\               # AI-MP v2 (pymavlink 直连)
├── ai_mp2.py                   # 主程序
├── mission.py                  # 航点模块
├── verify.py                   # 验证模块
└── ai_screen.py                # 屏幕控制工具

D:\oezcon\ardupilot_scripts\    # Lua 脚本
├── balloon_drop_v4.lua         # 简化版（遥控器触发）
├── balloon_drop_v3.lua         # 三重检测版
└── balloon_drop.lua            # 原始版
```

### 配置文件
- `D:\oezcon\ai_mp\FEATURE_CHECKLIST.md` — 功能清单
- `D:\oezcon\ai_mp\TODO.md` — 开发进度
- `D:\oezcon\AI_MP_PLAN.md` — 架构计划
- `D:\oezcon\ai_mavproxy\test_report.txt` — 测试报告

---

## 五、下一步行动

### 重启后立即执行
1. 检查网络连接
2. 通过代理下载 MP 源码
3. 解压到 `D:\oezcon\MissionPlanner\`
4. 在 MP 源码中添加 CLI 接口
5. 编译测试

### 关键命令
```bash
# 下载 MP 源码
python -c "import requests; requests.get('https://github.com/ArduPilot/MissionPlanner/archive/refs/heads/master.zip', proxies={'https':'http://127.0.0.1:7890'})"

# 测试 AI-MP
python D:\oezcon\ai_mavproxy\full_auto_test.py

# 启动 SITL
wsl -e bash -c "cd /root/ardupilot/Tools/autotest && python3 sim_vehicle.py -v ArduPlane"
```
