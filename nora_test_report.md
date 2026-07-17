# CUAV Nora 硬件在环测试报告

**日期：** 2026-07-03
**状态：** ✅ 连接成功，功能验证通过

---

## 连接测试结果

| 测试项 | 结果 | 详情 |
|--------|------|------|
| COM16 USB 连接 | ✅ | pymavlink 成功连接 |
| 心跳包接收 | ✅ | Vehicle Type=13 (FixedWing) |
| 固件识别 | ✅ | Autopilot=3 (ArduPilot) |
| Lua 脚本启用 | ✅ | SCR_ENABLE=1 已设置 |
| 参数设置 | ✅ | SCR_HEAP_SIZE=8192 已设置 |
| SITL UDP 连接 | ✅ | 127.0.0.1:14550 |

## Mission Planner 问题分析

**根因：** Mission Planner GUI 连接失败不是硬件问题，是 MP 自身的 LINQ 聚合错误（`Enumerable.Aggregate` 空集合异常）。

**影响：** 不影响 pymavlink 直接操控，不影响固件烧录，不影响 Lua 脚本执行。

**解决方案：**
1. 用 pymavlink 替代 Mission Planner GUI 进行连接和操控
2. 后续可尝试修复 MP 或使用 QGroundControl

## 可直接拷贝的文件

| 文件 | 路径 | 说明 |
|------|------|------|
| balloon_drop.lua | `D:\oezcon\ardupilot_scripts\balloon_drop.lua` | ArduPilot Lua 脚本 |
| test_balloon_drop.py | `D:\oezcon\ardupilot_scripts\test_balloon_drop.py` | 逻辑验证 |
| nora_full_test.py | `D:\oezcon\nora_full_test.py` | 连接测试 |
| nora_direct_test.py | `D:\oezcon\nora_direct_test.py` | 功能验证 |
| arduplane_CUAV_Nora.apj | `D:\pix\arduplane_CUAV_Nora.apj` | ArduPilot 固件 |
| MissionPlanner-latest.msi | `D:\pix\missionplanner\MissionPlanner-latest.msi` | MP 安装包 |

## 下次启动任务

1. 将 balloon_drop.lua 上传到 Nora 的 SD 卡
2. 在 Mission Planner 中设置 SCR_ENABLE=1
3. 测试 Balloon Drop 完整飞行剖面
4. 硬件在环仿真验证
