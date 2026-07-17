# CUAV Nora 硬件在环仿真 — 任务清单

**日期：** 2026-07-03
**目标：** 解决 Nora USB 连接 → 刷固件 → 硬件在环仿真

---

## 阶段 1：USB 驱动问题
- [x] 1.1 确认 USB 芯片型号（VID_1209 PID_5740，CuAV 专用）
- [x] 1.2 下载驱动（CP210x + STM32 VCP）
- [x] 1.3 安装驱动（Windows 已识别 COM15/COM16）
- [x] 1.4 验证设备管理器中出现正确 COM 口
- [x] 1.5 pymavlink 能通过 COM16 USB 连接 ✅
- [ ] 1.6 Mission Planner GUI 连接（MP LINQ bug，待修复）

## 阶段 2：固件
- [x] 2.1 pymavlink 连接 Nora ✅
- [x] 2.2 下载 ArduPilot 固件（arduplane_CUAV_Nora.apj 1.5MB）
- [x] 2.3 当前固件正常（ArduPilot, Autopilot=3），暂不需要重刷
- [x] 2.4 验证固件版本 ✅

## 阶段 3：功能验证
- [x] 3.1 pymavlink 连接验证 ✅
- [x] 3.2 SCR_ENABLE=1 已设置 ✅
- [x] 3.3 Lua 脚本支持确认 ✅
- [x] 3.4 AI-MP v2 全功能工具完成 ✅
- [ ] 3.5 上传 balloon_drop.lua 到 Nora SD 卡
- [ ] 3.6 测试 Balloon Drop 完整飞行剖面

---

## 进度记录

### 2026-07-03 14:00
- 任务清单创建
- 开始排查 USB 驱动问题

### 2026-07-03 15:40
- ✅ USB 连接确认：VID_1209 PID_5740，COM15/COM16 已识别
- ✅ pymavlink 直连测试通过：COM16 @ 115200
- ✅ 固件确认：ArduPilot, Autopilot=3, Vehicle=13 (FixedWing)
- ✅ Lua 脚本支持：SCR_ENABLE=1 已设置
- ⚠️ Mission Planner GUI 连接失败（LINQ Aggregate bug）
- 💡 pymavlink 可完全替代 MP GUI 操控 Nora

### 2026-07-03 16:25
- ✅ AI-MP v2 全功能工具完成（`D:\oezcon\ai_mp\ai_mp2.py`）
- ✅ 操作日志系统：每步操作显示时间、目的、详情、结果
- ✅ 支持命令：status/monitor/arm/disarm/mode/param-set/param-get/reboot/upload-script/balloon-drop-test
- ✅ Balloon Drop 测试验证通过（19种模式可用，心跳1秒间隔）
- ✅ 参数可设置：SCR_ENABLE=1, SCR_HEAP_SIZE=8192, ARSPD_USE=1
- ⚠️ 参数读取需要飞控重启后生效（pymavlink 限制）
- 📋 下一步：上传 balloon_drop.lua 到 SD 卡，实飞测试
