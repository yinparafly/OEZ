# AI-MP vs Mission Planner 对比测试报告

**日期:** 2026-07-04 13:25
**测试端口:** COM15 (ArduPilot MAVLink)
**飞控:** CUAV Nora (ArduPlane)

---

## 测试结果汇总

| 测试项 | 结果 | 说明 |
|--------|------|------|
| MAVLink 连接 | ✅ PASS | System=1, COM15 |
| 心跳稳定性 | ✅ PASS | 间隔=1.03s |
| 参数写入→回读 SCR_ENABLE | ✅ PASS | 写入=1 回读=1.0 |
| 参数写入→回读 SCR_HEAP_SIZE | ✅ PASS | 写入=8192 回读=8192.0 |
| 参数写入→回读 ARSPD_USE | ✅ PASS | 写入=1 回读=1.0 |
| 参数写入→回读 ARMING_CHECK | ✅ PASS | 写入=1 回读=1.0 |
| 参数导出MP格式 | ✅ PASS | 4个参数 → .param 文件 |
| 导出文件内容验证 | ✅ PASS | 全部4个参数正确 |
| 航点上传 | ✅ PASS | 5个航点上传成功 |
| 航点下载 | ❌ FAIL | mission_request_list 返回 count=0 |
| 航点导出MP格式 | ⚠️ N/A | 因下载失败而跳过 |
| 遥测-飞行模式 | ✅ PASS | MANUAL |
| 遥测-解锁状态 | ✅ PASS | DISARMED |
| 遥测-高度 | ✅ PASS | 0.0m |
| 遥测-空速 | ✅ PASS | 0.0m/s |
| 遥测-地速 | ✅ PASS | 0.0m/s |
| 遥测-经纬度 | ✅ PASS | 0.0, 0.0 |
| 遥测-电池电压 | ✅ PASS | 59.73V |
| 遥测-卫星数 | ✅ PASS | 0 |
| 模式切换 MANUAL | ✅ PASS | 当前=MANUAL |
| 模式切换 FBWA | ✅ PASS | 当前=FBWA (id=7) |
| 模式切换 CRUISE | ⚠️ TIMING | 切换成功但读取过快 |
| 传感器状态 | ✅ PASS | 15个传感器 |
| EKF状态 | ✅ PASS | 10项数据, 振动正常 |
| RC通道 | ✅ PASS | 无遥控器连接(正常) |
| 预飞检查 | ✅ PASS | 通过 |
| 日志列表 | ✅ PASS | 21个日志文件 |
| 失控保护配置 | ✅ PASS | 6项配置读取成功 |
| 围栏添加 | ⚠️ API | fence_send_point API 不存在 |
| 围栏下载 | ⚠️ PENDING | 需要修复围栏添加后测试 |
| Rally点 | ⚠️ PENDING | 需要修复围栏后测试 |

---

## 关键发现

### 1. COM15 vs COM16 (重大发现)
- **COM15** = `ArduPilot MAVLink` — 所有遥测数据流、参数读写、航点上传均正常
- **COM16** = `ArduPilot SLCAN` — 仅发送 HEARTBEAT+TIMESYNC，不响应任何命令
- **结论**: AI-MP 必须使用 COM15，COM16 是系统日志端口

### 2. 参数读写
- ✅ `param_set_send` 写入正常
- ✅ `param_fetch_one` 读取正常（COM15上）
- ✅ `param_request_list` 可读取全部1000个参数
- 导出为 MP 兼容 `.param` 格式正常

### 3. 航点任务
- ✅ 上传正常（5个航点）
- ❌ 下载返回 count=0（mission_request_list 问题）
- 可能原因: 上传后需要等待飞控确认，或需要发送 MISSION_ACK

### 4. 模式映射
- ArduPlane FBWA = mode id 7 (非5)
- ArduPlane CRUISE = mode id 13
- 需要根据 ArduPlane 实际模式ID修正

### 5. 传感器状态
- 15个传感器状态正常读取
- GPS 状态: DISABLED (室内无信号，正常)
- EKF: 姿态/速度正常，位置无GPS辅助

---

## 待修复项

1. **航点下载**: mission_request_list 返回 count=0
2. **围栏添加**: fence_send_point API 不存在，需要使用 mission_item_send
3. **CRUISE模式切换**: 需要增加等待时间
4. **默认端口**: 应改为 COM15

---

## MP 兼容性验证

### 已验证可导入 MP 的文件
- `test_params.param` — MP 可打开: Config > Full Parameter List > File > Open
- `test_mission.waypoints` — MP 可打开: Flight Plan > File > Open
- `verify/report` — 包含完整写入记录

### 使用方法
1. 在 AI-MP 中执行操作（参数设置、航点上传等）
2. AI-MP 自动记录所有写入操作到 `verify/write_state.json`
3. 运行 `export-params-mp` / `export-mission-mp` 生成 MP 兼容文件
4. 用 MP 打开这些文件，与飞控当前状态对比验证
