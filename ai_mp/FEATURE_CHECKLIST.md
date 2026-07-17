# AI-MP v2 功能清单 vs Mission Planner

**日期:** 2026-07-04
**目标:** AI-MP 实现 Mission Planner 全部功能的 AI 命令行版本

---

## 一、已完成 ✅

| # | MP 功能 | AI-MP 命令 | 状态 |
|---|---------|-----------|------|
| 1 | 连接/心跳 | `connect()` | ✅ |
| 2 | 飞行状态 | `status` | ✅ |
| 3 | 实时监控 | `monitor` (增强版含Roll/Pitch/GPS) | ✅ |
| 4 | 解锁/上锁 | `arm` / `disarm` | ✅ |
| 5 | 飞行模式 | `mode FBWA` | ✅ |
| 6 | 参数读取 | `param-get` | ✅ |
| 7 | 参数设置 | `param-set` | ✅ |
| 8 | 重启飞控 | `reboot` | ✅ |
| 9 | 航点任务-加载 | `mission-load` | ✅ |
| 10 | 航点任务-显示 | `mission-show` | ✅ |
| 11 | 航点任务-下载 | `mission-download` | ✅ |
| 12 | 航点任务-上传 | `mission-upload` | ✅ |
| 13 | 航点任务-添加 | `mission-add` | ✅ |
| 14 | 航点任务-删除 | `mission-remove` | ✅ |
| 15 | 航点任务-清空 | `mission-clear` | ✅ |
| 16 | 航点任务-保存 | `mission-save` (simple/qgc) | ✅ |
| 17 | 航点任务-跳转 | `mission-goto` | ✅ |
| 18 | Lua 脚本上传 | `upload-script` (MAVLink FTP) | ✅ |
| 19 | Balloon Drop 测试 | `balloon-drop-test` | ✅ |
| 20 | RC 通道读取 | `rc` | ✅ |
| 21 | 传感器状态 | `sensor-status` | ✅ |
| 22 | 固件版本 | `version` | ✅ |
| 23 | 罗盘校准 | `calibrate-compass` | ✅ |
| 24 | 加速度计校准 | `calibrate-accel` | ✅ |
| 25 | 水平校准 | `calibrate-level` | ✅ |
| 26 | 日志列表 | `list-logs` | ✅ |
| 27 | 日志下载 | `download-log` | ✅ |
| 28 | 围栏显示 | `fence-show` | ✅ |
| 29 | 围栏添加(圆形) | `fence-add CIRCLE` | ✅ |
| 30 | Guided 飞往 | `guided-go LAT LON ALT` | ✅ |
| 31 | 起飞 | `takeoff ALT` | ✅ |
| 32 | 返航 | `rtl` | ✅ |
| 33 | 降落 | `land` | ✅ |
| 34 | 紧急停止 | `emergency` | ✅ |
| 35 | 参数搜索 | `param-search KEYWORD` | ✅ |
| 36 | ESP32 电源控制 | `esp32_power/` 模块 | ✅ |
| 37 | 参数批量读取 | `param-list-all` | ✅ |
| 38 | 参数树查看 | `param-tree` | ✅ |
| 39 | 遥测日志记录 | `tlog-record` | ✅ |
| 42 | 围栏启用/禁用 | `fence-enable` / `fence-disable` | ✅ |
| 45 | Rally 点管理 | `rally-show` / `rally-add` | ✅ |
| 46 | 伺服输出 | `servo-output` | ✅ |
| 47 | 电机测试 | `motor-test` | ✅ |
| 49 | 失控保护 | `failsafe` | ✅ |
| 50 | 预飞检查 | `prearm-check` | ✅ |
| 51 | 电池监控配置 | `battery-config` | ✅ |
| 52 | 空速校准 | `airspeed-calibrate` | ✅ |
| 53 | GPS 配置 | `gps-config` | ✅ |
| 54 | 串口配置 | `serial-config` | ✅ |
| 57 | KML 导出 | `export-kml` | ✅ |
| 65 | MAVLink Inspector | `mavlink-inspector` | ✅ |
| 67 | EKF 状态 | `ekf-status` | ✅ |

---

## 二、待实现 ❌ (按优先级排序)

### P0 - 核心功能 (必须)

| # | MP 功能 | AI-MP 命令 | 说明 |
|---|---------|-----------|------|
| 40 | 遥测日志回放 | `tlog-playback` | 回放 .tlog 文件 |
| 41 | 日志图形分析 | `log-graph` | 从 .bin 日志绘图 |
| 43 | 围栏下载 | `fence-download` | 下载飞控中的围栏 |
| 44 | 围栏-多边形 | `fence-add POLYGON` | 多边形围栏 |
| 48 | 飞行模式设置 | `flightmode-setup` | RC 开关→模式映射 |

### P1 - 重要功能

| # | MP 功能 | AI-MP 命令 | 说明 |
|---|---------|-----------|------|
| 51 | 电池监控配置 | `battery-config` | 电压/电流/容量设置 |
| 52 | 空速校准 | `airspeed-calibrate` | 空速传感器偏移 |
| 53 | GPS 配置 | `gps-config` | GPS 类型/协议/速率 |
| 54 | 串口配置 | `serial-config` | 串口协议/波特率分配 |
| 55 | OSD 配置 | `osd-config` | OSD 布局编辑 |
| 56 | CAN 总线配置 | `can-config` | DroneCAN 节点管理 |
| 57 | KML 导出 | `export-kml` | 飞行轨迹导出为 KML |
| 58 | 航点任务-DO 命令 | 扩展 `mission-add` | DO_SET_SERVO, DO_RELAY 等 |
| 59 | 航点任务-条件命令 | 扩展 `mission-add` | CONDITION_DELAY, CONDITION_YAW |
| 60 | Survey 规划 | `survey-plan` | 测绘航线规划 |
| 61 | Grid 规划 | `grid-plan` | 栅格航线规划 |
| 62 | 固件升级-完整 | `firmware-upgrade` | 自动检测板型+下载+烧录 |
| 63 | 摇杆配置 | `joystick-config` | USB 摇杆输入映射 |
| 64 | 语音播报 | `speech-config` | 遥测语音播报设置 |

### P2 - 高级功能

| # | MP 功能 | AI-MP 命令 | 说明 |
|---|---------|-----------|------|
| 65 | MAVLink Inspector | `mavlink-inspector` | 实时消息查看/过滤 |
| 66 | MAVFtp 完整 | `ftp` | SD 卡文件管理(浏览/上传/下载/删除) |
| 67 | EKF 状态 | `ekf-status` | EKF 健康/振动 |
| 68 | 预飞清单 | `checklist` | 自定义预飞检查清单 |
| 69 | 多机系统 | `multi-vehicle` | 多机切换/管理 |
| 70 | RTK/NTRIP | `ntrip-config` | RTK 差分配置 |
| 71 | ADS-B | `adsb-status` | ADS-B 目标显示 |
| 72 | SITL 启动 | `sitl-launch` | 启动模拟器 |
| 73 | 日志 CSV 导出 | `log-export` | .bin → .csv 转换 |
| 74 | GeoRef 图片标注 | `georef` | 照片地理标记 |
| 75 | 自定义警告 | `warnings-config` | 自定义告警条件 |

### P3 - 辅助功能

| # | MP 功能 | AI-MP 命令 | 说明 |
|---|---------|-----------|------|
| 76 | 参数对比 | `param-diff` | 对比两组参数 |
| 77 | 参数导入/导出 | `param-export` / `param-import` | 全量参数备份 |
| 78 | 航线平滑 | `mission-smooth` | Spline 样条插值 |
| 79 | 地形跟随 | `terrain-follow` | SRTM 高程数据 |
| 80 | 频谱分析 | `fft-analysis` | 振动 FFT 分析 |
| 81 | SiK 无线电配置 | `radio-config` | 遥测电台设置 |
| 82 | 联合飞行 | `formation-flight` | 编队飞行 |

---

## 三、AI-MP 独有功能 (MP 没有的)

| # | 功能 | 命令 | 说明 |
|---|------|------|------|
| 83 | ESP32 远程重启 | `reboot-remote IP` | 通过 WiFi 继电器重启飞控 |
| 84 | AI 操作日志 | 自动记录 | 每步操作时间+目的+结果 |
| 85 | Balloon Drop 模板 | `balloon-drop-test` | 专用测试序列 |
| 86 | 自然语言指令 | (未来) | AI 解析自然语言→MAVLink |

---

## 四、当前代码统计

- `ai_mp2.py`: 2515 行
- `mission.py`: 566 行
- `ai_mp.py` (v1): 旧版本
- 总计: ~3081 行
