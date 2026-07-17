--[[
  Balloon Drop Launch Script for ArduPilot Fixed-Wing
  
  飞行剖面（垂直俯冲释放）：
    Phase 1 - FREE_FALL:  头部朝下(-90°)，保持航向，油门=0，检测空速
    Phase 2 - PULL_UP:    2.0g过载拉起至水平
    Phase 3 - MOTOR_START: 电机启动，加速到巡航速度
    Phase 4 - CRUISE:      切换 AUTO 模式执行航点任务
  
  飞机参数（基于真实飞机推导）：
    翼展 2.0m, 翼面积 0.55m², 质量 5.0kg
    巡航空速 22m/s, 失速速度 15m/s (水平)
    展弦比 7.27, CLmax=0.65, 升力线斜率 4.93/rad
  
  版本: v2.0 (适配真实飞机参数)
  日期: 2026-07-05
--]]

-- ============================================================
-- 飞机参数 (从真实飞机推导)
-- ============================================================
local PARAMS = {
  CRUISE_SPEED   = 22.0,   -- 巡航空速 m/s
  STALL_SPEED    = 15.0,   -- 失速速度 m/s (水平飞行)
  TRIGGER_AIRSPEED= 12.0,  -- 触发空速 m/s (俯冲时无重力失速风险, 12m/s足够拉起)
  LOAD_FACTOR    = 2.0,    -- 拉起过载 g
  MOTOR_RAMP_TIME= 2.0,    -- 电机渐增时间 s
  CRUISE_THROTTLE= 50,     -- 巡航油门 % (5kg飞机)
  PITCH_THRESHOLD= 0,      -- 拉起完成俯仰角 deg (拉到水平)
  DIVE_PITCH     = -80,    -- 俯冲俯仰角 deg (接近垂直)
}

-- ============================================================
-- 阶段定义
-- ============================================================
local PHASE = {
  FREE_FALL   = 0,
  PULL_UP     = 1,
  MOTOR_START = 2,
  CRUISE      = 3,
  DONE        = 4,
}

local phase = PHASE.FREE_FALL
local initial_yaw = nil
local pullup_start_time = nil
local motor_start_time = nil
local trigger_airspeed = 0
local pullup_omega = 0

-- ============================================================
-- 初始化
-- ============================================================
function init()
  trigger_airspeed = PARAMS.TRIGGER_AIRSPEED
  pullup_omega = (PARAMS.LOAD_FACTOR - 1) * 9.81 / trigger_airspeed
  
  gcs:send_text(MAVSEVERITY.INFO, 
    string.format("BalloonDrop v2: Trigger=%.1f m/s, Load=%.1fg, Omega=%.1fdeg/s, R=%.0fm", 
    trigger_airspeed, PARAMS.LOAD_FACTOR, math.deg(pullup_omega), 
    trigger_airspeed^2/((PARAMS.LOAD_FACTOR-1)*9.81)))
end

-- ============================================================
-- Phase 1: 自由下落 (头部朝下 -80°~-90°)
-- ============================================================
function update_free_fall()
  local airspeed = ahrs:get_airspeed()
  
  -- 锁定初始航向
  if initial_yaw == nil then
    initial_yaw = ahrs:get_yaw()
    gcs:send_text(MAVSEVERITY.INFO, 
      string.format("BalloonDrop: Free fall, heading=%.0f deg", math.deg(initial_yaw)))
  end
  
  -- 保持俯冲姿态 + 航向，油门=0
  vehicle:set_target_attitude(math.rad(PARAMS.DIVE_PITCH), 0, initial_yaw)
  vehicle:set_target_throttle_zero()
  
  -- 检测空速触发 (必须 > 失速速度)
  if airspeed >= trigger_airspeed then
    phase = PHASE.PULL_UP
    pullup_start_time = millis()
    gcs:send_text(MAVSEVERITY.INFO, 
      string.format("BalloonDrop: PULL-UP at V=%.1fm/s (>stall %.1f)", airspeed, PARAMS.STALL_SPEED))
  end
end

-- ============================================================
-- Phase 2: 拉起 (2.0g过载，从-80°拉到0°)
-- ============================================================
function update_pull_up()
  local airspeed = ahrs:get_airspeed()
  local roll, pitch, yaw = ahrs:get_euler_rpY()
  
  -- 动态计算角速度 (空速越低，角速度越大)
  if airspeed < 1.0 then airspeed = 1.0 end
  pullup_omega = (PARAMS.LOAD_FACTOR - 1) * 9.81 / airspeed
  
  -- 设置俯仰角速度（正值=抬头）
  local target_pitch = pitch + pullup_omega * 0.1  -- 0.1s步进
  vehicle:set_target_attitude(target_pitch, 0, initial_yaw)
  vehicle:set_target_throttle_zero()
  
  -- 检查是否拉起到水平
  local pitch_deg = math.deg(pitch)
  if pitch_deg >= PARAMS.PITCH_THRESHOLD then
    phase = PHASE.MOTOR_START
    motor_start_time = millis()
    gcs:send_text(MAVSEVERITY.INFO, 
      string.format("BalloonDrop: Level! pitch=%.1f deg, V=%.1fm/s", pitch_deg, airspeed))
  end
end

-- ============================================================
-- Phase 3: 电机启动 (油门渐增到巡航)
-- ============================================================
function update_motor_start()
  local elapsed = (millis() - motor_start_time) / 1000.0
  local progress = math.min(elapsed / PARAMS.MOTOR_RAMP_TIME, 1.0)
  
  -- 油门线性递增
  local throttle = PARAMS.CRUISE_THROTTLE * progress
  vehicle:set_target_throttle(throttle)
  vehicle:set_target_attitude(0, 0, initial_yaw)
  
  if progress >= 1.0 then
    phase = PHASE.CRUISE
    gcs:send_text(MAVSEVERITY.INFO, "BalloonDrop: Cruise throttle, switching AUTO")
  end
end

-- ============================================================
-- Phase 4: 巡航 (切换AUTO执行航点)
-- ============================================================
function update_cruise()
  vehicle:set_mode("AUTO")
  phase = PHASE.DONE
  gcs:send_text(MAVSEVERITY.INFO, "BalloonDrop: AUTO mode engaged")
end

-- ============================================================
-- 主更新函数 (10Hz)
-- ============================================================
function update()
  if phase == PHASE.FREE_FALL then
    update_free_fall()
  elseif phase == PHASE.PULL_UP then
    update_pull_up()
  elseif phase == PHASE.MOTOR_START then
    update_motor_start()
  elseif phase == PHASE.CRUISE then
    update_cruise()
  end
  
  return update, 100  -- 100ms = 10Hz
end

-- ============================================================
-- 启动
-- ============================================================
init()
return update, 1000  -- 首次延迟 1s 启动
