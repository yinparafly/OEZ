-- Balloon Drop v8 - ArduPlane API
-- 垂直俯冲释放, 12m/s触发, 2.0g拉起
-- 使用 set_target_throttle_rate_rpy 控制
local INFO=6
local phase=0; local t_motor=0
local V_TRIG=12.0; local N=2.0; local G=9.81

function init()
  gcs:send_text(INFO, "BD v8: Trig=12 N=2g")
end

function update()
  local aspd = ahrs:airspeed_EAS()
  
  if phase==0 then
    -- Phase 1: 自由俯冲, 油门=0, 负俯仰角速度(低头)
    vehicle:set_target_throttle_rate_rpy(0, 0, -1.5, 0)
    if aspd>=V_TRIG then phase=1; gcs:send_text(INFO, string.format("BD: Pull %.1f",aspd)) end
    
  elseif phase==1 then
    -- Phase 2: 拉起, 油门=0, 正俯仰角速度(抬头)
    local omega = (N-1)*G/math.max(aspd,1)
    vehicle:set_target_throttle_rate_rpy(0, 0, omega, 0)
    local p = math.deg(ahrs:get_pitch_rad())
    if p>=-5 then phase=2; t_motor=millis(); gcs:send_text(INFO, "BD: Motor") end
    
  elseif phase==2 then
    -- Phase 3: 电机启动, 油门渐增
    local thr = math.min((millis()-t_motor)/2000, 1.0)
    vehicle:set_target_throttle_rate_rpy(thr, 0, 0, 0)
    if thr>=1.0 then phase=3; vehicle:set_mode("AUTO"); gcs:send_text(INFO, "BD: AUTO") end
  end
  return update, 100
end

init()
return update, 1000
