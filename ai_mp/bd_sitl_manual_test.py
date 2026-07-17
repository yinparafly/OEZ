#!/usr/bin/env python3
"""
Balloon Drop SITL v4 - MANUAL模式RC控制
MANUAL模式: RC直接映射到舵面, 不经过姿态控制器
"""
import time
from pymavlink import mavutil

print("=== Balloon Drop v4 - MANUAL Mode Test ===")
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
m.wait_heartbeat()
print(f"Connected sys={m.target_system}")

# Params
m.mav.param_set_send(m.target_system, m.target_component, b'SCR_HEAP_SIZE', 200000.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
m.mav.param_set_send(m.target_system, m.target_component, b'PTCH_LIM_MIN_DEG', -90.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
time.sleep(1)

# Reboot
m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(6)
m.close()

m = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
m.wait_heartbeat()
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
time.sleep(3)

# Arm
print("Arming...")
m.arducopter_arm()
time.sleep(2)

# MANUAL mode (mode 0) - direct surface control
print("Switching to MANUAL mode (direct surface control)...")
m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
time.sleep(2)

# Phase 1: Nose dive via RC override
# In MANUAL: ch1=aileron, ch2=elevator, ch3=throttle, ch4=rudder
# elevator=1000 means full nose-down
print("Phase 1: Nose dive (elevator=1000, throttle=1000)")
start = time.time()
while time.time() - start < 30:
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        0, 0, 1500, 1000, 1000, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg and msg.get_type() == 'VFR_HUD':
        elapsed = time.time() - start
        if int(elapsed * 2) % 6 == 0:
            print(f"  [{elapsed:.1f}s] aspd={msg.airspeed:.1f} alt={msg.alt:.0f}")
        if msg.airspeed >= 12:
            print(f"  >>> TRIGGER at {msg.airspeed:.1f} m/s!")
            break

# Phase 2: Pull up
print("Phase 2: Pull up (elevator=1800, throttle=1000)")
pullup_start = time.time()
while time.time() - pullup_start < 10:
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        0, 0, 1500, 1800, 1000, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg and msg.get_type() == 'VFR_HUD':
        print(f"  [{time.time()-pullup_start:.1f}s] aspd={msg.airspeed:.1f} alt={msg.alt:.0f}")

# Phase 3: Level + throttle
print("Phase 3: Level + throttle up")
for _ in range(50):
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        0, 0, 1500, 1500, 1600, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(0.1)

m.arducopter_disarm()
m.close()
print("Done!")
