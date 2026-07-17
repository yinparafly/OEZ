#!/usr/bin/env python3
"""
Balloon Drop v3 - 用RC override直接控制
绕过FBWA姿态控制器, 直接操作升降舵
"""
import time
from pymavlink import mavutil

print("=== Balloon Drop v3 RC Override Test ===")
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

# FBWA mode
m.set_mode_fbwa()
time.sleep(1)

# Phase 1: Nose down, throttle 0 - command via RC override
# ch1=aileron(1500=level), ch2=elevator(1000=nose down), ch3=throttle(1000=off), ch4=rudder(1500=neutral)
print("Phase 1: Nose dive (elevator=1000, throttle=1000)")
start = time.time()
while time.time() - start < 30:
    # Override: aileron=1500, elevator=1000(nose down), throttle=1000(off), rudder=1500
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        0, 0, 1500, 1000, 1000, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg and msg.get_type() == 'VFR_HUD':
        print(f"  [{time.time()-start:.1f}s] aspd={msg.airspeed:.1f} alt={msg.alt:.0f}")
        if msg.airspeed >= 12:
            print("  >>> TRIGGER! Switching to pull-up")
            break

# Phase 2: Pull up - elevator up (1800), throttle still 0
print("Phase 2: Pull up (elevator=1800)")
pullup_start = time.time()
while time.time() - pullup_start < 10:
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        0, 0, 1500, 1800, 1000, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg and msg.get_type() == 'VFR_HUD':
        print(f"  [{time.time()-pullup_start:.1f}s] aspd={msg.airspeed:.1f} alt={msg.alt:.0f}")

# Phase 3: Level, throttle up
print("Phase 3: Level + throttle up")
m.mav.rc_channels_override_send(
    m.target_system, m.target_component,
    0, 0, 1500, 1500, 1600, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
)
time.sleep(5)

# Disarm
m.arducopter_disarm()
m.close()
print("Done!")
