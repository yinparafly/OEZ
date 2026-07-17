#!/usr/bin/env python3
"""
Balloon Drop v5 - 完整流程: 起飞→爬升→释放→测试
MANUAL模式起飞, 爬升后切FBWA释放
"""
import time
from pymavlink import mavutil

print("=== Balloon Drop v5 - Full Flight Test ===")
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

# Arm + MANUAL mode
print("Arm + MANUAL mode...")
m.arducopter_arm()
time.sleep(1)
m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
time.sleep(2)

# Takeoff: full throttle + slight nose up
print("Takeoff (throttle=2000, elevator=1400)...")
for i in range(200):  # 20 seconds
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        0, 0, 1500, 1400, 2000, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    if i % 50 == 0:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg and msg.get_type() == 'VFR_HUD':
            print(f"  [{i/10:.0f}s] aspd={msg.airspeed:.1f} alt={msg.alt:.0f}")
    time.sleep(0.1)

# Climb: reduce throttle, maintain nose up
print("Climb (throttle=1600, elevator=1400)...")
for i in range(400):  # 40 seconds
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        0, 0, 1500, 1400, 1600, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    if i % 100 == 0:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg and msg.get_type() == 'VFR_HUD':
            print(f"  [{20+i/10:.0f}s] aspd={msg.airspeed:.1f} alt={msg.alt:.0f}")
    time.sleep(0.1)

# Get current state
alt_now = 0
aspd_now = 0
msg = m.recv_match(blocking=True, timeout=2)
if msg and msg.get_type() == 'VFR_HUD':
    alt_now = msg.alt
    aspd_now = msg.airspeed

print(f"\nCurrent: alt={alt_now:.0f}m aspd={aspd_now:.1f}m/s")

# ===== RELEASE =====
print("\n===== RELEASE =====")
print("Switch to FBWA, Lua script should take over")

# Release override
m.mav.rc_channels_override_send(
    m.target_system, m.target_component,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
)
time.sleep(0.5)

# Switch to FBWA
m.set_mode_fbwa()
time.sleep(1)

# Monitor
print("Monitoring free fall (30s)...")
start = time.time()
try:
    while time.time() - start < 30:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == 'VFR_HUD':
            elapsed = time.time() - start
            if int(elapsed) % 3 == 0:
                print(f"  [{elapsed:.1f}s] aspd={msg.airspeed:.1f} alt={msg.alt:.0f}")
        if msg.get_type() == 'STATUSTEXT' and 'BD' in msg.text and 'Running' not in msg.text:
            print(f"  [{time.time()-start:.1f}s] >>> {msg.text}")
except KeyboardInterrupt:
    pass

m.arducopter_disarm()
m.close()
print("Done!")
