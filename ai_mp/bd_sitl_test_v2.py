#!/usr/bin/env python3
"""
Balloon Drop SITL 完整集成测试
解锁 → 检查脚本 → 触发 → 验证阶段切换
"""
import time
from pymavlink import mavutil

print("=" * 60)
print("Balloon Drop SITL Integration Test v2")
print("=" * 60)

# Connect
print("\n[1] Connect SITL...")
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
m.wait_heartbeat()
print(f"  Connected! sys={m.target_system}")

# Set params
print("\n[2] Set params...")
m.mav.param_set_send(m.target_system, m.target_component, b'SCR_HEAP_SIZE', 200000.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
m.mav.param_set_send(m.target_system, m.target_component, b'PTCH_LIM_MIN_DEG', -90.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
time.sleep(1)

# Reboot to apply params and load script
print("\n[3] Reboot SITL...")
m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(6)
m.close()

# Reconnect
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
m.wait_heartbeat()
print(f"  Reconnected! sys={m.target_system}")

# Request data
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

# Wait for script to load
print("\n[4] Wait for script...")
time.sleep(5)

# Arm
print("\n[5] Arm...")
m.arducopter_arm()
time.sleep(2)

# Set FBWA mode
print("\n[6] Set FBWA mode...")
m.set_mode_fbwa()
time.sleep(1)

# Monitor for 30 seconds
print("\n[7] Monitoring (30s)...")
print("  Script should auto-trigger when airspeed >= 12 m/s")
print()
start = time.time()
script_events = []
while time.time() - start < 30:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg is None:
        continue
    t = msg.get_type()
    if t == 'STATUSTEXT':
        text = msg.text
        if 'BD' in text or 'BalloonDrop' in text or 'Lua' in text:
            elapsed = time.time() - start
            print(f"  [{elapsed:.1f}s] {text}")
            script_events.append((elapsed, text))
    if t == 'VFR_HUD':
        aspd = msg.airspeed
        alt = msg.alt
        if int(time.time() - start) % 5 == 0:
            print(f"  [{time.time()-start:.1f}s] HUD: aspd={aspd:.1f}m/s alt={alt:.1f}m")

# Summary
print("\n" + "=" * 60)
print("Test Summary")
print("=" * 60)
print(f"  Duration: {time.time()-start:.1f}s")
print(f"  Script events: {len(script_events)}")
for t, text in script_events:
    print(f"    [{t:.1f}s] {text}")

phases_seen = set()
for _, text in script_events:
    if 'Fall' in text: phases_seen.add('FREE_FALL')
    if 'Pull' in text: phases_seen.add('PULL_UP')
    if 'Motor' in text: phases_seen.add('MOTOR_START')
    if 'AUTO' in text: phases_seen.add('CRUISE')
print(f"  Phases detected: {phases_seen}")

m.arducopter_disarm()
m.close()
print("\nDone!")
