#!/usr/bin/env python3
"""
Balloon Drop SITL 仿真 v2
直接在SITL地面启动，手动爬升，然后释放
"""
import time
from pymavlink import mavutil

print("=" * 60)
print("Balloon Drop SITL Simulation v2")
print("=" * 60)

m = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
m.wait_heartbeat()
print(f"[1] Connected sys={m.target_system}")

# Params
for name, val in [(b'SCR_HEAP_SIZE', 200000.0), (b'PTCH_LIM_MIN_DEG', -90.0), (b'PTCH_LIM_MAX_DEG', 30.0)]:
    m.mav.param_set_send(m.target_system, m.target_component, name, val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
time.sleep(1)

# Reboot
m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(6)
m.close()

m = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
m.wait_heartbeat()
print(f"[2] Reconnected")
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
time.sleep(3)

# Arm
print("[3] Arm...")
m.arducopter_arm()
time.sleep(2)

# Use FBWA to climb - command pitch up and throttle up
print("[4] Climb in FBWA (throttle=80%, pitch=10deg up)...")
m.set_mode_fbwa()
time.sleep(1)

# Command climb via RC override: ch3=throttle 80%, ch2=pitch up
# RC channels: 1=aileron, 2=elevator, 3=throttle, 4=rudder
# PWM: 1000=min, 1500=mid, 2000=max
for i in range(600):  # 60 seconds at 10Hz
    # Override: aileron=1500 (level), elevator=1300 (nose up), throttle=1600 (80%), rudder=1500
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        0, 0, 1500, 1300, 1600, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    if i % 50 == 0:
        print(f"    [{i/10:.0f}s] climbing...")
    time.sleep(0.1)

# Check altitude
print("[5] Checking altitude...")
m.mav.rc_channels_override_send(m.target_system, m.target_component, 0,0,1500,1500,1500,1500,0,0,0,0,0,0,0,0,0,0,0,0)
time.sleep(2)

# Get current alt
alt_now = 0
for _ in range(10):
    msg = m.recv_match(blocking=True, timeout=2)
    if msg and msg.get_type() == 'VFR_HUD':
        alt_now = msg.alt
        print(f"    Current alt: {alt_now:.0f}m aspd: {msg.airspeed:.1f}m/s")
        break

print(f"\n[6] ===== RELEASE! =====")
print(f"    Altitude: {alt_now:.0f}m")
print("    Switch FBWA, nose down -80, throttle 0")

# Release: switch to FBWA, nose down, throttle off
# The Lua script should take over
m.set_mode_fbwa()
time.sleep(0.5)

# Monitor free fall
print("[7] Monitoring free fall...")
print(f"    Trigger at 12 m/s\n")

start = time.time()
phases = []

try:
    while time.time() - start < 45:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        t = msg.get_type()
        elapsed = time.time() - start
        
        if t == 'STATUSTEXT':
            text = msg.text
            if ('BD' in text) and ('Running' not in text) and ('Time:' not in text):
                if not phases or phases[-1][1] != text:
                    print(f"  [{elapsed:.1f}s] >>> {text}")
                    phases.append((elapsed, text))
        
        if t == 'VFR_HUD' and int(elapsed * 2) % 6 == 0:
            print(f"  [{elapsed:.1f}s] aspd={msg.airspeed:.1f}m/s alt={msg.alt:.0f}m")

except KeyboardInterrupt:
    pass

# Summary
print("\n" + "=" * 60)
print("Summary")
print("=" * 60)
print(f"  Duration: {time.time()-start:.1f}s")
print(f"  Phase transitions: {len(phases)}")
for t, text in phases:
    print(f"    [{t:.1f}s] {text}")

if not phases:
    print("  No BD transitions - script may need FBWA attitude control")

m.arducopter_disarm()
m.close()
