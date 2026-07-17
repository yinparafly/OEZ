#!/usr/bin/env python3
"""
Balloon Drop SITL 集成测试
连接 SITL，启用 Lua 脚本，验证阶段切换
"""
import time
import sys
from pymavlink import mavutil

def main():
    print("=" * 60)
    print("Balloon Drop SITL Integration Test")
    print("=" * 60)

    # Connect to SITL
    print("\n[1] Connecting to SITL...")
    master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
    master.wait_heartbeat()
    print(f"  Connected! System={master.target_system}, Component={master.target_component}")

    # Check heartbeat
    print(f"  Type: {master.mav_type}")

    # Request data streams
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # Wait for first HEARTBEAT
    print("\n[2] Waiting for stable connection...")
    time.sleep(2)

    # Check vehicle mode
    master.wait_heartbeat()
    print(f"  Current mode: {master.flightmode}")

    # Enable scripting parameters
    print("\n[3] Setting parameters for Lua script...")
    params = {
        'SCR_ENABLE': 1,
        'SCR_HEAP_SIZE': 8192,
        'SCR_LOOP_RATE': 10,
    }
    for name, value in params.items():
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode('ascii'), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.5)
        print(f"  {name} = {value}")

    # Wait for params to take effect
    time.sleep(3)

    # Set FBWA mode
    print("\n[4] Setting FBWA mode...")
    master.set_mode_fbwa()
    time.sleep(1)
    print(f"  Mode: {master.flightmode}")

    # Arm
    print("\n[5] Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("  Armed!")

    # Wait a moment for Lua script to initialize
    time.sleep(2)

    # Monitor for MAVLink messages (especially script output)
    print("\n[6] Monitoring for phase transitions...")
    print("  (Press Ctrl+C to stop)")
    print()

    start_time = time.time()
    messages_received = {}
    script_messages = []
    current_mode = "UNKNOWN"

    try:
        while time.time() - start_time < 30:  # 30 seconds timeout
            msg = master.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue

            msg_type = msg.get_type()
            messages_received[msg_type] = messages_received.get(msg_type, 0) + 1

            # Look for STATUSTEXT (Lua script output)
            if msg_type == 'STATUSTEXT':
                text = msg.text
                print(f"  [{time.time() - start_time:.1f}s] STATUSTEXT: {text}")
                if 'BalloonDrop' in text:
                    script_messages.append(text)

            # Look for heartbeat mode changes
            if msg_type == 'HEARTBEAT':
                # ArduPilot custom_mode for plane
                custom_mode = msg.custom_mode
                mode_names = {0:'STABILIZE', 2:'RTL', 4:'AUTO', 5:'LOITER', 9:'FBWA', 10:'FBWB', 14:'CIRCLE', 15:'CIRCLE', 16:'TRIM', 26:'MANUAL'}
                mode_name = mode_names.get(custom_mode, f'MODE({custom_mode})')
                if mode_name != current_mode:
                    current_mode = mode_name
                    print(f"  [{time.time() - start_time:.1f}s] Mode: {mode_name}")

            # Look for airspeed
            if msg_type == 'VFR_HUD':
                print(f"  [{time.time() - start_time:.1f}s] HUD: alt={msg.alt:.1f}m, airspeed={msg.airspeed:.1f}m/s")

    except KeyboardInterrupt:
        print("\n  Test interrupted by user")

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    print(f"  Duration: {time.time() - start_time:.1f}s")
    print(f"  Message types received: {len(messages_received)}")
    for msg_type, count in sorted(messages_received.items(), key=lambda x: -x[1])[:10]:
        print(f"    {msg_type}: {count}")

    print(f"\n  BalloonDrop script messages: {len(script_messages)}")
    for m in script_messages:
        print(f"    {m}")

    if script_messages:
        print("\n  ✅ Lua script is running and outputting messages!")
    else:
        print("\n  ⚠️  No BalloonDrop messages detected. Script may not be loaded.")
        print("  Check: SCR_ENABLE=1, script file in APM/scripts/ directory")

    # Disarm
    print("\n[7] Disarming...")
    master.arducopter_disarm()
    time.sleep(2)
    print("  Disarmed")

    master.close()
    print("\n  Test complete!")

if __name__ == '__main__':
    main()
