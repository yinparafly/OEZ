from pymavlink import mavutil
import time

print("Connecting to SITL via UDP 127.0.0.1:14550...")
try:
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    print("Waiting for heartbeat...")
    master.wait_heartbeat(timeout=10)
    print(f"Connected! System: {master.target_system}, Component: {master.target_component}")
    print(f"Vehicle type: {master.mav_type}")
    
    # Request data stream
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )
    
    # Wait for some messages
    print("\nReceiving messages for 3 seconds...")
    start = time.time()
    msg_count = 0
    while time.time() - start < 3:
        msg = master.recv_match(type=['HEARTBEAT', 'ATTITUDE', 'GLOBAL_POSITION_INT'], blocking=True, timeout=1)
        if msg:
            msg_count += 1
            if msg.get_type() == 'HEARTBEAT':
                print(f"  HEARTBEAT: type={msg.type}, autopilot={msg.autopilot}")
            elif msg.get_type() == 'ATTITUDE':
                print(f"  ATTITUDE: roll={math.degrees(msg.roll):.1f}, pitch={math.degrees(msg.pitch):.1f}")
            elif msg.get_type() == 'GLOBAL_POSITION_INT':
                print(f"  POSITION: lat={msg.lat/1e7:.6f}, lon={msg.lon/1e7:.6f}, alt={msg.alt/1000:.1f}m")
    
    print(f"\nTotal messages received: {msg_count}")
    print("SITL UDP connection works!")
    master.close()
    
except Exception as e:
    print(f"Connection failed: {e}")

import math
