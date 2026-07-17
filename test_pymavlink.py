from pymavlink import mavutil
import time

print("Testing connection to COM16 @ 115200...")
try:
    master = mavutil.mavlink_connection('COM16', baud=115200)
    print("Waiting for heartbeat...")
    master.wait_heartbeat(timeout=10)
    print(f"Connected! System: {master.target_system}, Component: {master.target_component}")
    print(f"Vehicle type: {master.mav_type}")
    master.close()
except Exception as e:
    print(f"Connection failed: {e}")

print("\nTesting connection to COM15 @ 115200...")
try:
    master = mavutil.mavlink_connection('COM15', baud=115200)
    print("Waiting for heartbeat...")
    master.wait_heartbeat(timeout=10)
    print(f"Connected! System: {master.target_system}, Component: {master.target_component}")
    master.close()
except Exception as e:
    print(f"Connection failed: {e}")
