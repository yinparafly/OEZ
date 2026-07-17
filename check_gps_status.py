#!/usr/bin/env python3
"""
检查Nora飞控GPS状态
"""

from pymavlink import mavutil
import time

PORT = 'COM15'
BAUD = 115200

print(f"=== Checking Nora GPS Status on {PORT} ===")

mav = mavutil.mavlink_connection(PORT, baud=BAUD)
msg = mav.wait_heartbeat(timeout=10)
if not msg:
    print("Failed to connect!")
    exit(1)

print("Connected!\n")

# 请求GPS数据流
print("Requesting GPS data stream...")
mav.mav.request_data_stream_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    2,  # 2 Hz
    1   # start
)

# 等待并读取GPS信息
print("\nReading GPS data (5 seconds)...\n")

gps_count = 0
start_time = time.time()

while time.time() - start_time < 5:
    # GPS_RAW_INT - 原始GPS数据
    msg = mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
    if msg:
        gps_count += 1
        fix = msg.fix_type
        fix_text = {
            0: "NO FIX",
            1: "NO FIX (2D)",
            2: "2D FIX",
            3: "3D FIX",
            4: "DGPS",
            5: "RTK Float",
            6: "RTK Fixed"
        }.get(fix, f"Unknown ({fix})")
        
        print(f"Fix Type: {fix_text}")
        print(f"Satellites Visible: {msg.satellites_visible}")
        print(f"Latitude: {msg.lat/1e7:.7f}")
        print(f"Longitude: {msg.lon/1e7:.7f}")
        print(f"Altitude: {msg.alt/1000:.2f} m")
        print(f"HDOP: {msg.eph/100:.2f}" if msg.eph != 65535 else "HDOP: N/A")
        print(f"VDOP: {msg.epv/100:.2f}" if msg.epv != 65535 else "VDOP: N/A")
        print()

# GPS_GLOBAL_ORIGIN - GPS原点
msg = mav.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True, timeout=1)
if msg:
    print(f"GPS Origin: {msg.latitude/1e7:.7f}, {msg.longitude/1e7:.7f}")

# GLOBAL_POSITION_INT - 全局位置
msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
if msg:
    print(f"\nCurrent Position:")
    print(f"  Lat: {msg.lat/1e7:.7f}")
    print(f"  Lon: {msg.lon/1e7:.7f}")
    print(f"  Alt: {msg.alt/1000:.2f} m")

mav.close()

if gps_count == 0:
    print("\n*** No GPS data received! ***")
    print("Check:")
    print("  1. GPS module connected to Nora GPS port")
    print("  2. GPS转发器是否正常工作")
    print("  3. GPS串口配置 (GPS_TYPE, GPS_BAUD)")
else:
    print(f"\nGPS is working! ({gps_count} messages received)")
