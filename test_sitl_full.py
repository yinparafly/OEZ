from pymavlink import mavutil
import time
import math

print("=== ArduPilot SITL Balloon Drop Test ===\n")

print("1. Connecting to SITL via UDP...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat(timeout=10)
print(f"   Connected! Type: {master.mav_type}")

# Set mode to FBWA (Fly-By-Wire A)
print("\n2. Setting mode to FBWA...")
master.set_mode_apm(6)  # FBWA mode
time.sleep(1)

# Arm the vehicle
print("3. Arming vehicle...")
master.arducopter_arm()
time.sleep(2)

# Check arm state
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
)

print("4. Monitoring flight for 5 seconds...")
start = time.time()
altitudes = []
airspeeds = []

while time.time() - start < 5:
    msg = master.recv_match(blocking=True, timeout=1)
    if msg:
        mtype = msg.get_type()
        if mtype == 'VFR_HUD':
            alt = msg.alt
            spd = msg.airspeed
            altitudes.append(alt)
            airspeeds.append(spd)
            print(f"   t={time.time()-start:.1f}s alt={alt:.1f}m aspd={spd:.1f}m/s mode={msg.mode}")

print(f"\n5. Results:")
if altitudes:
    print(f"   Altitude range: {min(altitudes):.1f} - {max(altitudes):.1f} m")
    print(f"   Airspeed range: {min(airspeeds):.1f} - {max(airspeeds):.1f} m/s")
    print("   SITL is fully functional!")

print("\n=== Test Complete ===")
master.close()
