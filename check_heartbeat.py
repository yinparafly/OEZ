from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('COM16', baud=115200)
master.wait_heartbeat(timeout=10)

print(f"System ID: {master.target_system}")
print(f"Component ID: {master.target_component}")
print(f"Vehicle type: {master.mav_type}")
print(f"Autopilot: {master.autopilot}")
print(f"Base mode: {master.base_mode}")
print(f"Custom mode: {master.custom_mode}")

# Try to get parameters
print("\nRequesting parameters...")
master.param_fetch_all()
time.sleep(3)

# Check some key params
params_to_check = ['FRAME_CLASS', 'SYSID_THISMAV', 'SERIAL0_BAUD', 'SERIAL1_BAUD', 
                   'SERIAL2_BAUD', 'ARSPD_USE', 'AHRS_TYPE']
for p in params_to_check:
    val = master.param_fetch_one(p)
    if val:
        print(f"  {p} = {val}")

master.close()
