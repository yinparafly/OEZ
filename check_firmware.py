import serial
import struct
import time

s = serial.Serial('COM16', 115200, timeout=3)
print("Checking firmware type on COM16...")

start = time.time()
while time.time() - start < 3:
    data = s.read(300)
    if len(data) == 0:
        continue
    
    i = 0
    while i < len(data) - 10:
        if data[i] == 0xfd:  # MAVLink v2
            msg_id = data[i+5] | (data[i+6] << 8) | (data[i+7] << 16)
            sysid = data[i+3]
            compid = data[i+4]
            
            if msg_id == 0:  # HEARTBEAT
                # Parse heartbeat
                if i + 14 < len(data):
                    type_val = data[i+9]  # vehicle type
                    autopilot = data[i+10]  # autopilot type
                    
                    vehicle_types = {0:'Generic',1:'Fixed Wing',2:'Quad',12:'Helicopter',22:'Ground Rover',26:'Airship'}
                    autopilot_types = {0:'Generic',3:'ArduPilot',12:'PX4'}
                    
                    vtype = vehicle_types.get(type_val, f'Unknown({type_val})')
                    ap = autopilot_types.get(autopilot, f'Unknown({autopilot})')
                    
                    print(f"  HEARTBEAT: vehicle={vtype}({type_val}), autopilot={ap}({autopilot}), sysid={sysid}")
                    
                    if autopilot == 12:
                        print(f"\n  *** PX4 FIRMWARE DETECTED ***")
                        print(f"  Mission Planner may not work well with PX4 over serial.")
                        print(f"  Consider flashing ArduPilot firmware.")
                    elif autopilot == 3:
                        print(f"\n  ArduPilot firmware detected - should work with Mission Planner.")

s.close()
