import serial
import struct
import time

s = serial.Serial('COM16', 115200, timeout=3)
print("Listening on COM16 @ 115200 for 5 seconds...")

start = time.time()
heartbeat_count = 0
total_bytes = 0
msg_types = {}

while time.time() - start < 5:
    data = s.read(300)
    if len(data) == 0:
        continue
    total_bytes += len(data)
    
    # Parse MAVLink v2 messages
    i = 0
    while i < len(data) - 3:
        if data[i] == 0xfd:  # MAVLink v2 start
            if i + 3 < len(data):
                msg_len = data[i+1]
                msg_id = data[i+5] | (data[i+6] << 8) | (data[i+7] << 16) if i+7 < len(data) else 0
                
                if msg_id == 0:  # HEARTBEAT
                    heartbeat_count += 1
                    sysid = data[i+3] if i+3 < len(data) else 0
                    compid = data[i+4] if i+4 < len(data) else 0
                    print(f"  HEARTBEAT: sysid={sysid}, compid={compid}")
                
                if msg_id not in msg_types:
                    msg_types[msg_id] = 0
                msg_types[msg_id] += 1
                
                i += msg_len + 10  # skip to next message
            else:
                i += 1
        else:
            i += 1

s.close()

print(f"\nResults:")
print(f"  Total bytes: {total_bytes}")
print(f"  Heartbeats: {heartbeat_count}")
print(f"  Message types: {msg_types}")
print(f"  Heartbeat rate: {heartbeat_count/5:.1f} Hz")
