"""Test FTP and param read from Nora"""
from pymavlink import mavutil
import time, struct

master = mavutil.mavlink_connection('COM16', baud=115200, source_system=255)
master.wait_heartbeat(timeout=10)
print(f'Connected: sys={master.target_system} type={master.mav_type}')

# FTP payload is 251 bytes
FTP_SIZE = 251

def ftp_send(session, opcode, size=0, offset=0, data=b''):
    """Build and send FTP packet"""
    header = struct.pack('<BII', opcode, size, offset)
    payload_bytes = header + data
    # Pad to 251 bytes
    payload_bytes = payload_bytes.ljust(FTP_SIZE, b'\x00')
    payload = list(payload_bytes[:FTP_SIZE])
    master.mav.file_transfer_protocol_send(0, 1, 1, payload)

def ftp_recv(timeout=3):
    """Receive FTP response"""
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='FILE_TRANSFER_PROTOCOL', blocking=True, timeout=1)
        if msg:
            return msg
    return None

# === Test 1: FTP List / ===
print("\n=== FTP List / ===")
ftp_send(1, 10, data=b'/\x00')
msg = ftp_recv()
if msg:
    print(f'  opcode={msg.opcode} size={msg.size} offset={msg.offset}')
    if msg.size > 0:
        data = bytes(msg.payload[:msg.size])
        parts = data.split(b'\x00')
        for p in parts:
            p = p.strip()
            if p:
                try:
                    print(f'    -> {p.decode("ascii")}')
                except:
                    pass
else:
    print('  No FTP response')

# === Test 2: FTP List /APM/ ===
print("\n=== FTP List /APM/ ===")
ftp_send(1, 10, data=b'/APM/\x00')
msg = ftp_recv()
if msg:
    print(f'  opcode={msg.opcode} size={msg.size}')
    if msg.size > 0:
        data = bytes(msg.payload[:msg.size])
        parts = data.split(b'\x00')
        for p in parts:
            p = p.strip()
            if p:
                try:
                    print(f'    -> {p.decode("ascii")}')
                except:
                    pass
else:
    print('  No FTP response')

# === Test 3: FTP List /APM/scripts/ ===
print("\n=== FTP List /APM/scripts/ ===")
ftp_send(1, 10, data=b'/APM/scripts/\x00')
msg = ftp_recv()
if msg:
    print(f'  opcode={msg.opcode} size={msg.size}')
    if msg.size > 0:
        data = bytes(msg.payload[:msg.size])
        parts = data.split(b'\x00')
        for p in parts:
            p = p.strip()
            if p:
                try:
                    print(f'    -> {p.decode("ascii")}')
                except:
                    pass
else:
    print('  No FTP response')

master.close()
print("\nDone.")
