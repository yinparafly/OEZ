"""
AI-MP 固件烧录工具
通过 MAVLink 将 ArduPilot 固件烧录到飞控

ArduPilot 固件烧录协议:
1. 发送 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (param1=3) 进入 bootloader
2. 通过 MAVLink FTP 上传固件
3. 飞控自动重启

或者使用更简单的方式:
1. 发送 sysid 要求进入 bootloader
2. 用 MAVLink MAV_CMD_DO_UPGRADE_FIRMWARE
"""

import sys, os, time, struct, json
sys.path.insert(0, 'D:/oezcon/ai_mp')
from pymavlink import mavutil

def flash_firmware(port, firmware_path, baud=115200):
    """烧录固件到飞控"""
    print("=" * 60)
    print("  ArduPilot 固件烧录")
    print("=" * 60)
    
    # 读取固件
    if not os.path.exists(firmware_path):
        print("ERROR: 固件文件不存在: %s" % firmware_path)
        return False
    
    with open(firmware_path, 'rb') as f:
        fw_data = f.read()
    
    print("固件: %s" % firmware_path)
    print("大小: %d bytes (%.1f KB)" % (len(fw_data), len(fw_data) / 1024))
    
    # 连接
    print("\n连接飞控...")
    master = mavutil.mavlink_connection(port, baud=baud, source_system=255)
    master.wait_heartbeat(timeout=10)
    print("  Connected: sys=%d type=%d" % (master.target_system, master.mav_type))
    
    # 方法1: 使用 MAV_CMD_DO_UPGRADE_FIRMWARE
    print("\n尝试 MAV_CMD_DO_UPGRADE_FIRMWARE...")
    
    # 检查是否支持
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 3, 0, 0, 0, 0, 0, 0  # param1=3: enter bootloader
    )
    
    # 等待 ACK
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack:
        print("  ACK: cmd=%d result=%d" % (ack.command, ack.result))
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("  已进入 bootloader 模式")
    
    # 等待 bootloader 响应
    print("等待 bootloader...")
    time.sleep(3)
    
    # 尝试通过 FTP 上传固件
    print("开始上传固件...")
    
    SESSION_ID = 1
    CHUNK_SIZE = 236
    offset = 0
    total = len(fw_data)
    
    while offset < total:
        chunk = fw_data[offset:offset + CHUNK_SIZE]
        
        # 构建 FTP payload: opcode(1) + size(4) + offset(4) + data = 251 bytes total
        header = struct.pack('<BII', 5, len(chunk), offset)
        payload_bytes = header + chunk
        # 确保恰好 251 字节
        payload_bytes = payload_bytes.ljust(251, b'\x00')
        payload = list(payload_bytes[:251])
        
        master.mav.file_transfer_protocol_send(
            0, master.target_system, master.target_component,
            payload
        )
        
        offset += CHUNK_SIZE
        time.sleep(0.02)
        
        if (offset // CHUNK_SIZE) % 50 == 0:
            pct = offset * 100 // total
            print("  上传进度: %d%% (%d/%d bytes)" % (pct, offset, total))
    
    # 关闭文件
    close_payload = struct.pack('<BII', 6, 0, 0)
    close_bytes = close_payload.ljust(251, b'\x00')
    master.mav.file_transfer_protocol_send(
        0, master.target_system, master.target_component,
        list(close_bytes[:251])
    )
    
    # 发送重启命令
    print("\n发送重启命令...")
    time.sleep(2)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    
    print("固件烧录完成！等待飞控重启...")
    master.close()
    return True


if __name__ == '__main__':
    port = 'COM16'
    firmware = 'D:/pix/arduplane_CUAV_Nora.apj'
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        firmware = sys.argv[2]
    
    flash_firmware(port, firmware)
