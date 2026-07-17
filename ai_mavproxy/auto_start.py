"""
AI-MP 自动化: 连接 Nora + UDP 转发给 MP
"""
import sys, time, socket
sys.path.insert(0, 'D:/oezcon/ai_mavproxy')
from pymavlink import mavutil

NORA_PORT = 'COM15'
BAUD = 115200
MP_UDP_PORT = 14550

def main():
    print('=== AI-MP Auto Start ===')
    
    # 连接 Nora
    print('\n[1] Connecting to Nora...')
    master = mavutil.mavlink_connection(NORA_PORT, baud=BAUD, source_system=255)
    master.wait_heartbeat(timeout=10)
    print('[OK] Nora: sys=%d type=%d mode=%s' % (
        master.target_system, master.mav_type, master.flightmode))
    
    # 创建 UDP socket 用于转发
    print('\n[2] Setting up UDP forward...')
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mp_addr = ('127.0.0.1', MP_UDP_PORT)
    print('[OK] UDP target: %s:%d' % mp_addr)
    
    # 请求数据流
    print('\n[3] Requesting data streams...')
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        0, 4, 1
    )
    
    print('\n[4] System running...')
    print('    Nora: COM15 connected')
    print('    UDP: forwarding to %d' % MP_UDP_PORT)
    print('')
    print('    In MP: Select UDP, enter %d, click Connect' % MP_UDP_PORT)
    print('    Ctrl+C to stop')
    
    try:
        while True:
            msg = master.recv_match(blocking=True, timeout=0.5)
            if msg:
                buf = msg.get_msgbuf()
                udp_sock.sendto(buf, mp_addr)
    except KeyboardInterrupt:
        pass
    
    udp_sock.close()
    master.close()
    print('\nDone')

if __name__ == '__main__':
    main()
