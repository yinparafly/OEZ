"""
UDP 转发器: COM15 (Nora) → UDP 14550 (MP)
让 Mission Planner 可以看到 Nora 的实时数据
"""
import sys, time, socket, threading
sys.path.insert(0, 'D:/oezcon/ai_mavproxy')
from core.connection import Connection

UDP_PORT = 14550
NORA_PORT = 'COM15'

def main():
    # 创建 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', UDP_PORT))
    print('UDP Forward: 127.0.0.1:%d' % UDP_PORT)
    
    # 连接 Nora
    conn = Connection(NORA_PORT, 115200)
    if not conn.connect():
        print('Failed to connect to Nora')
        return
    
    print('Nora connected on %s' % NORA_PORT)
    print('Connect MP to UDP %d' % UDP_PORT)
    print('Ctrl+C to stop')
    
    # 转发数据
    try:
        while True:
            msg = conn.master.recv_match(blocking=True, timeout=0.5)
            if msg:
                buf = msg.get_msgbuf()
                sock.sendto(buf, ('127.0.0.1', UDP_PORT))
    except KeyboardInterrupt:
        pass
    
    conn.disconnect()
    sock.close()

if __name__ == '__main__':
    main()
