"""
TCP 转发器: COM15 (Nora) → TCP 5760 (MP)
让 Mission Planner 可以看到 Nora 的实时数据
"""
import sys, time, socket, threading
sys.path.insert(0, 'D:/oezcon/ai_mavproxy')
from core.connection import Connection

PORT_FORWARD = 5760
PORT_NORA = 'COM15'

def forward_to_client(master, client):
    """转发 Nora 数据到 MP 客户端"""
    while True:
        try:
            msg = master.recv_match(blocking=True, timeout=0.5)
            if msg:
                client.sendall(msg.get_msgbuf())
        except:
            break

def main():
    # 启动 TCP 服务器
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', PORT_FORWARD))
    server.listen(1)
    print('TCP Forward Server: 127.0.0.1:%d' % PORT_FORWARD)
    print('Connect MP to this port to see Nora data')
    print('')
    
    # 连接 Nora
    conn = Connection(PORT_NORA, 115200)
    if not conn.connect():
        print('Failed to connect to Nora')
        return
    
    print('Nora connected on %s' % PORT_NORA)
    print('Waiting for MP connection on TCP %d...' % PORT_FORWARD)
    print('Press Ctrl+C to stop')
    print('')
    
    server.settimeout(1)
    running = True
    
    try:
        while running:
            # 接受 MP 连接
            try:
                client, addr = server.accept()
                print('[%s] MP connected from %s' % (time.strftime('%H:%M:%S'), addr))
                t = threading.Thread(target=forward_to_client, args=(conn.master, client))
                t.daemon = True
                t.start()
            except socket.timeout:
                pass
            except KeyboardInterrupt:
                running = False
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    
    conn.disconnect()
    server.close()
    print('Server stopped')

if __name__ == '__main__':
    main()
