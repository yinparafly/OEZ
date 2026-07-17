"""
AI-MP → MP 交接脚本
AI-MP 执行任务后自动把控制权交给 MP
"""
import sys, os, time, subprocess
sys.path.insert(0, 'D:/oezcon/ai_mavproxy')
from core.connection import Connection
from modules.param_manager import ParamManager
from modules.flight_control import FlightControl
from modules.telemetry import Telemetry

NORA_PORT = 'COM15'
MP_EXE = r'D:\pix\missionplanner\MissionPlanner\MissionPlanner.exe'

def run_ai_tasks():
    """执行 AI-MP 任务"""
    print('=== AI-MP Tasks ===')
    
    conn = Connection(NORA_PORT, 115200)
    if not conn.connect():
        return False
    
    params = ParamManager(conn)
    fc = FlightControl(conn)
    tel = Telemetry(conn)
    tel.request_streams()
    
    # 任务 1: 获取状态
    print('\n[Task 1] Getting status...')
    status = tel.get_full_status(timeout=3)
    print('  Mode=%s Alt=%.1f Armed=%s' % (
        status.get('flightmode'), status.get('altitude', 0), status.get('armed')))
    
    # 任务 2: 参数检查
    print('\n[Task 2] Parameter check...')
    for p in ['SCR_ENABLE', 'SCR_HEAP_SIZE', 'ARMING_CHECK']:
        val = params.get(p)
        print('  %s = %s' % (p, val))
    
    # 任务 3: 验证 Lua 脚本状态
    print('\n[Task 3] Checking Lua script...')
    # 检查 STATUSTEXT
    start = time.time()
    while time.time() - start < 3:
        msg = conn.master.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if msg:
            print('  MSG: %s' % msg.text)
    
    # 断开连接
    conn.disconnect()
    print('\n[Done] AI-MP tasks completed, COM15 freed')
    return True

def launch_mp():
    """启动 MP"""
    print('\n=== Launching Mission Planner ===')
    if os.path.exists(MP_EXE):
        subprocess.Popen([MP_EXE])
        print('MP started. Connect to COM15.')
        return True
    else:
        print('MP not found at %s' % MP_EXE)
        return False

def main():
    print('=' * 60)
    print('  AI-MP → MP Handoff')
    print('=' * 60)
    
    # Step 1: 执行 AI-MP 任务
    run_ai_tasks()
    
    # Step 2: 启动 MP
    time.sleep(2)  # 等待端口释放
    launch_mp()
    
    print('\n' + '=' * 60)
    print('  Done!')
    print('  MP should auto-connect to COM15')
    print('  Check the HUD for flight data')
    print('=' * 60)

if __name__ == '__main__':
    main()
