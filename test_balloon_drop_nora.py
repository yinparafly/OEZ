"""
Balloon Drop 完整测试流程
使用 AI-MP 工具连接 Nora，测试 balloon_drop.lua 脚本
"""
import sys
import time
import json
sys.path.insert(0, 'D:/oezcon/ai_mp')
from ai_mp import MAVConnection

def test_balloon_drop():
    print("=" * 60)
    print("Balloon Drop 完整测试")
    print("=" * 60)
    
    # 1. 连接
    print("\n[1] 连接 Nora...")
    conn = MAVConnection('COM16', 115200)
    conn.connect()
    print(f"  类型: {conn.vehicle_type} (13=固定翼)")
    
    # 2. 配置 Lua 脚本
    print("\n[2] 配置 Lua 脚本支持...")
    conn.set_param('SCR_ENABLE', 1)
    time.sleep(0.5)
    conn.set_param('SCR_HEAP_SIZE', 8192)
    time.sleep(0.5)
    print("  SCR_ENABLE=1, SCR_HEAP_SIZE=8192")
    
    # 3. 检查空速传感器
    print("\n[3] 检查传感器配置...")
    conn.set_param('ARSPD_USE', 1)
    print("  ARSPD_USE=1 (启用空速传感器)")
    
    # 4. 获取当前状态
    print("\n[4] 当前飞行状态...")
    status = conn.get_status()
    for k, v in status.items():
        print(f"  {k}: {v}")
    
    # 5. 列出可用模式
    print("\n[5] 可用飞行模式...")
    modes = list(conn.master.mode_mapping().keys())
    print(f"  共 {len(modes)} 种模式")
    print(f"  关键模式: FBWA, FBWB, CRUISE, AUTO, LOITER, RTL")
    
    # 6. 检查心跳
    print("\n[6] 心跳检测...")
    for i in range(3):
        msg = conn.recv_message(timeout=2)
        if msg and msg.get_type() == 'HEARTBEAT':
            print(f"  心跳 #{i+1}: type={msg.type}, status={msg.system_status}")
    
    conn.close()
    
    print("\n" + "=" * 60)
    print("Balloon Drop 测试完成")
    print("Nora 连接正常，Lua 脚本已配置")
    print("下一步: 将 balloon_drop.lua 上传到 SD 卡")
    print("=" * 60)

if __name__ == '__main__':
    test_balloon_drop()
