"""
HIL 仿真飞行测试脚本
测试 Balloon Drop 全流程：释放 → 自由下落 → 拉起 → 电机启动 → 巡航

使用方式:
  1. SITL 模式: python hil_test.py --mode sitl
  2. 真机模式: python hil_test.py --mode real --port COM15
  3. 带舵机测试: python hil_test.py --mode real --port COM15 --servos
"""

import sys, os, time, argparse
sys.path.insert(0, 'D:/oezcon/ai_mavproxy')
from pymavlink import mavutil


class HILTest:
    """HIL 仿真飞行测试"""
    
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.master = None
    
    def connect(self):
        """连接飞控"""
        print("Connecting to %s..." % self.port)
        self.master = mavutil.mavlink_connection(self.port, baud=self.baud, source_system=255)
        self.master.wait_heartbeat(timeout=10)
        print("Connected: sys=%d type=%d" % (self.master.target_system, self.master.mav_type))
        
        # 请求数据流
        for sid in range(0, 7):
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                sid, 4, 1
            )
        return True
    
    def get_status(self):
        """获取状态"""
        status = {}
        start = time.time()
        while time.time() - start < 2:
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if not msg:
                continue
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT':
                status['mode'] = msg.custom_mode
                status['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            elif mtype == 'VFR_HUD':
                status['alt'] = msg.alt
                status['aspd'] = msg.airspeed
                status['gs'] = msg.groundspeed
                status['heading'] = msg.heading
                status['throttle'] = msg.throttle
            elif mtype == 'ATTITUDE':
                import math
                status['roll'] = math.degrees(msg.roll)
                status['pitch'] = math.degrees(msg.pitch)
                status['yaw'] = math.degrees(msg.yaw)
            elif mtype == 'SYS_STATUS':
                status['voltage'] = msg.voltage_battery / 1000
            elif mtype == 'STATUSTEXT':
                status['last_msg'] = msg.text
        return status
    
    def set_rc(self, channel, value):
        """设置 RC 通道值"""
        rc_channels = [0] * 18
        rc_channels[channel - 1] = value
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channels
        )
    
    def set_mode(self, mode_id):
        """设置飞行模式"""
        self.master.set_mode(mode_id)
    
    def arm(self):
        """解锁"""
        self.master.arducopter_arm()
    
    def disarm(self):
        """上锁"""
        self.master.arducopter_disarm()
    
    def send_lua_message(self, text):
        """发送 STATUSTEXT 消息（模拟 Lua 脚本输出）"""
        # 通过 RC 通道触发 Lua 脚本
        pass


def test_sitl(port):
    """SITL 测试模式"""
    print("=" * 60)
    print("  HIL Test: SITL Mode")
    print("=" * 60)
    
    hil = HILTest(port)
    if not hil.connect():
        return
    
    # 等待遥测数据
    print("\nWaiting for telemetry...")
    time.sleep(3)
    
    # 获取状态
    status = hil.get_status()
    print("\nInitial status:")
    for k, v in sorted(status.items()):
        print("  %s: %s" % (k, v))
    
    # 模拟释放触发（通过 RC 通道）
    print("\nSimulating release trigger (RC8 low)...")
    hil.set_rc(8, 900)  # 低值触发
    time.sleep(2)
    
    # 检查 Lua 脚本消息
    status = hil.get_status()
    if 'last_msg' in status:
        print("Lua message: %s" % status['last_msg'])
    
    # 等待几秒观察控制响应
    print("\nMonitoring for 5 seconds...")
    for i in range(5):
        status = hil.get_status()
        print("  [%ds] Alt=%.1f ASpd=%.1f Roll=%.1f Pitch=%.1f" % (
            i+1, status.get('alt', 0), status.get('aspd', 0),
            status.get('roll', 0), status.get('pitch', 0)))
        time.sleep(1)
    
    hil.master.close()
    print("\nSITL test completed")


def test_real(port):
    """真机测试模式"""
    print("=" * 60)
    print("  HIL Test: Real Hardware Mode")
    print("=" * 60)
    
    hil = HILTest(port)
    if not hil.connect():
        return
    
    # 获取状态
    status = hil.get_status()
    print("\nNora status:")
    for k, v in sorted(status.items()):
        print("  %s: %s" % (k, v))
    
    # 检查 Lua 脚本消息
    print("\nChecking for Lua script output (10s)...")
    start = time.time()
    while time.time() - start < 10:
        msg = hil.master.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if msg:
            print("  STATUSTEXT: %s" % msg.text)
    
    hil.master.close()
    print("\nReal hardware test completed")


def main():
    parser = argparse.ArgumentParser(description="HIL Simulation Test")
    parser.add_argument('--mode', choices=['sitl', 'real'], default='real')
    parser.add_argument('--port', default='COM15')
    args = parser.parse_args()
    
    if args.mode == 'sitl':
        test_sitl(args.port)
    else:
        test_real(args.port)


if __name__ == '__main__':
    main()
