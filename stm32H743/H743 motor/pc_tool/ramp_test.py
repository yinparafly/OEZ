"""
油门爬坡扫描（找电机启动点）
规则：遵守 工作要求.md —— 测到转动立即停机，全程 ≤5s 运转，绝不长时间空转。
流程: PWM 20 起步，每步 +10（≈+0.01ms），等 2.5s 斜坡爬到位后查 RPM；
      检测到 rpm!=0 → 记录启动油门 → PWM 0 停机退出。

用法:  python ramp_test.py [起始‰] [步进‰] [每步等待秒]
"""
import sys
import time
import serial
import serial.tools.list_ports

BAUD = 921600
START = int(sys.argv[1]) if len(sys.argv) > 1 else 20
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 10
WAIT = float(sys.argv[3]) if len(sys.argv) > 3 else 2.5
MAX_PWM = 500

ser = None


def send(cmd, wait=0.3):
    if not ser:
        return ""
    try:
        ser.reset_input_buffer()
        ser.write((cmd + "\r\n").encode())
        time.sleep(wait)
        return ser.read(ser.in_waiting or 256).decode("utf-8", "replace")
    except Exception as e:
        return f"[err {e}]"


def get_rpm():
    r = send("RPM", 0.2)
    for line in r.splitlines():
        if "rpm = " in line:
            s = line.split("rpm = ")[1]
            v = int(s.split(" rpm")[0].strip())
            return v, line.strip()
    return None, r.strip()


def find_port():
    for cfg in [p.device for p in serial.tools.list_ports.comports()]:
        try:
            s = serial.Serial(cfg, BAUD, timeout=0.3)
            s.reset_input_buffer()
            s.write(b"ID\r\n")
            time.sleep(0.4)
            data = s.read(512).decode("utf-8", "replace")
            s.close()
            if "ABI-MONITOR" in data:
                return cfg
        except Exception:
            continue
    return None


def main():
    global ser
    port = find_port()
    if not port:
        print("[失败] 未找到板子串口")
        sys.exit(1)
    print(f"[OK] {port}")
    ser = serial.Serial(port, BAUD, timeout=0.3)

    try:
        print(f"--- PWM 0 复位 ---"); print(send("PWM 0", 0.5))

        pwm = START
        while pwm <= MAX_PWM:
            r = send(f"PWM {pwm}", 0.3)
            print(f"--- PWM {pwm} ({1.02 + pwm*0.98/1000:.3f}ms) 等待 {WAIT}s ---")
            time.sleep(WAIT)
            rpm, line = get_rpm()
            print(f"    {line}")
            if rpm is not None and rpm != 0:
                print(f"\n>>> 电机在 PWM {pwm} 启动，rpm = {rpm}")
                break
            pwm += STEP
        else:
            print(f"\n>>> 到 PWM {MAX_PWM} 仍未启动")
    finally:
        print("--- PWM 0 停机 ---")
        if ser:
            try:
                ser.reset_input_buffer()
                ser.write(b"PWM 0\r\n")
                time.sleep(0.5)
                print(ser.read(256).decode("utf-8", "replace").strip())
            except Exception as e:
                print(f"[停机命令失败] {e}")
        ser.close()
    print("--- 完成，已停机 ---")


if __name__ == "__main__":
    main()