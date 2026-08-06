"""
Task 3 自动触发验证：ARM → PWM 驱动电机转 → 自动触发(转速过阈+整圈) → 验证 SNAP 点数
流程: PWM 0 复位 → ARM → PWM 700（稳定 ~5500rpm）→ 轮询 SNAP 直到 state=3
     → 验证点数范围(回溯400+0.8s窗口) → PWM 0 停机
任何路径 finally 必发 PWM 0。
"""
import sys
import time
import serial
import serial.tools.list_ports

BAUD = 921600
PWM = int(sys.argv[1]) if len(sys.argv) > 1 else 700
TIMEOUT = 20.0
ser = None


def send(cmd, wait=0.25):
    if not ser:
        return ""
    try:
        ser.reset_input_buffer()
        ser.write((cmd + "\r\n").encode())
        time.sleep(wait)
        return ser.read(ser.in_waiting or 256).decode("utf-8", "replace")
    except Exception as e:
        return f"[err {e}]"


def get_snap():
    r = send("SNAP", 0.2)
    for line in r.splitlines():
        if "snap:" in line:
            s = line.split("snap: ")[1]
            st = int(s.split("state=")[1].split()[0])
            ct = int(s.split("count=")[1].split()[0])
            rd = int(s.split("ready=")[1].split()[0])
            return st, ct, rd
    return None


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
        print("--- PWM 0 复位 ---"); print(send("PWM 0", 0.5))
        print("--- ARM ---"); print(send("ARM", 0.3))
        print(f"--- PWM {PWM} 驱动电机，等待自动触发 ---")
        print(send(f"PWM {PWM}", 0.3))

        t0 = time.time()
        st, ct, rd = get_snap()
        print(f"[{0:4.1f}s] state={st} count={ct} ready={rd}")
        while time.time() - t0 < TIMEOUT:
            st, ct, rd = get_snap()
            print(f"[{time.time()-t0:4.1f}s] state={st} count={ct} ready={rd}")
            if st == 3 and ct > 0:
                break
            time.sleep(0.5)

        ok = False
        if st == 3 and ct > 0:
            exp_min = 400              # 回溯
            exp_max = 400 + 31000      # 0.8s 窗口上限
            if exp_min <= ct <= exp_max:
                ok = True
                print(f"\n>>> 验证通过: state=3, 点数 {ct} (预期 {exp_min}..{exp_max})")
            else:
                print(f"\n>>> 点数异常: {ct} 超出预期 {exp_min}..{exp_max}")
        else:
            print(f"\n>>> 未触发: state={st} count={ct} (超时 {TIMEOUT}s)")

        return ok
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
    sys.exit(0 if main() else 1)