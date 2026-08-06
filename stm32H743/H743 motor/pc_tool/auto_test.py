"""
电机编码器自测脚本（遵守 工作要求.md 规则）
- 探测 COM 口（发 ID 找板子）
- PWM 0 复位 → PWM 300 运行 ≤5 秒采样 → PWM 0 停机
- 任何路径 finally 必发 PWM 0

用法:  python auto_test.py [PWM值,默认300] [运行秒数,默认4]
"""
import sys
import time
import serial
import serial.tools.list_ports

BAUD = 921600
PWM = int(sys.argv[1]) if len(sys.argv) > 1 else 300
DUR = int(sys.argv[2]) if len(sys.argv) > 2 else 4
CANDIDATES = ["COM7", "COM8", "COM21"]

ser = None


def send(cmd, wait=0.15):
    if not ser:
        return ""
    try:
        ser.reset_input_buffer()
        ser.write((cmd + "\r\n").encode())
        time.sleep(wait)
        return ser.read(ser.in_waiting or 256).decode("utf-8", "replace")
    except Exception as e:
        return f"[err {e}]"


def find_port():
    for cfg in CANDIDATES + [p.device for p in serial.tools.list_ports.comports()]:
        if cfg in CANDIDATES and cfg not in [p.device for p in serial.tools.list_ports.comports()]:
            if cfg in CANDIDATES[:1]:  # 只提示不报错
                pass
        try:
            s = serial.Serial(cfg, BAUD, timeout=0.3)
            s.reset_input_buffer()
            s.write(b"ID\r\n")
            time.sleep(0.5)
            data = s.read(512).decode("utf-8", "replace")
            s.close()
            if "ABI-MONITOR" in data:
                return cfg, data
        except Exception:
            continue
    return None, ""


def main():
    global ser
    port, banner = find_port()
    if not port:
        print("[失败] 未找到板子串口（确认串口线已接且未被占用）")
        sys.exit(1)
    print(f"[OK] {port}: {banner.strip().splitlines()[-1]}")

    ser = serial.Serial(port, BAUD, timeout=0.3)
    try:
        print(f"--- PWM 0 复位 ---"); print(send("PWM 0"))
        print(f"--- 基线 CNT ---");   print(send("CNT"))
        print(f"--- PWM {PWM} 运行 {DUR}s ---")
        print(send(f"PWM {PWM}", 0.3))
        t0 = time.time()
        while time.time() - t0 < DUR:
            r = send("RPM", 0.2)
            s = r.strip()
            print(f"[{time.time()-t0:4.1f}s] rpm/cnt/idx: {s}")
            time.sleep(0.5)
    finally:
        print(f"--- PWM 0 停机 ---")
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