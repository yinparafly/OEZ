"""连续采样 CNT/RPM，观察测速状态（诊断用，不转电机）"""
import sys
import time
import serial
import serial.tools.list_ports

BAUD = 921600
ser = None


def send(cmd, wait=0.2):
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())
    time.sleep(wait)
    return ser.read(ser.in_waiting or 512).decode("utf-8", "replace")


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
        print("--- 初始 DBG ---"); print(send("DBG"))
        for i in range(8):
            print(send("CNT"))
            r = send("RPM")
            print(f"  -> {r.strip()}")
            time.sleep(0.8)
    finally:
        ser.close()
    print("--- 完成 ---")


if __name__ == "__main__":
    main()