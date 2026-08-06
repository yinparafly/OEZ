"""PWM 300 下连续读 CNT/RPM（诊断 dpos 异常）"""
import time
import serial
import serial.tools.list_ports

BAUD = 921600
ser = None


def send(cmd, wait=0.15):
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
    ser = serial.Serial(port, BAUD, timeout=0.3)
    try:
        print("--- PWM 0 ---"); print(send("PWM 0", 0.5))
        print("--- PWM 300 后连续采样 CNT ---")
        print(send("PWM 300", 0.5))
        for i in range(10):
            print(send("CNT"))
            time.sleep(0.5)
    finally:
        if ser:
            ser.reset_input_buffer()
            ser.write(b"PWM 0\r\n")
            time.sleep(0.4)
        ser.close()
    print("--- 完成（已停机） ---")


if __name__ == "__main__":
    main()