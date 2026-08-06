"""
Task 4 SD 自动备份验证
流程: ID -> SD INIT(探测卡) -> ARM -> PWM -> 等 DONE -> 手动 SD SAVE 确认写卡/报错
     -> LS 看文件 -> PWM 0 停机
未插卡时验证无量路径返回码；插卡后重跑则验证写文件+不覆盖。"""
import sys
import time
import serial
import serial.tools.list_ports

BAUD = 921600
PWM = int(sys.argv[1]) if len(sys.argv) > 1 else 600


def open_ser():
    for p in serial.tools.list_ports.comports():
        if "CP210x" in (p.description or "") or "USB" in (p.description or ""):
            try:
                return serial.Serial(p.device, BAUD, timeout=1.0)
            except Exception:
                continue
    raise SystemExit("no USB-serial found")


def send(ser, cmd, wait=0.35):
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())
    time.sleep(wait)
    return ser.read(ser.in_waiting or 512).decode("utf-8", "replace")


def snap_state(ser):
    r = send(ser, "SNAP", 0.25)
    for line in r.splitlines():
        if "snap:" in line:
            s = line.split("snap: ")[1]
            st = int(s.split("state=")[1].split()[0])
            ct = int(s.split("count=")[1].split()[0])
            rd = int(s.split("ready=")[1].split()[0])
            return st, ct, rd
    return None


def main():
    ser = open_ser()
    try:
        time.sleep(0.3)
        send(ser, "ID", 0.3)

        r = send(ser, "SD INIT", 1.0)
        print(f"[SD INIT] {r.splitlines()[-1] if r.splitlines() else ''}")

        send(ser, "PWM 0", 0.3)
        send(ser, "ARM", 0.3)
        send(ser, f"PWM {PWM}", 0.5)
        time.sleep(1.0)
        send(ser, "PWM 0", 0.3)          # 触发只需过阈+一整圈（<1s），转够即停

        st = None
        t0 = time.time()
        while time.time() - t0 < 10:
            st = snap_state(ser)
            if st and st[0] == 3:
                break
            time.sleep(0.3)
        if not st:
            print("[FAIL] no SNAP reply")
            return
        print(f"[SNAP] state={st[0]} count={st[1]} ready={st[2]}")

        r = send(ser, "SD SAVE", 2.0)
        print(f"[SD SAVE ret] {[l.strip() for l in r.splitlines() if l.strip()][-2:]}")

        r = send(ser, "SD LS", 2.0)
        lines = [l.strip() for l in r.splitlines() if l.strip()]
        print(f"[SD LS] {lines[-3:]}")

        r = send(ser, "SNAP", 0.25)
        print(f"[after SAVE auto] {[l.strip() for l in r.splitlines() if 'ready=' in l]}")
    finally:
        send(ser, "PWM 0", 0.3)
        send(ser, "DISARM", 0.2)
        ser.close()


if __name__ == "__main__":
    main()