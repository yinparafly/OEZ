"""
Task 5 Flash 芯片保存验证
流程: ID -> FLASH(初始) -> ARM + PWM(短暂) -> 等 DONE -> FLASH(确认已自动保存)
     -> DUMP(收原始帧, 校验 preamble/<IHH/crc32) -> OpenOCD 复位
     -> FLASH(复位后数据仍在, 证明掉电不丢) -> DUMP 再取一次
需: 串口(CP210x), openocd 线程在 PATH/工作目录可复位, pyserial
"""
import sys
import time
import struct
import subprocess
import zlib
import serial
import serial.tools.list_ports

BAUD = 921600
PWM = int(sys.argv[1]) if len(sys.argv) > 1 else 600
OPENOCD = r"D:\openocd\xpack-openocd-0.12.0-7\bin\openocd.exe"


def open_ser():
    for p in serial.tools.list_ports.comports():
        if "CP210x" in (p.description or ""):
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


def flash_info(ser):
    import re
    for _ in range(5):
        r = send(ser, "FLASH", 1.0)
        lines = [l for l in r.splitlines() if "flash:" in l]
        if not lines:
            time.sleep(0.5)
            continue
        kv = {}
        for k, v in re.findall(r"(\w+)=(\d+)", lines[-1]):
            kv[k] = int(v)
        if kv:
            return kv
        time.sleep(0.5)
    return {}


def read_dump(ser):
    """触发 DUMP 并读回完整帧, 返回 dict 或 None"""
    ser.reset_input_buffer()
    ser.write(b"DUMP\r\n")
    data = bytearray()
    got_pre = False
    t0 = time.time()
    while time.time() - t0 < 12:
        chunk = ser.read(ser.in_waiting or 512)
        if not chunk:
            time.sleep(0.05)
            continue
        data += chunk
        if not got_pre:
            idx = data.find(b"\xaa" * 10 + b"\x55")
            if idx >= 0:
                data = data[idx:]
                got_pre = True
        if got_pre and len(data) >= 11 + 8:
            n = struct.unpack_from("<H", data, 11 + 4)[0]
            need = 11 + 8 + n * 16 + 4
            if len(data) >= need:
                return parse_frame(bytes(data[:need]))
    return None


def parse_frame(fr):
    assert fr[0:10] == b"\xaa" * 10 and fr[10] == 0x55, "preamble"
    magic, n, hz = struct.unpack_from("<IHH", fr, 11)
    pts = fr[11 + 8: 11 + 8 + n * 16]
    crc = struct.unpack_from("<I", fr, 11 + 8 + n * 16)[0]
    crc_ok = (zlib.crc32(pts) & 0xFFFFFFFF) == crc
    return {"magic": magic, "n": n, "hz": hz, "pts": len(pts), "crc_ok": crc_ok}


def reset_board():
    """openocd reset run (需要 stlink). 返回 True 成功"""
    try:
        subprocess.run(
            [OPENOCD, "-s", r"D:\openocd\xpack-openocd-0.12.0-7\scripts",
             "-f", "interface/stlink.cfg", "-f", "target/stm32h7x.cfg",
             "-c", "init", "-c", "reset run", "-c", "exit"],
            capture_output=True, timeout=30)
        return True
    except Exception:
        return False


def main():
    ser = open_ser()
    try:
        time.sleep(0.3)
        send(ser, "ID", 0.3)
        f0 = flash_info(ser)
        print(f"[FLASH 初始] {f0}")

        send(ser, "PWM 0", 0.3)
        send(ser, "ARM", 0.3)
        send(ser, f"PWM {PWM}", 0.5)
        time.sleep(1.0)
        send(ser, "PWM 0", 0.3)

        st = None
        t0 = time.time()
        while time.time() - t0 < 10:
            st = snap_state(ser)
            if st and st[0] == 3:
                break
            time.sleep(0.3)
        print(f"[SNAP] state={st[0]} count={st[1]} ready={st[2]}" if st else "[FAIL] no SNAP")

        time.sleep(0.5)          # 等自动保存打印完
        send(ser, "SNAP", 0.3)
        f1 = flash_info(ser)
        print(f"[FLASH 自动保存后] {f1}")
        assert f1.get("saved", 0) == 1, "自动保存未发生"
        assert f1.get("stored", 0) == st[1], "Flash 点数与 snap 不一致"

        r = read_dump(ser)
        if r is None:
            print("[FAIL] DUMP 未收到有效帧")
        else:
            print(f"[DUMP] {r}")
            assert r["magic"] == 0xAB1C0002
            assert r["n"] == st[1]
            assert r["crc_ok"], "crc32 校验失败"

        # 复位：模拟断电重启
        print("[RESET] openocd reset ...")
        if reset_board():
            time.sleep(1.5)
            ser.flushInput()
        else:
            print("[WARN] reset failed, 跳过复位验证")

        send(ser, "ID", 0.3)
        f2 = flash_info(ser)
        print(f"[FLASH 复位后] {f2}")
        assert f2.get("data", 0) == 1 and f2.get("stored", 0) == st[1], \
            "复位后 Flash 数据丢失"

        r2 = read_dump(ser)
        print(f"[DUMP 复位后] {r2}" if r2 else "[FAIL] 复位后 DUMP 失败")
        if r2:
            assert r2["n"] == st[1] and r2["crc_ok"]

        print("[PASS] Flash 芯片保存验证通过")
    finally:
        try:
            send(ser, "PWM 0", 0.3)
            send(ser, "DISARM", 0.2)
        except Exception:
            pass
        ser.close()


if __name__ == "__main__":
    main()