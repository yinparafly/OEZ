"""
弹射测试（Task 8 动态滤波验证）
- 探测 COM（ID 找板子）
- 读取 FILT SHOW（显示当前动态滤波状态）
- MONITOR START (ARM) → PWM 拉速 ×1.8s → PWM 0 停机
- 等 # RECORD done → LOG DUMP 拉记录数据分析高速段跳动

用法:  python blast_test.py [PWM 默认900] [运行秒 默认1.8]
"""
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
import time
import serial
import serial.tools.list_ports

BAUD = 921600
PWM = int(sys.argv[1]) if len(sys.argv) > 1 else 900
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 1.8
CANDIDATES = ["COM7", "COM8", "COM21", "COM22"]


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    for cfg in [c for c in CANDIDATES if c not in ports] + ports:
        try:
            s = serial.Serial(cfg, BAUD, timeout=0.3)
            s.reset_input_buffer()
            s.write(b"ID\r\n")
            time.sleep(0.5)
            data = s.read(1024).decode("utf-8", "replace")
            s.close()
            if "ABI-MONITOR" in data:
                return cfg, data
        except Exception:
            continue
    return None, ""


def main():
    port, banner = find_port()
    if not port:
        print("[失败] 未找到板子（确认串口线已接）")
        sys.exit(1)
    print(f"[OK] {port}: {banner.strip().splitlines()[-1]}")

    ser = serial.Serial(port, BAUD, timeout=0.5)
    ser.reset_input_buffer()

    def send(cmd, wait=0.4):
        ser.reset_input_buffer()
        ser.write((cmd + "\r\n").encode())
        time.sleep(wait)
        return ser.read(ser.in_waiting or 512).decode("utf-8", "replace")

    try:
        print("\n===== 1) 当前动态滤波状态 (FILT SHOW) =====")
        print(send("FILT SHOW", 0.5))
        print(send("CFG SHOW", 0.3))

        print("\n===== 2) PWM 0 复位 + ARM =====")
        print(send("PWM 0", 0.3))
        print(send("MONITOR START", 0.3))

        print(f"\n===== 3) 弹射 PWM {PWM} × {DUR}s =====")
        ser.write((f"PWM {PWM}\r\n").encode())
        t0 = time.time()
        while time.time() - t0 < DUR:
            time.sleep(0.05)
        print(f"[运行结束 {time.time()-t0:.1f}s] PWM 0 停机")
        ser.write(b"PWM 0\r\n")
        time.sleep(1.5)

        print("\n===== 4) 等待触发完成 (# RECORD done) =====")
        done = False
        for _ in range(60):
            n = ser.in_waiting
            if n:
                data = ser.read(n).decode("utf-8", "replace")
                if "RECORD done" in data:
                    print("[触发完成]")
                    print(data)
                    done = True
                    break
                if "TRIGGER" in data and not done:
                    print(data)
            time.sleep(0.1)
        if not done:
            print("[超时] 没等到 RECORD done，停 B）")

        print("\n===== 5) SNAP 状态 + LOG DUMP (PC 帧拉取) =====")
        print(send("SNAP? ", 0.2))
        dump = send("LOG DUMP USB", 1.5)
        lines = dump.splitlines()
        print(f"  共 {len([l for l in lines if l.startswith('D,')])} 点 D, 帧")
        # 打印 高速段 rpm 抽样（尾部 260 行，绘制尖刺）
        dlines = [l for l in lines if l.startswith("D,")]
        if dlines:
            nums = []
            for l in dlines:
                p = l.split(",")
                try:
                    nums.append(float(p[2]) if p[2].strip().lower() not in ("inf", "nan") else 0.0)
                except Exception:
                    return
            # 显示 max / min / 按 5 分位看跳动
            mx, mn = max(nums), min(nums)
            print(f"\n   rpm 范围: min={mn:.0f} max={mx:.0f}")
            print("   最后 60 点 rpm（看高速段跳动）:")
            for i, v in enumerate(nums[-60:]):
                print(f"     [{i:2d}] {v:8.1f}")
    finally:
        ser.reset_input_buffer()
        ser.write(b"PWM 0\r\n")
        time.sleep(0.3)


if __name__ == "__main__":
    main()