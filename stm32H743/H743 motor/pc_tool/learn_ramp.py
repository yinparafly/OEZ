"""
电机油门特性自学习（空载特性学习）
流程：8 档连续爬升，每档：PWM 设置 → 等 2s 斜坡到位 → 保持油门读 1s（多次采样取均值）
     → 下一档；全程连续运转（用户确认放宽工作要求第1条）；超 10000rpm 立即停机。

输出：pc_tool/calib/learn_YYYYMMDD_HHMMSS.csv + learn_notes.md（空载声明）

用法:  python learn_ramp.py [起始‰] [终止‰] [档数,默认8]
"""
import sys
import time
import csv
import os
from datetime import datetime
import serial
import serial.tools.list_ports

BAUD = 921600
START_PWM = int(sys.argv[1]) if len(sys.argv) > 1 else 0
END_PWM = int(sys.argv[2]) if len(sys.argv) > 2 else 1000
GEARS = int(sys.argv[3]) if len(sys.argv) > 3 else 8
RAMP_S = 2.0      # 每档斜坡等待
HOLD_S = 1.0      # 每档记录时长
SAMPLES = 3       # 每档采样次数
MAX_RPM = 10000   # 超限立即停机
CALIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib")

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


def get_rpm():
    r = send("RPM", 0.25)
    for line in r.splitlines():
        if "rpm = " in line:
            s = line.split("rpm = ")[1]
            v = int(s.split(" rpm")[0].strip())
            return v
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

    pwm_steps = []
    if GEARS <= 1:
        pwm_steps = [END_PWM]
    else:
        step = (END_PWM - START_PWM) / GEARS
        pwm_steps = [int(round(START_PWM + step * (i + 1))) for i in range(GEARS)]
        if pwm_steps[-1] != END_PWM:
            pwm_steps[-1] = END_PWM
        pwm_steps = [p for p in pwm_steps if p > 0]

    rows = []
    start_ms = 1.02 + START_PWM * 0.98 / 1000.0
    end_ms = 1.02 + END_PWM * 0.98 / 1000.0
    print(f"--- 油门自学习：{START_PWM}‰({start_ms:.3f}ms) → {END_PWM}‰({end_ms:.3f}ms), {GEARS} 档 ---")

    try:
        print("--- PWM 0 复位 ---"); print(send("PWM 0", 0.5))
        t_run0 = time.time()

        for pwm in pwm_steps:
            ms = 1.02 + pwm * 0.98 / 1000.0
            print(f"--- 档 {pwm}‰ ({ms:.3f}ms) 爬坡 {RAMP_S}s ---")
            send(f"PWM {pwm}", 0.1)
            time.sleep(RAMP_S)

            vals = []
            for i in range(SAMPLES):
                v = get_rpm()
                if v is not None:
                    vals.append(v)
                    print(f"    sample{i}: {v} rpm")
                if v is not None and v >= MAX_RPM:
                    print(f"\n>>> 超限 {v} ≥ {MAX_RPM}rpm, 立即停机")
                    vals = [v]
                    break
                if i < SAMPLES - 1:
                    time.sleep((HOLD_S - 0.25 * SAMPLES) / (SAMPLES - 1))

            if not vals:
                avg = 0
                print("    (无读数)")
            else:
                avg = sum(vals) // len(vals)
            rows.append((pwm, avg, vals))
            print(f"    => {pwm}‰ : {avg} rpm")

            if avg >= MAX_RPM:
                break
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

    # ---------------- 结果输出 ----------------
    t_run = time.time() - t_run0 if 't_run0' in dir() else 0
    os.makedirs(CALIB_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(CALIB_DIR, f"learn_{stamp}.csv")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["pwm_permille", "pulse_ms", "rpm_avg", "samples"])
        for pwm, avg, vals in rows:
            w.writerow([pwm, round(1.02 + pwm * 0.98 / 1000.0, 4), avg, vals])
    print(f"\n[CSV] {csv_path}")

    # 启动油门：第一个 rpm>0
    start_p = next((p for p, a, _ in rows if a > 0), None)
    print("--- 学习结果 ---")
    for pwm, avg, vals in rows:
        flag = " <-- 启动点" if pwm == start_p else ""
        print(f"  PWM {pwm:4d}‰ ({1.02 + pwm*0.98/1000:.3f}ms) : {avg:6d} rpm{flag}")
    if start_p is not None:
        print(f">>> 启动油门 ≈ {start_p}‰")
    else:
        print(">>> 未检测到启动点（全部 rpm=0？）")

    # 备注文档（空载声明）
    notes_path = os.path.join(CALIB_DIR, "learn_notes.md")
    is_new = not os.path.exists(notes_path)
    with open(notes_path, "a", encoding="utf-8") as f:
        if is_new:
            f.write("# 电机油门特性学习记录\n\n")
            f.write("> **重要声明：本表为当前型号电机 + 当前载荷（空载）下测得。**\n")
            f.write("> 载荷改变（加负载/换桨/换结构）、换电机/电调/电池/接线后，油门-转速特性会变化，**必须重新学习**。\n\n")
            f.write("| 时间 | 档位范围 | 启动油门 | 备注 |\n|------|---------|---------|------|\n")
        f.write(f"| {stamp} | {START_PWM}~{END_PWM}‰ | {start_p}‰ | 总运转{t_run:.0f}s |\n")
    print(f"[NOTES] {notes_path}")
    print("--- 完成，已停机 ---")


if __name__ == "__main__":
    main()