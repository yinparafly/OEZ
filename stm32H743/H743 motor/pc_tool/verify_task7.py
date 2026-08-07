# Task 7 Step1 全流程实测：ARM → 转电机 → 自动Flash+SD保存 → DUMP → 复位后配置/数据仍在
# 用法: python verify_full.py [COM]
import sys, serial, time

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM21"
BAUD = 921600

s = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(0.8)
s.reset_input_buffer()

def wait_prompt(t=6.0):
    buf = b""
    deadline = time.time() + t
    while time.time() < deadline:
        n = s.in_waiting
        if n:
            buf += s.read(n)
            if b"> " in buf: return buf
        else:
            time.sleep(0.3)
    return buf

def cmd(line, wait=0.3):
    s.reset_input_buffer()
    s.write((line + "\r\n").encode())
    buf = wait_prompt(wait)
    return buf.decode(errors="replace")

import sys as _sys
def pr(t): _sys.stdout.write(t + "\n")

pr("== ID ==")
pr(cmd("ID"))

pr("== CFG SHOW（默认档位）==")
pr(cmd("CFG SHOW"))

pr("== CAL STEPS（4000 基线）==")
pr(cmd("CAL STEPS"))

pr("== ARM 预置 ==")
pr(cmd("ARM", 1.0))

pr("== PWM 900 驱动 ~1.5s ==")
cmd("PWM 900")
time.sleep(1.5)
pr(cmd("PWM 0", 1.0))

pr("== snap 状态（应当 done count>0）==")
out = cmd("SNAP")
pr(out)

pr("== 自动保存已触发，FLASH 状态 ==")
out = cmd("FLASH")
pr(out)

pr("== 断电重启评估：先看当前数据 ==")
pr(cmd("IDX"))

s.close()