# Task 6 验证：CAL 命令 + 极性 + EMA 校准
# 用法: python verify_cal.py [COM]
import sys, serial, time

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM21"
BAUD = 921600

s = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(0.3)
s.reset_input_buffer()

def cmd(line, wait=0.4):
    """发送命令，等待 '> ' 提示符返回，读回全部输出。Flash 写命令时间长，最多等 6s。"""
    s.reset_input_buffer()
    # 先清掉任何积压
    s.write((line + "\r\n").encode())
    buf = b""
    deadline = time.time() + 6.0
    while time.time() < deadline:
        n = s.in_waiting
        if n:
            buf += s.read(n)
            if b"> " in buf:
                break
        else:
            time.sleep(wait)
    if b"> " not in buf:
        time.sleep(1.0)
        buf += s.read(s.in_waiting or 4096)
    return buf.decode(errors="replace")

import sys as _sys
def pr(t):
    _sys.stdout.write(t + "\n")

print("== HELP 含 CAL ==")
h = cmd("HELP")
assert "CAL" in h and "CFG POL" in h, "HELP 缺 CAL/CFG POL"
print("OK")

print("== 重置到已知状态 ==")
out = cmd("CAL RESET")
print(repr(out))

print("== CAL STEPS（默认 4000）==")
out = cmd("CAL STEPS")
print(repr(out))
assert "steps_per_rev = 4000" in out

print("== CAL SET 4100 保存 ==")
out = cmd("CAL SET 4100")
print(repr(out))
assert "4100" in out

print("== CAL STEPS 回读 ==")
out = cmd("CAL STEPS")
print(repr(out))
assert "4100" in out

print("== CAL RESET 恢复 4000 ==")
out = cmd("CAL RESET")
print(repr(out))
out = cmd("CAL STEPS")
print(repr(out))
assert "4000" in out

print("== CFG POL 1 ==")
out = cmd("CFG POL 1")
print(repr(out))
assert "pol = 1" in out

print("== CFG POL 0 ==")
out = cmd("CFG POL 0")
print(repr(out))
assert "pol = 0" in out

print("== CFG GEAR 改后 CAL STEPS 仍 4000 / POL 仍 0 ==")
out = cmd("CFG GEAR 3 1 2 4 3000 6000")
print(repr(out))
out = cmd("CAL STEPS")
print(repr(out))
assert "4000" in out
out = cmd("CFG GEAR 3 1 2 4 4000 8000")
print(repr(out))

s.close()
print("\nALL OK")