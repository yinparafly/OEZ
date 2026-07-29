"""
Pin Finder: 全自动识别绿灯 GPIO
方法: 先关所有引脚 → 只开一个 → 摄像头检测绿灯变化
测试模式: 300ms ON + 700ms OFF × 5  (区别于平时行为)
二次确认: 700ms ON + 300ms OFF × 3  (不同节奏)
"""

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time

TEST_PINS = [
    ("A","PA2",0x0004),("A","PA3",0x0008),("A","PA8",0x0100),("A","PA11",0x0800),
    ("A","PA12",0x1000),("A","PA15",0x8000),
    ("B","PB0",0x0001),("B","PB1",0x0002),("B","PB2",0x0004),
    ("B","PB5",0x0020),("B","PB6",0x0040),("B","PB7",0x0080),("B","PB8",0x0100),
    ("B","PB9",0x0200),("B","PB12",0x1000),("B","PB13",0x2000),("B","PB14",0x4000),("B","PB15",0x8000),
    ("C","PC0",0x0001),("C","PC1",0x0002),("C","PC2",0x0004),("C","PC3",0x0008),
    ("C","PC4",0x0010),("C","PC5",0x0020),("C","PC6",0x0040),("C","PC7",0x0080),
    ("C","PC8",0x0100),("C","PC9",0x0200),("C","PC10",0x0400),("C","PC11",0x0800),
    ("C","PC12",0x1000),("C","PC14",0x4000),("C","PC15",0x8000),
]

W = 960; H = 640

def build_cmd(port, pin_mask):
    return bytes([ord('T'), ord(port)-ord('A'), (pin_mask>>8)&0xFF, pin_mask&0xFF])

def build_verify_cmd(port, pin_mask):
    return bytes([ord('V'), ord(port)-ord('A'), (pin_mask>>8)&0xFF, pin_mask&0xFF])

class SerialCtrl:
    def __init__(self):
        self.ser = None; self.last_cmd = ""
    def auto_connect(self):
        for p in [p.device for p in serial.tools.list_ports.comports()]:
            try:
                self.ser = serial.Serial(p, 115200, timeout=1)
                time.sleep(0.5); self.ser.reset_input_buffer()
                print(f"  串口: {p}")
                return True
            except: continue
        return False
    def send(self, data, label=""):
        if not self.ser: return
        self.last_cmd = label or str(data)
        self.ser.write(data); self.ser.flush()

def find_green_led(cap, ser):
    """发送 ON/OFF，帧差分定位绿灯"""
    ser.send(b'2', '2 OFF')
    time.sleep(0.3)
    off_frames = []
    for _ in range(5):
        ret, f = cap.read()
        if ret: off_frames.append(cv2.resize(f, (W, H)))
        time.sleep(0.05)
    if not off_frames: return None
    bg = np.median(np.array(off_frames), axis=0).astype(np.uint8)
    bg_gray = cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY)

    ser.send(b'1', '1 ON')
    time.sleep(0.3)
    on_frames = []
    for _ in range(5):
        ret, f = cap.read()
        if ret: on_frames.append(cv2.resize(f, (W, H)))
        time.sleep(0.05)
    if not on_frames: return None
    fg = np.median(np.array(on_frames), axis=0).astype(np.uint8)
    fg_gray = cv2.cvtColor(fg, cv2.COLOR_BGR2GRAY)

    diff = cv2.absdiff(fg_gray, bg_gray)
    _, th = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
    th = cv2.erode(th, None, iterations=1)
    th = cv2.dilate(th, None, iterations=2)
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 15: return None
    x, y, w, h = cv2.boundingRect(c)
    return (x-8, y-8, w+16, h+16)

def measure(frame, roi):
    if roi is None: return 0
    x, y, w, h = roi
    r = frame[y:y+h, x:x+w]
    if r.size == 0: return 0
    return r[:,:,1].mean()

def detect_flash(cap, roi, pin_name, pin_idx, total, ser_cmd, n=55, d=0.1):
    vals = []
    for i in range(n):
        ret, f = cap.read()
        if not ret: continue
        f = cv2.resize(f, (W, H))
        v = measure(f, roi)
        vals.append(v)
        stat = "ON" if v > 20 else "OFF"
        draw_ui(f, roi, v, f"{stat} [{i+1}/{n}]", pin_idx, pin_name, total, ser_cmd)
        cv2.imshow("Pin Finder", f)
        cv2.waitKey(1)
        time.sleep(d)
    if not vals: return False, 0
    ch = max(vals) - min(vals)
    return ch > 20, ch

def draw_ui(f, roi, val, status, pin_idx, pin_name, total, serial_cmd, found=False):
    h, w = f.shape[:2]
    ov = f.copy()
    cv2.rectangle(ov, (0,0), (w,50), (30,30,30), -1)
    cv2.putText(ov, f"Pin: {pin_name}  [{pin_idx+1}/{total}]", (15,20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.putText(ov, f"Green: {val:.0f}  {status}", (15,40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
    px = w - 240
    cv2.rectangle(ov, (px,0), (w,110), (40,40,40), -1)
    cv2.putText(ov, f"TX: {serial_cmd}", (px+8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,50), 1)
    cv2.putText(ov, f"Value: {val:.0f}", (px+8, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1)
    cv2.putText(ov, f"Status: {status}", (px+8, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0) if "ON" in status else (100,100,100), 1)
    pw = w - 40; by = h - 25
    cv2.rectangle(ov, (20,by), (20+pw, by+10), (60,60,60), -1)
    if total > 0:
        fill = int(pw * (pin_idx+1) / total)
        cv2.rectangle(ov, (20,by), (20+fill, by+10), (0,180,0), -1)
    cv2.putText(ov, f"{(pin_idx+1)/total*100:.0f}%", (w//2-20, by-3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)
    if roi:
        x, y, rw, rh = roi
        cv2.rectangle(ov, (x,y), (x+rw, y+rh), (0,255,0), 2)
    if found:
        cv2.putText(ov, f">>> FOUND: {pin_name} <<<", (w//2-180, h//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 4)
    cv2.addWeighted(ov, 0.7, f, 0.3, 0, f)

def main():
    print("=== STM32 Pin Finder ===")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("摄像头失败"); return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

    print("自动连接串口...")
    ser = SerialCtrl()
    if not ser.auto_connect():
        print("无串口"); cap.release(); return

    print("帧差分定位绿灯...")
    roi = None
    for _ in range(3):
        roi = find_green_led(cap, ser)
        if roi: break
        time.sleep(0.2)
    if not roi:
        print("未找到绿灯, 用画面中心")
        roi = (W//2-20, H//2-20, 40, 40)
    print(f"  ROI: {roi}")

    ser.send(b'2', '2 OFF')
    time.sleep(0.3)
    ret, f = cap.read()
    if ret: f = cv2.resize(f, (W, H))
    off_v = measure(f, roi) if ret else 0
    ser.send(b'1', '1 ON')
    time.sleep(0.3)
    ret, f = cap.read()
    if ret: f = cv2.resize(f, (W, H))
    on_v = measure(f, roi) if ret else 0
    print(f"  OFF={off_v:.0f} ON={on_v:.0f} diff={on_v-off_v:.0f}")

    print(f"\n扫描 {len(TEST_PINS)} 个引脚 (全部测试, 不下结论)...")
    cv2.namedWindow("Pin Finder")
    matches = []
    ser.send(b'2', '2 OFF')

    for idx, (port, pin_name, pin_mask) in enumerate(TEST_PINS):
        ret, f = cap.read()
        if not ret: continue
        f = cv2.resize(f, (W, H))

        ser.send(build_cmd(port, pin_mask), f"T {port} 0x{pin_mask:04X}")
        flashed, change = detect_flash(cap, roi, pin_name, idx, len(TEST_PINS), ser.last_cmd, 55, 0.1)

        if flashed:
            matches.append((pin_name, change, idx))
            print(f"\n[{idx+1}/{len(TEST_PINS)}] {pin_name} ⚡ Δ={change:.0f}")
        else:
            print(".", end="", flush=True)

        ret, f = cap.read()
        if ret: f = cv2.resize(f, (W, H))
        draw_ui(f, roi, measure(f, roi), "WAIT", idx, pin_name, len(TEST_PINS), ser.last_cmd)
        cv2.imshow("Pin Finder", f)
        if (cv2.waitKey(1) & 0xFF) == 27: break

    print(f"\n\n=== 扫描结果: {len(matches)} 个引脚触发绿灯 ===")
    for name, ch, idx in matches:
        print(f"  {name}  Δ={ch:.0f}")

    detected = None
    if len(matches) == 1:
        candidate = matches[0]
        port, pin_mask = TEST_PINS[candidate[2]][0], TEST_PINS[candidate[2]][2]
        print(f"\n唯一匹配, 二次确认 {candidate[0]}...")
        ser.send(build_verify_cmd(port, pin_mask), f"V {port} 0x{pin_mask:04X}")
        confirmed, ch2 = detect_flash(cap, roi, candidate[0], candidate[2], len(TEST_PINS), ser.last_cmd, 40, 0.1)
        if confirmed:
            print(f"  确认! Δ={ch2:.0f}")
            detected = candidate[0]
            ret, f = cap.read()
            if ret: f = cv2.resize(f, (W, H))
            draw_ui(f, roi, measure(f, roi), "CONFIRMED!", candidate[2], detected, len(TEST_PINS), ser.last_cmd, True)
            cv2.imshow("Pin Finder", f)
            cv2.waitKey(5000)
    elif len(matches) == 0:
        print("\n没有引脚触发绿灯 — 可能是电源指示灯")
    else:
        print(f"\n{len(matches)} 个引脚触发, 需要排查串扰或噪声")

    if ser.ser: ser.ser.close()
    cap.release()
    cv2.destroyAllWindows()

    if detected:
        print(f"\n=== 绿灯 = {detected} ===")
    else:
        print("\n=== 未找到 ===")
    input("按 Enter 退出")

if __name__ == "__main__":
    main()
