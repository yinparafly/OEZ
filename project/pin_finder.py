"""
Pin Finder: 全自动 - 摄像头识别 STM32 绿灯 GPIO
自动找串口 + 自动找绿灯 + 自动全扫描
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

class SerialCtrl:
    def __init__(self):
        self.ser = None; self.last_cmd = ""
    def auto_connect(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports: return False
        for p in ports:
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
    def test_pin(self, port, pin_mask):
        self.send(build_cmd(port, pin_mask), f"T {port} 0x{pin_mask:04X}")

def find_green_led(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (40, 60, 60), (85, 255, 255))
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 20: return None
    x, y, w, h = cv2.boundingRect(c)
    return (x-10, y-10, w+20, h+20)

def measure_green(frame, roi):
    if roi is None: return 0
    x, y, w, h = roi
    r = frame[y:y+h, x:x+w]
    if r.size == 0: return 0
    g = r[:,:,1].mean(); rv = r[:,:,2].mean(); b = r[:,:,0].mean()
    return g - (rv + b) / 2

def detect_flash(cap, roi, pin_name, pin_idx, total, ser_cmd, n=20, d=0.1):
    vals = []
    for i in range(n):
        ret, f = cap.read()
        if not ret: continue
        f = cv2.resize(f, (W, H))
        v = measure_green(f, roi)
        vals.append(v)
        stat = "FLASH" if v > 25 else "OFF"
        draw_ui(f, roi, v, f"{stat} [{i+1}/{n}]", pin_idx, pin_name, total, ser_cmd)
        cv2.imshow("Pin Finder", f)
        cv2.waitKey(1)
        time.sleep(d)
    if not vals: return False, 0
    ch = max(vals)-min(vals)
    return ch > 25, ch

def draw_ui(f, roi, val, status, pin_idx, pin_name, total, serial_cmd, found=False):
    h, w = f.shape[:2]
    ov = f.copy()
    # top bar
    cv2.rectangle(ov, (0,0), (w,50), (30,30,30), -1)
    cv2.putText(ov, f"Pin: {pin_name}  [{pin_idx+1}/{total}]", (15,20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.putText(ov, f"Green: {val:.0f}  {status}", (15,40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
    # right panel
    px = w - 240
    cv2.rectangle(ov, (px,0), (w,110), (40,40,40), -1)
    cv2.putText(ov, f"TX: {serial_cmd}", (px+8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,50), 1)
    cv2.putText(ov, f"Value: {val:.0f}", (px+8, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1)
    cv2.putText(ov, f"Status: {status}", (px+8, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0) if status=="ON" else (100,100,100), 1)
    # progress bar
    pw = w - 40; by = h - 25
    cv2.rectangle(ov, (20,by), (20+pw, by+10), (60,60,60), -1)
    if total > 0:
        fill = int(pw * (pin_idx+1) / total)
        cv2.rectangle(ov, (20,by), (20+fill, by+10), (0,180,0), -1)
    cv2.putText(ov, f"{(pin_idx+1)/total*100:.0f}%", (w//2-20, by-3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)
    # ROI box
    if roi:
        x, y, rw, rh = roi
        cv2.rectangle(ov, (x,y), (x+rw, y+rh), (0,255,0), 2)
    if found:
        cv2.putText(ov, f">>> FOUND: {pin_name} <<<", (w//2-180, h//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 4)
    cv2.addWeighted(ov, 0.7, f, 0.3, 0, f)

def main():
    print("=== STM32 Pin Finder 全自动 ===")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("摄像头失败"); input("按 Enter 退出"); return

    print("自动连接串口...")
    ser = SerialCtrl()
    if not ser.auto_connect():
        print("无串口"); cap.release(); input("按 Enter 退出"); return

    print("自动寻找绿灯...")
    roi = None
    for _ in range(30):
        ret, f = cap.read()
        if ret:
            f = cv2.resize(f, (W, H))
            roi = find_green_led(f)
            if roi: break
        time.sleep(0.1)
    if not roi:
        print("未找到绿灯, 使用画面中心")
        roi = (W//2-20, H//2-20, 40, 40)

    print(f"  绿灯位置: {roi}")
    print("验证检测...")
    ser.send(b'1', '1 ON')
    time.sleep(0.3)
    ret, f = cap.read()
    if ret: f = cv2.resize(f, (W, H))
    on_val = measure_green(f, roi) if ret else 0

    ser.send(b'2', '2 OFF')
    time.sleep(0.3)
    ret, f = cap.read()
    if ret: f = cv2.resize(f, (W, H))
    off_val = measure_green(f, roi) if ret else 0

    diff = on_val - off_val
    print(f"  ON={on_val:.0f} OFF={off_val:.0f} diff={diff:.0f}")
    if diff < 20:
        print("  差值过小, 继续尝试...")

    print(f"\n扫描 {len(TEST_PINS)} 个引脚...")
    cv2.namedWindow("Pin Finder")
    detected = None

    for idx, (port, pin_name, pin_mask) in enumerate(TEST_PINS):
        ret, f = cap.read()
        if not ret: continue
        f = cv2.resize(f, (W, H))
        val = measure_green(f, roi)
        stat = "ON" if val > 25 else "OFF"

        ser.send(build_cmd(port, pin_mask), f"T {port} 0x{pin_mask:04X}")
        flashed, change = detect_flash(cap, roi, pin_name, idx, len(TEST_PINS), ser.last_cmd, 20, 0.1)
        time.sleep(0.5)

        if flashed:
            print(f"\n[{idx+1}/{len(TEST_PINS)}] {pin_name} *** 匹配! Δ={change:.0f} ***")
            detected = pin_name
            ret, f = cap.read()
            if ret: f = cv2.resize(f, (W, H))
            draw_ui(f, roi, val, f"FLASH! Δ={change:.0f}", idx, pin_name, len(TEST_PINS), ser.last_cmd, True)
            cv2.imshow("Pin Finder", f)
            cv2.waitKey(3000)
            break
        else:
            print(".", end="", flush=True)

        ret, f = cap.read()
        if ret: f = cv2.resize(f, (W, H))
        draw_ui(f, roi, measure_green(f, roi), stat, idx, pin_name, len(TEST_PINS), ser.last_cmd)
        cv2.imshow("Pin Finder", f)
        if (cv2.waitKey(1) & 0xFF) == 27: break

    ser.ser.close() if ser.ser else None
    cap.release()
    cv2.destroyAllWindows()

    if detected:
        print(f"\n\n=== 结果: 绿灯 = {detected} ===")
    else:
        print("\n\n=== 未检测到 ===")
    input("按 Enter 退出")

if __name__ == "__main__":
    main()
