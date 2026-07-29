"""
Pin Finder: 实时模式 - 全帧率 30fps, 每引脚 500ms
T = 设 HIGH, K = 关断
无 sleep 延时, 帧间用 waitKey 自然等待
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
        for p in [p.device for p in serial.tools.list_ports.comports()]:
            try:
                self.ser = serial.Serial(p, 115200, timeout=0.01)
                time.sleep(0.3); self.ser.reset_input_buffer()
                print(f"  串口: {p}")
                return True
            except: continue
        return False
    def send(self, data, label=""):
        if not self.ser: return
        self.last_cmd = label or str(data)
        self.ser.write(data); self.ser.flush()

def measure(frame, roi):
    if roi is None: return 0
    x, y, w, h = roi
    r = frame[y:y+h, x:x+w]
    if r.size == 0: return 0
    g = r[:,:,1].mean(); rv = r[:,:,2].mean(); b = r[:,:,0].mean()
    return g - (rv + b) * 0.5

def draw_ui(f, roi, val, status, pin_idx, pin_name, total, ser_cmd, found=False):
    h, w = f.shape[:2]; ov = f.copy()
    cv2.rectangle(ov, (0,0), (w,50), (30,30,30), -1)
    cv2.putText(ov, f"Pin: {pin_name}  [{pin_idx+1}/{total}]", (15,20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.putText(ov, f"Green: {val:.0f}  {status}", (15,40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
    px = w - 260
    cv2.rectangle(ov, (px,0), (w,130), (40,40,40), -1)
    cv2.putText(ov, f"TX: {ser_cmd}", (px+8, 20),
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

def run():
    print("=== STM32 Pin Finder 实时模式 ===")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened(): print("摄像头失败"); return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

    ser = SerialCtrl()
    if not ser.auto_connect(): print("无串口"); cap.release(); return

    ret, f = cap.read()
    if ret: f = cv2.resize(f, (W, H))
    print("点击绿灯位置 → +/-调大小 → SPACE确认")
    pt = [None]; sz = [24]
    def cb(e, x, y, f, p):
        if e == cv2.EVENT_LBUTTONDOWN: pt[0] = (x, y)
    cv2.namedWindow("Select ROI")
    cv2.setMouseCallback("Select ROI", cb)
    while True:
        d = f.copy()
        if pt[0]:
            x, y = pt[0]; s = sz[0]//2
            cv2.circle(d, (x, y), 3, (0,255,0), -1)
            cv2.rectangle(d, (x-s, y-s), (x+s, y+s), (0,255,0), 2)
            cv2.putText(d, f"ROI {sz[0]}x{sz[0]} +- SPACE ok", (20,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.imshow("Select ROI", d)
        k = cv2.waitKey(30) & 0xFF
        if k == 27: cv2.destroyWindow("Select ROI"); cap.release(); return
        if k == 43: sz[0] = min(120, sz[0] + 4)
        if k == 45: sz[0] = max(8, sz[0] - 4)
        if k == 32 and pt[0]: break
    cv2.destroyWindow("Select ROI")
    x, y = pt[0]; s = sz[0]//2
    roi = (max(0,x-s), max(0,y-s), sz[0], sz[0])
    print(f"  ROI: {roi}")

    print(f"\n扫描 {len(TEST_PINS)} 个引脚 (实时模式)...")
    cv2.namedWindow("Pin Finder")
    results = []  # (pin_name, Δ)
    pin_idx = 0
    state = "ON_CAPTURE"
    on_vals = []; off_vals = []
    on_count = 10; off_count = 10
    on_done = 0; off_done = 0
    port, pin_name, pin_mask = TEST_PINS[0]

    ser.send(build_cmd(port, pin_mask), f"T {port} 0x{pin_mask:04X}")
    state = "ON_CAPTURE"; on_vals = []; on_done = 0

    while pin_idx < len(TEST_PINS):
        ret, f = cap.read()
        if not ret: continue
        f = cv2.resize(f, (W, H))
        val = measure(f, roi)

        port, pin_name, pin_mask = TEST_PINS[pin_idx]
        status = ""

        if state == "ON_CAPTURE":
            if on_done < on_count:
                on_vals.append(val); on_done += 1
                status = f"ON {on_done}/{on_count}"
            else:
                ser.send(b'K', 'K')
                state = "OFF_CAPTURE"; off_vals = []; off_done = 0
                status = "OFF..."
                continue

        elif state == "OFF_CAPTURE":
            if off_done < off_count:
                off_vals.append(val); off_done += 1
                status = f"OFF {off_done}/{off_count}"
            else:
                on_avg = sum(on_vals) / len(on_vals)
                off_avg = sum(off_vals) / len(off_vals)
                delta = on_avg - off_avg
                results.append((pin_name, delta))
                print(f"\n[{pin_idx+1}/{len(TEST_PINS)}] {pin_name}  Δ={delta:.0f}  {'***' if delta > 25 else ''}")

                pin_idx += 1
                if pin_idx >= len(TEST_PINS):
                    break

                nport, npin_name, npin_mask = TEST_PINS[pin_idx]
                ser.send(build_cmd(nport, npin_mask), f"T {nport} 0x{npin_mask:04X}")
                state = "ON_CAPTURE"; on_vals = []; on_done = 0
                status = "NEXT..."
                continue

        draw_ui(f, roi, val, status, pin_idx, pin_name, len(TEST_PINS), ser.last_cmd)
        cv2.imshow("Pin Finder", f)
        if (cv2.waitKey(1) & 0xFF) == 27: break

    ser.send(b'K', 'K')
    ser.ser.close(); cap.release(); cv2.destroyAllWindows()

    print(f"\n\n=== 扫描结果 ===")
    results.sort(key=lambda x: -x[1])
    for name, d in results:
        flag = " <<<" if d > 25 else ""
        print(f"  {name}: Δ={d:.0f}{flag}")

    best_d = max(r[1] for r in results)
    avg_d = sum(r[1] for r in results) / len(results)
    if best_d > avg_d * 2 and best_d > 25:
        winner = max(results, key=lambda x: x[1])
        print(f"\n=== 绿灯 = {winner[0]} (Δ={winner[1]:.0f}) ===")
    elif best_d > 25:
        winner = max(results, key=lambda x: x[1])
        print(f"\n=== 可能 = {winner[0]} (Δ={winner[1]:.0f}, 平均Δ={avg_d:.0f}) ===")
    else:
        print(f"\n=== 未检测到可控引脚 ===")
    input("Enter 退出")

if __name__ == "__main__":
    run()
