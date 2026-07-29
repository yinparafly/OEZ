"""
Pin Finder: 摄像头自动识别 STM32 绿灯 GPIO
UI 显示: 摄像头画面 + 亮度值 + 当前引脚 + 串口指令
"""

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
import sys


TEST_PINS = [
    ("A", "PA2",  0x0004), ("A", "PA3",  0x0008),
    ("A", "PA8",  0x0100), ("A", "PA11", 0x0800),
    ("A", "PA12", 0x1000), ("A", "PA15", 0x8000),
    ("B", "PB0",  0x0001), ("B", "PB1",  0x0002),
    ("B", "PB2",  0x0004),
    ("B", "PB5",  0x0020), ("B", "PB6",  0x0040),
    ("B", "PB7",  0x0080), ("B", "PB8",  0x0100),
    ("B", "PB9",  0x0200), ("B", "PB12", 0x1000),
    ("B", "PB13", 0x2000), ("B", "PB14", 0x4000),
    ("B", "PB15", 0x8000),
    ("C", "PC0",  0x0001), ("C", "PC1",  0x0002),
    ("C", "PC2",  0x0004), ("C", "PC3",  0x0008),
    ("C", "PC4",  0x0010), ("C", "PC5",  0x0020),
    ("C", "PC6",  0x0040), ("C", "PC7",  0x0080),
    ("C", "PC8",  0x0100), ("C", "PC9",  0x0200),
    ("C", "PC10", 0x0400), ("C", "PC11", 0x0800),
    ("C", "PC12", 0x1000), ("C", "PC14", 0x4000),
    ("C", "PC15", 0x8000),
]

WINDOW_W = 960
WINDOW_H = 640
ROI_SIZE = 30


def build_test_command(port, pin_mask):
    port_num = ord(port) - ord('A')
    hi = (pin_mask >> 8) & 0xFF
    lo = pin_mask & 0xFF
    return bytes([ord('T'), port_num, hi, lo])


class SerialCtrl:
    def __init__(self):
        self.ser = None
        self.connected = False
        self.last_cmd = ""

    def list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port_name, baud=115200):
        self.ser = serial.Serial(port_name, baud, timeout=2)
        self.connected = True
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        return True

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.connected = False

    def send(self, data, label=""):
        if not self.connected:
            return
        self.last_cmd = label or str(data)
        self.ser.write(data)
        self.ser.flush()

    def test_pin(self, port, pin_mask):
        cmd = build_test_command(port, pin_mask)
        label = f"T {port} 0x{pin_mask:04X}"
        self.send(cmd, label)
        time.sleep(2.5)

    def send_char(self, c, label=""):
        self.send(bytes([c]), label or f"'{chr(c) if 32 <= c < 127 else hex(c)}'")


class GreenDetector:
    def __init__(self):
        self.roi = None
        self.baseline = 0
        self.threshold = 25
        self.current_val = 0
        self.status = "WAITING"

    def select_roi(self, frame):
        print("点击绿灯位置 → 按 SPACE 确认 (ESC 取消)")
        click_pt = [None]
        confirmed = [False]

        def cb(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                click_pt[0] = (x, y)

        cv2.namedWindow("Select ROI")
        cv2.setMouseCallback("Select ROI", cb)
        while True:
            disp = frame.copy()
            if click_pt[0]:
                x, y = click_pt[0]
                cv2.circle(disp, (x, y), 5, (0, 255, 0), -1)
                cv2.rectangle(disp, (x - ROI_SIZE, y - ROI_SIZE),
                              (x + ROI_SIZE, y + ROI_SIZE), (0, 255, 0), 1)
            cv2.putText(disp, "Click GREEN LED -> SPACE", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Select ROI", disp)
            k = cv2.waitKey(30) & 0xFF
            if k == 32 and click_pt[0]:
                confirmed[0] = True
                break
            if k == 27:
                break
        cv2.destroyWindow("Select ROI")
        if confirmed[0]:
            x, y = click_pt[0]
            self.roi = (max(0, x - ROI_SIZE), max(0, y - ROI_SIZE),
                        ROI_SIZE * 2, ROI_SIZE * 2)
            return True
        return False

    def measure(self, frame):
        if self.roi is None:
            return 0
        x, y, w, h = self.roi
        roi = frame[y:y+h, x:x+w]
        if roi.size == 0:
            return 0
        g = roi[:, :, 1].mean()
        r = roi[:, :, 2].mean()
        b = roi[:, :, 0].mean()
        return g - (r + b) / 2

    def update(self, frame):
        self.current_val = self.measure(frame)
        self.status = "ON" if self.current_val > self.threshold else "OFF"

    def detect_flash(self, cap, num_frames=15, delay_ms=100):
        vals = []
        for _ in range(num_frames):
            ret, frame = cap.read()
            if ret:
                frame = cv2.resize(frame, (WINDOW_W, WINDOW_H))
                vals.append(self.measure(frame))
            time.sleep(delay_ms / 1000.0)
        if not vals:
            return False
        change = max(vals) - min(vals)
        return change > self.threshold, max(vals), min(vals), change


def draw_ui(frame, detector, serial_ctrl, pin_idx, pin_name, total, status_line):
    h, w = frame.shape[:2]

    overlay = frame.copy()

    # top bar
    cv2.rectangle(overlay, (0, 0), (w, 55), (30, 30, 30), -1)
    cv2.putText(overlay, "STM32 Pin Finder", (15, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.putText(overlay, f"Pin: {pin_name}  [{pin_idx+1}/{total}]", (15, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # right info panel
    px = w - 260
    cv2.rectangle(overlay, (px, 0), (w, 130), (40, 40, 40), -1)
    cv2.putText(overlay, "Green LED", (px + 8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    val = detector.current_val
    cv2.putText(overlay, f"Value: {val:.0f}", (px + 8, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    on_color = (0, 255, 0) if detector.status == "ON" else (100, 100, 100)
    cv2.circle(overlay, (px + 160, 14), 6, on_color, -1)
    cv2.putText(overlay, detector.status, (px + 172, 19),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, on_color, 1)

    # serial cmd
    cv2.putText(overlay, "Serial TX:", (px + 8, 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    cv2.putText(overlay, serial_ctrl.last_cmd, (px + 8, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 50), 1)

    # status line
    cv2.putText(overlay, status_line, (px + 8, 115),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 100), 1)

    # progress bar at bottom
    prog_w = w - 40
    bar_h = 12
    bar_y = h - 30
    cv2.rectangle(overlay, (20, bar_y), (20 + prog_w, bar_y + bar_h), (60, 60, 60), -1)
    if total > 0:
        fill = int(prog_w * (pin_idx + 1) / total)
        cv2.rectangle(overlay, (20, bar_y), (20 + fill, bar_y + bar_h), (0, 180, 0), -1)
    pct = (pin_idx + 1) / total * 100 if total > 0 else 0
    cv2.putText(overlay, f"{pct:.0f}%", (w // 2 - 20, bar_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # draw ROI rectangle on the green LED
    if detector.roi:
        x, y, rw, rh = detector.roi
        cv2.rectangle(overlay, (x, y), (x + rw, y + rh), (0, 255, 0), 2)
        cv2.putText(overlay, "GREEN", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    # blend overlay
    alpha = 0.7
    frame[:] = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)


def main():
    print("=" * 50)
    print("STM32 Pin Finder - 摄像头识别绿灯 GPIO")
    print("=" * 50)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("错误: 无法打开摄像头")
        fallback_manual()
        return

    ret, frame = cap.read()
    if not ret:
        print("错误: 无法读取摄像头")
        cap.release()
        fallback_manual()
        return
    frame = cv2.resize(frame, (WINDOW_W, WINDOW_H))

    detector = GreenDetector()
    if not detector.select_roi(frame):
        print("未选择 ROI")
        cap.release()
        return
    print(f"ROI: {detector.roi}")

    ser = SerialCtrl()
    ports = ser.list_ports()
    if not ports:
        print("错误: 无串口可用")
        cap.release()
        return

    print("\n可用串口:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p}")
    try:
        idx = int(input("选择串口号: "))
        ser.connect(ports[idx])
        print("串口已连接")
    except Exception as e:
        print(f"串口连接失败: {e}")
        cap.release()
        return

    # === 验证阶段 ===
    print("\n=== 验证摄像头检测 ===")
    cv2.namedWindow("Pin Finder")

    ser.send_char(ord('1'), "1=Green ON")
    time.sleep(0.3)
    ret, frame = cap.read()
    frame = cv2.resize(frame, (WINDOW_W, WINDOW_H)) if ret else frame
    val_on = detector.measure(frame)

    ser.send_char(ord('2'), "2=Green OFF")
    time.sleep(0.3)
    ret, frame = cap.read()
    frame = cv2.resize(frame, (WINDOW_W, WINDOW_H)) if ret else frame
    val_off = detector.measure(frame)

    diff = val_on - val_off
    print(f"  ON={val_on:.1f}  OFF={val_off:.1f}  diff={diff:.1f} (threshold=25)")
    if diff < 25:
        print("  ⚠ 差值偏小，可能需要调整光线或摄像头位置")
        ans = input("  继续? (y/n): ")
        if ans.lower() != 'y':
            ser.disconnect()
            cap.release()
            return

    # === 扫描阶段 ===
    print("\n=== 开始扫描引脚 ===")
    detected = None

    for idx, (port, pin_name, pin_mask) in enumerate(TEST_PINS):
        ret, frame = cap.read()
        if not ret:
            continue
        frame = cv2.resize(frame, (WINDOW_W, WINDOW_H))

        # set baseline (pin off)
        detector.update(frame)

        # send test command
        ser.test_pin(port, pin_mask)

        # detect flash
        flashed, _, _, change = detector.detect_flash(cap, 15, 100)

        if flashed:
            status = f"CHANGE! {change:.0f}"
            print(f"\n[{idx+1}/{len(TEST_PINS)}] {pin_name} *** 匹配! *** ")
            detected = pin_name
            ret, frame = cap.read()
            frame = cv2.resize(frame, (WINDOW_W, WINDOW_H)) if ret else frame
            draw_ui(frame, detector, ser, idx, pin_name, len(TEST_PINS), status)
            cv2.putText(frame, f">>> FOUND: {pin_name} <<<",
                        (WINDOW_W//2 - 150, WINDOW_H//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            cv2.imshow("Pin Finder", frame)
            cv2.waitKey(3000)
            break
        else:
            status = f"x ({change:.0f})"
            print(f".", end="", flush=True)

        # update UI
        ret, frame = cap.read()
        if ret:
            frame = cv2.resize(frame, (WINDOW_W, WINDOW_H))
        detector.update(frame)
        draw_ui(frame, detector, ser, idx, pin_name, len(TEST_PINS), status)
        cv2.imshow("Pin Finder", frame)

        key = cv2.waitKey(30) & 0xFF
        if key == 27:
            break

    ser.disconnect()
    cap.release()
    cv2.destroyAllWindows()

    if detected:
        print(f"\n=== 结果: 绿灯 = {detected} ===")
    else:
        print("\n=== 未检测到匹配引脚 ===")


def fallback_manual():
    import serial as ser_mod
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        print("无串口")
        return
    print("\n可用串口:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p}")
    try:
        idx = int(input("选择串口号: "))
        ser = ser_mod.Serial(ports[idx], 115200, timeout=2)
        time.sleep(0.5)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"错误: {e}")
        return
    print("\n依次测试, 观察绿灯:")
    for i, (port, pin_name, pin_mask) in enumerate(TEST_PINS):
        cmd = build_test_command(port, pin_mask)
        ser.write(cmd); ser.flush()
        time.sleep(1.5)
        ans = input(f"[{i+1}/{len(TEST_PINS)}] {pin_name} 绿灯? (y/n/q): ")
        if ans == 'y':
            print(f"\n=== 绿灯 = {pin_name} ===")
            break
        if ans == 'q':
            break
    ser.close()


if __name__ == "__main__":
    main()
