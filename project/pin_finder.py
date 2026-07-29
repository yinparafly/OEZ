"""
Pin Finder: 用摄像头自动识别 STM32 上哪个 GPIO 控制红灯
用法:
  1. 连接 ST-Link (给板子供电)
  2. ST-Link VCP 虚拟串口连接 PC (或外部 USB 转串口)
  3. 摄像头对准板子，确保能看到红灯
  4. 运行: python pin_finder.py
  5. 在视频窗口中点击红灯位置
  6. 脚本自动测试每个引脚，检测红灯变化
"""

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
import sys
import tkinter as tk
from tkinter import ttk, messagebox
from threading import Thread, Event


# 要测试的引脚列表 (port, pin_name, pin_mask)
TEST_PINS = [
    # GPIOA
    ("A", "PA2",  0x0004), ("A", "PA3",  0x0008),
    ("A", "PA8",  0x0100), ("A", "PA11", 0x0800),
    ("A", "PA12", 0x1000), ("A", "PA15", 0x8000),
    # GPIOB
    ("B", "PB1",  0x0002), ("B", "PB2",  0x0004),
    ("B", "PB5",  0x0020), ("B", "PB6",  0x0040),
    ("B", "PB7",  0x0080), ("B", "PB8",  0x0100),
    ("B", "PB9",  0x0200), ("B", "PB12", 0x1000),
    ("B", "PB13", 0x2000), ("B", "PB14", 0x4000),
    ("B", "PB15", 0x8000),
    # GPIOC
    ("C", "PC0",  0x0001), ("C", "PC1",  0x0002),
    ("C", "PC2",  0x0004), ("C", "PC3",  0x0008),
    ("C", "PC4",  0x0010), ("C", "PC5",  0x0020),
    ("C", "PC6",  0x0040), ("C", "PC7",  0x0080),
    ("C", "PC8",  0x0100), ("C", "PC9",  0x0200),
    ("C", "PC10", 0x0400), ("C", "PC11", 0x0800),
    ("C", "PC12", 0x1000), ("C", "PC14", 0x4000),
    ("C", "PC15", 0x8000),
]


def build_test_command(port, pin_mask):
    """构造测试命令: 'T' + port_byte (A=0, B=1, C=2) + pin_mask_hi + pin_mask_lo"""
    port_num = ord(port) - ord('A')
    hi = (pin_mask >> 8) & 0xFF
    lo = pin_mask & 0xFF
    return bytes([ord('T'), port_num, hi, lo])


class SerialController:
    def __init__(self):
        self.ser = None
        self.port = None
        self.connected = False

    def list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port_name, baud=115200):
        self.ser = serial.Serial(port_name, baud, timeout=2)
        self.port = port_name
        self.connected = True
        time.sleep(0.5)  # let STM32 stabilize
        self.ser.reset_input_buffer()
        return True

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.connected = False

    def test_pin(self, port, pin_mask):
        """发送测试命令，等待引脚测试完成 (5次闪烁 ≈ 2s)"""
        if not self.connected:
            return False
        cmd = build_test_command(port, pin_mask)
        try:
            self.ser.write(cmd)
            self.ser.flush()
            # 等待: 串口传输(~1ms) + STM32处理 + 5次闪烁(2000ms) + 安全余量
            time.sleep(2.5)
            return True
        except Exception:
            self.connected = False
            return False


class LEDDetector:
    """用 OpenCV 检测 LED 区域亮度变化"""

    def __init__(self):
        self.roi = None  # (x, y, w, h)
        self.baseline = None
        self.threshold = 25  # 亮度变化阈值

    def select_roi(self, frame):
        """让用户点击选择 LED 区域"""
        print("请在视频窗口中点击红灯位置，然后按 SPACE 确认")
        roi_selector = ROISelector(frame)
        self.roi = roi_selector.run()
        return self.roi is not None

    def measure(self, frame):
        """测量 ROI 区域的红通道亮度"""
        if self.roi is None:
            return 0
        x, y, w, h = self.roi
        roi_frame = frame[y:y+h, x:x+w]
        if roi_frame.size == 0:
            return 0
        red = roi_frame[:, :, 2].mean()
        green = roi_frame[:, :, 1].mean()
        blue = roi_frame[:, :, 0].mean()
        return red - (green + blue) / 2

    def set_baseline(self, frame):
        """设置基准亮度 (拍照 baseline)"""
        self.baseline = self.measure(frame)

    def detect_flash(self, cap, num_frames=15, delay_ms=100):
        """连续拍多帧，检测是否有显著亮度变化（LED闪烁）"""
        if self.baseline is None:
            return False
        values = []
        for _ in range(num_frames):
            ret, frame = cap.read()
            if ret:
                frame = cv2.resize(frame, (960, 540))
                val = self.measure(frame)
                values.append(val)
            time.sleep(delay_ms / 1000.0)

        if not values:
            return False

        max_val = max(values)
        min_val = min(values)
        change = max_val - min_val
        return change > self.threshold, max_val, min_val, change


class ROISelector:
    """鼠标点击选择 ROI"""

    def __init__(self, frame):
        self.frame = frame.copy()
        self.click_point = None
        self.confirmed = False
        self.window_name = "Click on the RED LED, then press SPACE"

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_point = (x, y)
            display = self.frame.copy()
            cv2.circle(display, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(display, f"({x},{y})", (x+10, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(self.window_name, display)

    def run(self):
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        cv2.imshow(self.window_name, self.frame)

        while True:
            key = cv2.waitKey(30) & 0xFF
            if key == 32 and self.click_point:  # SPACE
                self.confirmed = True
                break
            if key == 27:  # ESC
                break

        cv2.destroyWindow(self.window_name)

        if self.confirmed and self.click_point:
            x, y = self.click_point
            return (max(0, x-15), max(0, y-15), 30, 30)  # 30x30 ROI
        return None


class AutoTester:
    def __init__(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera")

        self.detector = LEDDetector()
        self.serial_ctrl = SerialController()
        self.running = False
        self.result = None  # (port, pin_name) that controls the red LED

    def select_serial_port(self):
        ports = self.serial_ctrl.list_ports()
        if not ports:
            print("未发现串口！请连接 ST-Link VCP 或 USB 转串口模块")
            return False

        print("\n可用串口:")
        for i, p in enumerate(ports):
            print(f"  [{i}] {p}")

        try:
            idx = int(input("选择串口号: "))
            port_name = ports[idx]
        except (ValueError, IndexError):
            print("无效选择")
            return False

        print(f"连接 {port_name}...")
        try:
            self.serial_ctrl.connect(port_name)
            print("连接成功")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def run_scan(self):
        """运行全自动引脚扫描"""
        if not self.serial_ctrl.connected:
            print("未连接串口")
            return

        # 先让用户选择 ROI
        ret, frame = self.cap.read()
        if not ret:
            print("无法读取摄像头")
            return

        frame = cv2.resize(frame, (960, 540))
        if not self.detector.select_roi(frame):
            print("未选择 ROI，退出")
            return

        print(f"ROI 已选择: {self.detector.roi}")
        print("\n=== 第一步: 绿灯验证（确认摄像头检测方案可靠）===")

        # 绿灯亮
        self.serial_ctrl.ser.write(b"1"); time.sleep(0.3)
        ret, frame = self.cap.read()
        frame = cv2.resize(frame, (960, 540)) if ret else frame
        if ret:
            val_on = self.detector.measure(frame)
            print(f"  绿灯 ON  亮度={val_on:.1f}")
        # 绿灯灭
        self.serial_ctrl.ser.write(b"2"); time.sleep(0.3)
        ret, frame = self.cap.read()
        frame = cv2.resize(frame, (960, 540)) if ret else frame
        if ret:
            val_off = self.detector.measure(frame)
            print(f"  绿灯 OFF 亮度={val_off:.1f}")
        # 绿灯闪烁（变化间隔）验证 detect_flash
        self.serial_ctrl.ser.write(b"g"); time.sleep(0.1)
        self.detector.set_baseline(frame)
        self.serial_ctrl.ser.write(b"1"); time.sleep(0.05)
        self.serial_ctrl.ser.write(b"2"); time.sleep(0.05)
        self.serial_ctrl.ser.write(b"1"); time.sleep(0.05)
        self.serial_ctrl.ser.write(b"2")
        flashed, _, _, ch = self.detector.detect_flash(self.cap, num_frames=10, delay_ms=80)
        print(f"  绿灯闪烁检测: {'✓ 通过' if flashed else '✗ 未检测到'} (maxΔ={ch:.1f})")

        if not flashed:
            print("\n⚠ 摄像头未检测到绿灯变化！请检查:")
            print("  1. 摄像头是否对准板子")
            print("  2. ROI 是否点击在绿灯上")
            print("  3. 光线是否合适")
            ans = input("  继续扫描红灯? (y/n): ")
            if ans.lower() != 'y':
                return

        print("\n=== 第二步: 红灯引脚扫描 ===")
        self.running = True
        detected_pin = None
        detected_idx = -1

        cv2.namedWindow("Pin Finder")
        cv2.putText(frame, "SCANNING...", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Pin Finder", frame)
        cv2.waitKey(500)

        for idx, (port, pin_name, pin_mask) in enumerate(TEST_PINS):
            if not self.running:
                break

            print(f"\n[{idx+1}/{len(TEST_PINS)}] 测试 {pin_name}...", end="", flush=True)

            # 1. 拍照 baseline (灯灭状态)
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, (960, 540))
                self.detector.set_baseline(frame)

            # 2. 发送串口命令 → STM32 开始闪烁该引脚
            #    注意: 发送后必须等待足够长时间，等 STM32 确实执行完操作
            self.serial_ctrl.test_pin(port, pin_mask)

            # 3. 连续拍多帧，检测闪烁
            #    固件 TestPin: 5次翻转 × (200ms on + 200ms off) = 2000ms
            #    摄像头以 100ms 间隔拍 15 帧 = 1500ms 覆盖期
            if ret:
                flashed, max_v, min_v, change = self.detector.detect_flash(
                    self.cap, num_frames=15, delay_ms=100
                )
                if flashed:
                    print(f"  *** 红灯变化！change={change:.1f}, pin={pin_name} ***")
                    detected_pin = pin_name
                    detected_idx = idx

                    display = frame.copy()
                    cv2.putText(display, f"FOUND: {pin_name}",
                                (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                1.5, (0, 0, 255), 3)
                    cv2.rectangle(display,
                                  (self.detector.roi[0], self.detector.roi[1]),
                                  (self.detector.roi[0] + self.detector.roi[2],
                                   self.detector.roi[1] + self.detector.roi[3]),
                                  (0, 255, 0), 2)
                    cv2.imshow("Pin Finder", display)
                    cv2.waitKey(2000)
                    break
                else:
                    print(f" 无变化 (maxΔ={change:.1f})")
            else:
                print(" 摄像头错误")

            # 显示进度
            cv2.putText(frame, f"[{idx+1}/{len(TEST_PINS)}] {pin_name}",
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2)
            roi = self.detector.roi
            cv2.rectangle(frame, (roi[0], roi[1]),
                          (roi[0]+roi[2], roi[1]+roi[3]),
                          (0, 255, 0), 1)
            cv2.imshow("Pin Finder", frame)
            cv2.waitKey(100)

        self.running = False

        if detected_pin:
            self.result = detected_pin
            print(f"\n=== 结果: 红灯控制引脚 = {detected_pin} ===")
        else:
            print("\n=== 未检测到红灯变化 ===")
            print("可能原因: 红灯是电源指示灯(不可控), 或摄像头角度不对")

        cv2.destroyWindow("Pin Finder")

    def cleanup(self):
        self.running = False
        self.serial_ctrl.disconnect()
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


def main():
    print("=" * 50)
    print("STM32 Pin Finder - 用摄像头自动识别 LED 控制引脚")
    print("=" * 50)

    try:
        tester = AutoTester(0)
    except RuntimeError as e:
        print(f"错误: {e}")
        print("请确保摄像头已连接")
        # Fallback: 无摄像头模式，通过串口读数
        print("\n无摄像头模式: 输入引脚号手动测试")
        fallback_manual()
        return

    if not tester.select_serial_port():
        tester.cleanup()
        return

    tester.run_scan()

    if tester.result:
        print(f"\n红灯控制引脚: {tester.result}")
        print("请告诉我引脚号，我更新固件代码。")

    tester.cleanup()


def fallback_manual():
    """无摄像头时的手动测试模式"""
    import serial

    # 找串口
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        print("无串口可用，退出")
        return

    print("\n可用串口:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p}")
    try:
        idx = int(input("选择串口号: "))
        ser = serial.Serial(ports[idx], 115200, timeout=2)
        time.sleep(0.5)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"串口错误: {e}")
        return

    print("\n依次测试引脚... 观察红灯是否闪烁")
    for i, (port, pin_name, pin_mask) in enumerate(TEST_PINS):
        cmd = build_test_command(port, pin_mask)
        ser.write(cmd)
        time.sleep(1.5)
        ans = input(f"[{i+1}/{len(TEST_PINS)}] {pin_name} 红灯闪了吗? (y/n/q): ")
        if ans.lower() == 'y':
            print(f"\n=== 结果: 红灯 = {pin_name} ===")
            break
        if ans.lower() == 'q':
            break

    ser.close()


if __name__ == "__main__":
    main()
