"""
摄像头扫描仪 - 纯视觉版本 (不需要串口)
=====================================
固件自动运行，摄像头同时观看绿灯和红灯：
  1. 绿灯闪 XX 次 → 暂停 → YY 次 (两位编码 = 引脚号)
  2. 然后被测引脚开始闪烁 (500→100ms 递减)
  3. 如果红灯也跟着闪，说明这个引脚控制红灯

运行: python cam_scanner.py
"""

import cv2
import numpy as np
import time


class LEDTracker:
    def __init__(self):
        self.roi_green = None
        self.roi_red = None
        self.threshold = 25

    def select_rois(self, frame):
        """让用户依次点击绿灯和红灯位置"""
        frame_h, frame_w = frame.shape[:2]
        display = cv2.resize(frame, (960, 540))

        print("\n=== 请依次点击两个LED ===")

        # 选绿灯
        cv2.namedWindow("Click GREEN LED then SPACE")
        cv2.setMouseCallback("Click GREEN LED then SPACE",
                             lambda e, x, y, f, p: self._click_cb(e, x, y, f, p, "green"))
        cv2.imshow("Click GREEN LED then SPACE", display)
        self._click_done = False
        self._click_pos = None
        while True:
            k = cv2.waitKey(30) & 0xFF
            if k == 32 and self._click_pos:
                self._click_done = True
                break
            if k == 27:
                cv2.destroyAllWindows()
                return False
        cv2.destroyWindow("Click GREEN LED then SPACE")
        x, y = self._click_pos
        self.roi_green = (max(0, x-15), max(0, y-15), 40, 40)

        # 选红灯
        cv2.namedWindow("Click RED LED then SPACE")
        cv2.setMouseCallback("Click RED LED then SPACE",
                             lambda e, x, y, f, p: self._click_cb(e, x, y, f, p, "red"))
        cv2.imshow("Click RED LED then SPACE", display)
        self._click_done = False
        self._click_pos = None
        while True:
            k = cv2.waitKey(30) & 0xFF
            if k == 32 and self._click_pos:
                self._click_done = True
                break
            if k == 27:
                cv2.destroyAllWindows()
                return False
        cv2.destroyWindow("Click RED LED then SPACE")
        x, y = self._click_pos
        self.roi_red = (max(0, x-15), max(0, y-15), 40, 40)

        return True

    def _click_cb(self, event, x, y, flags, param, led_name):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._click_pos = (x, y)
            self._click_done = True

    def measure_green(self, frame):
        if self.roi_green is None:
            return 0
        x, y, w, h = self.roi_green
        roi = frame[y:y+h, x:x+w]
        if roi.size == 0:
            return 0
        g = roi[:, :, 1].mean()
        r = roi[:, :, 2].mean()
        b = roi[:, :, 0].mean()
        return g - (r + b) / 2

    def measure_red(self, frame):
        if self.roi_red is None:
            return 0
        x, y, w, h = self.roi_red
        roi = frame[y:y+h, x:x+w]
        if roi.size == 0:
            return 0
        r = roi[:, :, 2].mean()
        g = roi[:, :, 1].mean()
        b = roi[:, :, 0].mean()
        return r - (g + b) / 2

    def is_on(self, frame, led):
        """检测 LED 是否亮着"""
        val = self.measure_green(frame) if led == "green" else self.measure_red(frame)
        return val > self.threshold


class BlinkDecoder:
    """解码绿灯闪烁编码"""

    def __init__(self):
        self.state = "IDLE"
        self.tens = 0
        self.ones = 0
        self.last_on = False
        self.blink_count = 0
        self.in_blink = False
        self.blink_start = 0
        self.digit = 0
        self.decoded_number = None

    def feed(self, frame, tracker):
        is_on = tracker.is_on(frame, "green")
        now = time.time()

        if self.state == "IDLE":
            if is_on:
                self.state = "TENS"
                self.blink_count = 0
                self.in_blink = True
                self.blink_start = now
                self.last_on = True

        elif self.state == "TENS":
            if is_on and not self.last_on:
                self.blink_count += 1
            elif not is_on and self.last_on:
                pass
            self.last_on = is_on

            if not is_on and (now - self.blink_start > 2.0):
                if self.blink_count > 0:
                    self.tens = self.blink_count
                else:
                    self.tens = 0
                self.state = "DIGIT_GAP"
                self.blink_start = now

        elif self.state == "DIGIT_GAP":
            if is_on and (now - self.blink_start > 0.4):
                self.state = "ONES"
                self.blink_count = 0
                self.in_blink = True
                self.last_on = True

        elif self.state == "ONES":
            if is_on and not self.last_on:
                self.blink_count += 1
            self.last_on = is_on

            if not is_on and (now - self.blink_start > 2.0):
                self.ones = self.blink_count
                self.decoded_number = self.tens * 10 + self.ones
                self.state = "DONE"

        elif self.state == "DONE":
            if not is_on and (now - self.blink_start > 3.0):
                self.state = "IDLE"
                return self.decoded_number

        return None


class RedDetector:
    def __init__(self):
        self.baseline = None
        self.changed = False
        self.last_change_val = 0

    def set_baseline(self, frame, tracker):
        self.baseline = tracker.measure_red(frame)

    def check(self, frame, tracker):
        val = tracker.measure_red(frame)
        if self.baseline is None:
            self.baseline = val
            return False
        diff = abs(val - self.baseline)
        if diff > 20:
            self.changed = True
            self.last_change_val = diff
            return True
        return False

    def reset(self, frame, tracker):
        self.baseline = tracker.measure_red(frame)
        self.changed = False


def main():
    print("=" * 50)
    print("STM32 Pin Finder - 纯视觉模式")
    print("=" * 50)
    print("无需串口，摄像头自动识别")
    print("\n操作步骤:")
    print("  1. 摄像头对准板子 (确保能看到绿灯和红灯)")
    print("  2. 按 Reset 或重新上电")
    print("  3. 在弹出的窗口中依次点击绿灯/红灯位置")
    print("  4. 程序自动解码引脚编号，检测红灯变化")
    print()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("错误: 无法打开摄像头")
        return

    ret, frame = cap.read()
    if not ret:
        print("错误: 无法读取摄像头画面")
        cap.release()
        return

    frame = cv2.resize(frame, (960, 540))

    tracker = LEDTracker()
    if not tracker.select_rois(frame):
        print("未选择 ROI，退出")
        cap.release()
        return

    print(f"\n绿灯 ROI: {tracker.roi_green}")
    print(f"红灯 ROI: {tracker.roi_red}")
    print("\n== 开始监测，请按 Reset... ==")

    decoder = BlinkDecoder()
    red_detector = RedDetector()
    frame_count = 0
    last_pin = -1

    cv2.namedWindow("Pin Scanner")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (960, 540))
        frame_count += 1

        cv2.rectangle(frame,
                      (tracker.roi_green[0], tracker.roi_green[1]),
                      (tracker.roi_green[0]+tracker.roi_green[2],
                       tracker.roi_green[1]+tracker.roi_green[3]),
                      (0, 255, 0), 2)
        cv2.putText(frame, "GREEN", (tracker.roi_green[0], tracker.roi_green[1]-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.rectangle(frame,
                      (tracker.roi_red[0], tracker.roi_red[1]),
                      (tracker.roi_red[0]+tracker.roi_red[2],
                       tracker.roi_red[1]+tracker.roi_red[3]),
                      (0, 0, 255), 2)
        cv2.putText(frame, "RED", (tracker.roi_red[0], tracker.roi_red[1]-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        green_val = tracker.measure_green(frame)
        red_val = tracker.measure_red(frame)
        green_on = tracker.is_on(frame, "green")
        red_on = tracker.is_on(frame, "red")

        pin_num = decoder.feed(frame, tracker)
        if pin_num is not None:
            print(f"  [解码] 引脚 #{pin_num}")
            red_detector.reset(frame, tracker)
            last_pin = pin_num

        if red_detector.check(frame, tracker):
            print(f"  *** 红灯变化！当前测试引脚 = #{last_pin} ***")
            cv2.putText(frame, f"!!! RED CHANGED at PIN #{last_pin} !!!",
                        (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        cv2.putText(frame, f"Green: {green_val:.0f} ({'ON' if green_on else 'OFF'})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Red: {red_val:.0f} ({'ON' if red_on else 'OFF'})",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, f"State: {decoder.state} Pin: {decoder.decoded_number}",
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"T:{decoder.tens} O:{decoder.ones} Blinks:{decoder.blink_count}",
                    (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("Pin Scanner", frame)

        key = cv2.waitKey(30) & 0xFF
        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

    if last_pin >= 0:
        print(f"\n=== 结果: 红灯由引脚 #{last_pin} 控制 ===")


if __name__ == "__main__":
    main()
