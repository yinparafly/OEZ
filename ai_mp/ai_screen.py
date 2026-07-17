"""
AI Screen Control - 屏幕识别+鼠标控制工具
使用 Windows 原生 API，无需额外依赖

功能:
1. 截取屏幕 → 保存为图片 → AI 读取分析
2. 鼠标点击指定位置
3. 键盘输入
4. 查找屏幕上的文字/按钮位置

使用:
  python ai_screen.py screenshot              # 截取全屏
  python ai_screen.py screenshot region x y w h  # 截取区域
  python ai_screen.py click x y               # 点击坐标
  python ai_screen.py doubleclick x y         # 双击
  python ai_screen.py type "text"             # 输入文字
  python ai_screen.py find_window "Mission Planner"  # 查找窗口
  python ai_screen.py activate "Mission Planner"     # 激活窗口
  python ai_screen.py list_windows            # 列出所有窗口
"""

import sys
import os
import time
import ctypes
import ctypes.wintypes
import struct

# Windows API constants
SW_RESTORE = 9
SW_SHOW = 5
HWND_TOP = 0
SWP_NOMOVE = 0x0002
SWP_NOSIZE = 0x0001

user32 = ctypes.windll.user32
kernel32 = ctypes.windll.kernel32
gdi32 = ctypes.windll.gdi32

OUTPUT_DIR = "D:/oezcon/ai_mp/screenshots"
os.makedirs(OUTPUT_DIR, exist_ok=True)


class RECT(ctypes.Structure):
    _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long),
                ("right", ctypes.c_long), ("bottom", ctypes.c_long)]


def screenshot(filename=None, region=None):
    """截取屏幕 (使用 mss)"""
    import mss
    import mss.tools
    
    if filename is None:
        filename = f"screenshot_{time.strftime('%Y%m%d_%H%M%S')}.png"
    filepath = os.path.join(OUTPUT_DIR, filename)
    
    with mss.MSS() as sct:
        if region:
            x, y, w, h = region
            monitor = {"left": x, "top": y, "width": w, "height": h}
        else:
            monitor = sct.monitors[1]  # Primary monitor
        
        img = sct.grab(monitor)
        mss.tools.to_png(img.rgb, img.size, output=filepath)
        print(f"Screenshot saved: {filepath} ({img.width}x{img.height})")
    
    return filepath


def click(x, y, button='left'):
    """点击指定位置"""
    if button == 'right':
        flag = 0x0002  # MOUSEEVENTF_RIGHTDOWN
        flag_up = 0x0004  # MOUSEEVENTF_RIGHTUP
    else:
        flag = 0x0002  # MOUSEEVENTF_LEFTDOWN
        flag_up = 0x0004  # MOUSEEVENTF_LEFTUP
    
    user32.SetCursorPos(x, y)
    time.sleep(0.05)
    user32.mouse_event(flag, 0, 0, 0, 0)
    time.sleep(0.05)
    user32.mouse_event(flag_up, 0, 0, 0, 0)
    print(f"Clicked ({x}, {y}) [{button}]")


def doubleclick(x, y):
    """双击"""
    click(x, y)
    time.sleep(0.05)
    click(x, y)
    print(f"Double-clicked ({x}, {y})")


def type_text(text):
    """输入文字"""
    for char in text:
        # 使用 SendInput
        ctypes.windll.user32.SendInput(1, ctypes.byref(
            ctypes.Structure('_KEYBDINPUT', [
                ('wVk', ctypes.c_ushort, 0),
                ('wScan', ctypes.c_ushort, ord(char)),
                ('dwFlags', ctypes.c_ulong, 0x0004),  # KEYEVENTF_UNICODE
                ('time', ctypes.c_ulong, 0),
                ('dwExtraInfo', ctypes.POINTER(ctypes.c_ulong), 0)
            ])
        ), ctypes.sizeof(ctypes.Structure))
        time.sleep(0.01)
    print(f"Typed: {text}")


def find_window(title):
    """查找窗口"""
    result = []
    
    @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.wintypes.HWND, ctypes.wintypes.LPARAM)
    def callback(hwnd, lparam):
        if user32.IsWindowVisible(hwnd):
            length = user32.GetWindowTextLengthW(hwnd)
            if length > 0:
                buf = ctypes.create_unicode_buffer(length + 1)
                user32.GetWindowTextW(hwnd, buf, length + 1)
                if title.lower() in buf.value.lower():
                    result.append((hwnd, buf.value))
        return True
    
    user32.EnumWindows(callback, 0)
    return result


def activate_window(title):
    """激活窗口"""
    windows = find_window(title)
    if windows:
        hwnd, name = windows[0]
        user32.SetForegroundWindow(hwnd)
        user32.ShowWindow(hwnd, SW_RESTORE)
        print(f"Activated: {name} (hwnd={hwnd})")
        return hwnd
    else:
        print(f"Window not found: {title}")
        return None


def list_windows():
    """列出所有可见窗口"""
    result = []
    
    @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.wintypes.HWND, ctypes.wintypes.LPARAM)
    def callback(hwnd, lparam):
        if user32.IsWindowVisible(hwnd):
            length = user32.GetWindowTextLengthW(hwnd)
            if length > 0:
                buf = ctypes.create_unicode_buffer(length + 1)
                user32.GetWindowTextW(hwnd, buf, length + 1)
                rect = RECT()
                user32.GetWindowRect(hwnd, ctypes.byref(rect))
                result.append({
                    'hwnd': hwnd,
                    'title': buf.value,
                    'rect': (rect.left, rect.top, rect.right, rect.bottom),
                })
        return True
    
    user32.EnumWindows(callback, 0)
    
    print(f"\n{'='*70}")
    print(f"  Windows ({len(result)} visible)")
    print(f"{'='*70}")
    for w in result:
        r = w['rect']
        print(f"  {w['hwnd']:>10}  {w['title'][:50]:<50}  ({r[0]},{r[1]})-({r[2]},{r[3]})")
    print(f"{'='*70}")
    return result


def click_window_center(title):
    """点击窗口中心"""
    windows = find_window(title)
    if windows:
        hwnd, name = windows[0]
        rect = RECT()
        user32.GetWindowRect(hwnd, ctypes.byref(rect))
        cx = (rect.left + rect.right) // 2
        cy = (rect.top + rect.bottom) // 2
        click(cx, cy)
        print("Clicked center of '%s' (%d, %d)" % (name, cx, cy))
        return (cx, cy)
    else:
        print(f"Window not found: {title}")
        return None


# ============================================================
# Main
# ============================================================
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)
    
    cmd = sys.argv[1].lower()
    
    if cmd == 'screenshot':
        region = None
        if len(sys.argv) >= 6:
            region = (int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]))
        screenshot(region=region)
    
    elif cmd == 'click':
        x, y = int(sys.argv[2]), int(sys.argv[3])
        button = sys.argv[4] if len(sys.argv) > 4 else 'left'
        click(x, y, button)
    
    elif cmd == 'doubleclick':
        x, y = int(sys.argv[2]), int(sys.argv[3])
        doubleclick(x, y)
    
    elif cmd == 'type':
        text = ' '.join(sys.argv[2:])
        type_text(text)
    
    elif cmd == 'find_window':
        title = ' '.join(sys.argv[2:])
        find_window(title)
    
    elif cmd == 'activate':
        title = ' '.join(sys.argv[2:])
        activate_window(title)
    
    elif cmd == 'list_windows':
        list_windows()
    
    elif cmd == 'click_center':
        title = ' '.join(sys.argv[2:])
        click_window_center(title)
    
    else:
        print(f"Unknown command: {cmd}")
        print(__doc__)
