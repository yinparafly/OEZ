#!/usr/bin/env python3
"""
Windows 端实时查看器
自动刷新显示最新的飞行帧
"""
import os
import time
import glob
import tkinter as tk
from PIL import Image, ImageTk

WATCH_DIR = r"D:\oezcon\ai_mp\flight_frames"
REFRESH_MS = 200  # 5Hz 刷新

class FlightViewer:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ArduPlane Flight Display - Live")
        self.root.configure(bg='#0d1117')

        self.label = tk.Label(self.root, bg='#0d1117')
        self.label.pack()

        self.status = tk.Label(self.root, text="Waiting for frames...",
                              bg='#0d1117', fg='#aaaaaa', font=('Consolas', 10))
        self.status.pack()

        self.last_file = None
        self.update()

    def update(self):
        frames = sorted(glob.glob(os.path.join(WATCH_DIR, "frame_*.png")))
        if frames:
            latest = frames[-1]
            if latest != self.last_file:
                try:
                    img = Image.open(latest)
                    # 缩放到合适大小
                    w, h = img.size
                    max_w = self.root.winfo_screenwidth() - 100
                    max_h = self.root.winfo_screenheight() - 150
                    scale = min(max_w/w, max_h/h, 1.0)
                    img = img.resize((int(w*scale), int(h*scale)), Image.LANCZOS)

                    self.photo = ImageTk.PhotoImage(img)
                    self.label.config(image=self.photo)
                    self.last_file = latest

                    fname = os.path.basename(latest)
                    self.status.config(text=f"Frame: {fname} | Frames: {len(frames)} | Refresh: {REFRESH_MS}ms")
                except Exception as e:
                    self.status.config(text=f"Error: {e}")

        self.root.after(REFRESH_MS, self.update)

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    viewer = FlightViewer()
    viewer.run()
