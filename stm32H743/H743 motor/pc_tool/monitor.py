"""
ABI 监控器 PC 端监视工具（串口交互）
用法:  python monitor.py [COM口] [波特率]
默认:  921600 / COM7
- 实时打印固件回显（RPM ON 每秒回显 rpm/cnt/idx/div）
- 键盘输入命令直接发送给固件（CNT / RPM ON / ID / HELP ...）
"""
import sys
import threading
import time
import serial

BAUD = 921600


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    if len(sys.argv) > 2:
        global BAUD
        BAUD = int(sys.argv[2])

    ser = serial.Serial(timeout=0.1)
    ser.port = port
    ser.baudrate = BAUD
    ser.bytesize = 8
    ser.parity = "N"
    ser.stopbits = 1
    try:
        ser.open()
    except Exception as e:
        print(f"[错误] 无法打开 {port}: {e}")
        sys.exit(1)

    print(f"已连接 {port} @ {BAUD}  输入命令回车发送，Ctrl+C 退出")

    def reader():
        try:
            while True:
                data = ser.read(256)
                if data:
                    sys.stdout.write(data.decode("utf-8", "replace"))
                    sys.stdout.flush()
        except serial.SerialException:
            pass

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    try:
        while True:
            line = input()
            ser.write((line + "\r\n").encode())
    except (KeyboardInterrupt, EOFError):
        print("\n退出")
    finally:
        ser.close()


if __name__ == "__main__":
    main()