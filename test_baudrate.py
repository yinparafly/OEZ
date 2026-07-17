import serial
import time

# 测试多个波特率
baudrates = [115200, 57600, 921600, 230400]

for port in ['COM15', 'COM16']:
    for baud in baudrates:
        try:
            s = serial.Serial(port, baud, timeout=2)
            data = s.read(50)
            s.close()
            if len(data) > 0 and data[0] == 0xfd:  # MAVLink v2 marker
                print(f'{port} @ {baud}: MAVLink data found! ({len(data)} bytes)')
            elif len(data) > 0:
                print(f'{port} @ {baud}: Data ({len(data)} bytes) but not MAVLink: {data[:10]}')
            else:
                print(f'{port} @ {baud}: No data')
        except Exception as e:
            print(f'{port} @ {baud}: Error - {e}')
        time.sleep(0.3)
