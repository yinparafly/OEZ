import serial
import time

for port in ['COM15', 'COM16']:
    try:
        s = serial.Serial(port, 115200, timeout=2)
        data = s.read(100)
        s.close()
        if len(data) > 0:
            print(f'{port}: Got {len(data)} bytes - {data[:20]}')
        else:
            print(f'{port}: No data (timeout)')
    except Exception as e:
        print(f'{port}: Error - {e}')
    time.sleep(0.5)
