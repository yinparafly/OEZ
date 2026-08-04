#!/usr/bin/env python3
"""采集 DUMP BIN 数据、自动清洗 L 遥测帧、保存纯 BIN 文件。

用法：
  python capture_clean_bin.py COM23 921600 snap_001.bin

数据格式（C2000 v3 20B）：Preamble(11B) + Magic(4B LE) + n(2B) + hz(2B) + 点阵(n×20B) + CRC(4B) + "# BIN END"
"""
import serial, sys, time

def capture(port: str, baud: int, out_path: str):
    s = serial.Serial(port, baud, timeout=30.0)
    s.write(b'DUMP BIN\r\n')
    time.sleep(0.3)
    raw = b''
    # 读取直到收到 BIN END 标记
    while True:
        chunk = s.read(4096)
        if not chunk:
            break
        raw += chunk
        if b'BIN END' in raw:
            break
    # 清洗：查找 preamble 签名 (10*AA + 55)
    pre = b'\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\xaa\x55'
    start = raw.find(pre)
    if start < 0:
        print(f'ERROR: preamble not found in {len(raw)} bytes')
        with open(out_path, 'wb') as f: f.write(raw)
        return
    end = raw.find(b'# BIN END', start)
    if end < 0:
        print(f'ERROR: BIN END not found')
        return
    end += len(b'# BIN END')
    clean = raw[start:end]
    # 找 "# BIN END N" 中的 N（确认点数）
    bine = clean.rfind(b'# BIN END ')
    n_str = clean[bine+9:end].decode('ascii','ignore').strip()
    size = len(clean)
    print(f'Captured {len(raw)}B → clean {size}B  n={n_str}  saved → {out_path}')
    with open(out_path, 'wb') as f:
        f.write(clean)

if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM23'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
    out  = sys.argv[3] if len(sys.argv) > 3 else 'snap_clean_auto.bin'
    capture(port, baud, out)
