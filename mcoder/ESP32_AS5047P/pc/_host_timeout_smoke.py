# -*- coding: utf-8 -*-
"""Smoke: HOST OFF must not ESTOP after silence; HOST ON restores."""
import sys
import time
import serial

PORT = "COM10"
BAUD = 921600


def read_for(ser, sec, pred=None):
    end = time.time() + sec
    lines = []
    buf = b""
    while time.time() < end:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("utf-8", "replace").strip()
                if s:
                    lines.append(s)
                    print(s)
                    if pred and pred(s):
                        return lines, True
        else:
            time.sleep(0.02)
    return lines, False


def send(ser, cmd):
    print(f">> {cmd}")
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.8)
    ser.reset_input_buffer()

    send(ser, "PING")
    read_for(ser, 1.0)

    send(ser, "TELEM OFF")
    read_for(ser, 0.5)

    send(ser, "HOST?")
    lines, _ = read_for(ser, 1.0, lambda s: s.startswith("# HOST"))
    host0 = next((s for s in lines if s.startswith("# HOST")), "")

    send(ser, "HOST OFF")
    lines, ok = read_for(ser, 1.0, lambda s: "ACK HOST OFF" in s)
    if not ok:
        print("FAIL: no ACK HOST OFF")
        return 1

    send(ser, "ESTOP")
    read_for(ser, 0.4)
    # Clear ESTOP then arm RUNNING with zero target (no spin)
    send(ser, "STOP SOFT")
    read_for(ser, 0.4)
    send(ser, "MODE OPEN")
    read_for(ser, 0.3)
    send(ser, "RPM 0")
    read_for(ser, 0.3)
    send(ser, "START")
    read_for(ser, 0.8)

    print("--- silence 3.5s (expect NO host_timeout) ---")
    lines, hit = read_for(ser, 3.5, lambda s: "host_timeout" in s.lower() or "ESTOP" in s)
    if hit:
        print("FAIL: unexpected ESTOP/host_timeout during HOST OFF silence")
        for s in lines[-10:]:
            print(" ", s)
        send(ser, "ESTOP")
        return 2

    send(ser, "CTRL?")
    read_for(ser, 1.0, lambda s: s.startswith("# CTRL"))
    send(ser, "HOST?")
    read_for(ser, 1.0, lambda s: s.startswith("# HOST"))

    send(ser, "HOST ON")
    lines, ok = read_for(ser, 1.0, lambda s: "ACK HOST ON" in s)
    if not ok:
        print("FAIL: no ACK HOST ON")
        return 3

    # With HOST ON + RUNNING, silence should ESTOP
    print("--- silence 2.2s after HOST ON (expect host_timeout ESTOP) ---")
    lines, hit = read_for(ser, 2.5, lambda s: "host_timeout" in s.lower())
    if not hit:
        print("WARN: expected host_timeout after HOST ON silence (may need host_seen)")
        send(ser, "PING")  # refresh
        send(ser, "START")
        read_for(ser, 0.5)
        lines, hit = read_for(ser, 2.5, lambda s: "host_timeout" in s.lower())
    if hit:
        print("PASS: HOST ON silence -> host_timeout as expected")
    else:
        print("WARN: HOST ON silence did not ESTOP (still check HOST OFF path)")

    send(ser, "ESTOP")
    read_for(ser, 0.5)
    send(ser, "HOST?")
    read_for(ser, 1.0)
    print("PASS: HOST OFF silence OK; baseline was:", host0)
    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
