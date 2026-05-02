#!/usr/bin/env python3
"""Simple serial terminal for servo_test.ino"""

import serial
import serial.tools.list_ports
import threading
import time

PORT = "COM6"
BAUD = 9600

MENU = """
Commands:
  b  - sweep base servo (pin 52)
  c  - sweep camera servo (pin 36)
  a  - sweep both
  n  - center both (90 deg)
  0  - move both to 0 deg
  5  - move both to 90 deg
  9  - move both to 180 deg
  q  - quit
"""

def reader(ser):
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print(f"  < {line}")
        except:
            break

def main():
    # List ports and let user pick if COM6 not found
    ports = [p.device for p in serial.tools.list_ports.comports()]
    port = PORT
    if PORT not in ports:
        print(f"Available ports: {ports}")
        port = input(f"COM port [{PORT}]: ").strip() or PORT

    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except Exception as e:
        print(f"Could not open {port}: {e}")
        input("Press Enter to exit...")
        return

    print(f"Connected to {port} at {BAUD} baud")
    print("Waiting for Arduino to boot...")
    time.sleep(2)

    t = threading.Thread(target=reader, args=(ser,), daemon=True)
    t.start()

    print(MENU)

    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == "q":
            break
        elif cmd in ("b", "c", "a", "n", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"):
            ser.write(cmd.encode())
        elif cmd == "":
            pass
        else:
            print("Unknown command. ", end="")
            print(MENU)

    ser.close()
    print("Disconnected.")

if __name__ == "__main__":
    main()
