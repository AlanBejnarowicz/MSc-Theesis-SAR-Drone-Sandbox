# ais_forwarder.py  — runs on RPi Zero
import serial
import socket
import time

AIS_PORT   = "/dev/ttyACM0"   # or /dev/ttyUSB0 — check with: ls /dev/tty*
AIS_BAUD   = 38400
PC_HOST    = "localhost"  # your PC's local IP
PC_PORT    = 5001

def connect_tcp():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((PC_HOST, PC_PORT))
            print(f"Connected to PC {PC_HOST}:{PC_PORT}")
            return s
        except Exception as e:
            print(f"TCP connect failed: {e}, retrying in 5s...")
            time.sleep(5)

def main():
    sock = connect_tcp()
    with serial.Serial(AIS_PORT, AIS_BAUD, timeout=2) as ser:
        while True:
            try:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if line.startswith("!AIVDM") or line.startswith("!AIVDO"):
                    sock.sendall((line + "\n").encode())
            except (socket.error, serial.SerialException) as e:
                print(f"Error: {e} — reconnecting...")
                sock = connect_tcp()

if __name__ == "__main__":
    main()
