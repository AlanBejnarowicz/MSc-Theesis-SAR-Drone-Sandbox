# ais_forwarder.py  — runs on RPi Zero / Orange Pi
import serial
import socket
import time

AIS_PORT = "/dev/ais"
AIS_BAUD = 38400
PC_HOST  = "localhost"
PC_PORT  = 5001

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

def connect_serial():
    while True:
        try:
            ser = serial.Serial(AIS_PORT, AIS_BAUD, timeout=2)
            print(f"Serial opened: {AIS_PORT}")
            return ser
        except serial.SerialException as e:
            print(f"Serial connect failed: {e}, retrying in 5s...")
            time.sleep(5)

def main():
    sock = connect_tcp()
    ser  = connect_serial()

    while True:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("!AIVDM") or line.startswith("!AIVDO"):
                sock.sendall((line + "\n").encode())

        except serial.SerialException as e:
            print(f"Serial error: {e} — reconnecting in 5s...")
            time.sleep(5)
            try:
                ser.close()
            except:
                pass
            ser = connect_serial()

        except socket.error as e:
            print(f"TCP error: {e} — reconnecting in 5s...")
            time.sleep(5)
            sock = connect_tcp()

if __name__ == "__main__":
    main()