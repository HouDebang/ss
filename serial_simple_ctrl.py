import serial
import argparse
import threading
import time

# 用于控制接收线程是否输出
print_lock = threading.Lock()

def read_serial():
    while True:
        with print_lock:
            data = ser.readline().decode('latin-1')
            if data:
                print(f"Received: {data}", end='')

def main():
    global ser
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')

    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, baudrate=115200, dsrdtr=None)
        ser.setRTS(False)
        ser.setDTR(False)
    except serial.SerialException as e:
        print(f"无法打开串口 {args.port}: {e}")
        return

    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()

    try:
        while True:
            with print_lock:
                command = input("请输入 JSON 数据: ")
            ser.write(command.encode() + b'\n')
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()