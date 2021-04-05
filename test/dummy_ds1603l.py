import time
import serial

if __name__ == '__main__':
    with serial.Serial('/dev/ttyUSB1', 9600) as ser:
        for i in range(150, 200, 1):
            ser.write([0xff, 0x00, i, i-1])
            time.sleep(1)
