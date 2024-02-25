import serial
import time

# Serial port parameters
port = '/dev/ttyUSB0'   # Depends on the OS
baud = 115200
timeout = 3

# Open te serial port
ser = serial.Serial(port, baud, timeout=timeout)
ser.flush()

i=1
while(True):
    angle = i * 1.5
    message = "M" + str(angle) + "\n"
    ser.write(message.encode())
    print(message)
    i = i * -1
    time.sleep(2)

ser.close()