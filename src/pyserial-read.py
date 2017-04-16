import serial
port = '/dev/ttyUSB0'
baudrate = 115200
with serial.Serial(port, baudrate, timeout=1) as s:
    try:
        while True:
            data = s.readline().decode()
            print(data)
    except KeyboardInterrupt:
        exit
