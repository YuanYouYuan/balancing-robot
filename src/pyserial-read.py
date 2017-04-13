import serial
port = '/dev/ttyUSB0'
baudrate = 115200
file_path = '../data/IMU-data.csv'
with serial.Serial(port, baudrate, timeout=1) as s:
    with open(file_path, 'w') as f:
        try:
            while True:
                data = s.readline().decode()
                f.write(data)
                print(data)
        except KeyboardInterrupt:
            exit
