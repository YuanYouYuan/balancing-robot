import serial
port = '/dev/ttyUSB1'
baudrate = 115200
file_path = '../data/IMU-impact4-data.csv'
s = serial.Serial(port, baudrate, timeout=1)
f = open(file_path, 'w')
while True:
    try:
        data = s.readline().decode()
        f.write(data)
        print(data)
    except KeyboardInterrupt:
        f.close()
        s.close()
        break
