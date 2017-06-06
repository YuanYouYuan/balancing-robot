import socket
import numpy as np
from struct import unpack
import time

def main():
    server_address = ('192.168.0.134', 5000)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(server_address)

    print("Server is listening at ", server_address)
    s.listen(1)

    try:
        while True:
            print("waiting for connection")
            conn, addr = s.accept()
            conn.settimeout(1)
            try:
                print("connection from: " + str(addr))
                data_header = b''
                timer = time.time()
                while len(data_header) < 5:
                    if time.time() - timer > 2:
                        timeout = True
                        print("time is out")
                        break
                    else:
                        timeout = False
                        data_header += conn.recv(1)
                if not timeout:
                    data_header = unpack('<ci', data_header)
                    data_type = data_header[0]
                    data_size = data_header[1]
                    data_num = int(data_size / 4)
                    print(data_header)
                    if data_type == b'd':
                        data_value = b''
                        data_chan = 4
                        data_list = int(data_num / 4)
                        while len(data_value) < data_size:
                            data_value += conn.recv(4096)
                        print(len(data_value))
                        data_value = unpack('i'*data_num, data_value)
                        data_value = np.reshape(data_value, (data_list, 4))
                        save_data(data_value)
            except socket.timeout:
                print("timeout!!!!!!!!!!!")
            finally:
                conn.close()
                time.sleep(2)
    except KeyboardInterrupt:
        print('exit')
    finally:
        s.shutdown(socket.SHUT_RDWR)
        s.close()

def save_data(data_value):
    data_value_header = 'angle, angle_rate, angle_acce, power'
    np.savetxt('data.csv', data_value, header=data_value_header, delimiter=',')
    print("data saved")

if __name__ == '__main__':
    main()
