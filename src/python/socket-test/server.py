import socket
import numpy as np
from struct import unpack
import time

def main():
    server_address = ('127.0.0.1', 5000)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(server_address)

    print("Server is listening at ", server_address)
    s.listen(1)

    try:
        while True:
            print("Waiting for a connection")
            conn, addr = s.accept()
            conn.settimeout(1)
            try:
                print("connection from: " + str(addr))
                data = b''
                while len(data) < 5:
                    data += conn.recv(1)
                data = unpack('<ccccc', data)
                print(data)
            except socket.timeout:
                print("timeout")
            finally:
                conn.close()
                time.sleep(2)
    except KeyboardInterrupt:
        print('exit')
    finally:
        s.shutdown(socket.SHUT_RDWR)
        s.close()

if __name__ == '__main__':
    main()
