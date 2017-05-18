import socket
from struct import unpack

def main():
    server_address = ('192.168.0.103', 5001)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(server_address)

    print("Server is listening at ", server_address)
    s.listen(1)

    try:
        while True:
            conn, addr = s.accept()
            print("connection from: " + str(addr))
            try:
                while True:
                    buf = conn.recv(1)
                    print(buf)
                    if not buf:
                        break
            finally:
                conn.close()
    except KeyboardInterrupt:
        print('exit')
    finally:
        s.shutdown(socket.SHUT_RDWR)
        s.close()

if __name__ == '__main__':
    main()
