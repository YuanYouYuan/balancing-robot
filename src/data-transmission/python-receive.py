import socket
from struct import unpack

def main():
    host = "192.168.0.103"
    port = 5000

    s = socket.socket()
    s.bind((host, port))

    print("Server is listening at {}, port {}".format(host, port))
    s.listen(1)
    conn, addr = s.accept()
    print("connection from: " + str(addr))

    data = b''
    while True:
        buf = conn.recv(1024)
        data += buf
        if not buf:
            break


    conn.close()

    print(data)

if __name__ == '__main__':
    main()
