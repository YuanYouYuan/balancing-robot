import socket

def main():
    host = "192.168.0.198"
    port = 5000

    s = socket.socket()
    s.bind((host, port))

    print("Server is listening at {}, port {}".format(host, port))
    s.listen(1)
    conn, addr = s.accept()
    print("connection from: " + str(addr))
    while True:
        data = conn.recv(1024).decode()
        if not data:
            break;
        print("from connected user: " +str(data))
        data = str(data).upper()
        print("sending: " + str(data))
        conn.send(data.encode())
    conn.close()

if __name__ == '__main__':
    main()
