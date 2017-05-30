from struct import unpack
import socket

def main():
    host = "192.168.0.134"
    port = 5000
    s = socket.socket()
    s.bind((host, port))
    print("Server is listening at {}, port {}".format(host, port))
    s.listen(1)
    conn, addr = s.accept()
    print("connection from: " + str(addr))

    #while True:
    #    data += conn.recv(1024);
    #    if not data:
    #        break
    data = conn.recv(1024)
    print(len(data))
    data = unpack('ffffff', data)
    print(data)

if __name__ == '__main__':
    main()


    
