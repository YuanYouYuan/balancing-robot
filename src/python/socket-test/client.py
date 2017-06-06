import socket

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('192.168.0.134', 5000)
    print("Connect to localhost {}".format(server_address))
    s.connect(server_address)
    data = input("type something to send:   ")
    s.send(bytes(data, 'utf-8'))
    s.close()

if __name__ == '__main__':
    main()        

