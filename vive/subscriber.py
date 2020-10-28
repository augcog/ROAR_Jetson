import socket
if __name__ == "__main__":
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.bind(("192.168.1.5", 9001))
    while True:
        try:
            msg, addr = soc.recvfrom(1024)
            msg = msg.decode()
            print(msg)
        except Exception as e:
            print(e)
