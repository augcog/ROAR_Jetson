import socket

if __name__ == "__main__":
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.connect(("192.168.1.7", 9001))
    counter = 0
    while True:
        soc.sendall(f"im here {counter}".encode())
        counter += 1
