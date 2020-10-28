import socket

if __name__ == "__main__":
    HOST, PORT = "192.168.1.4", 8000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    counter = 0
    while True:
        sock.sendto(bytes(f"data {counter}" + "\n", "utf-8"), (HOST, PORT))
        received = str(sock.recv(1024), "utf-8")
        print("Received: {}".format(received))
        counter += 1
