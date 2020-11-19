import socket


def main():
    # Set up socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass  # Some systems don't support SO_REUSEPORT
    s.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_TTL, 20)
    s.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_LOOP, 1)

    addr = "192.168.42.7"
    port = 8000
    # Bind to the port
    s.bind(('', port))
    intf = socket.gethostbyname(socket.gethostname())
    s.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(intf))
    # s.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(addr) + socket.inet_aton(intf))

    while True:
        data, sender_addr = s.recvfrom(1024)
        # s.setsockopt(socket.SOL_IP, socket.IP_DROP_MEMBERSHIP, socket.inet_aton(addr) + socket.inet_aton('0.0.0.0'))
        print(data)


if __name__ == "__main__":
    main()



# import socket
#
#
# def main():
#     # Set up socket
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     s.bind(('192.168.42.7', 8000))
#
#     while True:
#         data, addr = s.recvfrom(1024)
#         print(data)
#
#
# if __name__ == "__main__":
#     main()