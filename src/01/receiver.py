import socket
from config import ENDPOINT as _ENDPOINT

if __name__ == "__main__":
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    sock.bind(_ENDPOINT)

    while True:
        data, addr = sock.recvfrom(65535)
        string = data.decode()
        print("{}:{}".format(addr,string))