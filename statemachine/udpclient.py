import socket


class Udpclient():
    def __init__(self, port, ip):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port = port
        self.ip = ip

    def send_msg(self, msg):
        self.socket.sendto(msg, (self.ip, self.port))



def main():
    ip = "192.168.5.4"
    port = 5632

    msg = "yoyoyoo bitches"

    client = Udpclient(port, ip)

    client.send_msg(msg)


if __name__ == "__main__":
    main()
