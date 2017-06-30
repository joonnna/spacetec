import socket
import time

class Udpclient():
    def __init__(self, port, ip):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port = port
        self.ip = ip

    def send_msg(self, msg):
        self.socket.sendto(msg, (self.ip, self.port))



def main():
    f = open("ptudata", "r")
    data = f.read()
    lines = data.split("\n")
    """
    test = lines[2]
    print str(test[172:181])
    print len(test)
    """
    #ip = "192.168.5.4"
    ip = "127.0.0.1"
    port = 5632

    msg = "yoyoyoo bitches"

    client = Udpclient(port, ip)

    for line in lines:
        if len(line) <= 1:
            continue
        client.send_msg(line)
        time.sleep(0.001)
    print "Done sending"

if __name__ == "__main__":
    main()
