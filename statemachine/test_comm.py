from comm import *


def main():

    ip = 192.168.5
    port = 5015


    comm = Communicaton(port, ip)

    comm.run()




if __name__ == "__main__":
    main()
