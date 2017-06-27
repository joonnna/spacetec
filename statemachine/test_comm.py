from comm import *

def main():


    ip = "192.168.5.4"
    port = 5632


    comm = Communication(port, ip)

    comm.run()




if __name__ == "__main__":
    main()
