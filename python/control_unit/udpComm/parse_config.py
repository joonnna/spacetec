import ConfigParser


def read_comm_config():
    config = ConfigParser.ConfigParser()
    config.read("/home/machinekit/machinekit/spacetec/configs/test_config.ini")
    dict = {}

    dict["ip"] = config.get("NETWORK", "IP")
    dict["port"] = config.getint("NETWORK", "PORT")

    dict["long"] = config.getfloat("GPS_CORDINATES", "LONGTITUDE")
    dict["lat"] = config.getfloat("GPS_CORDINATES", "LATITUDE")
    dict["height"] = config.getfloat("GPS_CORDINATES", "HEIGHT")

    return dict
