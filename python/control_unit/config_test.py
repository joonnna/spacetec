import ConfigParser

config = ConfigParser.ConfigParser()

config.read("../../configs/test_config.ini")

sig_lim = config.getfloat("SIGNAL", "SIGNAL_LIMIT")
print sig_lim
