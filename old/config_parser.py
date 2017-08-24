import ConfigParser
import PrintConfig


print_config = PrintConfig.PrintConfig()
print_config.show()


config = ConfigParser.ConfigParser()
config.read("../config.conf")
ip = config.get("NETWORK_CONF", "server_ip")
