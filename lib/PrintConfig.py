import ConfigParser


class PrintConfig(object):
    """docstring for PrintConfig"""

    def __init__(self, path):
        super(PrintConfig, self).__init__()
        self.config = ConfigParser.ConfigParser()
        self.config.read(path)

    def show(self):
        for x in self.config.sections():
            print(x)
            print(self.config.items(x))
            print("")


def main():
    config_printer = PrintConfig("../config.conf")

    config_printer.show()


if __name__ == "__main__":
    main()
