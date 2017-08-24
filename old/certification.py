#-*- coding: utf-8 -*-

import sys
import schedule  # see https://github.com/dbader/schedule
import serial
import ConfigParser

from import_manager import AutoSocket
from import_manager import PrintConfig
from import_manager import protocol


class certification(object):
    """docstring for certification"""

    def __init__(self, config_path):
        super(certification, self).__init__()

        PrintConfig.PrintConfig(config_path).show()

        config = ConfigParser.ConfigParser()
        config.read(config_path)

        """DEVICE_INFO"""
        self.device_id = config.getint("DEVICE_INFO", "id")

        """NETWORK_CONF"""
        server_ip = config.get("NETWORK_CONF", "server_ip")
        server_port = config.getint("NETWORK_CONF", "server_port")

        """IMPULSE_RADAR_CONF"""
        serial_path = config.get("IMPULSE_RADAR_CONF", "serial_path")
        serial_baudrate = config.getint(
            "IMPULSE_RADAR_CONF", "serial_baudrate")
        report_interval = config.getint(
            "IMPULSE_RADAR_CONF", "report_interval")

        self.seqnum = 0
        self.distance = 0

        self.radar_serial = serial.Serial(serial_path, serial_baudrate)
        self.socket = AutoSocket.AutoSocket(server_ip, server_port)

        self.radar_serial.close()
        self.radar_serial.open()

        schedule.every(report_interval).seconds.do(self.report)

    # def init()

    def report(self):

        if self.seqnum == 1000:
            sys.exit(1)
        else:
            self.seqnum += 1

        self.frame_buff = bytearray()

        """ HEADER """
        self.frame_buff.append(protocol.STX)  # stx
        self.frame_buff.append(0x00)  # Len
        self.frame_buff.append(protocol.REPORT_RADAR_DATA)  # CMD TYPE
        self.frame_buff.append(self.device_id)  # DEVICE ID

        """ BODY """
        self.frame_buff.append((self.seqnum >> 8) & 0xff)  # seqNum
        self.frame_buff.append((self.seqnum) & 0xff)  # seqNum
        self.frame_buff.append((int(self.distance) >> 8) & 0xff)  # DISTANCE HI
        self.frame_buff.append(int(self.distance) & 0xff)  # DISTANCE LOW

        """FOOTER"""
        self.frame_buff.append(protocol.ETX)  # ETX

        """Update Length Field"""
        self.frame_buff[1] = self.frame_buff.__len__() - 3
        self.socket.send(self.frame_buff)

        print("SeqNum : " + str(self.seqnum) + " " +
              "Distance : " + str(self.distance))

    def run(self):

        while True:
            schedule.run_pending()

            # print(rx_msg)
            try:
                rx_msg = self.radar_serial.readline()
                value = int(float(rx_msg[13:21]))
                self.distance = (self.distance * 0.95) + (value * 0.05)
            except Exception as e:
                print(e)


def main():

    ex = certification("config.conf")
    ex.run()

if __name__ == "__main__":
    main()
