#-*- coding: utf-8 -*-

import schedule  # see https://github.com/dbader/schedule
import serial
import ConfigParser
import datetime

from import_manager import AutoSocket
from import_manager import PrintConfig
from import_manager import protocol


class ReportDistance(object):
    """docstring for ReportDistance"""

    def __init__(self, config_path):
        super(ReportDistance, self).__init__()
        self.get_config(config_path)

        self.seqnum = 0
        self.distance = 0
        self.count = 0

        self.radar_serial = serial.Serial(
            self.serial_path, self.serial_baudrate)
        self.socket = AutoSocket.AutoSocket(self.server_ip, self.server_port)

        self.radar_serial.close()
        self.radar_serial.open()

        schedule.every(self.report_interval).seconds.do(self.report)

        self.f1 = open("log/raw_data.txt", 'a')
        self.f2 = open("log/average.txt", 'a')

    def get_config(self, config_path):

        PrintConfig.PrintConfig(config_path).show()

        config = ConfigParser.ConfigParser()
        config.read(config_path)

        """DEVICE_INFO"""
        self.device_id = config.getint("DEVICE_INFO", "id")

        """NETWORK_CONF"""
        self.server_ip = config.get("NETWORK_CONF", "server_ip")
        self.server_port = config.getint("NETWORK_CONF", "server_port")

        """IMPULSE_RADAR_CONF"""
        self.serial_path = config.get("IMPULSE_RADAR_CONF", "serial_path")
        self.serial_baudrate = config.getint(
            "IMPULSE_RADAR_CONF", "serial_baudrate")
        self.report_interval = config.getint(
            "IMPULSE_RADAR_CONF", "report_interval")

    def report(self):

        if self.seqnum >= 0xffff:
            self.seqnum = 0
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

        # print("Distance : " + str(self.distance))
        # print(self.count)
        self.f2.write(str(self.distance))

    def run(self):

        while True:
            schedule.run_pending()

            try:
                rx_msg = self.radar_serial.readline()
                print(rx_msg)
                self.f1.write(rx_msg + "\n")
                value = int(float(rx_msg[13:21]))
                self.distance = (self.distance * 0.95) + (value * 0.05)
                # self.count = self.count + 1
            except Exception as e:
                print(e)


def main():

    ex = ReportDistance("config.conf")
    ex.run()

if __name__ == "__main__":
    main()
