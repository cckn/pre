#-*- coding: utf-8 -*-

import schedule  # see https://github.com/dbader/schedule
import serial
import ConfigParser

from import_manager import AutoSocket
from import_manager import PrintConfig
from import_manager import protocol


class Report_certi(object):
    """docstring for Report_certi"""

    def __init__(self, config_path):
        super(Report_certi, self).__init__()
        self.get_config(config_path)
        self.seqnum = 0
        self.distance = 0
        self.count = 0
        self.num = 0

        self.serial = serial.Serial("/dev/serial0", 921600)
        self.socket = AutoSocket.AutoSocket(self.server_ip, self.server_port)

        self.serial.close()
        self.serial.open()
        self.serial.write(bytes('100 200\r', encoding='ascii'))

        schedule.every(self.report_interval).seconds.do(self.report)

        # self.f1 = open("log/raw_data.txt", 'a')
        # self.f2 = open("log/average.txt", 'a')

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

        self.frame_buff.append(int(self.distance) & 0xff)  # DISTANCE LOW

        """FOOTER"""
        self.frame_buff.append(protocol.ETX)  # ETX

        """Update Length Field"""
        self.frame_buff[1] = self.frame_buff.__len__() - 3
        self.socket.send(self.frame_buff)

        print("DeviceID : " + str(self.device_id) + " Seq : " +
              str(self.seqnum) + " Distance : " + str(self.distance))

    def run(self):

        while True:
            schedule.run_pending()

            try:
                self.distance = int(self.serial.readline())
                print(self.distance)

                # if self.num > 40:
                if self.distance == -1:
                    self.distance = 0
                elif self.distance > 255:
                    self.distance = 255
                # self.report()
                # print("send")z
                # self.num = 0
                # print(self.distance)

                # else:
                #     self.num = self.num + 1
            except Exception as e:
                print(e)


def main():

    ex = Report_certi("config.conf")

    ex.run()

if __name__ == "__main__":
    main()
