#-*- coding: utf-8 -*-

# import schedule  # see https://github.com/dbader/schedule
import threading

import serial
import ConfigParser
from collections import namedtuple

from import_manager import AutoSocket
from import_manager import PrintConfig
from import_manager import protocol


PrintConfig.PrintConfig("config.conf").show()

config = ConfigParser.ConfigParser()
config.read("config.conf")

"""DEVICE_INFO"""
device_id = config.getint("DEVICE_INFO", "id")

"""NETWORK_CONF"""
server_ip = config.get("NETWORK_CONF", "server_ip")
server_port = config.getint("NETWORK_CONF", "server_port")

"""GPS_CONF"""
serial_path = config.get("GPS_CONF", "serial_path")
serial_baudrate = config.getint("GPS_CONF", "serial_baudrate")
report_interval = config.getint("GPS_CONF", "report_interval")

"""GPS_DUMMY_CONF"""
tagid = config.getint("GPS_DUMMY_CONF", "tagid")
default_latitute = config.getfloat(
    "GPS_DUMMY_CONF", "default_latitute")
default_longitute = config.getfloat(
    "GPS_DUMMY_CONF", "default_longitute")
rand_value = config.getint("GPS_DUMMY_CONF", "rand_value")


device_serial = serial.Serial('/dev/ttyUSB1', 9600)
gps_socket = AutoSocket.AutoSocket(server_ip, server_port)


class ReportGPS(threading.Thread):
    """docstring for ReportGPS"""

    def __init__(self, config_path):
        super(ReportGPS, self).__init__()

        self.seqnum = 0
        self.buf = bytearray()
        self.start = False

        device_serial.close()
        device_serial.open()

        self.gps = namedtuple(
            "gps", "tagid, seqnum, NS, latitude, EW, longitude")
        self.gps_data = 0

    def update(self):

        self.tagid = self.buf[3]
        self.seqnum = self.buf[4]
        self.latitude = self.buf[5:14]
        self.longitude = self.buf[14:24]
        print(self.latitude)
        print(self.longitude)

        self.gps_data = self.gps(self.tagid, self.seqnum,
                                 "N", self.latitude, "E", self.longitude)

    def report(self):

        self.update()
        self.frame_buff = bytearray()

        """ HEADER """
        self.frame_buff.append(protocol.STX)  # stx
        self.frame_buff.append(0x00)  # Len
        self.frame_buff.append(0x02)  # CMD TYPE
        self.frame_buff.append(device_id)  # DEVICE ID

        """ BODY """
        self.frame_buff.append(((self.gps_data.tagid) >> 8) & 0xff)  # Tag id
        self.frame_buff.append((self.gps_data.tagid) & 0xff)  # Tag id
        self.frame_buff.append((self.gps_data.seqnum >> 8) & 0xff)  # seqNum
        self.frame_buff.append((self.gps_data.seqnum) & 0xff)  # seqNum

        """BODY - GPS DATA"""
        self.frame_buff.append('N')  # Tag id
        self.frame_buff = self.frame_buff + self.latitude
        self.frame_buff.append('E')  # Tag id
        self.frame_buff = self.frame_buff + self.longitude

        """FOOTER"""
        self.frame_buff.append(protocol.ETX)  # ETX

        """Update Length Field"""
        self.frame_buff[1] = self.frame_buff.__len__() - 3
        self.frame_buff[2] = 0x02
        gps_socket.send(self.frame_buff)

    def run(self):
        pass
        # while True:
        #     rx_msg = ord(device_serial.read())
        #     if self.start:
        #         self.buf.append(rx_msg)
        #         if rx_msg == 0x03 and self.buf[1] == 22 and len(self.buf) == 25:
        #             self.report()
        #             self.start = False

        #     else:
        #         if rx_msg == 0x02:
        #             self.start = True
        #             self.buf = bytearray()
        #             self.buf.append(rx_msg)
        #             # print("get")
        #             # self.socket.recv(1)


class test_th(threading.Thread):

    def __init__(self):
        super(test_th, self).__init__()

    def run(self):
        while True:
            print("TEST")

# def main():


if __name__ == "__main__":
    get_gps__ = ReportGPS("config.conf")
    get_gps__.demon = True
    get_gps__.start()

    testth = test_th()
    testth.demon = True
    testth.start()
