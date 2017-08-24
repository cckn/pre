#-*- coding: utf-8 -*-

# import schedule  # see https://github.com/dbader/schedule
import threading
import serial
import ConfigParser
import time
import datetime

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


device_serial = serial.Serial('/dev/ttyUSB1', 9600)
gps_socket = AutoSocket.AutoSocket(server_ip, server_port)


gps_start = False
alert_start = False
alert_count = 3


class get_gps_data(threading.Thread):
    """docstring for get_gps_data"""

    def __init__(self, config_path):
        super(get_gps_data, self).__init__()

        self.seqnum = 0
        self.buf = bytearray()

        device_serial.close()
        device_serial.open()

        self.gps = namedtuple(
            "gps", "tagid, seqnum, NS, latitude, EW, longitude")
        self.gps_data = 0

    def update(self):

        #        self.tagid = self.buf[3]
        self.tagid = 99
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
        self.frame_buff.append(protocol.REPORT_GPS_DATA)  # CMD TYPE
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
        gps_socket.send(self.frame_buff)

        print("DeviceID : " + str(device_id) +
              " TagID : " + str(self.tagid) +
              " Seq : " + str(self.seqnum) +
              " lati : " + str(self.latitude) +
              " long : " + str(self.longitude))

    def run(self):
        while True:
            rx_msg = ord(device_serial.read())
            global gps_start
            if gps_start:
                self.buf.append(rx_msg)
                if rx_msg == protocol.ETX and self.buf[1] == 22 and len(self.buf) == 25:
                    self.report()
                    gps_start = False

            else:
                if rx_msg == protocol.STX:
                    gps_start = True
                    self.buf = bytearray()
                    self.buf.append(rx_msg)
                    # print("get")
                    # self.socket.recv(1)


class resv_alert(threading.Thread):

    def __init__(self):
        super(resv_alert, self).__init__()
        self.buf = bytearray()

    def run(self):
        while True:
            global alert_start
            global alert_count
            rx_msg = ord(gps_socket.recv(1))
            # print(rx_msg)
            if alert_start:
                self.buf.append(rx_msg)
                if rx_msg == protocol.ETX and self.buf[1] == 5 and len(self.buf) == 8:
                    alert_start = False
                    alert_count = 3
                    # device_serial.write(
                    #     bytearray(b"\x02\x05\xFF\x00\x01\xFF\xFF\x03"))

            else:
                if rx_msg == protocol.STX:
                    alert_start = True
                    self.buf = bytearray()
                    self.buf.append(rx_msg)


class send_alert(threading.Thread):

    def __init__(self):
        super(send_alert, self).__init__()

    def run(self):
        global alert_count
        time.sleep(1)
        while True:
            if alert_count > 0:
                print(str(datetime.datetime.now().strftime(
                    '%H-%M-%S')) + " ALERT! : " + str(alert_count))
                if device_serial.isOpen():
                    device_serial.write(
                        bytearray(b"\x02\x05\xFF\x00\x01\xFF\xFF\x03"))
                alert_count = alert_count - 1
                time.sleep(1)


if __name__ == "__main__":

    resv_alert = resv_alert()
    resv_alert.demon = True
    resv_alert.start()

    send_alert = send_alert()
    send_alert.demon = True
    send_alert.start()

    get_gps_data = get_gps_data("config.conf")
    get_gps_data.demon = True
    get_gps_data.start()
