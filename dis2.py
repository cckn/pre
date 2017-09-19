#-*- coding: utf-8 -*-


import serial
import time
import threading

# import schedule  # see https://github.com/dbader/schedule

from import_manager import AutoSocket
from import_manager import protocol

danger = 0
warning = 0
none = 0

socket = AutoSocket.AutoSocket("192.168.0.20", 6005)
serial0 = serial.Serial("/dev/serial0", 921600)
serial0.close()
serial0.open()
serial0.write('100 200\r\n'.encode())


class get_warning(threading.Thread):
    """docstring for get_warning"""

    def __init__(self):
        super(get_warning, self).__init__()

    def run(self):
        global danger
        global warning
        global none

        while True:

            st = str(serial0.readline())
            # print(st)

            if st.find("Danger") != -1:
                danger += 1
                # print("danger :: " + str(danger))

            elif st.find("Warning") != -1:
                warning += 1
                # print("Warning :: " + str(warning))

            elif st.find("None") != -1:
                none += 1
                # print("None :: " + str(none))


class set_warning(threading.Thread):
    """docstring for set_warning"""
    global danger
    global warning
    global none

    def __init__(self):
        super(set_warning, self).__init__()

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
            if danger >= 35:
                print("danger")
            elif warning >= 35:
                print("warning")
            elif none >= 35:
                print("none")
            else:
                print("empty")
            # print("\t\t" +
            #       "  danger = " + str(danger) +
            #       "  warning = " + str(warning) +
            #       "  none = " + str(none))

            count_init()
            time.sleep(1)


def count_init():
    global danger
    global warning
    global none
    danger = danger * 0.5
    warning = warning * 0.5
    none = none * 0.5


t1 = get_warning()
t1.daemon = True
t1.start()

t2 = set_warning()
t2.daemon = True
t2.start()


while True:
    time.sleep(1)
