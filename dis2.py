#-*- coding: utf-8 -*-


import serial
import time
import threading

from import_manager import AutoSocket
from import_manager import protocol

cmd = bytearray("reset\r\n", 'utf-8')
cmd_dis = bytearray("100  2 00 \r\n ", 'utf-8')


threshold = 20

danger = 0
warning = 0
none = 0

# monitor_socket = AutoSocket.AutoSocket("192.168.2.99", 6005)

monitor_socket = AutoSocket.AutoSocket("192.168.2.6", 6005)
#server_socket = AutoSocket.AutoSocket("192.168.2.200", 5000)

sensor = serial.Serial("/dev/serial0", 921600)
sensor.close()
sensor.open()
time.sleep(5)

sensor.write(cmd_dis)

print(cmd_dis)


class get_warning(threading.Thread):
    """docstring for get_warning"""

    def __init__(self):
        super(get_warning, self).__init__()

    def run(self):
        global danger
        global warning
        global none

        while True:

            sensor_str = str(sensor.readline())
            # print(st)

            if sensor_str.find("bound") != -1:
                sensor.write(cmd_dis)
                print("init")

            elif sensor_str.find("start") != -1:
                print("start")

            if sensor_str.find("Danger") != -1:
                danger += 1
                # print("danger :: " + str(danger))

            elif sensor_str.find("Warning") != -1:
                warning += 1
                # print("Warning :: " + str(warning))

            elif sensor_str.find("None") != -1:
                none += 1
                # print("None :: " + str(none))


class set_warning(threading.Thread):
    """docstring for set_warning"""
    global danger
    global warning
    global none

    def __init__(self):
        super(set_warning, self).__init__()
        self.distance = 999
        self.seqnum = 0
        self.device_id = 1

        self.send_ct = 999

        self.danger_ct = 0
        self.warning_ct = 0
        self.none_ct = 0

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

        # self.frame_buff.append(int(self.distance) & 0xff)  # DISTANCE LOW

        """FOOTER"""
        self.frame_buff.append(protocol.ETX)  # ETX

        """Update Length Field"""
        self.frame_buff[1] = self.frame_buff.__len__() - 3
        print(self.distance)
        monitor_socket.send(self.frame_buff)
        if self.send_ct > 7:
 #           server_socket.send(self.frame_buff)
            self.send_ct = 0
        else:
            self.send_ct += 1

    def run(self):
        while True:
            if danger >= threshold:
                print("\tdanger")
                self.distance = 3
                self.danger_ct += 1
            elif warning >= threshold:
                print("\t\twarning")
                self.distance = 2
                self.warning_ct += 1
            elif none >= threshold:
                print("\t\t\tnone")
                self.distance = 1
                self.none_ct += 1
            else:
                print("\t\t\t\tempty")
                self.distance = 0
                self.none_ct += 1

            print("\t\t\t\t\t\t" +
                  "  danger = " + str(int(danger)) +
                  "  warning = " + str(int(warning)) +
                  "  none = " + str(int(none)))
            self.report()
            count_damper()
            time.sleep(1)


def count_damper():
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
