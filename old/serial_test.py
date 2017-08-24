#-*- coding: utf-8 -*-

import serial


radar_serial = serial.Serial("/dev/serial0", 921600)


radar_serial.close() # asdasdasd
radar_serial.open()

while True:
    print(radar_serial.read())

# radar_serial.
# rx_msg = radar_serial.readline()
