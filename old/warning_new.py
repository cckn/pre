import schedule  # see https://github.com/dbader/schedule
import serial
import socket


# HOST='192.168.0.36' #localhost
# PORT=6005
HOST = '192.168.0.11'  # localhost
PORT = 6000
device_id = 0x01

user_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


TEST_GPS_STRING = "$GPRMC,114455.532,A,3735.0079,N,12701.6446,E,0.000000,121.61,110706,,*0A"


def socket_auto_send(data):
    try:
        user_socket.send(data)
    except Exception as e:
        try:
            user_socket.close()
            global user_socket
            user_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            user_socket.connect((HOST, PORT))
            print "try"
        except Exception as e:
            print e


# def gps_parser():

#     frame_buff = bytearray()
#     frame_buff.append(0x02) # stx
#     frame_buff.append(0x1C) # Len
#     frame_buff.append(0x02) # CMD TYPE
#     frame_buff.append(device_id) # DEVICE ID
#     frame_buff.append(int(distance / 255) & 0xff) # DISTANCE HI
#     frame_buff.append(int(distance % 255) & 0xff) # DISTANCE LOW
#     frame_buff.append(0x03) # ETX

#     socket_auto_send(frame_buff)


# def send_gps_data():

#     frame_buff = bytearray()
#     frame_buff.append(0x02) # stx
#     frame_buff.append(0x1C) # Len
#     frame_buff.append(0x02) # CMD TYPE
#     frame_buff.append(device_id) # DEVICE ID
#     frame_buff.append(int(distance / 255) & 0xff) # DISTANCE HI
#     frame_buff.append(int(distance % 255) & 0xff) # DISTANCE LOW
#     frame_buff.append(0x03) # ETX

#     socket_auto_send(frame_buff)


def send_distance():

    frame_buff = bytearray()
    frame_buff.append(0x02)  # stx
    frame_buff.append(0x04)  # Len
    frame_buff.append(0x01)  # CMD TYPE
    frame_buff.append(device_id)  # DEVICE ID
    frame_buff.append(int(distance / 255) & 0xff)  # DISTANCE HI
    frame_buff.append(int(distance % 255) & 0xff)  # DISTANCE LOW
    frame_buff.append(0x03)  # ETX

    socket_auto_send(frame_buff)


if __name__ == "__main__":

    print "start"
    ser = serial.Serial('/dev/serial0', 921600)
    ser.close()
    ser.open()

    schedule.every(1).seconds.do(send_distance)

    distance = 0

    while True:
        schedule.run_pending()
        rx_msg = ser.readline()
        try:
            value = int(float(rx_msg[13:21]))
            distance = (distance * 0.95) + (value * 0.05)
        except Exception as e:
            pass
        finally:
            print rx_msg
            print distance
            # print value
