import schedule  # see https://github.com/dbader/schedule
import serial
import socket


buff = bytearray()
buff.append(0x02)
buff.append(0x04)
buff.append(0x01)
buff.append(0x01)
buff.append(0x00)
buff.append(0x00)
buff.append(0x03)

HOST = '192.168.0.36'  # localhost
PORT = 6005
# HOST='192.168.0.11' #localhost
# PORT=6000

# global s
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# try:

#     s.connect((HOST,PORT))

#     print "connet"
# except Exception as e:
#     print "not connect"


def send():
    try:
        buff[4] = int(distance / 255) & 0xff
        buff[5] = int(distance % 255) & 0xff

        s.send(buff)

    except Exception as e:

        try:
            s.close()
            global s
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print "try"
        except Exception as e:
            print e


if __name__ == "__main__":

    print "start"
    ser = serial.Serial('/dev/serial0', 921600)
    ser.close()
    ser.open()

    # schedule.every(1).seconds.do(send)

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
            print int(distance)

            if distance < 100:
                print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                pass
            # print value
