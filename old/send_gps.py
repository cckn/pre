

def main():
    # gps_serial.close()
    # gps_serial.open()

    schedule.every(1).seconds.do(gps_sender)
    gps_data_parser()
    while True:
        schedule.run_pending()


def gps_sender():
    gps_data_parser()
    send_gps_data()


def gps_data_parser():
    global seqnum
    global gps_data

    if seqnum > 0xffff:
        seqnum = 0
    else:
        seqnum += 1

    gps_data = gps(1, seqnum, random.randrange(50, 85),
                   "N", "3735.0079", "E", "12701.6446")

convert_to_only_degree


def send_gps_data():
    global gps_data

    frame_buff = bytearray()

    """ HEADER """
    frame_buff.append(0x02)  # STX
    frame_buff.append(0x1B)  # Length
    frame_buff.append(0x02)  # CMD TYPE
    frame_buff.append(device_id)  # DEVICE ID

    """ BODY """
    frame_buff.append((gps_data.tagid >> 8) & 0xff)  # Tag id
    frame_buff.append((gps_data.tagid) & 0xff)  # Tag id
    frame_buff.append((gps_data.seqnum >> 8) & 0xff)  # seqNum
    frame_buff.append((gps_data.seqnum) & 0xff)  # seqNum

    """BODY - GPS DATA"""
    frame_buff = frame_buff + bytearray(gps_data.NS)
    frame_buff = frame_buff + \
        bytearray("%.6f" % convert_to_only_degree(gps_data.latitude))
    frame_buff = frame_buff + bytearray(gps_data.EW)
    frame_buff = frame_buff + \
        bytearray("%.6f" % convert_to_only_degree(gps_data.longitude))

    """FOOTER"""
    frame_buff.append(0x03)  # ETX

    autosocket.send(frame_buff)


if __name__ == "__main__":
    main()
