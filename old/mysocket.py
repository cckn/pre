# -*- coding : cp949 -*-
import socket

HOST = '192.168.0.11'  # localhost
PORT = 6000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send(b'Hello, python')
data = s.recv(1024)
s.close()
print('Received', repr(data))
