# -*- coding: utf-8 -*-

import socket
import sys

HOST='' #your ip address
PORT= 8888 # any port 

ss=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print('Socket created')

ss.bind((HOST,PORT))
print('Socket bind complete')

ss.listen(10)
print('Socket now listening')

conn, addr = ss.accept()
print("socket accept")

x = 0
while (x<20) : #change x<20 to true for infinite loop
 
    #print("receiving data")
    data = conn.recv(128)
    #print("data received")
    #print(data)  #prints the raw data stream
    msg = data.decode("utf-8")
    info = msg.split("@")
    print(info[0])
    x = x+1 
#%%
