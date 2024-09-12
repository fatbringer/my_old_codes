

import serial
import datetime
import time

#formatting the filename as the current date, hours and minute
currtimedate = str(datetime.datetime.now())
date = currtimedate.split()[0]
currtime = currtimedate.split()[1]
splittime = currtime.split(":")
hourmin = splittime[0] + splittime[1]
filename = "/home/pi/Desktop/"+date+"-"+hourmin+".csv"

import csv
csvfile = open(filename,'a+')
filewrite = csv.writer(csvfile, delimiter=',')
filewrite.writerow(['Date & Time','Mass(g)'])

deviceID = ['/dev/ttyACM0', '/dev/ttyACM1']
ser = serial.Serial(deviceID[0], 9600)  # change accordingly to your serial port

print(ser.name, "is opened") #for debugging check which port is actually used


while True:
    try:
        summa = 0
        avg = 0
        ser.flushInput()
        
        for i in range (5): 
            ser_bytes = ser.readline() # reads the bytes of the serial input
        #   pls decode the bytes
            decoded = ser_bytes.decode("utf-8")
            massval = float(decoded)
            summa = summa + massval
            
        avg = summa / (i+1)
        print(avg)
        filewrite.writerow([datetime.datetime.now(),avg])
        csvfile.flush()


    except Exception as e:
        print(e) #using the ctrl-c to break the operation
 
print("terminating")
csvfile.close()

