
import serial
import datetime
import csv



#formatting the filename as the current date, hours and minute
currtimedate = str(datetime.datetime.now())
date = currtimedate.split()[0]
time = currtimedate.split()[1]
splittime = time.split(":")
hourmin = splittime[0] + splittime[1]
filename = "accels"+date+"-"+hourmin+".csv"


csvfile = open(filename,'a',newline='')
filewrite = csv.writer(csvfile, delimiter=',')
filewrite.writerow(['Date & Time','AccelX', 'AccelY','AccelZ'])

ser = serial.Serial('COM5', 115200)  # change accordingly to your serial port
ser.flushInput()

print(ser.name, "is opened") #for debugging check which port is actually used

loopcount = 0
while loopcount < 2048:
    try:
        ser.flushInput()    # flush the serial port to clear the queue so that data doesn't overlap and create erroneous data points
        loopcount = loopcount + 1
        print(loopcount)

        ser_bytes = ser.readline() # reads the bytes of the serial input
        #  pls decode the bytes
        decoded = ser_bytes.decode("utf-8")
        #print(decoded) #for debugging
        #print(type(decoded)) #debug

        #only necessary if incoming string needs splitting
        splitter = decoded.split(',')
        accelX = float(splitter[0])
        accelY = float(splitter[1])
        accelZ = float(splitter[2])

        #filewrite.writerow([datetime.datetime.now(), decoded])
        filewrite.writerow([datetime.datetime.now(), accelX, accelY, accelZ])

    except Exception as e:
        print(e)
        #print("Keyboard Interrupt") #using the ctrl-c to break the operation
        #ser.close()
        #break

print("terminating")
ser.close()
csvfile.close()
