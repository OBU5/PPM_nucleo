import serial
import csv
import numpy as np 

arr = np.zeros(44100)

# b'...' - a sequence of octets (integers between 0 and 255)
mode = b'#CON#'

#Connect the serial port
serial = serial.Serial("COM6", timeout=None, baudrate=2000000, xonxoff=False, rtscts=False, dsrdtr=False,bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE,parity=serial.PARITY_NONE) 

if serial.isOpen ():
    print ("Serial port is open")
else:
    print ("Serial port is not open")
    print(serial.name)         # check which port was really used

#serial.write(mode)     # write a string

#Get data from serial as fast as possible
buffer = []
terminate = 0
while terminate != 1:    
    waiting = serial.in_waiting                         # find num of bytes currently waiting in hardware
    for c in serial.read(waiting):                       # read them, convert to ascii
        if (chr(c) != ';'):
            buffer += chr(c) 
        else:
            terminate =1

#write data into file
receivedString = ''.join(buffer)
receivedRows = receivedString.split('\n')

#secondly we can store readed values into file
with open('test.csv', mode='w') as csvFile:
    csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
    index = 0
    while index < len(receivedRows):   
        csvFileWriter.writerow([index, receivedRows[index]])
        index+=1
#     print('done')

