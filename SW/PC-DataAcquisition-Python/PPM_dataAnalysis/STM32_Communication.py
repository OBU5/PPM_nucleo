import serial
import csv
import numpy as np 
import matplotlib.pyplot as plt 

arr = np.zeros(44100)

# b'...' - a sequence of octets (integers between 0 and 255)
mode = b'#CON#'

#Connect the serial port
serial = serial.Serial("COM9", timeout=None, baudrate=2000000, xonxoff=False, rtscts=False, dsrdtr=False,bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE,parity=serial.PARITY_NONE) 

if serial.isOpen ():
    print ("Serial port is open")
else:
    print ("Serial port is not open")
    print(serial.name)         # check which port was really used

#serial.write(mode)     # write a string

#Get data from serial as fast as possible
serial.write(b'100')
buffer = []
commandBuffer = []
dataBuffer = []
commandBuffer = []
terminate = 0
bufferStateIndex = 0    # 0 - wait for begining of command; 1 - command; 2 - method; 3 - count; 4 - data; 5 - found ending of message
while terminate != 1:    
    waiting = serial.in_waiting                         # find num of bytes currently waiting in hardware
    for c in serial.read(waiting):                       # read them, convert to ascii
        # begining of command was found
        if (chr(c) == '<'):
            bufferStateIndex = 1
            buffer += chr(c) 

        if (chr(c) != ':'):
            # 
            if (bufferStateIndex == 1):

            buffer += chr(c) 
        else:
            terminate =1



            
print("A")
#write data into file
receivedString = ''.join(buffer)
receivedRows = receivedString.split('\n')
del receivedRows[-2:] # delete last element

print("B")

# x axis values 
x = [] 
# corresponding y axis values 
y = [] 
#secondly we can store readed values into file
with open('test.csv', mode='w') as csvFile:
    csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
    index = 0
    print("C")
    while index < len(receivedRows):   
        csvFileWriter.writerow([index, receivedRows[index]])
        try:
            x.append(int(float(index)))
            y.append(int(float(receivedRows[index])))
        except:
            print("a")
        index+=1

