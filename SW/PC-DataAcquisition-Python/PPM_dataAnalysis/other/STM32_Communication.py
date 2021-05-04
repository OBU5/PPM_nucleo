import serial
import csv
import numpy as np 
import matplotlib.pyplot as plt 



arr = np.zeros(44100)

# b'...' - a sequence of octets (integers between 0 and 255)
mode = b'<MEAS:extADC:INF>'

#Connect the serial port
serial = serial.Serial("COM9", timeout=None, baudrate=2000000, xonxoff=False, rtscts=False, dsrdtr=False,bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE,parity=serial.PARITY_NONE) 

if serial.isOpen ():
    print ("Serial port is open")
else:
    print ("Serial port is not open")
    print(serial.name)          # check which port was really used

#serial.write(mode)             # write a string

#Get data from serial as fast as possible
buffer = []
commandBuffer = []
indexOfMeasurementBuffer = []
methodBuffer = []
dataBuffer = []
terminate = 0
bufferStateIndex = 0            # 0 - wait for begining of command; 1 - command; 1 - index of measurement; 3 - method; 4 - data; 5 - found ending of message
while terminate != 1:    
    waiting = serial.in_waiting                         # find num of bytes currently waiting in hardware
    for c in serial.read(waiting):                       # read them, convert to ascii
        currentCharacter = chr(c) 
        buffer.append(currentCharacter)
        # special character - begining of command was found
        if (currentCharacter == '<'):
            bufferStateIndex = 1
       
        # special character - change bufferStateIndex
        elif (currentCharacter == ':'):
            # next data will represent command -> change bufferStateIndex 
            if (bufferStateIndex == 1):
                bufferStateIndex = 2
                
            # next data will represent index of measurement -> change bufferStateIndex 
            elif (bufferStateIndex == 2):
                bufferStateIndex = 3
           
            # next data will represent method of measurement -> change bufferStateIndex 
            elif (bufferStateIndex == 3):
                bufferStateIndex = 4
                
        # special character - end of message
        elif (currentCharacter == '>'):
            terminate = 1
        # normal data
        else:
            if(bufferStateIndex == 0):
                print("missing specification of message")
            elif(bufferStateIndex == 1):
                commandBuffer.append(currentCharacter)
            elif(bufferStateIndex == 2):
                indexOfMeasurementBuffer.append(currentCharacter)
            elif(bufferStateIndex == 3):
                methodBuffer.append(currentCharacter)
            elif(bufferStateIndex == 4):
                dataBuffer.append(currentCharacter)
            else:
                print("missing specification of message")


    
print("A")
#write data into file
receivedMeasurementString = ''.join(dataBuffer)
receivedMeasurementRows = receivedMeasurementString.split('\n')


receivedString = ''.join(buffer)
receivedRows = receivedString.split('\n')
print("B")

#remove blank characters
while("" in receivedMeasurementRows) :
    receivedMeasurementRows.remove("")

#remove blank characters
while("" in receivedRows) :
    receivedRows.remove("")

# x axis values 
x = [] 
# corresponding y axis values 
y = [] 
#secondly we can store readed values into file
with open('test.csv', mode='w') as csvFile:
    csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
    index = 0
    print("C")
    while index < len(receivedMeasurementRows):   
        try:
            x.append(int(float(index)))
            y.append(int(float(receivedMeasurementRows[index])))
            csvFileWriter.writerow([receivedRows[index]])
        except:
            print(receivedMeasurementRows[index])
        index+=1

