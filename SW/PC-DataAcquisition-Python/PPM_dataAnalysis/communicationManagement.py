import csv
import numpy as np 
import matplotlib.pyplot as plt 

def sendMessage(port, baudrate, message):
    import serial
    serial = serial.Serial(port, timeout=None, baudrate=baudrate, xonxoff=False, rtscts=False, dsrdtr=False,bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE,parity=serial.PARITY_NONE) 
    if serial.isOpen ():
        print ("Sending command")
    else:
        print ("command wasnt send")
        print(serial.name)          # check which port was really used

    serial.write(message)             # write a string



def receiveMessage(port, baudrate, desiredCountOfReceivedMessages):
    import serial

    # b'...' - a sequence of octets (integers between 0 and 255)

    #Connect the serial port
    serial = serial.Serial(port, timeout=None, baudrate=baudrate, xonxoff=False, rtscts=False, dsrdtr=False,bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE,parity=serial.PARITY_NONE) 

    if serial.isOpen ():
        print ("waiting for new message")
    else:
        print ("Serial port is not open")
        print(serial.name)          # check which port was really used

    #serial.write(mode)             # write a string

    #Get data from serial as fast as possible
    buffer = []
    canAppendToBuffer = 0
    messageComplete = 0
    countOfReceivedMessages = 0
    while messageComplete != 1:    
        waiting = serial.in_waiting                         # find num of bytes currently waiting in hardware
        for c in serial.read(waiting):                       # read them, convert to ascii
            currentCharacter = chr(c) 
            # special character - begining of command was found
            if (currentCharacter == '<'):
                canAppendToBuffer = 1
            # special character - end of message
            elif (currentCharacter == '>'):
                countOfReceivedMessages += 1
                if(countOfReceivedMessages == desiredCountOfReceivedMessages):
                    messageComplete = 1            

            # normal data
            if(canAppendToBuffer == 1):
                buffer.append(currentCharacter)

    #write data into file
    receivedString = ''.join(buffer)
    receivedRows = receivedString.split('\n')

    #remove blank characters
    while("" in receivedRows) :
        receivedRows.remove("")
    
    return receivedRows

