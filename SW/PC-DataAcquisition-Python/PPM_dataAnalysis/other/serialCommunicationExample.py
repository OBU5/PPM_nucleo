import serial

# b'...' - a sequence of octets (integers between 0 and 255)
mode = b'#CON#'

#Connect the serial port
serial = serial.Serial("COM6", timeout=None, baudrate=2000000, xonxoff=False, rtscts=False, dsrdtr=False) 

if serial.isOpen ():
 print ("Serial port is open")
else:
 print ("Serial port is not open")
print(serial.name)         # check which port was really used

#serial.write(mode)     # write a string
index=0
while True:
    try:
        ser_bytes = serial.readline()
        decoded_bytes = float(ser_bytes[0:len(ser_bytes)-1].decode("utf-8"))
        print(decoded_bytes)
        index+=1

    except:
        print("Keyboard Interrupt")
        break
#Close the serial port
# serial.close ()
# if serial.isOpen ():
#  print ("Serial port is not closed")
# else:
#  print ("Serial port is closed")