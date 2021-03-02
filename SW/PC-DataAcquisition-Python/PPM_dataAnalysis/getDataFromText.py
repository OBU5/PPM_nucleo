import numpy as np 
import matplotlib.pyplot as plt 
fh = open('test.txt')

# x axis values 
x = [] 
# corresponding y axis values 
y = [] 

procesingDataFromMeasurement = 0
graphTitle = ""
measurementMethod = ""

index = 0
lineIndex = 0
try:
    for line in fh:
        lineIndex+=1
        foundHeadOfCommand = line.find("<")   #if there is "<" character in the line - command was found
        foundTailOfCommand = line.find(">")   #if there is "<" character in the line - command was found
        
        #Command line
        if foundHeadOfCommand >= 0:
            commandText = line[1:-2]           #remove first and last character
            commandArgs = commandText.split(":")

            if commandArgs[0] == 'INFO':
                print(commandText)        
            elif commandArgs[0] == 'MEAS':
                procesingDataFromMeasurement = 1
                graphTitle = commandArgs[1] +""+ commandArgs[2]
        
        #found end of command    
        elif foundTailOfCommand >= 0:
            if procesingDataFromMeasurement == 1 and commandArgs[2] == "extADC":
                fig = plt.figure(figsize=(30, 12))
                plt.subplot(2, 1, 1)
                plt.plot(x,y)            
                plt.xlabel('step') 
                plt.ylabel('Amplitude')
                plt.title(graphTitle) 

                plt.subplot(2, 5, 6)
                plt.xlim(0, 800)
                plt.plot(x,y)
                plt.xlabel('step') 
                plt.ylabel('Amplitude')

                plt.subplot(2, 5, 7)
                plt.xlim(22000, 22800)
                plt.plot(x,y)
                plt.xlabel('step') 
                plt.ylabel('Amplitude')

                plt.subplot(2, 5, 8)
                plt.xlim(44000, 44800)
                plt.plot(x,y)
                plt.xlabel('step') 
                plt.ylabel('Amplitude')

                plt.subplot(2, 5, 9)
                plt.xlim(66100, 66800)
                plt.plot(x,y)
                plt.xlabel('step') 
                plt.ylabel('Amplitude')

                plt.subplot(2, 5, 10)
                plt.xlim(87400, 88200)
                plt.plot(x,y)
                plt.xlabel('step') 
                plt.ylabel('Amplitude')
                
                fig.savefig(graphTitle + '.png', dpi=fig.dpi)

                #plt.show()




        #regular line
        else:
            try:
                x.append(int(float(index)))
                y.append(int(float(line)))
            except:
                print(index)
            index+=1


fh.close()


