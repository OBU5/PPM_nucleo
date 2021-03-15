import numpy as np 
import matplotlib.pyplot as plt 
fh = open('test.txt')
import csv


# x axis values 
x = [] 
# corresponding y axis values 
y = [] 

procesingDataFromMeasurement = 0
graphTitle = ""
measurementMethod = ""

arrayIndex = 0
lineIndex = 0
try:
    for line in fh:
        foundHeadOfCommand = line.find("<")   #if there is "<" character in the line - head of command was found
        foundTailOfCommand = line.find(">")   #if there is ">" character in the line - tail ofcommand was found
        
        #Command line
        if foundHeadOfCommand >= 0:

            # null arrays
            arrayIndex = 0
            x = [] 
            y = [] 
            
            #remove first and last character
            commandText = line[1:-2]           
            commandArgs = commandText.split(":")

            if commandArgs[0] == 'INFO':
                print(commandText)        
            elif commandArgs[0] == 'MEAS':
                procesingDataFromMeasurement = 1
                graphTitle = commandArgs[1] +""+ commandArgs[2]
        
        #found end of command    
        elif foundTailOfCommand >= 0:
            if procesingDataFromMeasurement == 1 and (commandArgs[2] == "extADC" or  commandArgs[2] == "intADC"):
                with open('generatedFiles/csv/' + graphTitle + '.csv', 'w', newline='') as csvFile:
                    csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
                    for i in range(arrayIndex):   
                        csvFileWriter.writerow([i, y[i]])
                fig = plt.figure(figsize=(30, 12))
                plt.subplot(2, 1, 1)
                plt.plot(x,y)            
                plt.xlabel('step') 
                plt.ylabel('Amplitude')
                plt.title(graphTitle) 

                plt.subplot(2, 5, 6)
                plt.xlim(0, 1800)
                plt.plot(x,y)
                plt.xlabel('step') 
                plt.ylabel('Amplitude')

                plt.subplot(2, 5, 7)
                plt.xlim(22000, 23800)
                plt.plot(x,y)
                plt.xlabel('step') 
                plt.ylabel('Amplitude')

                plt.subplot(2, 5, 8)
                plt.xlim(44000, 45800)
                plt.plot(x,y)
                plt.xlabel('step') 
                plt.ylabel('Amplitude')

                plt.subplot(2, 5, 9)
                plt.xlim(65100, 66800)
                plt.plot(x,y)
                plt.xlabel('step') 
                plt.ylabel('Amplitude')

                plt.subplot(2, 5, 10)
                plt.xlim(86400, 88200)
                plt.plot(x,y)
                plt.xlabel('step') 
                plt.ylabel('Amplitude')
                
                fig.savefig('generatedFiles/figures/' + graphTitle + '.png', dpi=fig.dpi)
                #plt.close(fig)
                plt.show()
            elif procesingDataFromMeasurement == 1 and commandArgs[2] == "comp" :
                with open('generatedFiles/csv/' + graphTitle + '.csv', 'w', newline='') as csvFile:
                    csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
                    for i in range(arrayIndex):   
                        csvFileWriter.writerow([i, y[i]])
                #print value
                average = 0
                averageFreq = 0
                averageMagInd = 0
                for i in range(10, 1500):
                    average = average + y[i]
                    
                average = average/(1500-10)
                averageFreq = 216000000/average
                averageMagInd = averageFreq/(42.5)
                print('Measurement ', commandArgs[1])
                print(round(average, 4),'[step]')
                print(round(averageFreq, 4), '[Hz]')
                print(round(averageMagInd, 4), '[uT]')
                print()


                #plot graph
                yMod = [216000000/yElement  for yElement in y]

                fig = plt.figure(figsize=(34, 12))
                plt.subplot(2, 1, 1)
                plt.plot(x,y)            
                plt.xlabel('step') 
                plt.ylabel('freq')
                plt.title(graphTitle) 

                plt.subplot(2, 5, 6)
                plt.xlim(0, 200)
                plt.ylim(1900, 2300)
                plt.plot(x,yMod)
                plt.xlabel('step') 
                plt.ylabel('freq')

                plt.subplot(2, 5, 7)
                plt.xlim(1000, 1200)
                plt.ylim(1900, 2300)
                plt.plot(x,yMod)
                plt.xlabel('step') 
                plt.ylabel('freq')

                plt.subplot(2, 5, 8)
                plt.xlim(1900, 2100)
                plt.ylim(1900, 2300)
                plt.plot(x,yMod)
                plt.xlabel('step') 
                plt.ylabel('freq')

                plt.subplot(2, 5, 9)
                plt.xlim(3000, 3200)
                plt.ylim(1900, 2300)
                plt.plot(x,yMod)
                plt.xlabel('step') 
                plt.ylabel('freq')

                plt.subplot(2, 5, 10)
                plt.xlim(3800, 4000)
                plt.ylim(1900, 2300)
                plt.plot(x,yMod)
                plt.xlabel('step') 
                plt.ylabel('freq')
                
                fig.savefig('generatedFiles/figures/' + graphTitle + '.png', dpi=fig.dpi)

               # plt.show()




        #regular line
        else:
            try:
                x.append(int(float(arrayIndex)))
                y.append(int(float(line)))

            except:
                print(arrayIndex)
            arrayIndex+=1
        lineIndex+=1
except:
    print('error on line - ', lineIndex)



fh.close()


