import numpy as np 
import matplotlib.pyplot as plt 
import csv
import math
from scipy.signal import hilbert, chirp
from scipy.fft import fft, fftfreq

import frequencyAnalysis as fa
import fileManagement as fm

comparator_startIndex = 25
comparator_endIndex = 400
adc_startIndex = 500
adc_stopIndex = 8820

fs = 216000000/4898

enableCSV = 0
enablePlot_measuredSignal = 0
enablePlot_comparator = 0

duration = 2.0
samples = int(fs*duration)

previousSample = 0

# x axis values 
x = [] 
# corresponding y axis values 
y = [] 


#hilbert transform 
f_hilbertTransform = 0
B_hilbertTransform = 0


f_fft_noZeroPadding = 0
B_fft_noZeroPadding = 0

f_fft_zeroPadding1 = 0
B_fft_zeroPadding1 = 0

f_fft_zeroPadding2 = 0
B_fft_zeroPadding2 = 0

f_comparator = 0
B_comparator = 0

            

procesingDataFromMeasurement = 0
graphTitle = ""
measurementMethod = ""

arrayIndex = 0
lineIndex = 0

# open file with measured data
fh = open('test.txt')
try:
    fm.appendColumnsToCSV(path='generatedFiles/results.csv',time="time", column1="hilbert transform", column2="fft without zero padding", column3= "fft with zero padding (bin width of 10 mHz)",column4= "fft with zero padding (bin width of 100 mHz)", column5 = "comparator")

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
            commandText = line[1:-1]
            print(commandText)
            commandArgs = commandText.split(":")

            if commandArgs[0] == 'INFO':
                print(commandText)
            elif commandArgs[0] == 'MEAS':
                procesingDataFromMeasurement = 1
                graphTitle = commandArgs[1] +""+ commandArgs[2]
        
        #found end of command
        elif foundTailOfCommand >= 0:
            #if there is a new sample, save existing samples
            if(commandArgs[1] != previousSample):
                print("Started execution of " + str(commandArgs[1]) +". sample")
                if(previousSample != 0):
                    # write result into file
                    fm.appendColumnsToCSV(path='generatedFiles/results.csv',time= ((int(previousSample) - 1) * 8), column1= B_hilbertTransform, column2= B_fft_noZeroPadding, column3= B_fft_zeroPadding1,column4= B_fft_zeroPadding2, column5 = B_comparator)
            previousSample = commandArgs[1]

            if procesingDataFromMeasurement == 1 and (commandArgs[2] == "extADC" or  commandArgs[2] == "intADC"):
                
                if(enableCSV):
                    #write into csv
                    fm.saveOneColumnToCSV(path = 'generatedFiles/csv/' + graphTitle + '.csv', column = y)
                
        
        
               
                # determine analytic signal
                
                f_hilbertTransform = fa.computeFrequencyWithHilbertTransform(signal = y, startSample = adc_startIndex, endSample = adc_stopIndex, fs = fs)
                B_hilbertTransform = f_hilbertTransform/(42.577478518 * pow(10, -3)) 
                print("    Calculations - ADC - hilbert transform")
                print('        f = '+ str(f_hilbertTransform)+ ' Hz')
                print('        B = '+ str(B_hilbertTransform)+ ' nT')

                
                
                f_fft_noZeroPadding = fa.computeFFTOfSignal(startSample = adc_startIndex, endSample = adc_stopIndex, signal = y, fs = fs)
                B_fft_noZeroPadding  = f_fft_noZeroPadding/(42.577478518 * pow(10, -3)) 
                print("    Calculations - ADC - with fft zero padding")
                print('        f = '+ str(f_fft_noZeroPadding)+ ' Hz')
                print('        B = '+ str(B_fft_noZeroPadding)+ ' nT')


                f_fft_zeroPadding1 = fa.computeFFTOfSignalWithZeroPadding(startSample = adc_startIndex, endSample = adc_stopIndex, signal = y, fs = fs, zeropadding = 4410000)
                B_fft_zeroPadding1  = f_fft_zeroPadding1/(42.577478518 * pow(10, -3)) 
                print("    Calculations - ADC - withou fft zero padding")
                print('        f = '+ str(f_fft_zeroPadding1)+ ' Hz')
                print('        B = '+ str(B_fft_zeroPadding1)+ ' nT')

                
                f_fft_zeroPadding2 = fa.computeFFTOfSignalWithZeroPadding(startSample = adc_startIndex, endSample = adc_stopIndex, signal = y, fs = fs, zeropadding = 441000)
                B_fft_zeroPadding2  = f_fft_zeroPadding2/(42.577478518 * pow(10, -3)) 
                print("    Calculations - ADC - withou fft zero padding")
                print('        f = '+ str(f_fft_zeroPadding2)+ ' Hz')
                print('        B = '+ str(B_fft_zeroPadding2)+ ' nT')


                #if(enablePlot_measuredSignal or enablePlot_hilbert or enablePlot_reconstructed or enablePlot_instantaneousAngle or enablePlot_instantaneousFrequency):
                   # print("    Plots")
                
                if(enablePlot_measuredSignal):
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
                    plt.close(fig)
                    
                   # print("        measured signal")

                #plt.show()
            elif procesingDataFromMeasurement == 1 and commandArgs[2] == "comp" :
                if(enableCSV):
                    saveOneColumnToCSV('generatedFiles/csv/' + graphTitle + '.csv', column=y)

                #print value
                average = 0
                skippedValues = 0
                for i in range(comparator_startIndex, comparator_endIndex):
                    #omit smaller than 35uT or greater than 60 uT
                    if(y[i] < 84573 or y[i] >144966 ):
                        skippedValues += 1
                    else:
                        average = average + y[i]
                average = average/(comparator_endIndex-comparator_startIndex - skippedValues)
                f_comparator = 216000000/average
                B_comparator = f_comparator/(42.577478518 * pow(10, -3))
                print("    Calculations - comparator")
                print('        f = '+ str(f_comparator)+ ' Hz')
                print('        B = '+ str(B_comparator)+ ' nT')


                #plot graph
                yMod = [216000000/yElement  for yElement in y]
                if(enablePlot_comparator):
                    fig = plt.figure(figsize=(34, 12))
                    plt.subplot(2, 1, 1)
                    plt.plot(x,y)            
                    plt.xlabel('step') 
                    plt.ylabel('freq')
                    plt.title(graphTitle) 

                    plt.subplot(2, 5, 6)
                    plt.xlim(0, 200)
                    plt.plot(x,y)            
                    plt.xlabel('step') 
                    plt.ylabel('freq')

                    plt.subplot(2, 5, 7)
                    plt.xlim(1000, 1200)
                    plt.plot(x,y)            
                    plt.xlabel('step') 
                    plt.ylabel('freq')

                    plt.subplot(2, 5, 8)
                    plt.xlim(1900, 2100)
                    plt.plot(x,y)            
                    plt.xlabel('step') 
                    plt.ylabel('freq')

                    plt.subplot(2, 5, 9)
                    plt.xlim(3000, 3200)
                    plt.plot(x,y)            
                    plt.xlabel('step') 
                    plt.ylabel('freq')

                    plt.subplot(2, 5, 10)
                    plt.xlim(3800, 4000)
                    plt.plot(x,y)            
                    plt.xlabel('step') 
                    plt.ylabel('freq')
                    
                    fig.savefig('generatedFiles/figures/' + graphTitle + '.png', dpi=fig.dpi)
                    plt.close()

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
except Exception as e:
    print(e)
    #print('error on line - ', lineIndex)



fh.close()

