import numpy as np 
import matplotlib.pyplot as plt 
fh = open('test.txt')
import csv
import math

from scipy.signal import hilbert, chirp

duration = 2.0
fs = 216000000/4898
samples = int(fs*duration)

enablePlot_measuredSignal = 1
enablePlot_hilbert = 0
enablePlot_reconstructed = 0
enablePlot_instantaneousAngle = 0
enablePlot_instantaneousFrequency = 1
enablePlot_comparator = 1


previousSample = 0

# x axis values 
x = [] 
# corresponding y axis values 
y = [] 


#hilbert transform 
signal = []
d_angle = []
d_freq = []
f_extADC = 0
hilbertStart = 29
hilbertEnd = 60000
hilbertSignalLength = hilbertEnd - hilbertStart
n = hilbertEnd - hilbertStart - 1

finalFrequency_comparator = 0

            

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
            if(commandArgs[1] != previousSample):
                print("Started execution of " + str(commandArgs[1]) +". sample")
            previousSample = commandArgs[1]

            if procesingDataFromMeasurement == 1 and (commandArgs[2] == "extADC" or  commandArgs[2] == "intADC"):
                with open('generatedFiles/csv/' + graphTitle + '.csv', 'w', newline='') as csvFile:
                    csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
                    for i in range(arrayIndex):   
                        csvFileWriter.writerow([y[i]])                    
                    csvFile.close()
               
               
                # determine analytic signal
                
                signal = []
                d_angle = []
                d_freq = []
                for k in range(len(y)):   
                    if((k >= hilbertStart) and (k < hilbertEnd)):
                        signal.append(y[k] - 32768)
                analytic_signal = hilbert(signal)
                #determine instantenous angle
                for k in range(len(analytic_signal)-1):  
                    d_angle.append(np.angle(analytic_signal[k+1]/analytic_signal[k]))

                #determine instantenous frequency
                for k in range(len(d_angle)):  
                    d_freq.append(d_angle[k] / (1/fs)/(2*math.pi))
                d_angle = np.cumsum(d_angle)

                # dtermine average frequency
                diff_d_angle = max(d_angle) - min(d_angle)
                tau=2 * math.pi * (n/fs) / diff_d_angle
                f_extADC = 1/tau
                B_extADC = f_extADC/(42.577478518 * pow(10, -3)) 
                reconstructedSig = np.zeros(hilbertEnd- hilbertStart)
                amplitude =max(np.abs(signal)*0.5)
                for k in range(hilbertEnd- hilbertStart):   
                    reconstructedSig[k] = (amplitude*math.cos(2*math.pi*f_extADC*(k)/fs))

                print("    Calculations - ADC")
                print('        f = '+ str(f_extADC)+ ' Hz')
                print('        B = '+ str(B_extADC)+ ' nT')
                # write result into file
                with open('generatedFiles/extADC_results.csv', 'a', newline='') as csvFile:
                    csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
                    csvFileWriter.writerow([commandArgs[1], f_extADC, B_extADC])
                    csvFile.close()



                if(enablePlot_measuredSignal or enablePlot_hilbert or enablePlot_reconstructed or enablePlot_instantaneousAngle or enablePlot_instantaneousFrequency):
                    print("    Plots")
                
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
                    
                    print("        measured signal")



                #plot - hilbert transform
                
                if(enablePlot_hilbert):
                    fig = plt.figure(figsize=(30, 12))
                    plt.subplot(2, 1, 1) 
                    plt.plot(signal, label='Signál')
                    plt.plot(np.imag(analytic_signal), label='Hilbertova transformace naměřených dat')
                    plt.xlabel("Vzorky")
                    plt.ylabel("Amplituda")                
                    plt.legend()         
                    plt.title(graphTitle + " Hilbertova transformace") 

                    plt.subplot(2, 4, 5)
                    plt.xlim(0, 400)
                    plt.plot(signal)
                    plt.plot(np.imag(analytic_signal))
                    plt.xlabel('step') 
                    plt.ylabel('Amplitude')

                    plt.subplot(2, 4, 6)
                    plt.xlim(hilbertSignalLength/3 - 200, hilbertSignalLength/3 + 200)
                    plt.plot(signal)
                    plt.plot(np.imag(analytic_signal))
                    plt.xlabel('step') 
                    plt.ylabel('Amplitude')

                    plt.subplot(2, 4, 7)
                    plt.xlim(hilbertSignalLength/3 * 2 - 200, hilbertSignalLength/3 * 2 + 200)
                    plt.plot(signal)
                    plt.plot(np.imag(analytic_signal))
                    plt.xlabel('step') 
                    plt.ylabel('Amplitude')

                    plt.subplot(2, 4, 8)
                    plt.xlim(hilbertSignalLength - 400, hilbertSignalLength)
                    plt.plot(signal)
                    plt.plot(np.imag(analytic_signal))
                    plt.xlabel('step') 
                    plt.ylabel('Amplitude')

                    fig.savefig('generatedFiles/figures/' + graphTitle + '_hilbert.png', dpi=fig.dpi)
                    plt.close(fig)
                    print("        measured signal and hilbert transform")

                # plot reconstructed signal vs measured signal
                
                if(enablePlot_reconstructed):
                    fig = plt.figure(figsize=(30, 12))
                    plt.subplot(2, 1, 1) 
                    plt.plot(signal, label='Naměřený signál')
                    plt.plot(reconstructedSig, label='Zrekonstruovaný signál')
                    plt.xlabel("Vzorky")
                    plt.ylabel("Amplituda")                
                    plt.legend()         
                    plt.title(graphTitle + " Rekonstrukce signálu") 

                    plt.subplot(2, 4, 5)
                    plt.xlim(0, 400)
                    plt.plot(signal)
                    plt.plot(reconstructedSig)
                    plt.xlabel('step') 
                    plt.ylabel('Amplitude')

                    plt.subplot(2, 4, 6)
                    plt.xlim(hilbertSignalLength/3 - 200, hilbertSignalLength/3 + 200)
                    plt.plot(signal)
                    plt.plot(reconstructedSig)
                    plt.xlabel('step') 
                    plt.ylabel('Amplitude')

                    plt.subplot(2, 4, 7)
                    plt.xlim(hilbertSignalLength/3 * 2 - 200, hilbertSignalLength/3 * 2 + 200)
                    plt.plot(signal)
                    plt.plot(reconstructedSig)
                    plt.xlabel('step') 
                    plt.ylabel('Amplitude')

                    plt.subplot(2, 4, 8)
                    plt.xlim(hilbertSignalLength - 400, hilbertSignalLength)
                    plt.plot(signal)
                    plt.plot(reconstructedSig)
                    plt.xlabel('step') 
                    plt.ylabel('Amplitude')

                    fig.savefig('generatedFiles/figures/' + graphTitle + '_reconstructed.png', dpi=fig.dpi)
                    plt.close(fig)
                    print("        measured signal and reconstructed signal")




                # plot instantaneous angle
                if(enablePlot_instantaneousAngle):

                    fig = plt.figure(figsize=(30, 12))
                    plt.subplot(2, 1, 1) 
                    plt.plot(d_angle)
                    plt.xlabel("Vzorky")
                    plt.ylabel("Úhel [rad]")   
                    plt.title(graphTitle + " Okamžitý úhel") 

                    plt.subplot(2, 4, 5)
                    plt.xlim(0, 400)
                    plt.plot(d_angle)
                    plt.xlabel("Vzorky")
                    plt.ylabel("Úhel [rad]")  

                    plt.subplot(2, 4, 6)
                    plt.xlim(hilbertSignalLength/3 - 200, hilbertSignalLength/3 + 200)
                    plt.plot(d_angle)
                    plt.xlabel("Vzorky")
                    plt.ylabel("Úhel [rad]")  

                    plt.subplot(2, 4, 7)
                    plt.xlim(hilbertSignalLength/3 * 2 - 200, hilbertSignalLength/3 * 2 + 200)
                    plt.plot(d_angle)
                    plt.xlabel("Vzorky")
                    plt.ylabel("Úhel [rad]")  

                    plt.subplot(2, 4, 8)
                    plt.xlim(hilbertSignalLength - 400, hilbertSignalLength)
                    plt.plot(d_angle)
                    plt.xlabel("Vzorky")
                    plt.ylabel("Úhel [rad]")  

                    fig.savefig('generatedFiles/figures/' + graphTitle + '_instantaneousAngle.png', dpi=fig.dpi)
                    plt.close(fig)
                    print("        instantaneous angle")


                
                # plot instantaneous frequency
                if(enablePlot_instantaneousFrequency):
                    fig = plt.figure(figsize=(30, 12))
                    plt.subplot(2, 1, 1) 
                    plt.plot(d_freq)
                    plt.ylim(1800, 2400)
                    plt.xlabel("Vzorky")
                    plt.ylabel("Frekvence [Hz]")   
                    plt.title(graphTitle + " Okamžitá frekvence") 

                    plt.subplot(2, 4, 5)
                    plt.xlim(0, 400)
                    plt.ylim(1800, 2400)
                    plt.plot(d_freq)
                    plt.xlabel("Vzorky")
                    plt.ylabel("Frekvence [Hz]")   

                    plt.subplot(2, 4, 6)
                    plt.xlim(hilbertSignalLength/3 - 200, hilbertSignalLength/3 + 200)
                    plt.ylim(1800, 2400)
                    plt.plot(d_freq)
                    plt.xlabel("Vzorky")
                    plt.ylabel("Frekvence [Hz]")   

                    plt.subplot(2, 4, 7)
                    plt.xlim(hilbertSignalLength/3 * 2 - 200, hilbertSignalLength/3 * 2 + 200)
                    plt.ylim(1800, 2400)
                    plt.plot(d_freq)
                    plt.xlabel("Vzorky")
                    plt.ylabel("Frekvence [Hz]")   

                    plt.subplot(2, 4, 8)
                    plt.xlim(hilbertSignalLength - 400, hilbertSignalLength)
                    plt.ylim(1800, 2400)
                    plt.plot(d_freq)
                    plt.xlabel("Vzorky")
                    plt.ylabel("Frekvence [Hz]")   

                    fig.savefig('generatedFiles/figures/' + graphTitle + '_instantaneousFrequency.png', dpi=fig.dpi)
                    plt.close(fig)                
                    print("        instantaneous frequency")

                #plt.show()
            elif procesingDataFromMeasurement == 1 and commandArgs[2] == "comp" :
                with open('generatedFiles/csv/' + graphTitle + '.csv', 'w', newline='') as csvFile:
                    csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
                    for i in range(arrayIndex):   
                        csvFileWriter.writerow([y[i]])
                    csvFile.close()

                #print value
                average = 0
                averageFreq = 0
                averageMagInd = 0
                startComp = 10
                stopComp = 1500
                for i in range(startComp, stopComp):
                    average = average + y[i]
                    
                average = average/(stopComp-startComp)
                averageFreq = 216000000/average
                averageMagInd = averageFreq/(42.577478518 * pow(10, -3))
                print("    Calculations - comparator")
                print('        f = '+ str(averageFreq)+ ' Hz')
                print('        B = '+ str(averageMagInd)+ ' nT')


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


