import numpy as np 
import matplotlib.pyplot as plt 
fh = open('test.txt')
import csv
import math
from scipy.signal import hilbert, chirp
from scipy.fft import fft, fftfreq


startComp = 25
stopComp = 400
hilbertStart = 500
hilbertEnd = 8820



enableCSV = 0
enablePlot_measuredSignal = 0
enablePlot_hilbert = 0
enablePlot_reconstructed = 0
enablePlot_instantaneousAngle = 0
enablePlot_instantaneousFrequency = 0
enablePlot_comparator = 0


def saveOneColumnToCSV(path, column):
    with open(path, 'w', newline='') as csvFile:
        csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
        for i in range(len(column)):   
            csvFileWriter.writerow([column[i]])                    
        csvFile.close()

def appendColumnsToCSV(path,time, column1, column2, column3, column4, column5):
    with open(path, 'a', newline='') as csvFile:
        csvFileWriter = csv.writer(csvFile, delimiter=',', quotechar='"', lineterminator='\n', quoting=csv.QUOTE_MINIMAL)
        csvFileWriter.writerow([time, column1,column2, column3, column4, column5])                    
        csvFile.close()

def computeFFTOfSignal(startSample, endSample, signal):    
    
    signalForFFT = []
    N = (endSample-startSample)

    for k in range(len(signal)):   
        if((k >= startSample) and (k < endSample)):
            signalForFFT.append(signal[k]- 32768)   
    y = np.zeros( N)
    y[:N] = signalForFFT
    y_fft = fft(y)
    x_fft = fftfreq(N, 1/fs)[:N//2]
    print(len(x_fft))
    
    locY = np.argmax(2.0/N * np.abs(y_fft[0:N//2])) # Find its location
    #plt.plot(2.0/totalN * np.abs(y_fft[0:totalN//2]))
    #plt.grid()
    #plt.show()
    frqY = x_fft[locY] # Get the actual frequency value
    return frqY

def computeFFTOfSignalWithZeroPadding(startSample, endSample, signal, zeropadding):    
    
    signalForFFT = []
    N = (endSample-startSample)

    for k in range(len(signal)):   
        if((k >= startSample) and (k < endSample)):
            signalForFFT.append(signal[k]- 32768)     
    totalN = zeropadding
    zeropadded_y = np.zeros(totalN)
    zeropadded_y[:N] = signalForFFT
    y_fft = fft(zeropadded_y)
    x_fft = fftfreq(totalN, 1/fs)[:totalN//2]
    print(len(x_fft))
    
    locY = np.argmax(2.0/totalN * np.abs(y_fft[0:totalN//2])) # Find its location
    #plt.plot(2.0/totalN * np.abs(y_fft[0:totalN//2]))
    #plt.grid()
    #plt.show()
    frqY = x_fft[locY] # Get the actual frequency value
    return frqY

def computeFrequencyWithHilbertTransform(hilbertStart, hilbertEnd):
    signal = []
    d_angle = []
    d_freq = []
    skippedEnd = 50
    hilbertSignalLength = hilbertEnd - hilbertStart - skippedEnd
    n = hilbertEnd - hilbertStart - skippedEnd - 1
    for k in range(len(y)):   
        if((k >= hilbertStart) and (k < hilbertEnd)):
            signal.append(y[k] - 32768) #shift signal to 0
    
    # xx = np.zeros(10)
    # analytic_signal=[]
    # a0=2/(np.pi*(0+1))
    # a2=2/(np.pi*(2+1))
    # a4=2/(np.pi*(4+1))
    # n=len(signal)
    # for k in range(len(signal)):   
    #     xx[0] = signal[k]
    #     analytic_signal.append(xx[4]+1j*((xx[0]-xx[9])*a0+(xx[1]-xx[7])*a2+(xx[3]-xx[5])*a4))
    #     xx[0] = signal[k]
    #     xx[1] = xx[0]
    #     xx[2] = xx[1]
    #     xx[3] = xx[2]
    #     xx[4] = xx[3]
    #     xx[5] = xx[4]
    #     xx[6] = xx[5]
    #     xx[7] = xx[6]
    #     xx[8] = xx[7]
    #     xx[9] = xx[8]
    analytic_signal = hilbert(signal)

    #omit last 50samples of analytic signal
    signal = signal[:len(signal)-skippedEnd]
    analytic_signal = analytic_signal[:len(analytic_signal)-skippedEnd]
    #d_angle = np.unwrap(np.angle(analytic_signal))
    #determine instantenous angle
    
    skippedBegining = 20
    for k in range(len(analytic_signal)-1):
        if(k > skippedBegining):  
            d_angle.append(np.angle(analytic_signal[k+1]/analytic_signal[k]))

    #determine instantenous frequency
    for k in range(len(d_angle)):  
        d_freq.append(d_angle[k] / (1/fs)/(2*math.pi))
    d_angle = np.cumsum(d_angle)

    # dtermine average frequency
    diff_d_angle = max(d_angle) - min(d_angle)
    tau=2 * math.pi * ((n-skippedBegining)/fs) / diff_d_angle
    f_average = 1/tau

    #Afterburner - too slow
    # optimizer_freq = 1 #+- 2 Hz 
    # optimizer_resolution = 400
    # evaluationOfOptimizer = []
    # reconstructedSig = []
    # amplitude = max(np.abs(signal)*0.5)

    # for j in range(optimizer_resolution):
    #     reconstructedSig = []
    #     for k in range(hilbertSignalLength):
    #         reconstructedSig.append(amplitude*math.cos(2*math.pi*(f_average + ((j/(optimizer_resolution/4))-optimizer_freq))*(k)/fs))
    #     tmp = np.sum(np.abs(np.subtract(signal,reconstructedSig)))
    #     evaluationOfOptimizer.append(tmp)
           
    # locY = np.argmin(evaluationOfOptimizer) # Find its location
    # print(f_average)
    # f_average = f_average + (locY/optimizer_resolution - optimizer_freq) # Get the actual frequency value
    # print(f_average)
            

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
        # print("        measured signal and hilbert transform")

    # plot reconstructed signal vs measured signal

    if(enablePlot_reconstructed):
        #create reconstructed signal with detemined frequency
        reconstructedSig = np.zeros(hilbertEnd- hilbertStart)
        amplitude =max(np.abs(signal)*0.5)
        #omit also last 50 samples
        for k in range(hilbertEnd- hilbertStart):   
            reconstructedSig[k] = (amplitude*math.cos(2*math.pi*f_average*(k)/fs))
        #plot
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
        # print("        measured signal and reconstructed signal")




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
        # print("        instantaneous angle")



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
        # print("        instantaneous frequency")

    return f_average






duration = 2.0
fs = 216000000/4898
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
try:
    appendColumnsToCSV(path='generatedFiles/results.csv',time="time", column1="hilbert transform", column2="fft without zero padding", column3= "fft with zero padding (bin width of 10 mHz)",column4= "fft with zero padding (bin width of 100 mHz)", column5 = "comparator")

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
            #if there is a new sample, save existing samples
            if(commandArgs[1] != previousSample):
                print("Started execution of " + str(commandArgs[1]) +". sample")
                if(previousSample != 0):
                    # write result into file
                    appendColumnsToCSV(path='generatedFiles/results.csv',time= ((int(previousSample) - 1) * 8), column1= B_hilbertTransform, column2= B_fft_noZeroPadding, column3= B_fft_zeroPadding1,column4= B_fft_zeroPadding2, column5 = B_comparator)
            previousSample = commandArgs[1]

            if procesingDataFromMeasurement == 1 and (commandArgs[2] == "extADC" or  commandArgs[2] == "intADC"):
                
                if(enableCSV):
                    #write into csv
                    saveOneColumnToCSV(path = 'generatedFiles/csv/' + graphTitle + '.csv', column = y)
                
        
        
               
                # determine analytic signal
                
                f_hilbertTransform = computeFrequencyWithHilbertTransform(hilbertStart, hilbertEnd)
                B_hilbertTransform = f_hilbertTransform/(42.577478518 * pow(10, -3)) 
                print("    Calculations - ADC - hilbert transform")
                print('        f = '+ str(f_hilbertTransform)+ ' Hz')
                print('        B = '+ str(B_hilbertTransform)+ ' nT')


                
                f_fft_noZeroPadding = computeFFTOfSignal(startSample = hilbertStart, endSample = hilbertEnd, signal = y)
                B_fft_noZeroPadding  = f_fft_noZeroPadding/(42.577478518 * pow(10, -3)) 
                print("    Calculations - ADC - with fft zero padding")
                print('        f = '+ str(f_fft_noZeroPadding)+ ' Hz')
                print('        B = '+ str(B_fft_noZeroPadding)+ ' nT')


                f_fft_zeroPadding1 = computeFFTOfSignalWithZeroPadding(startSample = hilbertStart, endSample = hilbertEnd, signal = y, zeropadding = 4410000)
                B_fft_zeroPadding1  = f_fft_zeroPadding1/(42.577478518 * pow(10, -3)) 
                print("    Calculations - ADC - withou fft zero padding")
                print('        f = '+ str(f_fft_zeroPadding1)+ ' Hz')
                print('        B = '+ str(B_fft_zeroPadding1)+ ' nT')

                
                f_fft_zeroPadding2 = computeFFTOfSignalWithZeroPadding(startSample = hilbertStart, endSample = hilbertEnd, signal = y, zeropadding = 441000)
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
                for i in range(startComp, stopComp):
                    #omit smaller than 35uT or greater than 60 uT
                    if(y[i] < 84573 or y[i] >144966 ):
                        skippedValues += 1
                    else:
                        average = average + y[i]
                average = average/(stopComp-startComp - skippedValues)
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

