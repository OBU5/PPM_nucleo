import numpy as np 
import matplotlib.pyplot as plt 
import math
from scipy.signal import hilbert, chirp
from scipy.fft import fft, fftfreq

enableCSV = 0
enablePlot_measuredSignal = 0
enablePlot_hilbert = 0
enablePlot_reconstructed = 0
enablePlot_instantaneousAngle = 0
enablePlot_instantaneousFrequency = 0
enablePlot_comparator = 0


def computeFFTOfSignal(signal, startSample, endSample, fs ):    
    
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

def computeFFTOfSignalWithZeroPadding(signal, startSample, endSample, fs, zeropadding):    
    
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

def computeFrequencyWithHilbertTransform(signal, startSample, endSample, fs):
    selectedSignal = []
    d_angle = []
    d_freq = []
    skippedEnd = 50
    hilbertSignalLength = endSample - startSample - skippedEnd
    n = endSample - startSample - skippedEnd - 1
    for k in range(len(signal)):   
        if((k >= startSample) and (k < endSample)):
            selectedSignal.append(signal[k] - 32768) #shift signal to 0
    
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
    analytic_signal = hilbert(selectedSignal)

    #omit last 50samples of analytic signal
    selectedSignal = selectedSignal[:len(selectedSignal)-skippedEnd]
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
        reconstructedSig = np.zeros(endSample- startSample)
        amplitude =max(np.abs(signal)*0.5)
        #omit also last 50 samples
        for k in range(endSample- startSample):   
            reconstructedSig[k] = (amplitude*math.cos(2*math.pi*f_average*(k)/fs))
        #plot
        fig = plt.figure(figsize=(30, 12))
        plt.subplot(2, 1, 1) 
        plt.plot(selectedSignal, label='Naměřený signál')
        plt.plot(reconstructedSig, label='Zrekonstruovaný signál')
        plt.xlabel("Vzorky")
        plt.ylabel("Amplituda")                
        plt.legend()         
        plt.title(graphTitle + " Rekonstrukce signálu") 

        plt.subplot(2, 4, 5)
        plt.xlim(0, 400)
        plt.plot(selectedSignal)
        plt.plot(reconstructedSig)
        plt.xlabel('step') 
        plt.ylabel('Amplitude')

        plt.subplot(2, 4, 6)
        plt.xlim(hilbertSignalLength/3 - 200, hilbertSignalLength/3 + 200)
        plt.plot(selectedSignal)
        plt.plot(reconstructedSig)
        plt.xlabel('step') 
        plt.ylabel('Amplitude')

        plt.subplot(2, 4, 7)
        plt.xlim(hilbertSignalLength/3 * 2 - 200, hilbertSignalLength/3 * 2 + 200)
        plt.plot(selectedSignal)
        plt.plot(reconstructedSig)
        plt.xlabel('step') 
        plt.ylabel('Amplitude')

        plt.subplot(2, 4, 8)
        plt.xlim(hilbertSignalLength - 400, hilbertSignalLength)
        plt.plot(selectedSignal)
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