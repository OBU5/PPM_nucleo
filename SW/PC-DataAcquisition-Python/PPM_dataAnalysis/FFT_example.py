import numpy as np 
import matplotlib.pyplot as plt 
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt



def computeFFTOfSignal(startSample, endSample, signal):    
    signalForFFT = []
    N = (endSample-startSample)
    for k in range(len(signal)):   
        if((k >= startSample) and (k < endSample)):
            signalForFFT.append(signal[k])

    y_fft = fft(signalForFFT)
    x_fft = fftfreq(N, 1/fs)[:N//2]
    plt.plot(x_fft, 2.0/N * np.abs(y_fft[0:N//2]))
    plt.grid()
    plt.show()


    zeroN = 20000
    totalN = zeroN + N
    zeropadded_y = np.zeros(zeroN + N)
    zeropadded_y[:N] = signalForFFT

    y_fft = fft(zeropadded_y)
    x_fft = fftfreq(totalN, 1/fs)[:totalN//2]
    plt.plot(x_fft, 2.0/totalN * np.abs(y_fft[0:totalN//2]))
    plt.grid()
    plt.show()


# Number of sample points
n = 600
# sample spacing
fs = 800.0
Ts = 1.0 / fs
x = np.linspace(0.0, n/fs, n, endpoint=False)
y = np.sin(50.0 * 2.0*np.pi*x) + 0.5*np.sin(80.0 * 2.0*np.pi*x)
computeFFTOfSignal(startSample = 0, endSample = 500, signal = y)
