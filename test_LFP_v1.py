from scipy.fft import fft, fftfreq
from scipy import signal
import twoWayRangingLib as func
import configparser
import numpy as np
import matplotlib.pyplot as plt



def getRefSignal(f0,duration,sr):
    # single-tone with frequency "f0" and duration "duration"
    # "sr" is the sampling rate
    Ns = duration * sr
    # t = np.r_[0.0:Ns]/sr
    t = np.arange(int(Ns))/sr
    return np.cos(2*np.pi*f0*t)

# Load Parameters
confFile = "UR_pyConfig.conf"
cp = configparser.ConfigParser()
cp.read(confFile)

f0 = cp.getint("SIGNAL","f0") 
duration = cp.getint("SIGNAL","duration") # microseconds
RATE = cp.getint("MIC","RATE")

RefSignal = func.getRefSignal(f0,duration/1000000.0,RATE)
RefSignal2 = getRefSignal(f0,duration/1000000.0,RATE)
xcorrelation = abs(np.correlate(RefSignal, RefSignal, mode = 'full'))

N = len(xcorrelation)


a = signal.argrelextrema(xcorrelation,np.greater)

plt.figure()
plt.plot(xcorrelation,'r-o')
plt.plot(a[0],xcorrelation[a[0]],'b*')
plt.show()

yf = fft(xcorrelation)
xf = fftfreq(N,1/RATE)[:N//2]
plt.figure()
plt.plot(xf,2.0/N * np.abs(yf[0:N//2]))
plt.show()


nyq = 0.5*RATE
normal_cutoff = 1000/nyq
order = 5
b, a  = signal.butter(order,normal_cutoff, btype='lowpass', analog = False)
filtered = signal.lfilter(b, a, xcorrelation)
plt.figure()
plt.plot(xcorrelation,'r-')
plt.plot(filtered,'b-o')
plt.show()
