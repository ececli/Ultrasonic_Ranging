import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq, rfft, rfftfreq

RATE = 64000

f0 = 22000
step = 500
fend = 30000

combined_abs_yf = np.zeros(320001)
while True:

    filename = 'data/Fulldata_20220330_'+str(f0)+'Hz.dat'
    fulldata = np.loadtxt(filename, delimiter = ",")
    print("length of "+filename+" is ",len(fulldata))

    DCoffset = np.mean(fulldata)
    normalAllData = fulldata-DCoffset
    
    yf = rfft(normalAllData)
    xf = rfftfreq(len(normalAllData), 1/RATE)
    combined_abs_yf[(xf>=f0-step/2) & (xf<f0+step/2)] = np.abs(yf[(xf>=f0-step/2) & (xf<f0+step/2)])
    f0 = f0 + step
    if f0>fend:
        break
    

    
    
    
    
    
# plt.figure()
# plt.plot(normalAllData)
# plt.show()


plt.figure()
plt.plot(xf,combined_abs_yf)
plt.show()
