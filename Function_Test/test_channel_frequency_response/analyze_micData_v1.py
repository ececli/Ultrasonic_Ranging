import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq, rfft, rfftfreq

RATE = 64000

filename = 'data/Fulldata_20220330_25000Hz.dat'
fulldata = np.loadtxt(filename, delimiter = ",")
print(len(fulldata))

DCoffset = np.mean(fulldata)
normalAllData = fulldata-DCoffset
plt.figure()
plt.plot(normalAllData)
plt.show()

yf = rfft(normalAllData)
xf = rfftfreq(len(normalAllData), 1/RATE)
plt.figure()
plt.plot(xf,np.abs(yf),'r-o')
plt.show()