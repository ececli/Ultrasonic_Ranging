import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq, rfft, rfftfreq

RATE = 64000

filename = 'Fulldata_20220413_0855.dat'
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

numBins = 100
lenEvaData = int(len(normalAllData)/numBins)
xf_eva = rfftfreq(lenEvaData, 1/RATE)
k=0
max_yf = []
cor_xf = []
while True:
    yf_eva = rfft(normalAllData[k:k+lenEvaData])
    k=k+lenEvaData
    ymax = np.max(np.abs(yf_eva[xf_eva > 19000]))
    corx = xf_eva[np.abs(yf_eva) == ymax]
    if len(corx)>1:
        print(k,corx)
    max_yf.append(ymax)
    cor_xf.append(corx[0])
    # max_yf.append(np.max(np.abs(yf_eva)))
    # cor_xf.append(xf_eva[np.argmax(np.abs(yf_eva))])
    
    if k >= len(normalAllData):
        break

print("Mean = ", np.mean(max_yf))
print("Std = ", np.std(max_yf))
print("Corresponding frequencies are ", np.unique(cor_xf))

plt.figure()
plt.plot(max_yf,'o')
plt.show()
    
