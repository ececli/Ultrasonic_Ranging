import sys
import time
import numpy as np
import twoWayRangingLib_v2 as func
import Impl_Pico_Lib as func2
import configparser
from scipy import signal
import matplotlib.pyplot as plt



fulldata = np.loadtxt('Fulldata_responder_19delay.dat',delimiter=',')
print(len(fulldata))
print(fulldata.shape)

# Load Parameters
confFile = "UR_pyConfig_v2.conf"
cp = configparser.ConfigParser()
cp.read(confFile)


RATE = cp.getint("MIC","RATE")
CHUNK = cp.getint("MIC", "CHUNK")


f0 = cp.getint("SIGNAL","f0") 
duration = cp.getfloat("SIGNAL","duration") # microseconds
THRESHOLD = cp.getfloat("SIGNAL","THRESHOLD")
THRESHOLD_TX = cp.getfloat("SIGNAL","THRESHOLD_TX")
NumRanging = cp.getint("SIGNAL","NumRanging")
IgnoredSamples = cp.getint("SIGNAL","IgnoredSamples")







RefSignal = func.getRefSignal(f0,duration,RATE, 0)
RefSignal2 = func.getRefSignal(f0,duration,RATE, np.pi/2)

NumSigSamples = len(RefSignal)


# Peak Shape:
peak_interval = int(NumSigSamples/100)
peak_width = int(NumSigSamples/100)

'''
wholeData = []
k=0
while True:
    wholeData.append(fulldata[k])
    k=k+1
    if k==len(fulldata):
        break
    
'''
wholeData = fulldata.flatten()
print(fulldata[0])
print(fulldata[1])
print(wholeData[0:100])
autoc = func.noncoherence(wholeData, RefSignal, RefSignal2)

Index1, peak1 = func.peakDetector(autoc,
                                  THRESHOLD,
                                  peak_interval,
                                  peak_width)

plt.figure()
plt.plot(autoc,'r-o')
plt.plot(Index1,autoc[Index1],'bs')
plt.show()


