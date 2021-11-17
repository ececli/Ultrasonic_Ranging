import numpy as np
import matplotlib.pyplot as plt
import os
import time
from scipy import signal
import queue
# from numba import jit


def getRefSignal(f0,duration,sr, phi=0):
    # single-tone with frequency "f0" and duration "duration"
    # "sr" is the sampling rate
    Ns = duration * sr
    # t = np.r_[0.0:Ns]/sr
    t = np.arange(int(Ns))/sr
    return np.sin(2*np.pi*f0*t + phi)

def genBPF(order,L,H,fs):
    l = L/(fs/2)
    h = H/(fs/2)
    if h>=1:
        sos = signal.butter(order, l, btype = 'high',output = 'sos')
    else:
        sos = signal.butter(order, [l,h], btype = 'bandpass',output = 'sos')
    return sos


def noncoherence(sig,refSignal1,refSignal2):
    # non-coherence method: use both sin and cos reference signals
    autoc1 = np.correlate(sig, refSignal1, mode = 'valid')
    autoc2 = np.correlate(sig, refSignal2, mode = 'valid')
    autoc = np.sqrt((autoc1*autoc1 + autoc2*autoc2)/2)
    return autoc

def peakDetector(seq,THRESHOLD, peak_interval, peak_width):
    Index, _ = signal.find_peaks(seq,
                                 height=THRESHOLD,
                                 distance=peak_interval,
                                 width=peak_width)
    return Index, seq[Index]



def peakFilter(Index, Peaks, TH = 0.8):
    feasiblePeakTH = TH*np.max(Peaks)
    feasiblePeak = Peaks[Peaks>=feasiblePeakTH]
    feasibleIndex = Index[Peaks>=feasiblePeakTH]
    return feasibleIndex[0], feasiblePeak[0]


def calAbsSampleIndex(counter, Index, CHUNK, NumIgnoredFrame = 0, NumReqFrames=0):
    return (counter-NumIgnoredFrame - NumReqFrames)*CHUNK + Index


def combineFrames(frames):
    return np.concatenate(frames)



# @jit(nopython=True)
def goertzel(x,k,N):
    w = 2*np.pi*k/N
    cw = np.cos(w)
    z1 = 0
    z2 = 0
    y = np.zeros(len(x))
    for k in range(len(x)):
        if k < N:

            z0 = x[k] + 2* cw * z1 - z2
        else:
            z0 = x[k] + 2* cw * z1 - z2 - x[k-N]
        y[k] = z2*z2 + z1*z1 - 2* cw * z1*z2
        z2 = z1
        z1 = z0
    return y

k=100
N=256
w = 2*np.pi*k/N
cw = np.cos(w)


# @jit(nopython=True)
def goertzel_RT(x,x_old,z1,z2):
    y = np.zeros(len(x))
    for k in range(len(x)):
        z0 = x[k] - x_old[k] + 2* cw * z1 - z2
        y[k] = z2*z2 + z1*z1 - 2* cw * z1*z2
        z2 = z1
        z1 = z0
    return y, z1, z2


CHUNK = 64
f0 = 25000
duration=0.004
RATE=64000
THRESHOLD = 7000
RefSignal = getRefSignal(f0,duration,RATE, 0)
RefSignal2 = getRefSignal(f0,duration,RATE, np.pi/2)

NumSigSamples = len(RefSignal)


# init functions


# Peak Shape:
peak_interval = int(NumSigSamples/10)
peak_width = int(NumSigSamples/100)


mic = []
rawData = []
counter = 0
with open('rec3_64.dat', "rb") as f:
    byte = f.read(4)
    while byte:
        # Do stuff with byte.
        counter = counter + 1
        intData = int.from_bytes(byte,'little',signed = True)
        rawData.append(intData)
        mic.append(intData>>14)
        byte = f.read(4)
        # if counter == 20:
        #     break


print('length of the data is ',len(mic))
print('-'*20)
print(len(np.unique(rawData)))
print(np.sort(np.unique(rawData)))
print(np.diff(np.sort(np.unique(rawData))))
print(np.min(np.diff(np.sort(np.unique(rawData)))))

print('-'*20)
print(len(np.unique(mic)))
print(np.sort(np.unique(mic)))
print(np.diff(np.sort(np.unique(mic))))
print(np.min(np.diff(np.sort(np.unique(mic)))))
print('-'*20)
sig = np.asarray(mic)


autoc = noncoherence(sig,RefSignal,RefSignal2)
Index1, peak1 = peakDetector(autoc,
                             THRESHOLD,
                             peak_interval,
                             peak_width)

print(Index1)
print(np.diff(Index1))

startTime = time.time()
autoc = noncoherence(sig,RefSignal,RefSignal2)
Duration = time.time()-startTime
print("Duration of old method is ",Duration)

startTime = time.time()
y = goertzel(sig,100,256)
Duration = time.time()-startTime
print("Duration of g method is ",Duration)

plt.figure(figsize=(20,8))
plt.plot(y,'r-o')
plt.show()

plt.figure(figsize=(20,8))
plt.plot(autoc,'b-o')
plt.show()




#######################################################
print("#"*20)
print('Start Simulation - Old Method')
print("#"*20)
CHUNK=64
initialStage_Duration = 1
warmUpStage_Duration = 1
NumIgnoredFrame = int(np.ceil(initialStage_Duration*RATE/CHUNK))
NumWarmUpFrame = int(np.ceil(warmUpStage_Duration*RATE/CHUNK))
micCounter = 0
counter = 0
Flag_initial = True
Flag_warmUp = True
Data_warmUp = []
DCOffset = 0
previousData = np.empty(0)
previousAutoc = np.empty(0)
TH = 7000
autocStartPointer = 0
absIndex = []

#######################################################

while True:
    micData = np.asarray(mic[micCounter:micCounter+CHUNK])
    micCounter = micCounter + CHUNK
    if micCounter > len(mic):
        print('done')
        print("Duration is ",time.time()-startTime)
        break

    counter = counter + 1

    if Flag_initial:
        if counter < NumIgnoredFrame:
            continue
        if counter == NumIgnoredFrame:
            Flag_initial = False
            counter = 0
            print('Beginning Data has been thrown away')
            continue

   

    if Flag_warmUp:
        if counter < NumWarmUpFrame:
            Data_warmUp.append(micData)
            continue
        if counter == NumWarmUpFrame:
            if len(Data_warmUp)>0:
                DCOffset = int(np.mean(combineFrames(Data_warmUp)))
            else:
                DCOffset = 0
            Flag_warmUp = False
            counter = 0
            print("DC Offset is ",DCOffset)
            continue


    ## for debug and record purposes:
    if counter == 1:
        startTime = time.time()
        print('Start Time is ',startTime)
    ## End



    currentData = micData - DCOffset
    wholeData = np.concatenate((previousData, currentData))
    if len(wholeData) < NumSigSamples:
        previousData = wholeData
        continue
    else:
        previousData = wholeData[-(NumSigSamples-1):]
    

    
    # Peak Detection Algorithm


    autoc = noncoherence(wholeData, RefSignal, RefSignal2)

    wholeAutoc = np.concatenate((previousAutoc, autoc))

    Index1, peak1 = peakDetector(wholeAutoc,
                                 TH,
                                 peak_interval,
                                 peak_width)
    if Index1.size>0: # if signal is detected
        ## For debug purpose, print out progress:
        # print("Signal Detected")
        ## End
        Index, Peak = peakFilter(Index1, peak1, TH = 1)
        if (Index>=(NumSigSamples/2)) and (Index<= (len(wholeAutoc)-(NumSigSamples/2))):
            # Calculate absolute Index
            # absIndex.append(autocStartPointer*CHUNK+Index)
            # Relationship between autocStartPointer and counter is that
            # autocStartPointer + np.ceil(signal_length*3/CHUNK) = counter
            # But in the very beginning, when len(wholeAutoc) is not full,
            # autocStartPointer = 0
            # Therefore, the complete form of autocStartPointer is
            # autocStartPointer = max(0,counter - int(np.ceil(NumSigSamples*3/CHUNK)))
            # For details of how to compute this number, see the mobile picture at
            # 12:34 pm 11/04/2021. 
            autocStartPointer = max(0,counter - int(np.ceil(NumSigSamples*3/CHUNK)))
            absIndex.append(autocStartPointer*CHUNK+Index)

            print(counter, len(wholeAutoc),Index1,absIndex[-1])
            previousAutoc = wholeAutoc[int(len(wholeAutoc)/CHUNK)*CHUNK:]
            continue


    # Prepare for the next loop
    if len(wholeAutoc) <= 2*NumSigSamples:
        previousAutoc = wholeAutoc
    else:
        previousAutoc = wholeAutoc[CHUNK:]
        # autocStartPointer = autocStartPointer + 1



print(np.unique(absIndex))
AAA = np.diff(np.unique(absIndex))
print(AAA)




#######################################################
print("#"*20)
print('Start Simulation - G Filter Method')
print("#"*20)

micCounter = 0
counter = 0
Flag_initial = True
Flag_warmUp = True

Data_warmUp = []
DCOffset = 0
keptData = np.zeros(NumSigSamples)
previousAutoc = np.empty(0)
autocStartPointer = 0
absIndex = []
TH = 1e7
z1=0
z2=0
#######################################################
while True:
    micData = np.asarray(mic[micCounter:micCounter+CHUNK])
    micCounter = micCounter + CHUNK
    if micCounter > len(mic):
        print('done')
        print("Duration is ",time.time()-startTime)
        break
    
    counter = counter + 1

    if Flag_initial:
        if counter < NumIgnoredFrame:
            continue
        if counter == NumIgnoredFrame:
            Flag_initial = False
            counter = 0
            print('Beginning Data has been thrown away')
            continue

   

    if Flag_warmUp:
        if counter < NumWarmUpFrame:
            Data_warmUp.append(micData)
            continue
        if counter == NumWarmUpFrame:
            if len(Data_warmUp)>0:
                DCOffset = int(np.mean(combineFrames(Data_warmUp)))
            else:
                DCOffset = 0
            Flag_warmUp = False
            counter = 0
            print("DC Offset is ",DCOffset)
            continue


    ## for debug and record purposes:
    if counter == 1:
        startTime = time.time()
        print('Start Time is ',startTime)
    ## End


    currentData = micData - DCOffset
    
    # Peak Detection Algorithm

    y,z1,z2 = goertzel_RT(currentData,keptData[0:CHUNK],z1,z2)
    keptData = np.concatenate((keptData[CHUNK:], currentData))

    wholeAutoc = np.concatenate((previousAutoc, y))
    if len(wholeAutoc) <= 2*NumSigSamples:
        previousAutoc = wholeAutoc
        continue
    else:
        previousAutoc = wholeAutoc[CHUNK:]

    Index1, peak1 = peakDetector(wholeAutoc,
                                 TH,
                                 peak_interval,
                                 peak_width)
    if Index1.size>0: # if signal is detected
        ## For debug purpose, print out progress:
        # print("Signal Detected")
        ## End
        Index, Peak = peakFilter(Index1, peak1, TH = 1)
        if (Index>=(NumSigSamples/2)) and (Index<= (len(wholeAutoc)-(NumSigSamples/2))):

            absIndex.append(counter*CHUNK-CHUNK+Index)

            print(counter, len(wholeAutoc),Index1,absIndex[-1])
            previousAutoc = wholeAutoc[int(len(wholeAutoc)/CHUNK)*CHUNK:]
            continue




print(np.unique(absIndex))
BBB = np.diff(np.unique(absIndex))
print(BBB)