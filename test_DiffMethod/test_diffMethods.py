import numpy as np
import matplotlib.pyplot as plt
import os
import time
from scipy import signal
from numba import jit
from cffi import FFI

'''
Case 1: RPi1: facing to the other RPi 1.3 meters away
Case 2: RPi1: facing paper wall 30 cm away
Case 3: RPi1: facing curtain 70 cm away
Case 4: RPi1: Under Table and Chairs
Case 5: RPi1: facing to the other RPi 35 cm away
Case 6: RPi1: facing to the other RPi 35 cm away with data loss
Case 7: RPi2: facing to the other RPi 35 cm away
Case 8: RPi2: facing to the other RPi 70 cm away
'''


dt = np.dtype([('counter', 'i4'),
               ('status', 'i4'),
               ('timestamp', 'f8')
               ])

dt_settings = np.dtype([('lag', 'i4'),
                        ('width', 'i4'), 
                        ('threshold', 'f8'),
                        ('influence', 'f8'),
                        ('mph', 'f8')
                        ])

dt_state = np.dtype([('avgFilter', 'f8'),
                     ('std2', 'f8'),
                     ('write_addr', 'i4'),
                     ('time_between', 'i4'),
                     ('length', 'i4'),
                     ('pk_idx', 'i4'),
                     ('pk', 'f8'),
                     ('pk_time', 'i4'),
                     ])

fd = open('RawData_pureRX_330cm_Rev.dat', 'rb')
raw = fd.read()
fd.close()

def getRefSignal(f0,duration,sr, phi=0):
    # single-tone with frequency "f0" and duration "duration"
    # "sr" is the sampling rate
    Ns = duration * sr
    # t = np.r_[0.0:Ns]/sr
    t = np.arange(int(Ns))/sr
    return np.sin(2*np.pi*f0*t + phi)


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



@jit(nopython=True)
def goertzel(x,k,N):
    k=100
    N=256
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
        z2 = z1
        z1 = z0
        y[k] = np.sqrt(z2*z2 + z1*z1 - 2* cw * z1*z2)
    return y


@jit
# Sae Woo's code
def sg_v2(x, k=25*4):
    Pxx = []
    N = 64*4
    w = 2*np.pi*k/N;
    cw = np.cos(w);
    c = 2*cw;
    sw = np.sin(w);
    z1=0;
    z2=0;
    #while (idx < (len(x)-1)):
    for idx in range(len(x)):
        if idx<N:
            z0 = x[idx] + c*z1 - z2;
        else:
            z0 = x[idx] - x[idx-N] + c*z1 -z2;
        z2 = z1;
        z1 = z0;

        P = np.sqrt(z2*z2 + z1*z1 - c * z1*z2)
        Pxx.append(P)
    return np.array(Pxx)


k=100
N=256
w = 2*np.pi*k/N
cw = np.cos(w)

@jit(nopython=True)
def goertzel_RT(x,x_old,z1,z2):
    y = np.zeros(len(x))
    for k in range(len(x)):
        z0 = x[k] - x_old[k] + 2* cw * z1 - z2
        z2 = z1
        z1 = z0
        y[k] = np.sqrt(z2*z2 + z1*z1 - 2* cw * z1*z2)
    return y, z1, z2



@jit
def sg_block_v1(x, offset, z, Pxx, c, block_size=64, k=25*4, window_size=64*4):
    z1, z2 = z
    for idx in range(block_size):
        z0 = x[offset + idx] - x[offset + idx-window_size] + c*z1 -z2;
        z2 = z1;
        z1 = z0;

        Pxx[idx] = np.sqrt(z2*z2 + z1*z1 - c * z1*z2)
    z[0] = z1
    z[1] = z2
    

@jit
def sg_block_v2(x, offset, z, Pxx, c, block_size, window_size):
    z1, z2 = z
    for idx in range(block_size):
        z0 = x[offset + idx] - x[offset + idx-window_size] + c*z1 -z2;
        z2 = z1;
        z1 = z0;
        Pxx[idx] = np.sqrt(z2*z2 + z1*z1 - c * z1*z2)
    z[0] = z1
    z[1] = z2


@jit
def peak_marking_v2(y, lag, threshold, influence, width, mph, signals): #, filteredY):
    filteredY = np.zeros(lag)
    avgFilter = 0
    stdFilter = 0
    std2 = stdFilter**2
    pks = []
    start, length, pk, pk_idx = -1, 0, -1, -1
    for i in range(0, len(y)):
        # if i == lag:
        #     print(avgFilter, stdFilter)
        oldValue = filteredY[i%lag] # store this for use to update mean and std
        if abs(y[i] - avgFilter) > threshold * stdFilter and (i>=lag):
            if (y[i] > avgFilter):
                signals[i] = 1
                if start<0:
                    start, length = i, 1
                    #print(start)
                else:
                    #print(i, start, length)
                    length += 1
                if y[i]>pk:
                    pk, pk_idx = y[i], i
            else:
                signals[i] = -1
                if (start>0) and (length>width) and (pk>mph):
                    pks.append([start, length, pk_idx, pk])
                    # print("-1", i, start, length, pk_idx, pk)
                    start, length, pk, pk_idx = -1, 0, -1, -1
            filteredY[i%lag] = influence * y[i] + (1 - influence) * filteredY[(i-1)%lag]
        else:
            signals[i] = 0
            filteredY[i%lag] = y[i]
            if (start>0) and (length>width) and (pk>mph):
                pks.append([start, length, pk_idx, pk])
                # print("0:", i, start, length, pk, pk_idx)
            start, length, pk, pk_idx = -1, 0, -1, -1
        prevAvg = avgFilter
        avgFilter = avgFilter + (filteredY[i%lag] - oldValue) / lag
        #print(prevAvg, avgFilter, filteredY[i%lag], oldValue)
        # avgFilter = np.mean(filteredY)
        std2 = std2 + (filteredY[i%lag] - oldValue)*(filteredY[i%lag] + oldValue - avgFilter - prevAvg) /lag
        stdFilter = std2**0.5
        #stdFilter = np.std(filteredY)
    return pks


@jit
def pk_mark(y, lag, threshold, influence, width, mph):
    """Find and mark peaks using z-scor algorithm
    
    Returns:  pks, signals
    pks -- list of peaks... [start, length, pk_idx, pk]
    signals -- numpy array that is the length of the input.  
            1 if in + peak, -1 in - peak, 0 not in peak
            
    Keyword arguments:
    y -- numpy array of input signal
    lag -- integer of how many points to compute mean and std over
    threshold -- double: multiples of std deviation that constitute a peak
    influence -- double: amount signal in the peak influences the running avg and std
    width -- integer:  minimum peak width to be added to the output
    mph: double: minimum peak hieght
    """
    signals = np.zeros(len(y))
    pks = peak_marking_v2(y, lag, threshold, influence, width, mph, signals)
    return pks, signals


@jit
def peak_marking_block(y, yLen, filteredY, settings, state, signals): #, filteredY):
    lag = settings.lag

    avgFilter = state.avgFilter
    std2 = state.std2
    stdFilter = state.std2**0.5
    pks = []
    # start, length, pk, pk_idx = -1, 0, -1, -1
    write_addr = state.write_addr
    time_between = state.time_between
    length = state.length
    pk = state.pk
    pk_idx = state.pk_idx
    pk_time = state.pk_time
    #print(write_addr, length, pk, pk_idx)
    for i in range(yLen):
        # if (i%64) == 0:
        #    print(avgFilter, stdFilter)
        write_addr += 1
        time_between += 1
        
        idx = write_addr % lag
        prev_idx = (write_addr -1 + lag) % lag
        
        oldValue = filteredY[idx] # store this for use to update mean and std

        if abs(y[i] - avgFilter) > settings.threshold * stdFilter and (write_addr>=lag):
            if (y[i] > avgFilter):
                signals[i] = 1
                #if start<0:
                if length==0:
                    start, length = i, 1
                    #print(start)
                else:
                    #print(i, start, length)
                    length += 1
                if y[i]>pk:
                    pk, pk_idx = y[i], i
                    pk_time = time_between
            else:
                signals[i] = -1
                if (length>0) and (length>settings.width) and (pk>settings.mph):
                    # pks.append([start, length, pk_idx, pk, pk_time])
                    pks.append([pk_time, length, pk])
                    print("-1", i, start, length, pk_idx, pk)
                    start, length, pk, pk_idx = -1, 0, -1, -1
                    time_between = time_between - pk_time
            filteredY[idx] = settings.influence * y[i] + (1 - settings.influence) * filteredY[prev_idx]
        else:
            signals[i] = 0
            filteredY[idx] = y[i]
            if (length>0) and (length>settings.width) and (pk>settings.mph):
                #pks.append([start, length, pk_idx, pk, pk_time])
                pks.append([pk_time, length, pk])
                #print("0:", i, start, length, pk, pk_idx, time_between, pk_time)
                time_between = time_between - pk_time 
            start, length, pk, pk_idx = -1, 0, -1, -1
        prevAvg = avgFilter
        avgFilter = avgFilter + (filteredY[idx] - oldValue) / lag
        #print(prevAvg, avgFilter, filteredY[i%lag], oldValue)
        # avgFilter = np.mean(filteredY)
        std2 = std2 + (filteredY[idx] - oldValue)*(filteredY[idx] + oldValue - avgFilter - prevAvg) /lag
        stdFilter = std2**0.5
        #stdFilter = np.std(filteredY)
    state.std2 = std2
    state.avgFilter = avgFilter
    state.write_addr = write_addr
    state.time_between = time_between
    state.length = length
    state.pk = pk
    state.pk_idx = pk_idx
    state.pk_time = pk_time

    return pks

@jit
def pk_mark_block(y, settings, state):
    """Find and mark peaks using z-scor algorithm
    
    Returns:  pks, signals
    pks -- list of peaks... [start, length, pk_idx, pk]
    signals -- numpy array that is the length of the input.  
            1 if in + peak, -1 in - peak, 0 not in peak
            
    Keyword arguments:
    y -- numpy array of input signal
    settings -- numpy record structure with parameters for identifying peaks
    state -- numpy record structure that stores the state of peak finding between blocks of data
    """
    signals = np.zeros(len(y))
    filteredY = np.zeros(settings.lag)
    pks = peak_marking_block(y, len(y), filteredY, settings, state, signals)
    print(state)
    return pks, signals





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


# For new data format
len_oneBuffer = 16+CHUNK*4
numBuffer = len(raw)/len_oneBuffer
print("There are totally %.2f buffers" % numBuffer)

counter_all = []
mic_data = []
for k in range(int(numBuffer)):
    header = np.frombuffer(raw[k*len_oneBuffer:k*len_oneBuffer+16], dtype=dt)
    mic = np.frombuffer(raw[k*len_oneBuffer+16:(k+1)*len_oneBuffer], dtype=np.int32)
    mic_data.append(mic>>14)
    counter = header[0][0]
    status = header[0][1]
    TS = header[0][2]
    if status:
        print(counter, status)
    counter_all.append(counter)


temp = np.unique(np.diff(counter_all))
if len(np.unique(np.diff(counter_all))) != 1:
    print("Difference between counter is ", temp)
    


sig = np.concatenate(mic_data)





autoc = noncoherence(sig,RefSignal,RefSignal2)
Index1, peak1 = peakDetector(autoc,
                             THRESHOLD,
                             peak_interval,
                             peak_width)


## Method 1: Sin-Cos filtering, and scipy peak finding:
startTime = time.time()
autoc = noncoherence(sig,RefSignal,RefSignal2)
Duration = time.time()-startTime
print("Duration of sin-cos filtering is ",Duration)

startTime = time.time()
Index1, peak1 = peakDetector(autoc,
                             THRESHOLD,
                             peak_interval,
                             peak_width)
Duration = time.time()-startTime
print("Duration of scipy peak finding algorithm is ",Duration)
print(Index1)
print(np.diff(Index1))




y2 = sg_v2(sig, k=100)
y = goertzel(sig,100,256)



startTime = time.time()
y2 = sg_v2(sig, k=100)
Duration = time.time()-startTime
print("Duration of goertzel filter method (2) is ",Duration)


startTime = time.time()
y = goertzel(sig,100,256)
Duration = time.time()-startTime
print("Duration of goertzel filter method is ",Duration)


#######################################################################
#######################################################################
## Part II: Evaluate the speed and filtering with chunk-data

result_1 = np.zeros(int(numBuffer)*CHUNK)
Duration_stat = np.zeros(int(numBuffer))
z1, z2 = 0, 0
keptData = np.zeros(NumSigSamples)
for k in range(int(numBuffer)):
    header = np.frombuffer(raw[k*len_oneBuffer:k*len_oneBuffer+16], dtype=dt)
    mic = np.frombuffer(raw[k*len_oneBuffer+16:(k+1)*len_oneBuffer], dtype=np.int32)
    mic = mic>>14
    startTime = time.time()
    y,z1,z2 = goertzel_RT(mic,keptData[0:CHUNK],z1,z2)
    keptData = np.concatenate((keptData[CHUNK:], mic))
    result_1[k*CHUNK:(k+1)*CHUNK] = y
    duration = time.time()-startTime
    Duration_stat[k] = duration

print("Chang's G-filter method")
print("total time is ",np.sum(Duration_stat))
print("mean time is ",np.mean(Duration_stat))
print("std is ",np.std(Duration_stat))



print("#"*20)
result_2 = np.zeros(int(numBuffer)*CHUNK)
Duration_stat = np.zeros(int(numBuffer))
previousData = np.zeros(NumSigSamples-1)
for k in range(int(numBuffer)):
    header = np.frombuffer(raw[k*len_oneBuffer:k*len_oneBuffer+16], dtype=dt)
    mic = np.frombuffer(raw[k*len_oneBuffer+16:(k+1)*len_oneBuffer], dtype=np.int32)
    mic = mic>>14
    startTime = time.time()
    wholeData = np.concatenate((previousData, mic))
    previousData = wholeData[-(NumSigSamples-1):]
    y = noncoherence(wholeData, RefSignal, RefSignal2)
    result_2[k*CHUNK:(k+1)*CHUNK] = y
    duration = time.time()-startTime
    Duration_stat[k] = duration

print("Chang's sin-cos method")
print("total time is ",np.sum(Duration_stat))
print("mean time is ",np.mean(Duration_stat))
print("std is ",np.std(Duration_stat))




print("#"*20)
result_3 = np.zeros(int(numBuffer)*CHUNK)
Duration_stat = np.zeros(int(numBuffer))
k=100
window_size=256
w = 2*np.pi*k/window_size
c = 2*np.cos(w)

z = np.zeros(2)
ring = np.zeros(512, np.int32)
write_addr = 0

for k in range(int(numBuffer)):
    header = np.frombuffer(raw[k*len_oneBuffer:k*len_oneBuffer+16], dtype=dt)
    mic = np.frombuffer(raw[k*len_oneBuffer+16:(k+1)*len_oneBuffer], dtype=np.int32)
    mic = mic>>14
    startTime = time.time()
    ring[(write_addr):(write_addr+CHUNK)] = mic
    Pxx = result_3[k*CHUNK:(k+1)*CHUNK]
    sg_block_v2(ring, write_addr, z, Pxx, c, 64, 256)
    # sg_block_v1(ring, write_addr, z, Pxx, c)
    write_addr += CHUNK
    write_addr &= (512-1)
    duration = time.time()-startTime
    Duration_stat[k] = duration

print("Sae Woo's g-filter method")
print("total time is ",np.sum(Duration_stat))
print("mean time is ",np.mean(Duration_stat))
print("std is ",np.std(Duration_stat))


# print(np.where(result_3 - result_1))




#######################################################################
#######################################################################
## Part III: Evaluate speed of peaking finding algorithm

'''
pks, s = pk_mark(result_3, 256, 2, 5e-4, 128, 300)

plt.figure()
plt.plot(result_3,'r-o')
for pk in pks:
    plt.plot(pk[2],pk[3],'bs')
plt.show()
'''

k=100
w = 2*np.pi*k/NumSigSamples
c = 2*np.cos(w)
z = np.zeros(2)
ring = np.zeros(512, np.int32)
write_addr = 0


settings_np = np.recarray(1, dtype=dt_settings)[0]
state = np.recarray(1, dtype=dt_state)[0]

settings_np.mph = 300
settings_np.lag = 256
settings_np.width = 128
settings_np.threshold = 2
settings_np.influence = 5e-4
#print('settings', settings_np)

state.avgFilter = 0
state.std2 = 0
state.write_addr = -1
state.length = 0
state.pk_idx = -1
state.pk = -1
state.time_between = -1
state.pk_time = -1


settings = settings_np
filteredY = np.zeros(settings.lag)
packet_s = np.zeros(CHUNK)
pks_blocks = []

Pxx = np.zeros(CHUNK)
Duration_filter = np.zeros(int(numBuffer))
Duration_peak = np.zeros(int(numBuffer))
for k in range(int(numBuffer)):
    header = np.frombuffer(raw[k*len_oneBuffer:k*len_oneBuffer+16], dtype=dt)
    mic = np.frombuffer(raw[k*len_oneBuffer+16:(k+1)*len_oneBuffer], dtype=np.int32)
    mic = mic>>14
    startTime = time.time()
    # ------------ Filtering ------------- #
    ring[(write_addr):(write_addr+CHUNK)] = mic
    sg_block_v2(ring, write_addr, z, Pxx, c, CHUNK, NumSigSamples)
    write_addr += CHUNK
    write_addr &= (512-1)
    ########################################
    duration = time.time()-startTime
    Duration_filter[k] = duration
    startTime = time.time()
    # ------ peak finding alrogithm ------ #
    result = peak_marking_block(Pxx, len(Pxx), filteredY, settings, state, packet_s)
    if result:
        pks_blocks.extend(result)
    
    ########################################
    duration = time.time()-startTime
    Duration_peak[k] = duration
    
print("#"*40)
print("#"*40)
print("Z-score Peaking Finding Method")
print("filtering function total time is ",np.sum(Duration_filter))
print("filtering function mean time is ",np.mean(Duration_filter))
print("filtering function std is ",np.std(Duration_filter))
print("peak finding function total time is ",np.sum(Duration_peak))
print("peak finding function mean time is ",np.mean(Duration_peak))
print("peak finding function std is ",np.std(Duration_peak))
print("#"*40)

'''
absIndex = 0
plt.figure()
plt.plot(result_3,'r-o')
for pk in pks_blocks:
    absIndex += pk[0]
    plt.plot(absIndex,pk[2],'bs')
plt.show()
'''

################################################################################
################################################################################


k=100
w = 2*np.pi*k/NumSigSamples
c = 2*np.cos(w)
z = np.zeros(2)
ring = np.zeros(512, np.int32)
write_addr = 0




previousAutoc = np.empty(0)

THRESHOLD = 2000
peak_interval = 256
peak_width = 192

pks_blocks_2 = []
Pxx = np.zeros(CHUNK)
Duration_filter = np.zeros(int(numBuffer))
Duration_peak = np.zeros(int(numBuffer))
for k in range(int(numBuffer)):
    header = np.frombuffer(raw[k*len_oneBuffer:k*len_oneBuffer+16], dtype=dt)
    mic = np.frombuffer(raw[k*len_oneBuffer+16:(k+1)*len_oneBuffer], dtype=np.int32)
    mic = mic>>14
    startTime = time.time()
    # ------------ Filtering ------------- #
    ring[(write_addr):(write_addr+CHUNK)] = mic
    sg_block_v2(ring, write_addr, z, Pxx, c, CHUNK, NumSigSamples)
    write_addr += CHUNK
    write_addr &= (512-1)
    ########################################
    Duration_filter[k] = time.time()-startTime
    startTime = time.time()
    # ------ peak finding alrogithm ------ #
    
    wholeAutoc = np.concatenate((previousAutoc, Pxx))
    if len(wholeAutoc) <= 2*NumSigSamples:
        previousAutoc = wholeAutoc
        continue
    else:
        previousAutoc = wholeAutoc[CHUNK:]
    
    Index1, _ = signal.find_peaks(wholeAutoc,
                                 height=THRESHOLD,
                                 distance=peak_interval,
                                 width=peak_width)
    if Index1.size>0: # if signal is detected
        ## For debug purpose, print out progress:
        # print("Signal Detected")
        ## End
        if Index1.size==1:
            Index = Index1
        else:
            Index = Index1[np.argmax(wholeAutoc[Index1])]
        # If the Index is not at the edges of the matched filter, then claim the signal is detected
        if (Index >= (NumSigSamples/2)) and (Index <= (len(wholeAutoc)-(NumSigSamples/2))):
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
            autocStartPointer = max(0,k - int(np.ceil(NumSigSamples*2/CHUNK)))
            absIndex = autocStartPointer*CHUNK+Index
            pks_blocks_2.extend(absIndex)
            previousAutoc = np.empty(0)
    
    
        
    
    ########################################
    Duration_peak[k] = time.time()-startTime
    
print("#"*40)
print("Scipy Peaking Finding Method")
print("filtering function total time is ",np.sum(Duration_filter))
print("filtering function mean time is ",np.mean(Duration_filter))
print("filtering function std is ",np.std(Duration_filter))
print("peak finding function total time is ",np.sum(Duration_peak))
print("peak finding function mean time is ",np.mean(Duration_peak))
print("peak finding function std is ",np.std(Duration_peak))
print("#"*40)



'''
plt.figure()
plt.plot(result_3,'r-o')
plt.plot(pks_blocks_2,result_3[pks_blocks_2],'bs')
plt.show()
'''
