# two way ranging - master: first send and then listen
# 
import RPi.GPIO as GPIO
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time



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

def findDeviceIndex(p):
    DEV_INDEX = -1
    for ii in range(0, p.get_device_count()):
        dev = p.get_device_info_by_index(ii)
        if 'snd_rpi_i2s' in dev['name']:
            DEV_INDEX = dev['index']
    return DEV_INDEX

def micWarmUp(stream,CHUNK,RATE,FORMAT,sec):
    # DCOffset = micWarmUp(stream,CHUNK,RATE,FORMAT,sec)
    counter_warmup = 0
    ave_data = []
    if sec >= 3: # use the data after 3 seconds to calculate DC offset
        removeFirst_N_frame = 3*RATE/CHUNK
    elif sec >= 2:
        removeFirst_N_frame = 2*RATE/CHUNK
    else:
        removeFirst_N_frame = 0
        
    if stream.is_stopped():
        stream.start_stream()
    while True:
        data = stream.read(CHUNK)
        ndata = preProcessingData(data,FORMAT)
        counter_warmup = counter_warmup + 1
        if counter_warmup >= removeFirst_N_frame:
            ave_data.append(ndata)
        if counter_warmup>= int(sec*RATE/CHUNK+1):
            stream.stop_stream()
            print("Mic - READY")
            return int(np.mean(np.concatenate(ave_data)))



def preProcessingData(data,FORMAT):
    if FORMAT == pyaudio.paFloat32:
        return np.frombuffer(data,dtype=np.float32)
    elif FORMAT == pyaudio.paInt32:
        ndata = np.frombuffer(data,dtype=np.int32)
        # For I2S mic only, which only has 18 bits data and 14 bits 0.
        # return (ndata>>14)/(2**17) # return float numbers
        return (ndata>>14)
    else:
        print("Please use Float32 or Int32")
        return np.frombuffer(data,dtype=np.int32)

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


def sendSignal(PIN,Duration):
    GPIO.output(PIN,True)
    time.sleep(Duration)
    GPIO.output(PIN,False)
    return


def calAbsSampleIndex(counter, Index, CHUNK, NumIgnoredFrame = 0, NumReqFrames=0):
    return (counter-NumIgnoredFrame - NumReqFrames)*CHUNK + Index

# Init Parameters

## Mic
warmUpSecond = 5
CHANNELS = 1
RATE = 64000
CHUNK = 2048
FORMAT = pyaudio.paInt32

## Speaker
pin_OUT = 5

## Signal
f0 = 25000
duration = 0.004 # seconds
THRESHOLD = 700
NumRanging = 100
TIMEOUTCOUNTS = 150
IgnoredSamples = 4096
TH_ratio_width_50 = 0.5



# init variables
SOUNDSPEED = 0.343 # m/ms
wrapsFix = 2**32 # constant

counter_NumRanging = 0



# for debug purpose, record all the data
fulldata = np.frompyfunc(list, 0, 1)(np.empty((NumRanging), dtype=object))
fullTS = np.frompyfunc(list, 0, 1)(np.empty((NumRanging), dtype=object))
T4T1Delay = np.zeros(NumRanging) # unit: microsecond
T3T2Delay = np.zeros(NumRanging)
Ranging_Record = np.zeros(NumRanging)
Peaks_record = np.zeros(NumRanging)

NumIgnoredFrame = int(np.ceil(IgnoredSamples/CHUNK))
NumReqFrames = int(np.ceil(RATE / CHUNK * duration) + 1.0)
RefSignal = getRefSignal(f0,duration,RATE, 0)
RefSignal2 = getRefSignal(f0,duration,RATE, np.pi/2)

NumSigSamples = len(RefSignal)
lenOutput = CHUNK*NumReqFrames-NumSigSamples+1
TH_MaxIndex = lenOutput - NumSigSamples


# init functions
# generate BPF
pre_BPfiltering = True
L = f0 - 2000
H = f0 + 2000
order = 9
sos = genBPF(order, L, H, fs=RATE)

# Peak Shape:
peak_interval = int(NumSigSamples/100)
peak_width = int(NumSigSamples/100)

# GPIO init
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_OUT,GPIO.OUT)
GPIO.output(pin_OUT,False)




# register mic    
p = pyaudio.PyAudio()

DEV_INDEX = findDeviceIndex(p)
if DEV_INDEX == -1:
    print("Error: No Mic Found!")
    exit(1)

# init Recording
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index = DEV_INDEX,
                frames_per_buffer=CHUNK)

print("Mic - ON")
# throw aray first n seconds data since mic is transient, i.e., not stable
DCOffset = micWarmUp(stream,CHUNK,RATE,FORMAT,warmUpSecond)
print("DC offset of this Mic is ",DCOffset)

Index_Record = []
fulldata = []
filteredData = []
frames = []
counter = 0
Flag_SendOut = False


stream.start_stream()
while True:
    data = stream.read(CHUNK)
    counter = counter + 1
    
    if Flag_SendOut:
        sendSignal(pin_OUT,0.0001)
        Flag_SendOut = False
        print("Signal Out Counter: ",counter)
    
    if counter <= NumIgnoredFrame:
        continue
    
    
    ndata = preProcessingData(data,FORMAT)-DCOffset
    ## for debug purpose:
    fulldata.append(ndata)
    ## End
    
    frames.append(ndata)

    if len(frames) < NumReqFrames:
        continue
    sig = np.concatenate(frames)
    if pre_BPfiltering:
        sig = signal.sosfiltfilt(sos, sig)

    autoc = noncoherence(sig,RefSignal,RefSignal2)
    Index1, peak1 = peakDetector(autoc,
                                     THRESHOLD,
                                     peak_interval,
                                     peak_width)
    
    if Index1.size>0: # signal detected
        Index, Peak = peakFilter(Index1, peak1, TH = 0.8)
        if Index <= TH_MaxIndex: # claim the peak is detected
            absIndex = calAbsSampleIndex(counter,
                                         Index,
                                         CHUNK,
                                         NumIgnoredFrame,
                                         NumReqFrames)
            Index_Record.append(absIndex)
            print("Received Counter: ",counter)
            
    else: # no peaks detected
        if counter > int(TIMEOUTCOUNTS):
            print("Time out")
            break
    
    frames.pop(0)
    if counter%100 == 0:
        Flag_SendOut = True

    



############################################################################
print("done")
# stream.stop_stream()
stream.stop_stream()
stream.close()
p.terminate()
GPIO.cleanup()


   
print(Index_Record)

print(np.diff(np.unique(Index_Record)))


recvSig = np.concatenate(fulldata)
filteredSig = signal.sosfiltfilt(sos, recvSig)
autocSig = noncoherence(filteredSig,RefSignal,RefSignal2)

plt.figure()
plt.plot(recvSig,'r-o')
plt.plot(filteredSig,'b-s')
plt.show()

plt.figure()
plt.plot(autocSig,'r-o')
plt.plot(Index_Record,autocSig[Index_Record],'bs')
plt.show()


    
