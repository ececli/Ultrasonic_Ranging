import time
import zmq
import sys
import numpy as np
import RPi.GPIO as GPIO
from scipy import signal

dt = np.dtype([('counter', 'i4'), ('status', 'i4'), ('timestamp', 'f8')])

def sendSignal(PIN,Duration):
    GPIO.output(PIN,True)
    time.sleep(Duration)
    GPIO.output(PIN,False)
    return

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


context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("ipc:///dev/shm/mic_data")
socket.setsockopt(zmq.SUBSCRIBE, b'')




CHUNK = 64 # buffer size
pin_OUT = 5
RATE = 64000 # Hz
f0 = 25000 # Hz
duration = 0.004 # second





initialStage_Duration = 5 # seconds
warmUpStage_Duration = 5 # seconds

NumSigSamples = int(duration*RATE)
NumIgnoredFrame = int(np.ceil(initialStage_Duration*RATE/CHUNK))
NumWarmUpFrame = int(np.ceil(warmUpStage_Duration*RATE/CHUNK))

# 
k = int((f0/RATE)*NumSigSamples)



counter = 0
COUNT_PRE = 0


Flag_SendSig = False
Flag_abnormal = False
Flag_initial = True
Flag_warmUp = True
Data_warmUp = []

try:
    while True:
        rawData = socket.recv()

        counter = counter + 1

        # Throw the first N second data
        if Flag_initial:
            if counter < NumIgnoredFrame:
                continue
            if counter == NumIgnoredFrame:
                Flag_initial = False
                counter = 0
                print('Beginning Data has been thrown away')
                continue


        header = np.frombuffer(rawData[:16], dtype=dt)
        mic_data = np.frombuffer(rawData[16:], dtype=np.int32)
        mic = mic_data >>14
        COUNT = header[0][0]
        status = header[0][1]
        TS = header[0][2] # no use so far

        
        # Warm up period: Collect data to calculate DC offset
        if Flag_warmUp:
            if counter < NumWarmUpFrame:
                Data_warmUp.append(mic)
                continue
            if counter == NumWarmUpFrame:
                if len(Data_warmUp)>0:
                    DCOffset = int(np.mean(np.concatenate(Data_warmUp)))
                else:
                    DCOffset = 0
                Flag_warmUp = False
                counter = 0
                COUNT_PRE = COUNT
                print("DC Offset is ",DCOffset)
                continue


        ## Check abnormal input data
        # Currently haven't figure out a solution when it happens
        if status:
            Flag_abnormal = True
            print('[Abnormal Mic Data] Input Overflow: ',counter,COUNT,status)

        if (COUNT - COUNT_PRE) != 1:
            Flag_abnormal = True
            print('[Abnormal Mic Data] Missing a Buffer at Subscriber: ',counter,COUNT,COUNT_PRE)
        COUNT_PRE = COUNT
        ## End of checking abnormal input data



        if Flag_SendSig:
            sendSignal(pin_OUT,1e-4)
            Flag_SendSig = False
            print('[Signal Sent] ',counter_NumRanging,counter)


        ## mic data is ready to use for filtering and peak detection
        filtered = 







except KeyboardInterrupt:
    GPIO.cleanup()
    print("Terminated by User")






