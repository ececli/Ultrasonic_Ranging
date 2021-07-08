# import configparser
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
# import time
import pigpio
# from scipy.signal import chirp


def sendSingleTone(pi_IO,pin,f0,duration,ratio):
    # duration is in microsecond
    pi_IO.hardware_PWM(pin,f0,ratio)
    startTS = pi_IO.get_current_tick()
    while (pi_IO.get_current_tick() - startTS) < duration:
        pass
    pi_IO.hardware_PWM(pin,0,0)
    return startTS


def sendChirp(pi_IO,pin,f0,f1,duration,sr,ratio):
    # duration is in microsecond
    Ns = int(np.round(duration*sr/1e6))
    f_all = np.linspace(f0,f1,Ns, endpoint = False)
    interval = duration/Ns # in microsecond
    TS0 = 0
    startTS = pi_IO.get_current_tick()
    for f in f_all:
        pi_IO.hardware_PWM(pin,int(f),ratio)
        TS0 = pi_IO.get_current_tick()
        while (pi_IO.get_current_tick() - TS0) < interval:
            pass
    return startTS

def findDeviceIndex(p):
    DEV_INDEX = -1
    for ii in range(0, p.get_device_count()):
        dev = p.get_device_info_by_index(ii)
        if 'snd_rpi_i2s' in dev['name']:
            DEV_INDEX = dev['index']
    return DEV_INDEX

def getRefSignal(f0,duration,sr, phi=0):
    # single-tone with frequency "f0" and duration "duration"
    # "sr" is the sampling rate
    Ns = duration * sr
    # t = np.r_[0.0:Ns]/sr
    t = np.arange(int(Ns))/sr
    return np.sin(2*np.pi*f0*t + phi)

def getRefChirp(f0,f1,duration,sr):
    # chirp signal from "f0" to "f1" with duration "duration"
    # "sr" is the sampling rate
    Ns = duration * sr
    # t = np.r_[0.0:Ns]/sr
    t = np.arange(int(Ns))/sr
    return signal.chirp(t,f0,t[-1],f1)

def matchedFilter(frames,refSignal):
    # simple peak detection
    sig = np.concatenate(frames)
    autoc = abs(np.correlate(sig, refSignal, mode = 'valid'))
    ave = np.mean(autoc)
    peak = np.max(autoc)
    Index = np.argmax(autoc)
    return ave,peak,Index


def LPF_PeakDetection(frames,refSignal,LPF_A,LPF_B):
    # low pass filter method: Use a LFP after the matched filter
    sig = np.concatenate(frames)
    autoc = abs(np.correlate(sig, refSignal, mode = 'valid'))
    filtered = signal.lfilter(LPF_B, LPF_A, autoc)
    ave = np.mean(filtered)
    peak = np.max(filtered)
    Index = np.argmax(filtered)
    return ave,peak,Index

def sincos_PeakDetection(frames,refSignal1,refSignal2):
    # sin-cos method: use two phases reference signals
    sig = np.concatenate(frames)
    autoc1 = np.correlate(sig, refSignal1, mode = 'valid')
    autoc2 = np.correlate(sig, refSignal2, mode = 'valid')
    autoc = np.sqrt((autoc1*autoc1 + autoc2*autoc2)/2)
    ave = np.mean(autoc)
    peak = np.max(autoc)
    Index = np.argmax(autoc)
    return ave,peak,Index



def Nader_PeakDetection(frames,refSignal,threshold):
    # Nader's method: Use all the local peaks from the raising edge to
    # fit a line, and use all the local peaks from the falling edge to
    # fit another line. Then the intersection of two line is the time
    # of the peak.
    sig = np.concatenate(frames)
    autoc = np.abs(np.correlate(sig, refSignal, mode = 'valid'))
    ave = np.mean(autoc)
    peak, Index = Nader_Method(autoc,threshold)
    return ave, peak, Index



def multi_PeakDetection(frames,refSignal1,refSignal2,LPF_A,LPF_B, threshold):
    # compare different methods at the same time
    sig = np.concatenate(frames)
    autoc1 = np.correlate(sig, refSignal1, mode = 'valid')
    autoc2 = np.correlate(sig, refSignal2, mode = 'valid')
    autoc = np.sqrt((autoc1*autoc1 + autoc2*autoc2)/2)
    abs_autoc1 = np.abs(autoc1)
    filtered = signal.lfilter(LPF_B, LPF_A, abs_autoc1)
    
    peak1 = np.max(abs_autoc1)
    peak2 = np.max(autoc)
    peak3 = np.max(filtered)
    
    Index1 = np.argmax(abs_autoc1)
    Index2 = np.argmax(autoc)
    Index3 = np.argmax(filtered)
    
    peak4, Index4 = Nader_Method(abs_autoc1,threshold)
    
    return peak1, peak2, peak3, peak4, Index1, Index2, Index3, Index4


def Nader_Method(autoc,threshold):
    peak = np.max(autoc)
    Index = np.argmax(autoc)
    if np.max(autoc) > threshold:
        # fit lines
        localPeaks = signal.argrelextrema(autoc, np.greater)
        aaa = autoc[localPeaks[0]]>=threshold
        goodPeaksIndex = localPeaks[0][aaa]
        goodPeaks = autoc[goodPeaksIndex]
        if len(goodPeaksIndex[goodPeaksIndex <= Index])>=2:
            z1 = np.polyfit(goodPeaksIndex[goodPeaksIndex<=Index], goodPeaks[goodPeaksIndex<=Index], 1)
        else:
            return peak,Index
        
        if len(goodPeaksIndex[goodPeaksIndex >= Index])>=2:
            z2 = np.polyfit(goodPeaksIndex[goodPeaksIndex>=Index], goodPeaks[goodPeaksIndex>=Index], 1)
        else:
            return peak,Index
        
        NaderIndex = int(np.round((z2[1] - z1[1])/(z1[0]-z2[0])))
        return peak,NaderIndex
    else:
        return peak,Index




def lookBack(curPeak, curTS, prePeak, preTS, continueFlag, THRESHOLD):
    signalDetected = False
    if curPeak > THRESHOLD or continueFlag == False:
        if continueFlag:
            continueFlag = False
            prePeak = curPeak
            preTS = curTS
        else: 
            # stream.stop_stream()            
            if curPeak <= prePeak:
                curTS = preTS
                curPeak = prePeak
            signalDetected = True
            print("Peak Detected: ", curPeak)
    return curPeak, curTS, prePeak, preTS, continueFlag, signalDetected

def index2TS(Index, frameTime, RATE, CHUNK):
    # frameTime is in microsecond
    # return TS is in microsecond
    return frameTime[0] + int(1000000*(Index/RATE - CHUNK/RATE))


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
        
def calDuration(T_start,T_end,wrapsFix):
    duration = T_end - T_start
    if duration < 0:
        duration = duration + wrapsFix
    return duration    

def getStat(a,label = " ", unit = " "):
    meanA = np.mean(a)
    stdA = np.std(a)
    
    print("Mean of "+label + " = " +str(meanA)+", Std of "+label + " = " +str(stdA))
    plt.figure()
    plt.plot(a,'b.')
    plt.xlabel('Index of Samples')
    plt.ylabel(label + "(" + unit + ")")
    plt.show()
    return meanA, stdA
    
def getOutputFig(fulldata,RefSignal,LPF_B,LPF_A):
    
    rcvSignal = np.concatenate(fulldata)
    xcorrelation = abs(np.correlate(rcvSignal, RefSignal, mode = 'valid'))
    filtered = signal.lfilter(LPF_B,LPF_A,xcorrelation)
    plt.figure()
    plt.plot(xcorrelation,'r-o')
    plt.plot(filtered,'b-o')
    plt.show()
    
def getOutputFig_IQMethod(fulldata, RefSignal1, RefSignal2):
    
    rcvSignal = np.concatenate(fulldata)
    autoc1 = np.correlate(rcvSignal, RefSignal1, mode = 'valid')
    autoc2 = np.correlate(rcvSignal, RefSignal2, mode = 'valid')
    autoc = np.sqrt((autoc1*autoc1 + autoc2*autoc2)/2)
    plt.figure()
    plt.plot(autoc,'b-o')
    plt.show()