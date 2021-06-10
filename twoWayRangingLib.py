# import configparser
import pyaudio
import numpy as np
# import matplotlib.pyplot as plt
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

def getRefSignal(f0,duration,sr):
    # single-tone with frequency "f0" and duration "duration"
    # "sr" is the sampling rate
    Ns = duration * sr
    # t = np.r_[0.0:Ns]/sr
    t = np.arange(int(Ns))/sr
    return np.sin(2*np.pi*f0*t)

def getRefChirp(f0,f1,duration,sr):
    # chirp signal from "f0" to "f1" with duration "duration"
    # "sr" is the sampling rate
    Ns = duration * sr
    # t = np.r_[0.0:Ns]/sr
    t = np.arange(int(Ns))/sr
    return signal.chirp(t,f0,t[-1],f1)

def matchedFilter(frames,refSignal):
    sig = np.concatenate(frames)
    autoc = abs(np.correlate(sig, refSignal, mode = 'valid'))
    ave = np.mean(autoc)
    peak = np.max(autoc)
    Index = np.argmax(autoc)
    return ave,peak,Index

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
    
