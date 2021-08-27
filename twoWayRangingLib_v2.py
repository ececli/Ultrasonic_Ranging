# Lib version 2: removed some of the functions which are not used in version 15 land later
# Note that for the version <=14, use twoWayRangingLib.py instead of this one.

# import configparser
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq
import time
import pigpio


#########################################################################################
#########################################################################################
def genWaveForm(f0, duration, pin):
    # Example: 
    # duration = 2000 # microsecond
    # f0 = 25000 # 10khz signal
    # pin_OUT = 12

    # Note: with f0=31250 Hz, duration can be 1600, 1920 or 3200 microseconds

    period = 1.0/f0*1e6 # period of the signal
    NumPeriod_Signal = int(duration/period)
    bit_time = int(period/2) # bit time in microsecond

    actual_f0 =  1/(bit_time*2)*1e6 # in Hz
    actual_duration = bit_time*2*NumPeriod_Signal # in microsecond

    if actual_f0 != f0:
        print("Warning: Actual f0 is ",actual_f0)
        
    if actual_duration != duration:
        print("Warning: Actual duration is ",actual_duration)

    PIN_MASK = 1<<pin

    # generate the wave form for one period
    pulse_1_period = [(PIN_MASK,0,bit_time),(0,PIN_MASK,bit_time)]
    # repeat
    pulses = pulse_1_period * NumPeriod_Signal
    wf = []
    for p in pulses:
        wf.append(pigpio.pulse(p[0], p[1], p[2]))
    
    return wf

def genWaveForm_2pin(f0, duration, pin1, pin2):
    # This function genreate wave form for two pins. The wave forms are reversed
    # for those two pins. By connecting these two pins to the buzzer, it could
    # provide +3V to -3V wave. 
    # Example: 
    # duration = 4000 # microsecond
    # f0 = 25000 # 10khz signal
    # pin1 = 12
    # pin2 = 4

    # Note: with f0=31250 Hz, duration can be 1600, 1920 or 3200 microseconds

    period = 1.0/f0*1e6 # period of the signal
    NumPeriod_Signal = int(duration/period)
    bit_time = int(period/2) # bit time in microsecond

    actual_f0 =  1/(bit_time*2)*1e6 # in Hz
    actual_duration = bit_time*2*NumPeriod_Signal # in microsecond

    if actual_f0 != f0:
        print("Warning: Actual f0 is ",actual_f0)
        
    if actual_duration != duration:
        print("Warning: Actual duration is ",actual_duration)

    PIN1_MASK = 1<<pin1
    PIN2_MASK = 1<<pin2

    # generate the wave form for one period
    pulse_1_period = [(PIN1_MASK,PIN2_MASK,bit_time),(PIN2_MASK,PIN1_MASK,bit_time)]
    pulse_end_period = (0,PIN2_MASK,bit_time)
    # repeat
    pulses = pulse_1_period * NumPeriod_Signal
    # make sure both pins become low level in the end
    pulses.append(pulse_end_period)
    
    wf = []
    for p in pulses:
        wf.append(pigpio.pulse(p[0], p[1], p[2]))   
    return wf



def genChirpWaveForm_2pin(f0, f1, duration, pin1, pin2):
    # This function genreate chirp wave form for two pins. The wave forms are reversed
    # for those two pins. By connecting these two pins to the buzzer, it could
    # provide +3V to -3V wave. 
    # Example: 
    # duration = 4000 # microsecond
    # f0 = 25000 # 10khz signal
    # f1 = 30000
    # pin1 = 12
    # pin2 = 4

    # Note: with f0=31250 Hz, duration can be 1600, 1920 or 3200 microseconds
    sr=1e6
    bit_time = 1
    w = getRefChirp(f0,f1,duration,sr)
    diff_w = np.diff(w)
    seq = np.zeros(len(w)-1)
    seq[diff_w>=0] = 1

    PIN1_MASK = 1<<pin1
    PIN2_MASK = 1<<pin2
    
    # seq[k]=1: (PIN1_MASK,PIN2_MASK,bit_time)
    # seq[k]=0: (PIN2_MASK,PIN1_MASK,bit_time)
    pulses = []
    for k in seq:
        if k:
            pulses.append((PIN1_MASK,PIN2_MASK,bit_time))
        else:
            pulses.append((PIN2_MASK,PIN1_MASK,bit_time))
    # Ending: Make sure both pins --> 0
    if seq[-1]:
        pulses.append((0,PIN1_MASK,bit_time))
    else:
        pulses.append((0,PIN2_MASK,bit_time))
    
    wf = []
    for p in pulses:
        wf.append(pigpio.pulse(p[0], p[1], p[2]))   
    return wf


def createWave(pi_IO, wf):
    pi_IO.wave_clear()
    pi_IO.wave_add_generic(wf)
    wid = pi_IO.wave_create()
    return wid

def deleteWave(pi_IO, wid):
    pi_IO.wave_delete(wid)

def sendWave(pi_IO, wid):
    # duration is in microsecond
    # this function uses wave function in pigpio
    # pro: controling sending time accurately.
    # con: cannot use arbitrary combination of f0 and duration
    # suggestion: f0=31250 Hz with duration =1600 or 1920 or 3200 microseconds
    # OR: f0=25000 with duration 2000
    # To use this function, use createWave and genWaveForm first.
    pi_IO.wave_send_once(wid)
    startTS = pi_IO.get_current_tick() # version 1
    return startTS

def sendWave_v2(pi_IO, wid):
    # duration is in microsecond
    # this function uses wave function in pigpio
    # pro: controling sending time accurately.
    # con: cannot use arbitrary combination of f0 and duration
    # suggestion: f0=31250 Hz with duration =1600 or 1920 or 3200 microseconds
    # OR: f0=25000 with duration 2000
    # To use this function, use createWave and genWaveForm first.
    pi_IO.wave_send_once(wid)
    startTS = time.time() # version 2
    return startTS

#########################################################################################
#########################################################################################

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

def getRefChirp(f0,f1,duration,sr, phi=0):
    # chirp signal from "f0" to "f1" with duration "duration"
    # "sr" is the sampling rate
    Ns = duration * sr
    # t = np.r_[0.0:Ns]/sr
    t = np.arange(int(Ns))/sr
    return signal.chirp(t,f0 = f0, t1 = duration,f1 = f1, phi = phi)


def genBPF(order,L,H,fs):
    l = L/(fs/2)
    h = H/(fs/2)
    if h>=1:
        sos = signal.butter(order, l, btype = 'high',output = 'sos')
    else:
        sos = signal.butter(order, [l,h], btype = 'bandpass',output = 'sos')
    return sos

def combineFrames(frames):
    return np.concatenate(frames)

def BPF_sos(sos, data):
    return signal.sosfiltfilt(sos, data)


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



def index2TS(Index, frameTime, RATE, CHUNK):
    # frameTime is in microsecond
    # return TS is in microsecond
    return frameTime[0] + int(1000000*(Index/RATE - CHUNK/RATE))

def index2TS_v2(Index, frameTime, RATE, CHUNK):
    # frameTime is in second
    # return TS is in second
    return frameTime[0] + (Index-CHUNK)/RATE

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
        print("wrap fix happens,duration = ",duration)
        duration = duration + wrapsFix
        print("fixed duration = ",duration)
    return duration    



#######################################################################################
#######################################################################################


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
    


def checkRawData(fulldata):
    rcvSignal = np.concatenate(fulldata)
    plt.figure()
    plt.plot(rcvSignal,'r-o')
    plt.show()
    return rcvSignal

def checkBPFilteredData(fulldata,sos,showRawData = True):
    rcvSignal = np.concatenate(fulldata)
    filtered = signal.sosfiltfilt(sos, rcvSignal)
    plt.figure()
    if showRawData:
        plt.plot(rcvSignal,'r-o')
    plt.plot(filtered,'b-*')
    plt.show()
    return filtered


def drawFFT(data,fs):
    N = len(data)
    T = 1/fs
    yf = fft(data)
    xf = fftfreq(N, T)[:N//2]
    aaa = 2.0/N * np.abs(yf[0:N//2])
    plt.figure()
    plt.plot(xf, aaa)
    plt.grid()
    plt.show()

def checkFFT(fulldata,sos,fs,showRawData = True):
    rcvSignal = np.concatenate(fulldata)
    filtered = signal.sosfiltfilt(sos, rcvSignal)
    if showRawData:
        drawFFT(rcvSignal,fs)
    drawFFT(filtered,fs)
        
    

def checkMFOutput(fulldata, RefSignal1, RefSignal2):
    
    rcvSignal = np.concatenate(fulldata)
    autoc1 = np.correlate(rcvSignal, RefSignal1, mode = 'valid')
    autoc2 = np.correlate(rcvSignal, RefSignal2, mode = 'valid')
    autoc = np.sqrt((autoc1*autoc1 + autoc2*autoc2)/2)
    plt.figure()
    plt.plot(autoc,'r-o')
    plt.show()
    return autoc
    
    
def getOutputFig_IQMethod2(fulldata,
                           RefSignal1,
                           RefSignal2,
                           THRESHOLD,
                           peak_interval,
                           peak_width,
                           recordedPeak=0,
                           preBPFilter = False,
                           sos = None):
    
    sig = combineFrames(fulldata)
    if preBPFilter:
        sig = BPF_sos(sos,sig)
    autoc = noncoherence(sig,RefSignal1,RefSignal2)

    peaks, properties = signal.find_peaks(autoc,
                                          height=THRESHOLD,
                                          distance=peak_interval,
                                          width=peak_width)
    print("Index of peaks: ", peaks)
    print(properties)
    
    plt.figure()
    plt.plot(autoc,'b-o')
    plt.plot(peaks, autoc[peaks],'rs')
    plt.hlines(y=properties["width_heights"],
               xmin=properties["left_ips"],
               xmax=properties["right_ips"])
    if recordedPeak>0:
        plt.axhline(y = recordedPeak,color="g")
    plt.show()





def errorStat(data, GT, offset=0, bin=100):
    fixedData = data - offset
    meanData = np.mean(fixedData)
    stdData = np.std(fixedData)
    
    Error = fixedData - GT
    meanAbsError = np.mean(np.abs(Error))
    meanError = np.mean(Error)
    
    sortedMAE = np.sort(np.abs(Error))
    RE95 = sortedMAE[int(len(fixedData)*0.95)-1]
    
    
    print("--------------------------------")
    print("Number of Valid Data is ", len(fixedData))
    print("Mean Distance = %.1f cm" % np.multiply(meanData,100))
    print("Std of Distance = %.1f cm" % np.multiply(stdData,100))
    print("Mean Error = %.1f cm" % np.multiply(meanError,100))
    print("Mean Absolute Error = %.1f cm" % np.multiply(meanAbsError,100))
    print("RE95 = %.1f cm" % np.multiply(RE95,100))
    print("--------------------------------")
    plt.figure()
    plt.plot(fixedData,'b.')
    plt.xlabel('Index of Samples')
    plt.ylabel("Distance (m)")
    plt.grid()
    plt.show()
    
    count, bins_count = np.histogram(np.abs(Error), bins=bin)
    pdf = count / sum(count)
    cdf = np.cumsum(pdf)
    plt.figure()
    axes = plt.gca()
    axes.set_ylim([0,1])
    plt.plot(bins_count[1:], cdf)
    plt.axhline(y=0.95,color="r")
    plt.axvline(x=RE95,color="r")
    plt.grid()
    plt.xlabel("Absolute Error (m)")
    plt.ylabel("Probability")
    plt.show()
    return meanError,meanAbsError
