# This code is for receiving the ultrasound and detecting if there is a single tone
import configparser
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time
import pigpio
import os
import twoWayRangingLib_v2 as func


# Load Parameters
confFile = "UR_pyConfig.conf"
cp = configparser.ConfigParser()
cp.read(confFile)

warmUpSecond = cp.getint("MIC","WARMUP_TIME")
CHANNELS = cp.getint("MIC","CHANNELS")
RATE = cp.getint("MIC","RATE")
CHUNK = cp.getint("MIC", "CHUNK")
FORMAT_SET = cp.get("MIC","FORMAT")
if FORMAT_SET == "Int":
    FORMAT = pyaudio.paInt32
elif FORMAT_SET == "Float":
    FORMAT = pyaudio.paFloat32
else:
    FORMAT = pyaudio.paInt32
    print("Unsupport Format. Have been Changed to Int32.")

THRESHOLD = cp.getfloat("SIGNAL","THRESHOLD")
IgnoredSamples = cp.getint("SIGNAL","IgnoredSamples")
TH_ratio_width_50 = cp.getfloat("SIGNAL","TH_ratio_width_50")
# f0 = cp.getint("SIGNAL","f0") 
# duration = cp.getint("SIGNAL","duration") # microseconds

f0 = 25000
duration = 4000
THRESHOLD = 10000
NumPeaks = 21
interval_signal = 2e6

SOUNDSPEED = 343.0 # m/s
########################################
pi_IO = pigpio.pi()

# init

NumIgnoredFrame = int(np.ceil(IgnoredSamples/CHUNK))
NumReqFrames = int(np.ceil(RATE / CHUNK * duration/1000000.0) + 1.0)
RefSignal = func.getRefSignal(f0,duration/1000000.0,RATE, 0)
RefSignal2 = func.getRefSignal(f0,duration/1000000.0,RATE, np.pi/2)

NumSigSamples = len(RefSignal)
lenOutput = CHUNK*NumReqFrames-NumSigSamples+1
TH_MaxIndex = lenOutput - NumSigSamples


# generate BPF
pre_BPfiltering = True
order = 9
L = f0-2000
H = f0+2000
sos = func.genBPF(order, L, H, fs = RATE)

# Peak Shape:
peak_interval = int(NumSigSamples/100)
peak_width = int(NumSigSamples/100)


p = pyaudio.PyAudio()

DEV_INDEX = func.findDeviceIndex(p)
if DEV_INDEX == -1:
    print("Error: No Mic Found!")
    exit(1)


# start Recording
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index = DEV_INDEX,
                frames_per_buffer=CHUNK)

print("Mic - ON")
# throw aray first n seconds data since mic is transient, i.e., not stable
DCOffset = func.micWarmUp(stream,CHUNK,RATE,FORMAT,warmUpSecond)
print("DC offset of this Mic is ",DCOffset)

# firstChunk = True
frames = []
frameTime = []
fulldata = []
fullTS = []
counter = 0
ready2recv_Flag = False
signalDetected1 = False

Index_Record = []
peakTS_Record = []
peakTS_Counter = 0

stream.start_stream()
while True:
    data = stream.read(CHUNK)
    currentTime = pi_IO.get_current_tick()
    counter = counter + 1
        
    if counter <= NumIgnoredFrame:
        continue
    
    if not ready2recv_Flag:
        ready2recv_Flag = True
        print("Mic - Ready to Receive")
        continue

    ndata = func.preProcessingData(data,FORMAT) - DCOffset
    fulldata.append(ndata)
    fullTS.append(currentTime)
    
    frames.append(ndata)
    frameTime.append(currentTime)
    
    if len(frames) < NumReqFrames:
        continue
    
    sig = func.combineFrames(frames)
    
    if pre_BPfiltering:
        sig_filtered = func.BPF_sos(sos, sig)
        autoc = func.noncoherence(sig_filtered,RefSignal,RefSignal2)
    else:
        autoc = func.noncoherence(sig,RefSignal,RefSignal2)
    Index1, peak1 = func.peakDetector(autoc,
                                     THRESHOLD,
                                     peak_interval,
                                     peak_width)
    
    if Index1.size>0: # signal detected
        Index, Peak = func.peakFilter(Index1, peak1, TH=0.8)
        peakTS1 = func.index2TS(Index, frameTime, RATE, CHUNK)
        signalDetected1 = True
        if Index <= TH_MaxIndex: # claim the peak is detected
            peakTS_Record.append(peakTS1)
            Index_Record.append(Index)
            peakTS_Counter = peakTS_Counter + 1
            signalDetected1 = False
            frames = []
            frameTime = []
            continue
    else:
        if signalDetected1: # seems impossible to happen in this case
            print("At ",counter)
            print("Signal previously detected but disappear!")
            break
        if peakTS_Counter >= NumPeaks:
            print("Detected %d Peaks" % peakTS_Counter)
            break
    frames.pop(0)
    frameTime.pop(0)
    


# stop Recording

stream.stop_stream()
stream.close()
p.terminate()
print("Mic - OFF")


TSError = np.diff(peakTS_Record)-interval_signal

print(TSError)

with open("TS_Error.csv","ab") as f:
    np.savetxt(f,TSError, fmt="%d",delimiter=",")
    

aaa = np.loadtxt("TS_Error.csv",delimiter=",")
EstiError = aaa*1e-6*SOUNDSPEED
func.errorStat_inputError(EstiError, bin=100)




# print(np.diff(Index_Peaks)-128000)

"""
rawData = func.checkRawData(fulldata)
_ = func.checkBPFilteredData(fulldata,sos,showRawData = True)
func.checkFFT(fulldata,sos,RATE,showRawData = True)

Index_Peaks = func.getOutputFig_IQMethod2(fulldata,
                            RefSignal,
                            RefSignal2,
                            THRESHOLD,
                            peak_interval,
                            peak_width,
                            0,
                            pre_BPfiltering,
                            sos)

"""


