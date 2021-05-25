# This code is for receiving the ultrasound and detecting if there is a single tone
import configparser
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time
import os




def getRefSignal(f0,duration,sr):
    Ns = duration * sr
    t = np.arange(int(Ns))/sr
    return np.sin(2*np.pi*f0*t)

# def getRefSignal_old(f0,duration,sr):
#     Ns = duration * sr
#     t = np.r_[0.0:Ns]/sr
#     # t = np.arange(int(Ns))/sr
#     return np.sin(2*np.pi*f0*t)

def preProcessingData(data,FORMAT):
    if FORMAT == pyaudio.paFloat32:
        return np.frombuffer(data,dtype=np.float32)
    elif FORMAT == pyaudio.paInt32:
        ndata = np.frombuffer(data,dtype=np.int32)
        # For I2S mic only, which only has 18 bits data and 14 bits 0.
        return (ndata>>14)/(2**17)
    else:
        print("Please use Float32 or Int32")
        return np.frombuffer(data,dtype=np.int32)

def micWarmUp(sec):
    counter_warmup = 0
    if stream.is_stopped():
        stream.start_stream()
    while True:
        data = stream.read(CHUNK)
        counter_warmup = counter_warmup + 1
        if counter_warmup>= int(sec*RATE/CHUNK+1):
            print("Mic - READY")
            return 
    
print("Mic - READY")
    

########################################
confFile = "UR_pyConfig.conf"

cp = configparser.ConfigParser()
cp.read(confFile)

CHANNELS = cp.getint("MIC","CHANNELS")
RATE = cp.getint("MIC","RATE")
CHUNK = cp.getint("MIC", "CHUNK")

f0 = cp.getint("SIGNAL","f0")
duration = cp.getint("SIGNAL","duration")

# parameters
FORMAT = pyaudio.paFloat32

# init
fulldata = []
fullTS = []


RefSignal = getRefSignal(f0,duration/1000000.0,RATE)

p = pyaudio.PyAudio()

for ii in range(0, p.get_device_count()):
   dev = p.get_device_info_by_index(ii)
   if 'snd_rpi_i2s' in dev['name']:
       DEV_INDEX = dev['index']


# start Recording
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index = DEV_INDEX,
                frames_per_buffer=CHUNK)

print("Mic - ON")
# throw aray first n seconds data since mic is transient, i.e., not stable
micWarmUp(5)


# firstChunk = True
counter = 0
while True:
    data = stream.read(CHUNK)
    currentTime = time.time()
    # if firstChunk:
    #     firstChunk = False
    #     continue
    ndata = preProcessingData(data,FORMAT)
    fulldata.append(ndata)
    fullTS.append(currentTime)
    counter = counter + 1
    if counter == 100:
        break

# stop Recording

stream.stop_stream()
stream.close()
p.terminate()
print("Mic - OFF")

rcvSigal = np.concatenate(fulldata)
plt.figure()
plt.plot(rcvSigal,'r-o')
plt.xlabel('Index of Samples')
plt.ylabel('Output of the Microphone')
plt.show()


xcorrelation = abs(signal.correlate(rcvSigal, RefSignal, mode = 'valid'))

plt.figure()
plt.plot(xcorrelation,'r.')
plt.xlabel('Index of Samples')
plt.ylabel('Output of the cross-correlation')
plt.show()