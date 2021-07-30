# This code is for receiving the ultrasound and detecting if there is a single tone
import configparser
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time
import os
import twoWayRangingLib as func


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



########################################


# init

NumIgnoredFrame = int(np.ceil(IgnoredSamples/CHUNK))
RefSignal = func.getRefSignal(f0,duration/1000000.0,RATE, 0)
RefSignal2 = func.getRefSignal(f0,duration/1000000.0,RATE, np.pi/2)
NumSigSamples = len(RefSignal)

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
DCOffset = func.micWarmUp(stream,CHUNK,RATE,FORMAT,warmUpSecond)
print("DC offset of this Mic is ",DCOffset)

# firstChunk = True
fulldata = []
fullTS = []
counter = 0
ready2recv_Flag = False
stream.start_stream()
while True:
    data = stream.read(CHUNK)
    currentTime = time.time()
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
    
    if counter == 1000:
        break

# stop Recording

stream.stop_stream()
stream.close()
p.terminate()
print("Mic - OFF")

rcvSignal = np.concatenate(fulldata)
plt.figure()
plt.plot(rcvSignal,'r-o')
plt.xlabel('Index of Samples')
plt.ylabel('Output of the Microphone')
plt.show()



# autoc = func.noncoherence(fulldata,RefSignal,RefSignal2)
# Index1, peak1 = func.NC_detector(autoc,THRESHOLD, NumSigSamples, th_ratio=TH_ratio_width_50)
        
# plt.figure()
# plt.plot(autoc,'r.')
# plt.plot(Index1, peak1,'bs')
# plt.xlabel('Index of Samples')
# plt.ylabel('Output of the cross-correlation')
# plt.show()

func.getOutputFig_IQMethod2(fulldata,
                            RefSignal,
                            RefSignal2,
                            THRESHOLD,
                            NumSigSamples,
                            th_ratio=TH_ratio_width_50)
