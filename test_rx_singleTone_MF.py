# This code is for receiving the ultrasound and detecting if there is a single tone

import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time




def getRefSignal(f0,duration,sr):
    Ns = duration * sr
    t = np.r_[0.0:Ns]/sr
    return np.sin(2*np.pi*f0*t)


########################################
# parameters
FORMAT = pyaudio.paFloat32
CHANNELS = 1
RATE = 64000
CHUNK = 8192


f0 = 24000
duration = 0.002 # seconds

# init
fulldata = []
fullTS = []
counter = 0

RefSignal = getRefSignal(f0,duration,RATE)

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
print("* recording")

firstChunk = True
while True:
    data = stream.read(CHUNK)
    currentTime = time.time()
    if firstChunk:
        firstChunk = False
        continue
    ndata = np.frombuffer(data,dtype=np.float32)
    fulldata.append(ndata)
    fullTS.append(currentTime)
    counter = counter + 1
    if counter == 100:
        break

# stop Recording
print("* done recording")
stream.stop_stream()
stream.close()
p.terminate()

rcvSigal = np.concatenate(fulldata)
plt.figure()
plt.plot(rcvSigal,'r.')
plt.show()


xcorrelation = abs(signal.correlate(rcvSigal, RefSignal, mode = 'valid'))

plt.figure()
plt.plot(xcorrelation,'r.')
plt.show()
