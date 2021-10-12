# This code is for receiving the ultrasound and detecting if there is a single tone
import configparser
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq
import time
import os
import twoWayRangingLib_v2 as func

# Load Parameters
confFile = "UR_pyConfig.conf"
cp = configparser.ConfigParser()
cp.read(confFile)

warmUpSecond = cp.getint("MIC","WARMUP_TIME")
CHANNELS = cp.getint("MIC","CHANNELS")
RATE = cp.getint("MIC","RATE")
CHUNK = 256
FORMAT_SET = cp.get("MIC","FORMAT")
if FORMAT_SET == "Int":
    FORMAT = pyaudio.paInt32
elif FORMAT_SET == "Float":
    FORMAT = pyaudio.paFloat32
else:
    FORMAT = pyaudio.paInt32
    print("Unsupport Format. Have been Changed to Int32.")

# duration = cp.getint("SIGNAL","duration") # microseconds





########################################


# init


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

stream.start_stream()
while True:
    data = stream.read(CHUNK)

# stop Recording

stream.stop_stream()
stream.close()
p.terminate()
print("Mic - OFF")
