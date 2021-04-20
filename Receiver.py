import pyaudio
import wave
import numpy as np
import librosa
import librosa.display
import matplotlib.pyplot as plt
from scipy import signal
import time

def pcm2float(data,dtype='float64'):
    data = np.asarray(data)
    ii = np.iinfo(data.dtype)
    abs_max = 2**(ii.bits-1)
    offset = ii.min + abs_max
    return (data.astype(dtype) - offset) / abs_max

def getChirpRef(f0,f1,duration,sr):
    Ns = duration * sr
    f = np.linspace(f0,f1,int(Ns))
    t = np.r_[0.0:Ns]/sr
    return np.sin(2*np.pi*f*t)


########################################
# parameters
FORMAT = pyaudio.paInt32
CHANNELS = 1
RATE = 44100
CHUNK = 8192

f1 = 26000
f0 = 25000
duration = 0.1

# init
frames = []
frameTime = []
metadata = []
peakdata = []
fulldata = []
fullTS = []
counter = 0

NumReqFrames = int(np.ceil(RATE / CHUNK * duration) + 1.0)

# generate chirp reference signal
chirpRef = getChirpRef(f0,f1,duration,RATE)

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


while True:
    data = stream.read(CHUNK)
    currentTime = time.time()
    # convert from int to float
    ndata = pcm2float(np.frombuffer(data,dtype=np.int32),dtype='float32')
    frames.append(ndata)
    frameTime.append(currentTime)
    fulldata.append(ndata)
    fullTS.append(currentTime)
    if len(frames) < NumReqFrames:
        continue
    sig = np.concatenate(frames)
    autoc = abs(signal.correlate(sig, chirpRef, mode = 'valid'))
    peak = np.max(autoc)
    Index = np.argmax(autoc)
    peakTS = frameTime[0] + Index/RATE
    metadata.append(peakTS)
    peakdata.append(peak)
    counter = counter + 1
    frames.pop(0)
    frameTime.pop(0)
    if counter == 400:
        break

# stop Recording
print("* done recording")
stream.stop_stream()
stream.close()
p.terminate()

# show the results
plt.figure()
plt.plot(metadata[1:],peakdata[1:])
plt.show()


# X = librosa.stft(np.concatenate(fulldata))
# Xdb = librosa.amplitude_to_db(abs(X))
# plt.figure(figsize=(14, 5))
# librosa.display.specshow(Xdb, sr=RATE, x_axis='time', y_axis='hz')
# plt.colorbar()
# plt.set_cmap("jet")
# plt.show()

pd = np.asarray(peakdata)
md = np.asarray(metadata)

peakIndex = np.where(pd>0.5*np.max(pd[1:]))
peakTSs_candi = md[peakIndex]
peakTSs = []
peakTSs.append(peakTSs_candi[0])
for k in peakTSs_candi:
    if k - peakTSs[-1] <=duration:
        continue
    peakTSs.append(k)

print(peakTSs)
