# two way ranging - listener: first listen and then send

import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time
import pigpio


def sendSingleTone(pi_IO,pin,f0,duration,ratio):
    pi_IO.hardware_PWM(pin,f0,ratio)
    startTS = pi_IO.get_current_tick()
    while (pi_IO.get_current_tick() - startTS) < duration:
        pass
    pi_IO.hardware_PWM(pin,0,0)
    return startTS

def findDeviceIndex(p):
    DEV_INDEX = -1
    for ii in range(0, p.get_device_count()):
        dev = p.get_device_info_by_index(ii)
        if 'snd_rpi_i2s' in dev['name']:
            DEV_INDEX = dev['index']
    return DEV_INDEX

def getRefSignal(f0,duration,sr):
    Ns = duration * sr
    t = np.r_[0.0:Ns]/sr
    return np.sin(2*np.pi*f0*t)

def matchedFilter(frames,refSignal):
    sig = np.concatenate(frames)
    autoc = abs(signal.correlate(sig, refSignal, mode = 'valid'))
    ave = np.mean(autoc)
    peak = np.max(autoc)
    Index = np.argmax(autoc)
    return ave,peak,Index



# Constant
FORMAT = pyaudio.paFloat32
CHANNELS = 1
RATE = 64000
CHUNK = 8192
pin_OUT = 12
ratio = 500000

# Parameters
f0 = 30000
duration = 2000 # microseconds
THRESHOLD = 0.003
NumRanging = 10

# init variables
fulldata = []
fullTS = []
# frames = []
# frameTime = []
counter_NumRanging = 0
T3T2Delay_micros = np.zeros(NumRanging)
T3T2Delay_NumSample = np.zeros(NumRanging)

NumReqFrames = int(np.ceil(RATE / CHUNK * duration/1000000.0) + 1.0)
RefSignal = getRefSignal(f0,duration/1000000.0,RATE)

# init
pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.hardware_PWM(pin_OUT,0,0)

p = pyaudio.PyAudio()

DEV_INDEX = findDeviceIndex(p)
if DEV_INDEX == -1:
    print("Error: No Mic Found!")

# start Recording
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index = DEV_INDEX,
                frames_per_buffer=CHUNK)
stream.stop_stream() # pause


while True:
    print(counter_NumRanging)
    frames = []
    frameTime = []
    counter = 0
    firstChunk = True
    signalDetected = False
    stream.start_stream()
    while True:
        data = stream.read(CHUNK)
        if signalDetected:
            stream.stop_stream()
            break
        # currentTime = time.time()
        currentTime = pi_IO.get_current_tick()
        counter = counter + 1
        if firstChunk:
            firstChunk = False
            continue
        ndata = np.frombuffer(data,dtype=np.float32)
        frames.append(ndata)
        frameTime.append(currentTime)
        if counter_NumRanging == 9:
            fulldata.append(ndata)
            fullTS.append(currentTime)

        if len(frames) < NumReqFrames:
            continue
        ave,peak,Index = matchedFilter(frames,RefSignal)
        if peak > THRESHOLD:
            # stream.stop_stream()
            print("Peak Detected: ",peak)
            peakTS = frameTime[0] + int(1000000*(Index/RATE - CHUNK/RATE)) # T2
            signalDetected = True
            continue
            # break
        frames.pop(0)
        frameTime.pop(0)
        if counter == 100:
            print("Time out")
            stream.stop_stream()
            break

    if signalDetected:
        # Send Signal Out
        T3 = sendSingleTone(pi_IO,pin_OUT,f0,duration,ratio)
        # print("Signal sent out")
        # print("Delay: T3-T2",T3-peakTS)
        T3T2Delay_micros[counter_NumRanging] = T3-peakTS
        T3T2Delay_NumSample[counter_NumRanging] = (NumReqFrames+1)*CHUNK-Index

    counter_NumRanging = counter_NumRanging + 1
    if counter_NumRanging>= NumRanging:
        break


print("done")

stream.close()
p.terminate()
pi_IO.stop()

print(T3T2Delay_micros)
print(T3T2Delay_NumSample)


rcvSigal = np.concatenate(fulldata)
plt.figure()
plt.plot(rcvSigal,'r.')
plt.show()


xcorrelation = abs(signal.correlate(rcvSigal, RefSignal, mode = 'valid'))

plt.figure()
plt.plot(xcorrelation,'r.')
plt.show()