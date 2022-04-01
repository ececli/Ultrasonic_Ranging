import pyaudio
import numpy as np
import matplotlib.pyplot as plt
# from scipy import signal
import time
from scipy.fft import fft, fftfreq, rfft, rfftfreq

def findDeviceIndex(p):
    DEV_INDEX = -1
    for ii in range(0, p.get_device_count()):
        dev = p.get_device_info_by_index(ii)
        if 'snd_rpi_i2s' in dev['name']:
            DEV_INDEX = dev['index']
    return DEV_INDEX



FORMAT = pyaudio.paInt32
RATE = 64000
CHANNELS = 1
CHUNK = 1024
IgnoreTime = 5 # second
recordTime = 0 # if 0, then means infinity


NumIgnoredFrame = int(np.ceil(IgnoreTime*RATE/CHUNK))
NumRecordFrame = int(np.ceil(recordTime*RATE/CHUNK))

p = pyaudio.PyAudio()

DEV_INDEX = findDeviceIndex(p)
if DEV_INDEX == -1:
    print("Error: No Mic Found!")
    exit(1)

# init Recording
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index = DEV_INDEX,
                frames_per_buffer=CHUNK)
stream.start_stream()
print("Mic - ON")

fulldata = []
counter = 0
Flag_Ignore = True
try:
    while True:
        data = stream.read(CHUNK)
        counter = counter + 1
        if Flag_Ignore:
            if counter < NumIgnoredFrame:
                continue
            if counter == NumIgnoredFrame:
                Flag_Ignore = False
                counter = 0
                print("Start Recording Now!")
                continue
            
        ndata = np.frombuffer(data,dtype=np.int32)
        
        fulldata.append(ndata>>14)
        
        if NumRecordFrame>0:
            if counter>=NumRecordFrame:
                break
                
            
            
        
except KeyboardInterrupt:
    print("Terminated by user")

stream.stop_stream()
stream.close()
p.terminate()
print("Recording Ends")
print("Total Count is ", counter)
print("Number of buffer data collected is ", len(fulldata))
filename = 'Fulldata'+time.strftime("_%Y%m%d_%H%M")+'.dat'
a_file = open(filename, "w")
allFullData = np.concatenate(fulldata)
np.savetxt(a_file, allFullData, fmt='%d', delimiter=',')
a_file.close()
print('Full Data written to file: ', filename)

DCoffset = np.mean(allFullData)
normalAllData = allFullData-DCoffset
plt.figure()
plt.plot(normalAllData)
plt.show()

yf = rfft(normalAllData)
xf = rfftfreq(len(normalAllData), 1/RATE)
plt.figure()
plt.plot(xf,np.abs(yf))
plt.show()