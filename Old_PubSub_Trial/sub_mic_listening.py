import sys
import zmq
import time
import numpy as np
import twoWayRangingLib_v2 as func
import Impl_Pico_Lib as func2
import configparser
from scipy import signal
import RPi.GPIO as GPIO
from myMQTT_Class import myMQTT
import pyaudio

# version 5 is the debugging version which write data to files.


# Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect ("ipc:///dev/shm/test")


# Subscribe to zipcode, default is NYC, 10001
topicfilter = b"10001"
# socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
socket.setsockopt(zmq.SUBSCRIBE, b"")




# Load Parameters



RATE = 64000
CHUNK = 64
pin_OUT = 5




initialStage_Duration = 5
warmUpStage_Duration = 5

NumIgnoredFrame = int(np.ceil(initialStage_Duration*RATE/CHUNK))
NumWarmUpFrame = int(np.ceil(warmUpStage_Duration*RATE/CHUNK))



# GPIO init
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_OUT,GPIO.OUT)
GPIO.output(pin_OUT,False)


fulldata = []
rawData = []




counter = 0
Flag_initial = True
Flag_warmUp = True
Data_warmUp = []
COUNT = []
STATUS = []
Flag_SendSig = False
maxCounter = 15000

while True:
    data = socket.recv_pyobj()
    counter = counter + 1

    if Flag_initial:
        if counter < NumIgnoredFrame:
            continue
        if counter == NumIgnoredFrame:
            Flag_initial = False
            counter = 0
            print('Beginning Data has been thrown away')
            continue


    mic = np.frombuffer(data['mic'], dtype=np.int32)
    rawData.append(data['mic'])
    mic = mic >> 14
    # print(type(data), data )
    # COUNT = data['count']
    # print(COUNT, counter, data['status'], TS, mic.mean())


    

    if Flag_warmUp:
        if counter < NumWarmUpFrame:
            Data_warmUp.append(mic)
            continue
        if counter == NumWarmUpFrame:
            if len(Data_warmUp)>0:
                DCOffset = int(np.mean(func.combineFrames(Data_warmUp)))
            else:
                DCOffset = 0
            Flag_warmUp = False
            counter = 0
            print("DC Offset is ",DCOffset)
            continue


    ## for debug and record purposes:
    fulldata.append(mic - DCOffset)

    COUNT.append([counter,data['count']])
    STATUS.append([counter,data['status']])
    
    if Flag_SendSig:
        func2.sendSignal(pin_OUT,1e-4)
        Flag_SendSig = False
        print('[Signal Sent at counter ] ',counter)
        # continue
    
    
    if counter%1000 == 0:
        Flag_SendSig = True


    if counter>=maxCounter:
        print("Finished")
        break



GPIO.cleanup()


allFullData = np.concatenate(fulldata)
a_file = open('Fulldata_initiatorOnly.dat', "w")
np.savetxt(a_file, allFullData, fmt='%d', delimiter=',')
a_file.close()
print('Data written to file.')
logData = open('RawData_initiatorOnly.dat','wb')

print("Length of the raw data is ",len(rawData))
for d in rawData:
    buf = bytes(d)
    logData.write(buf)
    mic = np.frombuffer(d, dtype=np.int32)

logData.close()

print('Finished Writing Raw Data to Files')


