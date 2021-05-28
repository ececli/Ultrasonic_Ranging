# two way ranging - master: first send and then listen
# v2 - send multiple time to test the performance
import configparser
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
# from scipy import signal
import time
import pigpio
import twoWayRangingLib as func
from myMQTT_Class import myMQTT

warmUpSecond = 2
FORMAT = pyaudio.paFloat32
confFile = "UR_pyConfig.conf"

cp = configparser.ConfigParser()
cp.read(confFile)

CHANNELS = cp.getint("MIC","CHANNELS")
RATE = cp.getint("MIC","RATE")
CHUNK = cp.getint("MIC", "CHUNK")

ratio = cp.getint("SPEAKER","ratio")
pin_OUT = cp.getint("SPEAKER","pin_OUT")

f0 = cp.getint("SIGNAL","f0") 
duration = cp.getint("SIGNAL","duration") # microseconds
THRESHOLD = cp.getfloat("SIGNAL","THRESHOLD")
NumRanging = cp.getint("SIGNAL","NumRanging")


broker_address = cp.get("COMMUNICATION",'broker_address')
topic1 = cp.get("COMMUNICATION",'topic1')
topic2 = cp.get("COMMUNICATION",'topic2')


# init variables
fulldata = []
fullTS = []
counter_NumRanging = 0
T4T1Delay_micros = np.zeros(NumRanging)
T4T1Delay_NumSample = np.zeros(NumRanging)

NumReqFrames = int(np.ceil(RATE / CHUNK * duration/1000000.0) + 1.0)
RefSignal = func.getRefSignal(f0,duration/1000000.0,RATE)

# init functions
pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.hardware_PWM(pin_OUT,0,0)

p = pyaudio.PyAudio()

DEV_INDEX = func.findDeviceIndex(p)
if DEV_INDEX == -1:
    print("Error: No Mic Found!")

# init Recording
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index = DEV_INDEX,
                frames_per_buffer=CHUNK)

print("Mic - ON")
# throw aray first sec seconds data since mic is transient, i.e., not stable
func.micWarmUp(stream,CHUNK,RATE,warmUpSecond)

stream.stop_stream() # pause

while True:
    time.sleep(1)
    print(counter_NumRanging)
    # Send Signal Out
    T1 = func.sendSingleTone(pi_IO,pin_OUT,f0,duration,ratio)

    # Turn on listening mode
    stream.start_stream()
    frames = []
    frameTime = []
    counter = 0
    firstChunk = True
    signalDetected = False
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
        ndata = func.preProcessingData(data,FORMAT)
        frames.append(ndata)
        frameTime.append(currentTime)
        # for debug purpose:
        if counter_NumRanging == 9:
            fulldata.append(ndata)
            fullTS.append(currentTime)

        if len(frames) < NumReqFrames:
            continue
        ave,peak,Index = func.matchedFilter(frames,RefSignal)

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


    # print("* done")
    if signalDetected:
        T4T1Delay_micros[counter_NumRanging] = peakTS - T1
        T4T1Delay_NumSample[counter_NumRanging] = (counter-NumReqFrames)*CHUNK+Index
        
    counter_NumRanging = counter_NumRanging + 1
    if counter_NumRanging >= NumRanging:
        break

print("done")
# stream.stop_stream()
stream.close()
p.terminate()
pi_IO.stop()
print("Mic - OFF")



mqttc = myMQTT(broker_address)
mqttc.registerTopic(topic1)
mqttc.registerTopic(topic2)


while True:
    if mqttc.checkTopicDataLength(topic1)==NumRanging and mqttc.checkTopicDataLength(topic2)==NumRanging:
        mqttc.closeClient()
        break


T3T2Delay_micros = mqttc.readTopicData(topic1)
T3T2Delay_NumSample = mqttc.readTopicData(topic2)

print("--------------------")
print(T4T1Delay_micros)
print(T4T1Delay_NumSample)
print("--------------------")
print(T3T2Delay_micros)
print(T3T2Delay_NumSample)
print("--------------------")

ToF1 = (T4T1Delay_micros - T3T2Delay_micros)/2/1000.0
ToF2 = (T4T1Delay_NumSample - T3T2Delay_NumSample)/RATE/2*1000.0

print("Mean of ToF = "+str(np.mean(ToF1))+", Std of ToF = "+ str(np.std(ToF1)))
print("Mean of ToF = "+str(np.mean(ToF2))+", Std of ToF = "+ str(np.std(ToF2)))
plt.figure()
plt.plot(ToF1,'r.')
plt.show()

plt.figure()
plt.plot(ToF2,'b.')
plt.show()

# For debug only:
rcvSignal = np.concatenate(fulldata)
plt.figure()
plt.plot(rcvSignal,'r.')
plt.show()

xcorrelation = abs(np.correlate(rcvSignal, RefSignal, mode = 'valid'))

plt.figure()
plt.plot(xcorrelation,'r.')
plt.show()