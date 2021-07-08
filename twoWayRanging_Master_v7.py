# two way ranging - master: first send and then listen
# v2 - send multiple time to test the performance
import configparser
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time
import pigpio
import twoWayRangingLib as func
from myMQTT_Class import myMQTT

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

ratio = cp.getint("SPEAKER","ratio")
pin_OUT = cp.getint("SPEAKER","pin_OUT")

f0 = cp.getint("SIGNAL","f0") 
duration = cp.getint("SIGNAL","duration") # microseconds
THRESHOLD = cp.getfloat("SIGNAL","THRESHOLD")
NumRanging = cp.getint("SIGNAL","NumRanging")
TIMEOUTCOUNTS = cp.getint("SIGNAL","TimeoutCounts")

broker_address = cp.get("COMMUNICATION",'broker_address')
topic1 = cp.get("COMMUNICATION",'topic1')
topic2 = cp.get("COMMUNICATION",'topic2')


# init variables
SOUNDSPEED = 0.343 # m/ms
wrapsFix = 2**32 # constant
# fulldata = []
# fullTS = []
# peak_pre = []
# peak_cur = []
counter_NumRanging = 0
T4T1Delay = np.zeros((NumRanging,4))
T3T2Delay = np.zeros((NumRanging,4))
Ranging_Record = np.zeros((NumRanging,4))


# for debug purpose, record all the data
fulldata = np.frompyfunc(list, 0, 1)(np.empty((NumRanging), dtype=object))
fullTS = np.frompyfunc(list, 0, 1)(np.empty((NumRanging), dtype=object))

NumReqFrames = int(np.ceil(RATE / CHUNK * duration/1000000.0) + 1.0)
RefSignal = func.getRefSignal(f0,duration/1000000.0,RATE, 0)
RefSignal2 = func.getRefSignal(f0,duration/1000000.0,RATE, np.pi/2)

# low pass filter method
nyq = 0.5*RATE
normal_cutoff = 1000/nyq
order = 5
LPF_B, LPF_A  = signal.butter(order,normal_cutoff, btype='lowpass', analog = False)

# init functions
pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.hardware_PWM(pin_OUT,0,0)

mqttc = myMQTT(broker_address)
mqttc.registerTopic(topic1)
mqttc.registerTopic(topic2)

# clear msg in topics
if mqttc.checkTopicDataLength(topic1)>0:
    mqttc.readTopicData(topic1)
if mqttc.checkTopicDataLength(topic2)>0:
    mqttc.readTopicData(topic2)
    
p = pyaudio.PyAudio()

DEV_INDEX = func.findDeviceIndex(p)
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

print("Mic - ON")
# throw aray first sec seconds data since mic is transient, i.e., not stable
# throw aray first n seconds data since mic is transient, i.e., not stable
DCOffset = func.micWarmUp(stream,CHUNK,RATE,FORMAT,warmUpSecond)
print("DC offset of this Mic is ",DCOffset)



while True:
    time.sleep(0.2)
    print(counter_NumRanging)
    # Send Signal Out
    T1 = func.sendSingleTone(pi_IO,pin_OUT,f0,duration,ratio)
    time.sleep(0.1)
    # Turn on listening mode
    
    frames = []
    frameTime = []
    counter = 0
    firstChunk = True
    
    prePeak1=0
    prePeakTS1=0
    continueFlag1 = True
    signalDetected1 = False
    
    prePeak2=0
    prePeakTS2=0
    continueFlag2 = True
    signalDetected2 = False
    
    prePeak3=0
    prePeakTS3=0
    continueFlag3 = True
    signalDetected3 = False
    
    prePeak4=0
    prePeakTS4=0
    continueFlag4 = True
    signalDetected4 = False
    
    # allSignalDetected = False
    # anySignalDetected = False
    
    stream.start_stream()
    while True:
        data = stream.read(CHUNK)
        # if allSignalDetected:
        #     stream.stop_stream()
        #     break
        # currentTime = time.time()
        currentTime = pi_IO.get_current_tick()
        counter = counter + 1
        if firstChunk:
            firstChunk = False
            continue
        ndata = func.preProcessingData(data,FORMAT)-DCOffset
        frames.append(ndata)
        frameTime.append(currentTime)
        # for debug purpose:
        fulldata[counter_NumRanging].append(ndata)
        fullTS[counter_NumRanging].append(currentTime)

        if len(frames) < NumReqFrames:
            continue
        # ave,peak,Index = func.matchedFilter(frames,RefSignal)
        # ave,peak,Index = func.LPF_PeakDetection(frames, RefSignal, LPF_A, LPF_B)
        # ave,peak,Index = func.sincos_PeakDetection(frames, RefSignal, RefSignal2)
        # ave,peak,Index = func.Nader_PeakDetection(frames,RefSignal,THRESHOLD)
        peak1, peak2, peak3, peak4, Index1, Index2, Index3, Index4 = func.multi_PeakDetection(frames,RefSignal,RefSignal2,LPF_A,LPF_B, THRESHOLD)
        peakTS1 = func.index2TS(Index1, frameTime, RATE, CHUNK)
        peakTS2 = func.index2TS(Index2, frameTime, RATE, CHUNK)
        peakTS3 = func.index2TS(Index3, frameTime, RATE, CHUNK)
        peakTS4 = func.index2TS(Index4, frameTime, RATE, CHUNK)
        if not signalDetected1:
            peak1, peakTS1, prePeak1, prePeakTS1, continueFlag1, signalDetected1= func.lookBack(peak1, peakTS1, prePeak1, prePeakTS1, continueFlag1, THRESHOLD)
        if not signalDetected2:
            peak2, peakTS2, prePeak2, prePeakTS2, continueFlag2, signalDetected2= func.lookBack(peak2, peakTS2, prePeak2, prePeakTS2, continueFlag2, THRESHOLD)
        if not signalDetected3:
            peak3, peakTS3, prePeak3, prePeakTS3, continueFlag3, signalDetected3= func.lookBack(peak3, peakTS3, prePeak3, prePeakTS3, continueFlag3, THRESHOLD)
        if not signalDetected4:
            peak4, peakTS4, prePeak4, prePeakTS4, continueFlag4, signalDetected4= func.lookBack(peak4, peakTS4, prePeak4, prePeakTS4, continueFlag4, THRESHOLD)
                       
        
        # if signalDetected1 or signalDetected2 or signalDetected3 or signalDetected4:
        #     anySignalDetected = True
        
        if signalDetected1 and signalDetected2 and signalDetected3 and signalDetected4:
            stream.stop_stream()
            break

        if counter == int(TIMEOUTCOUNTS):
            print("Time out")
            stream.stop_stream()
            break

        frames.pop(0)
        frameTime.pop(0)
        
    # print("* done")
    if signalDetected1 or signalDetected2 or signalDetected3 or signalDetected4:
        MSG = [0,0,0,0]
        if signalDetected1:
            T4_T1 = peakTS1 - T1
            if T4_T1 < 0:
                T4_T1 = T4_T1 + wrapsFix
            T4T1Delay[counter_NumRanging,0] = T4_T1
            MSG[0] = T4_T1
            
        if signalDetected2:
            T4_T1 = peakTS2 - T1
            if T4_T1 < 0:
                T4_T1 = T4_T1 + wrapsFix
            T4T1Delay[counter_NumRanging,1] = T4_T1
            MSG[1] = T4_T1
            
        if signalDetected3:
            T4_T1 = peakTS3 - T1
            if T4_T1 < 0:
                T4_T1 = T4_T1 + wrapsFix
            T4T1Delay[counter_NumRanging,2] = T4_T1
            MSG[2] = T4_T1
            
        if signalDetected4:
            T4_T1 = peakTS4 - T1
            if T4_T1 < 0:
                T4_T1 = T4_T1 + wrapsFix
            T4T1Delay[counter_NumRanging,3] = T4_T1
            MSG[3] = T4_T1
        
        while True:
            if mqttc.checkTopicDataLength(topic1)>=4:
                break
        T3_T2 = mqttc.readTopicData(topic1)
        Ranging = (np.asarray(MSG) - np.asarray(T3_T2))/2/1000.0*SOUNDSPEED
        Ranging_Record[counter_NumRanging,:] = Ranging
        T3T2Delay[counter_NumRanging,:] = T3_T2
        
    counter_NumRanging = counter_NumRanging + 1
    if counter_NumRanging >= NumRanging:
        break

############################################################################
print("done")
# stream.stop_stream()
stream.close()
p.terminate()
pi_IO.stop()
mqttc.closeClient()
print("Mic - OFF")



print("--------------------")
print(Ranging_Record)

plt.figure()
# ax = plt.subplot(1,1,1)
plt.plot(Ranging_Record[:,0],'r.',label="Simple")
plt.plot(Ranging_Record[:,3],'m.',label="Nader Method")
plt.plot(Ranging_Record[:,1],'b.',label="Non-coherent")
plt.plot(Ranging_Record[:,2],'k.',label="LFP")

plt.ylabel('Estimated Distance (m)')
plt.legend(loc="lower left")
plt.show()

print(np.mean(Ranging_Record,axis=0))
print(np.std(Ranging_Record,axis=0))


plt.figure()
plt.hist(Ranging_Record[:,0],bins='auto')
plt.xlabel('Distance (m)')
plt.show()

plt.figure()
plt.hist(Ranging_Record[:,1],bins='auto')
plt.xlabel('Distance (m)')
plt.show()

plt.figure()
plt.hist(Ranging_Record[:,2],bins='auto')
plt.xlabel('Distance (m)')
plt.show()

plt.figure()
plt.hist(Ranging_Record[:,3],bins='auto')
plt.xlabel('Distance (m)')
plt.show()


plt.figure()
plt.hist(Ranging_Record[:,0]-0.5,bins='auto', color='r')
plt.hist(Ranging_Record[:,1]-1,bins='auto', color='b')
plt.hist(Ranging_Record[:,2],bins='auto', color='k')
plt.hist(Ranging_Record[:,3],bins='auto', color='m')
plt.xlabel('Distance (m)')
plt.show()