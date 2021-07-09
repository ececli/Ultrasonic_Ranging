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
IgnoredSamples = cp.getint("SIGNAL","IgnoredSamples")

broker_address = cp.get("COMMUNICATION",'broker_address')
topic_t3t2 = cp.get("COMMUNICATION",'topic_t3t2')
topic_ready2recv = cp.get("COMMUNICATION",'topic_ready2recv')
topic_counter = cp.get("COMMUNICATION",'topic_counter')

MasterID = cp.getint("COMMUNICATION",'MasterID')
ListenerID = cp.getint("COMMUNICATION",'ListenerID')

# init variables
SOUNDSPEED = 0.343 # m/ms
wrapsFix = 2**32 # constant

counter_NumRanging = 0
T4T1Delay = np.zeros(NumRanging) # unit: microsecond
T3T2Delay = np.zeros(NumRanging)
Ranging_Record = np.zeros(NumRanging)


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

# communication
mqttc = myMQTT(broker_address)
mqttc.registerTopic(topic_t3t2)
mqttc.registerTopic(topic_ready2recv)
mqttc.registerTopic(topic_counter)


# clear msg in topics
if mqttc.checkTopicDataLength(topic_t3t2)>0:
    mqttc.readTopicData(topic_t3t2)
if mqttc.checkTopicDataLength(topic_ready2recv)>0:
    mqttc.readTopicData(topic_ready2recv)
if mqttc.checkTopicDataLength(topic_counter)>0:
    mqttc.readTopicData(topic_counter)

# register mic    
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

    time.sleep(0.1)
    while True:
        if mqttc.checkTopicDataLength(topic_ready2recv)>=1:
            ready2recv_Flag = mqttc.readTopicData(topic_ready2recv)
            if ready2recv_Flag[-1] == ListenerID:
                print(counter_NumRanging)
                break

    # Send Signal Out
    T1 = func.sendSingleTone(pi_IO,pin_OUT,f0,duration,ratio)
    # time.sleep(0.1)
    # Turn on listening mode
    
    frames = []
    frameTime = []
    counter = 0
    ready2recv_Flag = False
    # firstChunk = True
    
    prePeak1=0
    prePeakTS1=0
    continueFlag1 = True
    signalDetected1 = False

    
    stream.start_stream()
    while True:
        data = stream.read(CHUNK)
        currentTime = pi_IO.get_current_tick()
        counter = counter + 1
        
        ndata = func.preProcessingData(data,FORMAT)-DCOffset
        frames.append(ndata)
        frameTime.append(currentTime)
        # for debug purpose:
        fulldata[counter_NumRanging].append(ndata)
        fullTS[counter_NumRanging].append(currentTime)

        if len(frames) < NumReqFrames:
            continue
        # ave,peak,Index = func.matchedFilter(frames,RefSignal)
        # ave,peak1,Index1 = func.LPF_PeakDetection(frames, RefSignal, LPF_A, LPF_B)
        ave,peak1,Index1 = func.sincos_PeakDetection(frames, RefSignal, RefSignal2)
        # ave,peak,Index = func.Nader_PeakDetection(frames,RefSignal,THRESHOLD)
        # peak1, peak2, peak3, peak4, Index1, Index2, Index3, Index4 = func.multi_PeakDetection(frames,RefSignal,RefSignal2,LPF_A,LPF_B, THRESHOLD)
        
        GlobalIndex = (counter-NumReqFrames)*CHUNK+Index1
        if GlobalIndex <= IgnoredSamples:
            frames.pop(0)
            frameTime.pop(0)
            continue
        else:
            if not ready2recv_Flag:
                ready2recv_Flag = True
                mqttc.sendMsg(topic_ready2recv,MasterID)
        
        
        peakTS1 = func.index2TS(Index1, frameTime, RATE, CHUNK)
        if not signalDetected1:
            peak1, peakTS1, prePeak1, prePeakTS1, continueFlag1, signalDetected1= func.lookBack(peak1, peakTS1, prePeak1, prePeakTS1, continueFlag1, THRESHOLD)
            if signalDetected1:
                stream.stop_stream()
                break
        
        if counter == int(TIMEOUTCOUNTS):
            print("Time out")
            stream.stop_stream()
            break

        frames.pop(0)
        frameTime.pop(0)
        
    # print("* done")
    if signalDetected1:
        T4_T1 = func.calDuration(T1, peakTS1, wrapsFix)
        T4T1Delay[counter_NumRanging] = T4_T1
        
        while True:
            if mqttc.checkTopicDataLength(topic_t3t2)>=1:
                break
        T3_T2 = mqttc.readTopicData(topic_t3t2)[-1]
        
        Ranging = (T4_T1 - T3_T2)/2/1000.0*SOUNDSPEED
        Ranging_Record[counter_NumRanging] = Ranging
        T3T2Delay[counter_NumRanging] = T3_T2
        
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



func.getStat(Ranging_Record,label = "Distance 1", unit = "m")

# For debug only:
func.getOutputFig(fulldata[0],RefSignal,LPF_B,LPF_A)
func.getOutputFig_IQMethod(fulldata[0], RefSignal, RefSignal2)
