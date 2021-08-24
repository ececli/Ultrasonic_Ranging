# two way ranging - master: first send and then listen
# v10 - use pigpio wave related functions to generate wave to speaker
# The algorithm are based on the diagram
# 
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
pin1 = cp.getint("SPEAKER","pin_OUT_1")
pin2 = cp.getint("SPEAKER","pin_OUT_2")

f0 = cp.getint("SIGNAL","f0") 
duration = cp.getint("SIGNAL","duration") # microseconds
THRESHOLD = cp.getfloat("SIGNAL","THRESHOLD")
NumRanging = cp.getint("SIGNAL","NumRanging")
TIMEOUTCOUNTS = cp.getint("SIGNAL","TimeoutCounts")
IgnoredSamples = cp.getint("SIGNAL","IgnoredSamples")
TH_ratio_width_50 = cp.getfloat("SIGNAL","TH_ratio_width_50")

broker_address = cp.get("COMMUNICATION",'broker_address')
# broker_address = "192.168.197.238"
topic_t3t2 = cp.get("COMMUNICATION",'topic_t3t2')
topic_ready2recv = cp.get("COMMUNICATION",'topic_ready2recv')
topic_counter = cp.get("COMMUNICATION",'topic_counter')

MasterID = cp.getint("COMMUNICATION",'MasterID')
ListenerID = cp.getint("COMMUNICATION",'ListenerID')

Ranging_Offset = cp.getfloat("RANGING","Ranging_Offset")
Ranging_Max = cp.getint("RANGING",'Ranging_MAX')
Ranging_Min = cp.getint("RANGING",'Ranging_MIN')
# init variables
SOUNDSPEED = 0.343 # m/ms
wrapsFix = 2**32 # constant

counter_NumRanging = 0



# for debug purpose, record all the data
fulldata = np.frompyfunc(list, 0, 1)(np.empty((NumRanging), dtype=object))
fullTS = np.frompyfunc(list, 0, 1)(np.empty((NumRanging), dtype=object))
T4T1Delay = np.zeros(NumRanging) # unit: microsecond
T3T2Delay = np.zeros(NumRanging)
Ranging_Record = np.zeros(NumRanging)
Peaks_record = np.zeros(NumRanging)

NumIgnoredFrame = int(np.ceil(IgnoredSamples/CHUNK))
NumReqFrames = int(np.ceil(RATE / CHUNK * duration/1000000.0) + 1.0)
RefSignal = func.getRefSignal(f0,duration/1000000.0,RATE, 0)
RefSignal2 = func.getRefSignal(f0,duration/1000000.0,RATE, np.pi/2)

NumSigSamples = len(RefSignal)
lenOutput = CHUNK*NumReqFrames-NumSigSamples+1
TH_MaxIndex = lenOutput - NumSigSamples


# init functions
# generate BPF
L = f0-2000
H = f0 + 2000
order = 9
sos = func.genBPF(order, L, H, fs=RATE)

# GPIO init
pi_IO = pigpio.pi()
# pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.set_mode(pin1, pigpio.OUTPUT)
pi_IO.set_mode(pin2, pigpio.OUTPUT)
# generate wave form
# wf = func.genWaveForm(f0, duration, pin_OUT)
wf = func.genWaveForm_2pin(f0,duration, pin1, pin2)
wid = func.createWave(pi_IO, wf)

# setup communication
# broker_address = "192.168.68.118"
mqttc = myMQTT(broker_address)
mqttc.registerTopic(topic_t3t2)
mqttc.registerTopic(topic_ready2recv)
# mqttc.registerTopic(topic_counter)

# clear existing msg in topics
if mqttc.checkTopicDataLength(topic_t3t2)>0:
    mqttc.readTopicData(topic_t3t2)
if mqttc.checkTopicDataLength(topic_ready2recv)>0:
    mqttc.readTopicData(topic_ready2recv)
# if mqttc.checkTopicDataLength(topic_counter)>0:
#     mqttc.readTopicData(topic_counter)

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
# throw aray first n seconds data since mic is transient, i.e., not stable
DCOffset = func.micWarmUp(stream,CHUNK,RATE,FORMAT,warmUpSecond)
print("DC offset of this Mic is ",DCOffset)



while True:

    # time.sleep(0.1)
    while True:
        if mqttc.checkTopicDataLength(topic_ready2recv)>=1:
            ready2recv_Flag = mqttc.readTopicData(topic_ready2recv)
            if ready2recv_Flag[-1] == ListenerID: # only read last msg
                # print(counter_NumRanging)
                break

    # Send Signal Out
    T1 = func.sendWave(pi_IO, wid)
    #### End of Sending Part ####
    # time.sleep(0.1)
    # Turn on listening mode
    
    frames = []
    frameTime = []
    counter = 0
    ready2recv_Flag = False
    signalDetected1 = False

    
    stream.start_stream()
    while True:
        data = stream.read(CHUNK)
        currentTime = pi_IO.get_current_tick() # version 1
        # currentTime = time.time() # version 2
        counter = counter + 1
        
        if counter <= NumIgnoredFrame:
            continue
        
        if not ready2recv_Flag:
            ready2recv_Flag = True
            mqttc.sendMsg(topic_ready2recv,MasterID)
            continue
        
        ndata = func.preProcessingData(data,FORMAT)-DCOffset
        ## for debug purpose:
        fulldata[counter_NumRanging].append(ndata)
        fullTS[counter_NumRanging].append(currentTime)
        ## End
        
        # if signalDetected1:
        #     frames.append(ndata[0:NumSigSamples])
        # else:
        #     frames.append(ndata)
        frames.append(ndata)
        frameTime.append(currentTime)

        if len(frames) < NumReqFrames:
            continue
        autoc = func.noncoherence(frames,RefSignal,RefSignal2,True,sos)
        Index1, peak1 = func.NC_detector(autoc,
                                         THRESHOLD,
                                         int(NumSigSamples/2),
                                         th_ratio=1)
        
        if Index1.size>0: # signal detected
            if Index1.size>1: # multiple signal detected, interesting to see
                print("At ",counter_NumRanging,counter)
                print("multiple peaks detected!")
            Index, Peak = func.peakFilter(Index1, peak1, TH = 0.8)
            peakTS1 = func.index2TS(Index, frameTime, RATE, CHUNK)
            signalDetected1 = True
            if Index <= TH_MaxIndex: # claim the peak is detected
                break
        else: # no peaks detected
            if counter > int(TIMEOUTCOUNTS):
                print("Time out at ", counter_NumRanging)
                break
            if signalDetected1: # seems impossible to happen in this case
                print("At ",counter_NumRanging,counter)
                print("Signal previously detected but disappear!")
                break
        frames.pop(0)
        frameTime.pop(0)

        
    stream.stop_stream()
    if signalDetected1:
        T4_T1 = func.calDuration(T1, peakTS1, wrapsFix) # version 1
        T4T1Delay[counter_NumRanging] = T4_T1
        Peaks_record[counter_NumRanging] = Peak
        
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
func.deleteWave(pi_IO, wid)
pi_IO.stop()
mqttc.closeClient()



np.savetxt("Ranging.csv",Ranging_Record, fmt="%.4f", delimiter = ",")
func.getStat(Ranging_Record,label = "Distance 1", unit = "m")

# For debug only:
func.getOutputFig_IQMethod2(fulldata[0],
                            RefSignal,
                            RefSignal2,
                            THRESHOLD,
                            NumSigSamples,
                            th_ratio=TH_ratio_width_50)

plt.figure()
plt.plot(Peaks_record,'.')
plt.xlabel("Index of Trials")
plt.ylabel("Peak values")
plt.show()

Ranging_Record = Ranging_Record - Ranging_Offset
valid_Ranging = Ranging_Record[(Ranging_Record>Ranging_Min) & (Ranging_Record<Ranging_Max)]

plt.figure()
plt.hist(valid_Ranging,bins=30)
plt.show()


func.errorStat(valid_Ranging,GT = 1.78)

