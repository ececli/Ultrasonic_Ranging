# two way ranging - listener: first listen and then send
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
# pin_OUT = cp.getint("SPEAKER","pin_OUT")
pin1 = cp.getint("SPEAKER","pin_OUT_1")
pin2 = cp.getint("SPEAKER","pin_OUT_2")


duration = 2000 # microseconds
THRESHOLD = cp.getfloat("SIGNAL","THRESHOLD")
NumRanging = 100
TIMEOUTCOUNTS = cp.getint("SIGNAL","TimeoutCounts")
IgnoredSamples = cp.getint("SIGNAL","IgnoredSamples")
TH_ratio_width_50 = cp.getfloat("SIGNAL","TH_ratio_width_50")

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




# for debug purpose, record all the data
fulldata = np.frompyfunc(list, 0, 1)(np.empty((NumRanging*2), dtype=object))
fullTS = np.frompyfunc(list, 0, 1)(np.empty((NumRanging*2), dtype=object))
T3T2Delay = []
Peaks_record = []

NumIgnoredFrame = int(np.ceil(IgnoredSamples/CHUNK))
NumReqFrames = int(np.ceil(RATE / CHUNK * duration/1000000.0) + 1.0)
########################################################################
## Nader's reference signal
gamma = 625000.0
f_1 = 25000
f_s = 64000
kk = np.arange(256)
RefSignal = np.sin(np.pi*(f_1+gamma*(2*kk+1)/(2*f_s))*(2*kk+1)/f_s)
RefSignal2 = np.cos(np.pi*(f_1+gamma*(2*kk+1)/(2*f_s))*(2*kk+1)/f_s)
## End of Nader's reference signal
########################################################################
########################################################################

NumSigSamples = len(RefSignal)
lenOutput = CHUNK*NumReqFrames-NumSigSamples+1
TH_MaxIndex = lenOutput - NumSigSamples


# init functions
pi_IO = pigpio.pi()
# pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.set_mode(pin1,pigpio.OUTPUT)
pi_IO.set_mode(pin2,pigpio.OUTPUT)

# generate wave form
########################################################################
## Generate wave form based on Nader's bits seq
nader_bits = np.loadtxt("nader_4ms_bits.csv",delimiter=",")
bit_time = 1
PIN1_MASK = 1<<pin1
PIN2_MASK = 1<<pin2
pulses = []
for k in nader_bits:
    if k:
        pulses.append((PIN1_MASK,PIN2_MASK,bit_time))
    else:
        pulses.append((PIN2_MASK,PIN1_MASK,bit_time))
# Ending: Make sure both pins --> 0
if nader_bits[-1]:
    pulses.append((0,PIN1_MASK,bit_time))
else:
    pulses.append((0,PIN2_MASK,bit_time))
wf = []
for p in pulses:
    wf.append(pigpio.pulse(p[0], p[1], p[2]))
########################################################################
########################################################################
wid = func.createWave(pi_IO, wf)

# setup communication
# broker_address = "192.168.68.118"
mqttc = myMQTT(broker_address)
mqttc.registerTopic(topic_ready2recv)
# mqttc.registerTopic(topic_counter)

if mqttc.checkTopicDataLength(topic_ready2recv)>0:
    mqttc.readTopicData(topic_ready2recv)
# if mqttc.checkTopicDataLength(topic_counter)>0:
#     mqttc.readTopicData(topic_counter)

p = pyaudio.PyAudio()

DEV_INDEX = func.findDeviceIndex(p)
if DEV_INDEX == -1:
    print("Error: No Mic Found!")
    exit(1)

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

TimeOutFlag = False
TimeOutCount = 0
while True:
    # print(counter_NumRanging)
    if TimeOutFlag:
        break
    
    # receiving part starts here:
    frames = []
    frameTime = []
    counter = 0
    ready2recv_Flag = False
    signalDetected1 = False
    
    stream.start_stream()
    while True:
        data = stream.read(CHUNK)
        currentTime = pi_IO.get_current_tick()
        counter = counter + 1
        
        if counter <= NumIgnoredFrame:
            continue
        
        if not ready2recv_Flag:
            ready2recv_Flag = True
            mqttc.sendMsg(topic_ready2recv,ListenerID)
            continue

        ndata = func.preProcessingData(data,FORMAT)-DCOffset
        ## for debug purpose:
        fulldata[counter_NumRanging].append(ndata)
        fullTS[counter_NumRanging].append(currentTime)
        ## End
        
        if signalDetected1:
            frames.append(ndata[0:NumSigSamples])
        else:
            frames.append(ndata)
        frameTime.append(currentTime)

        if len(frames) < NumReqFrames:
            continue

        autoc = func.noncoherence(frames,RefSignal,RefSignal2)
        Index1, peak1 = func.NC_detector(autoc,
                                         THRESHOLD,
                                         int(NumSigSamples/10),
                                         th_ratio=0.01)
        
        if Index1.size>0: # signal detected
            if Index1.size>1: # multiple signal detected, interesting to see
                print("At ",counter_NumRanging,counter)
                print("multiple peaks detected!")
            Index, Peak = func.peakFilter(Index1, peak1, TH=0.8)
            peakTS1 = func.index2TS(Index, frameTime, RATE, CHUNK)
            signalDetected1 = True
            if Index <= TH_MaxIndex: # claim the peak is detected
                break
        else:
            if counter > int(TIMEOUTCOUNTS):
                print("Time out at ", counter_NumRanging)
                TimeOutCount = TimeOutCount + 1
                break
            if signalDetected1: # seems impossible to happen in this case
                print("At ",counter_NumRanging,counter)
                print("Signal previously detected but disappear!")
                break
        frames.pop(0)
        frameTime.pop(0)
        
        
    stream.stop_stream()
    if signalDetected1:
        # Send Signal Out
        while True:
            if mqttc.checkTopicDataLength(topic_ready2recv)>=1:
                ready2recv_Flag = mqttc.readTopicData(topic_ready2recv)
                if ready2recv_Flag[-1] == MasterID:
                    # print(counter_NumRanging)
                    break
        T3 = func.sendWave(pi_IO, wid)

        T3_T2 = func.calDuration(peakTS1, T3, wrapsFix)
        T3T2Delay.append(T3_T2)
        Peaks_record.append(Peak)

        mqttc.sendMsg(topic_t3t2,T3_T2)
        TimeOutCount = 0 # reset timeout counter
        # time.sleep(0.1)
    else:
        if TimeOutCount >=2:
            TimeOutFlag = True
        
    counter_NumRanging = counter_NumRanging + 1
    


print("done")

stream.close()
p.terminate()
func.deleteWave(pi_IO, wid)
pi_IO.stop()
mqttc.closeClient()




# For debug only:
func.getOutputFig_IQMethod2(fulldata[0],
                            RefSignal,
                            RefSignal2,
                            THRESHOLD,
                            NumSigSamples,
                            th_ratio=0.01)

plt.figure()
plt.plot(Peaks_record,'.')
plt.xlabel("Index of Trials")
plt.ylabel("Peak values")
plt.show()




