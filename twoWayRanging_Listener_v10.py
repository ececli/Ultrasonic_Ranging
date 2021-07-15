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
pin_OUT = cp.getint("SPEAKER","pin_OUT")

f0 = cp.getint("SIGNAL","f0") 
duration = cp.getint("SIGNAL","duration") # microseconds
THRESHOLD = cp.getfloat("SIGNAL","THRESHOLD")
NumRanging = cp.getint("SIGNAL","NumRanging")
TIMEOUTCOUNTS = cp.getint("SIGNAL","TimeoutCounts")
NumIgnoredFrame = cp.getint("SIGNAL","NumIgnoredFrame")

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

NumReqFrames = int(np.ceil(RATE / CHUNK * duration/1000000.0) + 1.0)
RefSignal = func.getRefSignal(f0,duration/1000000.0,RATE, 0)
RefSignal2 = func.getRefSignal(f0,duration/1000000.0,RATE, np.pi/2)

nyq = 0.5*RATE
normal_cutoff = 1000/nyq
order = 5
LPF_B, LPF_A  = signal.butter(order,normal_cutoff, btype='lowpass', analog = False)

# init functions
pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)

# generate wave form
wf = func.genWaveForm(f0, duration, pin_OUT)
wid = func.createWave(pi_IO, wf)

# setup communication
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

    prePeak1=0
    prePeakTS1=0
    continueFlag1 = True
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
        frames.append(ndata)
        frameTime.append(currentTime)
        # for debug purpose:
        fulldata[counter_NumRanging].append(ndata)
        fullTS[counter_NumRanging].append(currentTime)

        if len(frames) < NumReqFrames:
            continue
        # ave,peak,Index = func.matchedFilter(frames,RefSignal)
        ave,peak1,Index1 = func.LPF_PeakDetection(frames, RefSignal, LPF_A, LPF_B)
        # ave,peak1,Index1 = func.sincos_PeakDetection(frames, RefSignal, RefSignal2)
        # ave,peak,Index = func.Nader_PeakDetection(frames,RefSignal,THRESHOLD)
        # peak1, peak2, peak3, peak4, Index1, Index2, Index3, Index4 = func.multi_PeakDetection(frames,RefSignal,RefSignal2,LPF_A,LPF_B, THRESHOLD)
        
        peakTS1 = func.index2TS(Index1, frameTime, RATE, CHUNK)
        
        
        if continueFlag1:
            if peak1 > THRESHOLD:
                prePeak1 = peak1
                prePeakTS1 = peakTS1
                continueFlag1 = False
            else:
                ### TBD
                if counter > int(TIMEOUTCOUNTS):
                    # previously use int(TIMEOUTCOUNTS/2)
                    print("Time out at ", counter_NumRanging)
                    TimeOutCount = TimeOutCount + 1
                    break
            
            frames.pop(0)
            frameTime.pop(0)

        else:
            signalDetected1 = True
            if peak1<=prePeak1:
                peak1 = prePeak1
                peakTS1 = prePeakTS1
            
            # print("Peak: ",peak1)
            break
        
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
        Peaks_record.append(peak1)

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
func.getOutputFig(fulldata[0],RefSignal,LPF_B,LPF_A)
func.getOutputFig_IQMethod(fulldata[0], RefSignal, RefSignal2)


plt.figure()
plt.plot(Peaks_record,'.')
plt.xlabel("Index of Trials")
plt.ylabel("Peak values")
plt.show()
