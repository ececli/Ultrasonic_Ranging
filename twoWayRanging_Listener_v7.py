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
# frames = []
# frameTime = []
counter_NumRanging = 0
T3T2Delay_micros1 = []
T3T2Delay_micros2 = []
T3T2Delay_micros3 = []
T3T2Delay_micros4 = []


# for debug purpose, record all the data
fulldata = np.frompyfunc(list, 0, 1)(np.empty((NumRanging*2), dtype=object))
fullTS = np.frompyfunc(list, 0, 1)(np.empty((NumRanging*2), dtype=object))

NumReqFrames = int(np.ceil(RATE / CHUNK * duration/1000000.0) + 1.0)
RefSignal = func.getRefSignal(f0,duration/1000000.0,RATE, 0)
RefSignal2 = func.getRefSignal(f0,duration/1000000.0,RATE, np.pi/2)

nyq = 0.5*RATE
normal_cutoff = 1000/nyq
order = 5
LPF_B, LPF_A  = signal.butter(order,normal_cutoff, btype='lowpass', analog = False)

# init
pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.hardware_PWM(pin_OUT,0,0)

mqttc = myMQTT(broker_address)

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
# throw aray first sec seconds data since mic is transient, i.e., not stable
# throw aray first n seconds data since mic is transient, i.e., not stable
DCOffset = func.micWarmUp(stream,CHUNK,RATE,FORMAT,warmUpSecond)
print("DC offset of this Mic is ",DCOffset)

TimeOutFlag = False
TimeOutCount = 0
while True:
    print(counter_NumRanging)
    if TimeOutFlag:
        break
    frames = []
    frameTime = []
    counter = 0
    firstChunk = True
    
    
    prePeak1=0
    preIndex1=0
    continueFlag1 = True
    signalDetected1 = False
    
    prePeak2=0
    preIndex2=0
    continueFlag2 = True
    signalDetected2 = False
    
    prePeak3=0
    preIndex3=0
    continueFlag3 = True
    signalDetected3 = False
    
    prePeak4=0
    preIndex4=0
    continueFlag4 = True
    signalDetected4 = False
    
    stream.start_stream()
    while True:
        data = stream.read(CHUNK)
        # if signalDetected:
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
        if not signalDetected1:
            peak1, Index1, prePeak1, preIndex1, continueFlag1, signalDetected1= func.lookBack(peak1, Index1, prePeak1, preIndex1, continueFlag1, THRESHOLD)
            if signalDetected1:
                peakTS1 = func.index2TS(Index1, frameTime, RATE, CHUNK)
        if not signalDetected2:
            peak2, Index2, prePeak2, preIndex2, continueFlag2, signalDetected2= func.lookBack(peak2, Index2, prePeak2, preIndex2, continueFlag2, THRESHOLD)
            if signalDetected2:
                peakTS2 = func.index2TS(Index2, frameTime, RATE, CHUNK)
        if not signalDetected3:
            peak3, Index3, prePeak3, preIndex3, continueFlag3, signalDetected3= func.lookBack(peak3, Index3, prePeak3, preIndex3, continueFlag3, THRESHOLD)
            if signalDetected3:
                peakTS3 = func.index2TS(Index3, frameTime, RATE, CHUNK)
        if not signalDetected4:
            peak4, Index4, prePeak4, preIndex4, continueFlag4, signalDetected4= func.lookBack(peak4, Index4, prePeak4, preIndex4, continueFlag4, THRESHOLD)
            if signalDetected4:
                peakTS4 = func.index2TS(Index4, frameTime, RATE, CHUNK)            
        
        # if signalDetected1 or signalDetected2 or signalDetected3 or signalDetected4:
        #     anySignalDetected = True
        
        if signalDetected1 and signalDetected2 and signalDetected3 and signalDetected4:
            stream.stop_stream()
            break
        
        if counter == int(TIMEOUTCOUNTS/2):
            print("Time out")
            # TimeOutFlag = True
            stream.stop_stream()
            # for test purpose:
            TimeOutCount = TimeOutCount + 1
            break
        
        frames.pop(0)
        frameTime.pop(0)

    if signalDetected1 or signalDetected2 or signalDetected3 or signalDetected4:
        # Send Signal Out
        # add this only for method 1
        time.sleep(0.2)
        T3 = func.sendSingleTone(pi_IO,pin_OUT,f0,duration,ratio)
        MSG = [0,0,0,0]
        if signalDetected1:
            T3_T2 = T3-peakTS1
            if T3_T2 < 0:
                T3_T2 = T3_T2 + wrapsFix
            T3T2Delay_micros1.append(T3_T2)
            MSG[0] = T3_T2
        else:
            T3T2Delay_micros1.append(0)
                
        if signalDetected2:
            T3_T2 = T3-peakTS2
            if T3_T2 < 0:
                T3_T2 = T3_T2 + wrapsFix
            T3T2Delay_micros2.append(T3_T2)
            MSG[1] = T3_T2
        else:
            T3T2Delay_micros2.append(0)
            
        if signalDetected3:
            T3_T2 = T3-peakTS3
            if T3_T2 < 0:
                T3_T2 = T3_T2 + wrapsFix
            T3T2Delay_micros3.append(T3_T2)
            MSG[2] = T3_T2
        else:
            T3T2Delay_micros3.append(0)
            
        if signalDetected4:
            T3_T2 = T3-peakTS4
            if T3_T2 < 0:
                T3_T2 = T3_T2 + wrapsFix
            T3T2Delay_micros4.append(T3_T2)
            MSG[3] = T3_T2
        else:
            T3T2Delay_micros4.append(0)
        mqttc.sendMsg(topic1,MSG)
        TimeOutCount = 0 # reset timeout counter
        time.sleep(0.1)
    counter_NumRanging = counter_NumRanging + 1
    # if counter_NumRanging>= NumRanging:
    #     break
    if TimeOutCount >=2:
        TimeOutFlag = True


print("done")

stream.close()
p.terminate()
pi_IO.stop()
mqttc.closeClient()
print("Mic - OFF")


print(T3T2Delay_micros1)
print(T3T2Delay_micros2)
print(T3T2Delay_micros3)
print(T3T2Delay_micros4)

