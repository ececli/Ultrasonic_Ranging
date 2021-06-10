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
peak_pre = []
peak_cur = []
# frames = []
# frameTime = []
counter_NumRanging = 0
T3T2Delay_micros = []
T3T2Delay_NumSample = []

# for debug purpose, record all the data
fulldata = np.frompyfunc(list, 0, 1)(np.empty((NumRanging+1), dtype=object))
fullTS = np.frompyfunc(list, 0, 1)(np.empty((NumRanging+1), dtype=object))

NumReqFrames = int(np.ceil(RATE / CHUNK * duration/1000000.0) + 1.0)
RefSignal = func.getRefSignal(f0,duration/1000000.0,RATE)


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
while True:
    print(counter_NumRanging)
    if TimeOutFlag:
        break
    frames = []
    frameTime = []
    counter = 0
    firstChunk = True
    signalDetected = False
    continueFlag = True
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
        ndata = func.preProcessingData(data,FORMAT)-DCOffset
        frames.append(ndata)
        frameTime.append(currentTime)
        # for debug purpose:
        fulldata[counter_NumRanging].append(ndata)
        fullTS[counter_NumRanging].append(currentTime)

        if len(frames) < NumReqFrames:
            continue
        # ave,peak,Index = func.matchedFilter(frames,RefSignal)
        ave,peak,Index = func.LPF_PeakDetection(frames, RefSignal, LPF_A, LPF_B)
        if peak > THRESHOLD:
            if continueFlag: # first time detected, need to see one more frame
                continueFlag = False
                peak_pre.append(peak) # debug purpose
                peak_PRE = peak
                peakTS_PRE = frameTime[0] + int(1000000*(Index/RATE - CHUNK/RATE)) # T2
            else:
                # stream.stop_stream()
                print("Peak Detected: ",peak)
                peak_cur.append(peak) # debug purpose
                if peak <= peak_PRE:
                    peakTS = peakTS_PRE
                else:
                    peakTS = frameTime[0] + int(1000000*(Index/RATE - CHUNK/RATE)) # T2
                signalDetected = True
                continue
            # break
        frames.pop(0)
        frameTime.pop(0)
        if counter == TIMEOUTCOUNTS*10:
            print("Time out")
            TimeOutFlag = True
            stream.stop_stream()
            break

    if signalDetected:
        # Send Signal Out
        T3 = func.sendSingleTone(pi_IO,pin_OUT,f0,duration,ratio)
        T3_T2 = T3-peakTS
        if T3_T2 < 0:
            T3_T2 = T3_T2 + wrapsFix
        T3_T2_NamSample = (NumReqFrames+1)*CHUNK-Index
        T3T2Delay_micros.append(T3_T2)
        T3T2Delay_NumSample.append(T3_T2_NamSample)
        mqttc.sendMsg(topic1,T3_T2)
        mqttc.sendMsg(topic2,T3_T2_NamSample)

    counter_NumRanging = counter_NumRanging + 1
    # if counter_NumRanging>= NumRanging:
    #     break


print("done")

stream.close()
p.terminate()
pi_IO.stop()
mqttc.closeClient()
print("Mic - OFF")


print(T3T2Delay_micros)
print(T3T2Delay_NumSample)


# rcvSignal = np.concatenate(fulldata)
# xcorrelation = abs(np.correlate(rcvSignal, RefSignal, mode = 'valid'))

# plt.figure()
# plt.plot(xcorrelation,'r-o')
# plt.show()

