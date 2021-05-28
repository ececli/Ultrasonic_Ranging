# two way ranging - listener: first listen and then send
import configparser
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
# from scipy import signal
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

broker_address = cp.get("COMMUNICATION",'broker_address')
topic1 = cp.get("COMMUNICATION",'topic1')
topic2 = cp.get("COMMUNICATION",'topic2')


# init variables
fulldata = []
fullTS = []
# frames = []
# frameTime = []
counter_NumRanging = 0
T3T2Delay_micros = np.zeros(NumRanging)
T3T2Delay_NumSample = np.zeros(NumRanging)

NumReqFrames = int(np.ceil(RATE / CHUNK * duration/1000000.0) + 1.0)
RefSignal = func.getRefSignal(f0,duration/1000000.0,RATE)

# init
pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.hardware_PWM(pin_OUT,0,0)

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
func.micWarmUp(stream,CHUNK,RATE,warmUpSecond)

stream.stop_stream() # pause


while True:
    print(counter_NumRanging)
    frames = []
    frameTime = []
    counter = 0
    firstChunk = True
    signalDetected = False
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

    if signalDetected:
        # Send Signal Out
        T3 = func.sendSingleTone(pi_IO,pin_OUT,f0,duration,ratio)

        T3T2Delay_micros[counter_NumRanging] = T3-peakTS
        T3T2Delay_NumSample[counter_NumRanging] = (NumReqFrames+1)*CHUNK-Index

    counter_NumRanging = counter_NumRanging + 1
    if counter_NumRanging>= NumRanging:
        break


print("done")

stream.close()
p.terminate()
pi_IO.stop()
print("Mic - OFF")



time.sleep(5)
mqttc = myMQTT(broker_address)
mqttc.sendMsg(topic1,T3T2Delay_micros)
mqttc.sendMsg(topic2,T3T2Delay_NumSample)
mqttc.closeClient()


print(T3T2Delay_micros)
print(T3T2Delay_NumSample)


rcvSignal = np.concatenate(fulldata)
plt.figure()
plt.plot(rcvSignal,'r.')
plt.show()


xcorrelation = abs(np.correlate(rcvSignal, RefSignal, mode = 'valid'))

plt.figure()
plt.plot(xcorrelation,'r.')
plt.show()
