# two way ranging - initiator: first send and then listen
# 
import RPi.GPIO as GPIO
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time
import twoWayRangingLib_v2 as func
import Impl_Pico_Lib as func2
from myMQTT_Class import myMQTT
import configparser

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


pin_OUT = cp.getint("SPEAKER","pin_OUT")
pin_OUT = 5

f0 = cp.getint("SIGNAL","f0") 
duration = cp.getint("SIGNAL","duration") # microseconds
duration = 0.004 # seconds
THRESHOLD = cp.getfloat("SIGNAL","THRESHOLD")
NumRanging = cp.getint("SIGNAL","NumRanging")
NumRanging = 10
TIMEOUTCOUNTS = cp.getint("SIGNAL","TimeoutCounts")
IgnoredSamples = cp.getint("SIGNAL","IgnoredSamples")
TH_ratio_width_50 = cp.getfloat("SIGNAL","TH_ratio_width_50")

broker_address = cp.get("COMMUNICATION",'broker_address')
# broker_address = "192.168.197.238"
topic_t3t2 = cp.get("COMMUNICATION",'topic_t3t2')
topic_ready2recv = cp.get("COMMUNICATION",'topic_ready2recv')

MasterID = cp.getint("COMMUNICATION",'MasterID')
ListenerID = cp.getint("COMMUNICATION",'ListenerID')

Ranging_Offset = cp.getfloat("RANGING","Ranging_Offset")
Ranging_Max = cp.getint("RANGING",'Ranging_MAX')
Ranging_Min = cp.getint("RANGING",'Ranging_MIN')

# Init Parameters





# init variables
SOUNDSPEED = 343 # m/s

ID = MasterID
theOtherID = ListenerID

topic_tell_IamReady2Recv = topic_ready2recv+"/"+str(ID)
topic_check_ifOtherReady = topic_ready2recv+"/"+str(theOtherID)


NumIgnoredFrame = int(np.ceil(IgnoredSamples/CHUNK))
NumReqFrames = int(np.ceil(RATE / CHUNK * duration) + 1.0)
RefSignal = func.getRefSignal(f0,duration,RATE, 0)
RefSignal2 = func.getRefSignal(f0,duration,RATE, np.pi/2)

NumSigSamples = len(RefSignal)
lenOutput = CHUNK*NumReqFrames-NumSigSamples+1
TH_MaxIndex = lenOutput - NumSigSamples


# init functions
# generate BPF
pre_BPfiltering = True
L = f0 - 2000
H = f0 + 2000
order = 9
sos = func.genBPF(order, L, H, fs=RATE)

# Peak Shape:
peak_interval = int(NumSigSamples/100)
peak_width = int(NumSigSamples/100)

# GPIO init
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_OUT,GPIO.OUT)
GPIO.output(pin_OUT,False)

# setup communication
mqttc = myMQTT(broker_address)
mqttc.registerTopic(topic_t3t2)
mqttc.registerTopic(topic_tell_IamReady2Recv)
mqttc.registerTopic(topic_check_ifOtherReady)
# clear existing msg in topics
if mqttc.checkTopicDataLength(topic_t3t2)>0:
    mqttc.readTopicData(topic_t3t2)
if mqttc.checkTopicDataLength(topic_check_ifOtherReady)>0:
    mqttc.readTopicData(topic_check_ifOtherReady)

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


frames = []
counter = 0

Flag_Ready2Send = False
Flag_Ready2Recv = False
Flag_ExpTX = False
Flag_ExpRX = False
Flag_T1Ready = False
Flag_T4T1Ready = False

T4T1Ready_CD = 9999
Ready2Recv_CD = 9999 # Count Down

Ranging_Record = np.zeros(NumRanging)
sendOut_RecordCounter = np.zeros(NumRanging)
RecvTX_RecordCounter = np.zeros(NumRanging)
RecvRX_RecordCounter = np.zeros(NumRanging)

Index_Record = []
PeakCounter_Record = []
fulldata = []

counter_NumRanging = 0

stream.start_stream()
while True:
    data = stream.read(CHUNK)
    counter = counter + 1
    
    if counter <= NumIgnoredFrame:
        continue
    
    
    # Ready2Recv Count Down
    if Ready2Recv_CD:
        Ready2Recv_CD = Ready2Recv_CD - 1
        ## For debug purpose, print out progress:
        # print("Ready2Recv Count Down: ",Ready2Recv_CD)
        ## End
    else:
        ## For debug purpose, print out progress:
        print("Ready to Receive")
        ## End
        Flag_Ready2Recv = True
        Ready2Recv_CD = 9999
    
    
    # Send Ready2Recv Msg to the other device
    if Flag_Ready2Recv:
        ## For debug purpose, print out progress:
        print("Send out Ready-to-Receive to the other device")
        ## End
        mqttc.sendMsg(topic_tell_IamReady2Recv,ID)
        Flag_Ready2Recv = False
        Flag_ExpRX = True
        
    
    # Send out Single-Tone Signal
    if Flag_Ready2Send:
        ## For debug purpose, print out progress:
        print("Send out single-tone signal")
        ## End
        func2.sendSignal(pin_OUT,0.0001)
        Flag_Ready2Send = False
        Flag_ExpTX = True
        # TBD: everytime initiator sends out, count as a new counter and reset all the flags
        Flag_T1Ready = False # not sure if necessary
        Flag_T4T1Ready = False # not sure if necessary
        ## for debug and record purposes: 
        sendOut_RecordCounter[counter_NumRanging] = counter
        ## End
        

    # Read Ready2Send Msg
    if mqttc.checkTopicDataLength(topic_check_ifOtherReady)>=1:
        ## For debug purpose, print out progress:
        print("-----------------------------")
        print("Received Msg")
        ## End
        ready2recv_buffer = mqttc.readTopicData(topic_check_ifOtherReady)
        if ready2recv_buffer[-1] == theOtherID: # only read last msg
            ## For debug purpose, print out progress:
            print("Ready to Send")
            ## End
            Flag_Ready2Send = True

    
    ndata = func.preProcessingData(data,FORMAT)-DCOffset
    ## for debug and record purposes:
    fulldata.append(ndata)
    ## End
    
    frames.append(ndata)
    if len(frames) < NumReqFrames:
        continue
    
    # Peak Detection Algorithm
    sig = np.concatenate(frames)
    if pre_BPfiltering:
        sig = signal.sosfiltfilt(sos, sig)

    autoc = func.noncoherence(sig, RefSignal, RefSignal2)
    Index1, peak1 = func.peakDetector(autoc,
                                     THRESHOLD,
                                     peak_interval,
                                     peak_width)
    
    if Index1.size>0: # if signal is detected
        ## For debug purpose, print out progress:
        print("Signal Detected")
        ## End
        Index, Peak = func.peakFilter(Index1, peak1, TH = 0.8)
        if Index <= TH_MaxIndex: # claim the peak is detected
            absIndex = func2.calAbsSampleIndex(counter,
                                              Index,
                                              CHUNK,
                                              NumIgnoredFrame,
                                              NumReqFrames)
            
            ## For debug and record purposes:
            Index_Record.append(absIndex)
            PeakCounter_Record.append(counter)
            ## End
            if Flag_ExpTX:
                ## For debug purpose, print out progress:
                print("Received own signal")
                ## End
                T1 = absIndex
                Flag_ExpTX = False
                Flag_T1Ready = True
                Ready2Recv_CD = 3
                ## For debug and record purposes:
                RecvTX_RecordCounter[counter_NumRanging] = counter
                ## End
            if Flag_ExpRX:
                ## For debug purpose, print out progress:
                print("Received signal from the other device")
                ## End
                T4 = absIndex
                Flag_ExpRX = False
                ## For debug and record purposes:
                RecvRX_RecordCounter[counter_NumRanging] = counter
                ## End
                if Flag_T1Ready:
                    T4T1 = T4-T1
                    Flag_T1Ready = False
                    Flag_T4T1Ready = True
                    T4T1Ready_CD = 5
                else:
                    print("WARNING: Missing T1")
            
 
    
    if Flag_T4T1Ready:
        if mqttc.checkTopicDataLength(topic_t3t2)>=1:
            ## For debug purpose, print out progress:
            print("Received T3-T2")
            ## End
            T3T2 = mqttc.readTopicData(topic_t3t2)[-1]
            Flag_T4T1Ready = False
        
            Ranging = SOUNDSPEED*(T4T1 - T3T2)/2/RATE
            Ranging_Record[counter_NumRanging] = Ranging
            counter_NumRanging = counter_NumRanging + 1
        else:
            ## For debug purpose, print out progress:
            print("Waiting for T3-T2 from the other device")
            ## End
            if T4T1Ready_CD:
                T4T1Ready_CD = T4T1Ready_CD - 1
            else:
                Flag_T4T1Ready = False
                print("WARNING: Missing T3-T2")
                counter_NumRanging = counter_NumRanging + 1
    
    frames.pop(0)
    
    if counter_NumRanging >= NumRanging:
        break



############################################################################
print("done")
# stream.stop_stream()
stream.stop_stream()
stream.close()
p.terminate()
GPIO.cleanup()
mqttc.closeClient()

print(Ranging_Record)
   
# print(Index_Record)

# print(np.diff(np.unique(Index_Record)))


recvSig = np.concatenate(fulldata)
filteredSig = signal.sosfiltfilt(sos, recvSig)
autocSig = func.noncoherence(filteredSig,RefSignal,RefSignal2)

plt.figure()
plt.plot(recvSig,'r-o')
plt.plot(filteredSig,'b-s')
plt.show()

plt.figure()
plt.plot(autocSig,'r-o')
plt.plot(Index_Record,autocSig[Index_Record],'bs')
plt.show()


    


