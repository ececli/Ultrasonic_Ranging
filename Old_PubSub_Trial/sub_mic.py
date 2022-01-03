import sys
import zmq
import time
import numpy as np
import twoWayRangingLib_v2 as func
import Impl_Pico_Lib as func2
import configparser
from scipy import signal
import RPi.GPIO as GPIO
from myMQTT_Class import myMQTT
import pyaudio



def dataProcessing_process(role):

    if role == 'initiator':
        ID = 1
        theOtherID = 2
    elif role == 'responder':
        ID = 2
        theOtherID = 1
    else:
        ID = 2
        theOtherID = 1
        print("role options: 1. initiator, 2. responder (default)")



    # Socket to talk to server
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect ("ipc:///dev/shm/test")


    # Subscribe to zipcode, default is NYC, 10001
    topicfilter = b"10001"
    # socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
    socket.setsockopt(zmq.SUBSCRIBE, b"")




    # Load Parameters
    confFile = "UR_pyConfig_v2.conf"
    cp = configparser.ConfigParser()
    cp.read(confFile)


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
    duration = cp.getfloat("SIGNAL","duration") # microseconds
    THRESHOLD = cp.getfloat("SIGNAL","THRESHOLD")
    THRESHOLD_TX = cp.getfloat("SIGNAL","THRESHOLD_TX")
    NumRanging = cp.getint("SIGNAL","NumRanging")
    IgnoredSamples = cp.getint("SIGNAL","IgnoredSamples")


    broker_address = cp.get("COMMUNICATION",'broker_address')
    # broker_address = "192.168.197.238"
    topic_t3t2 = cp.get("COMMUNICATION",'topic_t3t2')
    topic_ready2recv = cp.get("COMMUNICATION",'topic_ready2recv')

    Ranging_Offset = cp.getfloat("RANGING","Ranging_Offset")
    Ranging_Max = cp.getint("RANGING",'Ranging_MAX')
    Ranging_Min = cp.getint("RANGING",'Ranging_MIN')

    # filtering
    order = cp.getint("FILTER", 'ORDER')
    pre_BPfiltering = cp.getboolean("FILTER", 'DO_BPF')
    safeBW = cp.getint("FILTER", 'SAFE_BW')

    # system
    Ready2Recv_CD_SET = cp.getint("SYSTEM", 'Ready2Recv_CD')
    T4T1Ready_CD_SET = cp.getint("SYSTEM", 'T4T1Ready_CD')
    # Init Parameters

    # init variables
    SOUNDSPEED = 343 # m/s

    topic_tell_IamReady2Recv = topic_ready2recv+"/"+str(ID)
    topic_check_ifOtherReady = topic_ready2recv+"/"+str(theOtherID)



    initialStage_Duration = 5
    warmUpStage_Duration = 5

    NumIgnoredFrame = int(np.ceil(initialStage_Duration*RATE/CHUNK))
    NumWarmUpFrame = int(np.ceil(warmUpStage_Duration*RATE/CHUNK))
    NumReqFrames = int(np.ceil(RATE / CHUNK * duration) + 1.0)
    RefSignal = func.getRefSignal(f0,duration,RATE, 0)
    RefSignal2 = func.getRefSignal(f0,duration,RATE, np.pi/2)

    NumSigSamples = len(RefSignal)
    TH_MaxIndex = (CHUNK*NumReqFrames-NumSigSamples+1) - NumSigSamples


    # init functions
    # generate BPF
    L = f0 - safeBW
    H = f0 + safeBW
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


    frames = []
    counter = 0
    counter_NumRanging = 0
    Flag_jumpOneFrame = False

    if ID == 1:
        Flag_ExpRX = False
        Flag_SendSig = True
    else:
        Flag_ExpRX = True
        Flag_SendSig = False
            

    Ranging_Record = np.zeros(NumRanging)
    T3T2_Record = np.zeros(NumRanging)
    T4T1_Record = np.zeros(NumRanging)

    sendOut_RecordCounter = []
    RecvTX_RecordCounter = []
    RecvRX_RecordCounter = []

    Index_Record = []
    PeakCounter_Record = []
    fulldata = np.frompyfunc(list, 0, 1)(np.empty((NumRanging), dtype=object))
    fulldata_temp = []



    prev = 0;
    counter = 0
    Flag_initial = True
    Flag_warmUp = True
    Data_warmUp = []
    while True:
        data = socket.recv_pyobj()
        counter = counter + 1

        if Flag_initial:
            if counter < NumIgnoredFrame:
                continue
            if counter == NumIgnoredFrame:
                Flag_initial = False
                counter = 0
                print('Beginning Data has been thrown away')
                continue


        mic = np.frombuffer(data['mic'], dtype=np.int32)
        mic = mic >> 14
        # print(mic.shape)
        TS = data['input_buffer_adc_time'];
        COUNT = data['count']
        print(COUNT, counter, data['status'], TS, mic.mean())


        

        if Flag_warmUp:
            if counter < NumWarmUpFrame:
                Data_warmUp.append(mic)
                continue
            if counter == NumWarmUpFrame:
                if len(Data_warmUp)>0:
                    DCOffset = int(np.mean(func.combineFrames(Data_warmUp)))
                else:
                    DCOffset = 0
                Flag_warmUp = False
                counter = 0
                print("DC Offset is ",DCOffset)
                continue


        ## for debug and record purposes:
        fulldata_temp.append(mic - DCOffset)
        if counter == 1:
            startTime = time.time()
            print('Start Time is ',startTime)
        ## End


        if Flag_jumpOneFrame:
            Flag_jumpOneFrame = False
            continue
            
        
        
        if Flag_SendSig:
            func2.sendSignal(pin_OUT,1e-4)
            Flag_SendSig = False
            continue

        frames.append(mic - DCOffset)
        if len(frames) < NumReqFrames:
            continue


        
        # Peak Detection Algorithm
        if Flag_ExpRX:
            TH = THRESHOLD
        else:
            TH = THRESHOLD_TX
                
        sig = np.concatenate(frames)
        if pre_BPfiltering:
            sig = signal.sosfiltfilt(sos, sig)

        autoc = func.noncoherence(sig, RefSignal, RefSignal2)
        Index1, peak1 = func.peakDetector(autoc,
                                         TH,
                                         peak_interval,
                                         peak_width)
        
        if Index1.size>0: # if signal is detected
            ## For debug purpose, print out progress:
            # print("Signal Detected")
            ## End
            Index, Peak = func.peakFilter(Index1, peak1, TH = 1)
            if Index <= TH_MaxIndex: # claim the peak is detected
                absIndex = func2.calAbsSampleIndex(counter,
                                                  Index,
                                                  CHUNK,
                                                  0,
                                                  NumReqFrames)
                
                ## For debug and record purposes:
                # Index_Record.append(absIndex)
                # PeakCounter_Record.append(counter)
                ## End
                if Flag_ExpRX:
                    ## For debug purpose, print out progress:
                    # print("Received signal from the other device")
                    ## End
                    T_RX = absIndex
                    Flag_ExpRX = False
                    Flag_SendSig = True
                    ## For debug and record purposes:
                    # RecvRX_RecordCounter.append(counter)
                    ## End
                    if ID == 1:
                        T4T1_Record[counter_NumRanging] =T_RX - T_TX
                        ## For debug and record purposes
                        fulldata[counter_NumRanging] = fulldata_temp
                        fulldata_temp = []
                        ## END
                        counter_NumRanging = counter_NumRanging + 1
                else:
                    ## For debug purpose, print out progress:
                    # print("Received own signal")
                    ## End
                    # 1. General process after receiving its own signal
                    T_TX = absIndex
                    Flag_ExpRX = True
                    ## For debug and record purposes:
                    # RecvTX_RecordCounter.append(self.counter)
                    ## End
                    # 2. For responder only:
                    if ID == 2:
                        T3T2_Record[counter_NumRanging] = T_TX - T_RX
                        ## For debug and record purposes
                        fulldata[counter_NumRanging] = fulldata_temp
                        fulldata_temp = []
                        ## End
                        counter_NumRanging = counter_NumRanging + 1

                frames = []
                Flag_jumpOneFrame = True
            else:
                frames.pop(0)
        else:
            frames.pop(0)
        
        if len(frames)>=NumReqFrames:
            print('BUG APPEAR!')
            print(counter_NumRanging,counter,len(frames),
                  Flag_ExpRX,Flag_SendSig,Flag_jumpOneFrame)
            break
        
        if counter_NumRanging>=NumRanging:
            print("Ranging Finished!")
            break


    GPIO.cleanup()
    Duration = time.time()-startTime
    print("Duration is ", Duration)
    if ID == 1:
        while True:
            if mqttc.checkTopicDataLength(topic_t3t2)>=NumRanging:
                break
        ## For debug purpose, print out progress:
        # print("Received T3-T2")
        ## End
        T3T2 = mqttc.readTopicData(topic_t3t2)
        print("Read all T3-T2")
        # print(T3T2)            
        Ranging_Record = SOUNDSPEED*(T4T1_Record - T3T2)/2/RATE
        a = Ranging_Record[(Ranging_Record>0) & (Ranging_Record<5)]
        print(T4T1_Record)
        print(a)
        print(len(a),np.mean(a),np.std(a))
        mqttc.closeClient()
    else:
        mqttc.sendMsg(topic_t3t2, T3T2_Record)
        print(T3T2_Record)
        print("Sending T3-T2 Status: Done")
        time.sleep(5)




if __name__ == '__main__':
    if len(sys.argv)>=2:
        role = sys.argv[1]
        print(role)
    else:
        role = 'responder'
        print('Please enter role. Otherwise, role is responder')
    
    dataProcessing_process(role)