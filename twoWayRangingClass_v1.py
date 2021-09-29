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



class TWR:
    
    # constant
    SOUNDSPEED = 343 # m/s
    initiatorID = 1
    responderID = 2
    
    
    
    def __init__(self, role, conf_file):
        self.conf_file = conf_file
        if role == "initiator":
            self.ID = TWR.initiatorID
            self.theOtherID = TWR.responderID
        elif role == "responder":
            self.ID = TWR.responderID
            self.theOtherID = TWR.initiatorID
        else:
            self.ID = TWR.responderID
            self.theOtherID = TWR.initiatorID
            print("role options: 1. initiator, 2. responder (default)")


    def load_parameters(self):
        # Load Parameters
        cp = configparser.ConfigParser()
        cp.read(self.conf_file)
        
        self.warmUpSecond = cp.getint("MIC","WARMUP_TIME")
        self.CHANNELS = cp.getint("MIC","CHANNELS")
        self.RATE = cp.getint("MIC","RATE")
        self.CHUNK = cp.getint("MIC", "CHUNK")
        FORMAT_SET = cp.get("MIC","FORMAT")
        if FORMAT_SET == "Int":
            self.FORMAT = pyaudio.paInt32
        elif FORMAT_SET == "Float":
            self.FORMAT = pyaudio.paFloat32
        else:
            self.FORMAT = pyaudio.paInt32
            print("Unsupport Format. Have been Changed to Int32.")


        self.pin_OUT = cp.getint("SPEAKER","pin_OUT")
        self.pin_OUT = 5

        self.f0 = cp.getint("SIGNAL","f0") 
        self.duration = cp.getfloat("SIGNAL","duration") # microseconds
        # self.duration = 0.004 # seconds
        self.THRESHOLD = cp.getfloat("SIGNAL","THRESHOLD")
        self.THRESHOLD_TX = cp.getfloat("SIGNAL","THRESHOLD_TX")
        self.NumRanging = cp.getint("SIGNAL","NumRanging")
        # self.NumRanging = 100
        self.IgnoredSamples = cp.getint("SIGNAL","IgnoredSamples")


        self.broker_address = cp.get("COMMUNICATION",'broker_address')
        # broker_address = "192.168.197.238"
        self.topic_t3t2 = cp.get("COMMUNICATION",'topic_t3t2')
        self.topic_ready2recv = cp.get("COMMUNICATION",'topic_ready2recv')


        self.Ranging_Offset = cp.getfloat("RANGING","Ranging_Offset")
        self.Ranging_Max = cp.getint("RANGING",'Ranging_MAX')
        self.Ranging_Min = cp.getint("RANGING",'Ranging_MIN')
        
        # filtering
        self.order = cp.getint("FILTER", 'ORDER')
        self.pre_BPfiltering = cp.getboolean("FILTER", 'DO_BPF')
        self.safeBW = cp.getint("FILTER", 'SAFE_BW')
        
        # system
        self.Ready2Recv_CD_SET = cp.getint("SYSTEM", 'Ready2Recv_CD')
        self.T4T1Ready_CD_SET = cp.getint("SYSTEM", 'T4T1Ready_CD')






    # Init Parameters


    def init_parameters(self):


        # init variables
        self.topic_tell_IamReady2Recv = self.topic_ready2recv+"/"+str(self.ID)
        self.topic_check_ifOtherReady = self.topic_ready2recv+"/"+str(self.theOtherID)


        self.NumIgnoredFrame = int(np.ceil(self.IgnoredSamples/self.CHUNK))
        self.NumReqFrames = int(np.ceil(self.RATE / self.CHUNK * self.duration) + 1.0)
        self.RefSignal = func.getRefSignal(self.f0,self.duration,self.RATE, 0)
        self.RefSignal2 = func.getRefSignal(self.f0,self.duration,self.RATE, np.pi/2)

        self.NumSigSamples = len(self.RefSignal)
        # lenOutput = CHUNK*NumReqFrames-NumSigSamples+1
        self.TH_MaxIndex = (self.CHUNK*self.NumReqFrames-self.NumSigSamples+1)- self.NumSigSamples

        # Peak Shape:
        self.peak_interval = int(self.NumSigSamples/100)
        self.peak_width = int(self.NumSigSamples/100)
        
        # Value changes in the loop
        self.frames = []
        self.counter = 0

        self.Flag_Ready2Send = False
        self.Flag_Ready2Recv = False
        self.Flag_ExpTX = False
        self.Flag_ExpRX = False
        self.Flag_T_TX_Ready = False
        self.Flag_T_RX_Ready = False
        self.Flag_T4T1Ready = False

        self.T4T1Ready_CD = 9999
        if self.ID == TWR.initiatorID:
            self.Ready2Recv_CD = 9999 # Count Down for initiator
        else:
            self.Ready2Recv_CD = self.Ready2Recv_CD_SET # Count Down for responder 

        self.Ranging_Record = np.zeros(self.NumRanging)
        # self.sendOut_RecordCounter = np.zeros(self.NumRanging)
        # self.RecvTX_RecordCounter = np.zeros(self.NumRanging)
        # self.RecvRX_RecordCounter = np.zeros(self.NumRanging)
        self.sendOut_RecordCounter = []
        self.RecvTX_RecordCounter = []
        self.RecvRX_RecordCounter = []
        self.T3T2_Record = []
        self.T4T1_Record = []
        

        self.Index_Record = []
        self.PeakCounter_Record = []
        self.fulldata = np.frompyfunc(list, 0, 1)(np.empty((self.NumRanging), dtype=object))
        self.fulldata_temp = []
        self.counter_NumRanging = 0
        self.oneLoopTime = []



    def init_GPIO(self):
        # GPIO init
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_OUT,GPIO.OUT)
        GPIO.output(self.pin_OUT,False)
        
    def init_communications(self):        
        # setup communication
        self.mqttc = myMQTT(self.broker_address)
        self.mqttc.registerTopic(self.topic_t3t2)
        self.mqttc.registerTopic(self.topic_tell_IamReady2Recv)
        self.mqttc.registerTopic(self.topic_check_ifOtherReady)
        # clear existing msg in topics
        if self.mqttc.checkTopicDataLength(self.topic_t3t2)>0:
            self.mqttc.readTopicData(self.topic_t3t2)
        if self.mqttc.checkTopicDataLength(self.topic_check_ifOtherReady)>0:
            self.mqttc.readTopicData(self.topic_check_ifOtherReady)

        return self.mqttc


    def init_BPF(self):
        # generate BPF
        L = self.f0 - self.safeBW
        H = self.f0 + self.safeBW
        self.sos = func.genBPF(self.order, L, H, fs=self.RATE)
        return self.sos

    def init_mic(self):
        # register mic    
        self.p = pyaudio.PyAudio()

        DEV_INDEX = func.findDeviceIndex(self.p)
        if DEV_INDEX == -1:
            print("Error: No Mic Found!")
            exit(1)

        # init Recording
        self.stream = self.p.open(format=self.FORMAT,
                                  channels=self.CHANNELS,
                                  rate=self.RATE,
                                  input=True,
                                  input_device_index = DEV_INDEX,
                                  frames_per_buffer=self.CHUNK)

        print("Mic - ON")
        # throw aray first n seconds data since mic is transient, i.e., not stable
        self.DCOffset = func.micWarmUp(self.stream,
                                       self.CHUNK,
                                       self.RATE,
                                       self.FORMAT,
                                       self.warmUpSecond)
        print("DC offset of this Mic is ",self.DCOffset)





    def initialize(self):
        self.load_parameters()
        self.init_parameters()
        self.init_GPIO()
        self.init_communications()
        self.init_BPF()
        self.init_mic()
        




    def ready2recv_countDown(self):
        # Ready2Recv Count Down
        if self.Ready2Recv_CD:
            self.Ready2Recv_CD = self.Ready2Recv_CD - 1
            ## For debug purpose, print out progress:
            # print("Ready2Recv Count Down: ",Ready2Recv_CD)
            ## End
        else:
            ## For debug purpose, print out progress:
            # print("Ready to Receive")
            ## End
            self.Flag_Ready2Recv = True
            self.Ready2Recv_CD = 9999

    def send_ready2recv_msg(self):
        # Send Ready2Recv Msg to the other device
        if self.Flag_Ready2Recv:
            ## For debug purpose, print out progress:
            # print("Send out Ready-to-Receive to the other device")
            ## End
            self.mqttc.sendMsg(self.topic_tell_IamReady2Recv,self.ID)
            self.Flag_Ready2Recv = False
            self.Flag_ExpRX = True
    
    
    def send_singleTone(self):
        # Send out Single-Tone Signal
        if self.Flag_Ready2Send:
            ## For debug purpose, print out progress:
            # print("Send out single-tone signal")
            ## End
            func2.sendSignal(self.pin_OUT,0.0001)
            self.Flag_Ready2Send = False
            self.Flag_ExpTX = True
            # TBD: everytime initiator sends out, count as a new counter and reset all the flags
            self.Flag_T_TX_Ready = False # not sure if necessary
            self.Flag_T4T1Ready = False # not sure if necessary
            ## for debug and record purposes: 
            self.sendOut_RecordCounter.append(self.counter)
            ## End
    
    
    def read_ready2recv_msg(self):
        # Read Ready2Send Msg
        if self.mqttc.checkTopicDataLength(self.topic_check_ifOtherReady)>=1:
            ## For debug purpose, print out progress:
            # print("-----------------------------")
            # print("Received Msg")
            ## End
            ready2recv_buffer = self.mqttc.readTopicData(self.topic_check_ifOtherReady)
            if ready2recv_buffer[-1] == self.theOtherID: # only read last msg
                ## For debug purpose, print out progress:
                # print("Ready to Send")
                ## End
                self.Flag_Ready2Send = True

    def peak_detection_algorithm(self):
        # Peak Detection Algorithm
        if self.Flag_ExpTX:
            TH = self.THRESHOLD_TX
        else:
            TH = self.THRESHOLD
        sig = np.concatenate(self.frames)
        if self.pre_BPfiltering:
            sig = signal.sosfiltfilt(self.sos, sig)

        autoc = func.noncoherence(sig, self.RefSignal, self.RefSignal2)
        Index1, peak1 = func.peakDetector(autoc,
                                         TH,
                                         self.peak_interval,
                                         self.peak_width)
        return Index1, peak1     
    
    
    def procTX(self):
        ## For debug purpose, print out progress:
        # print("Received own signal")
        ## End
        # 1. General process after receiving its own signal
        self.T_TX = self.absIndex
        self.Flag_ExpTX = False
        self.Flag_T_TX_Ready = True
        self.Ready2Recv_CD = self.Ready2Recv_CD_SET
        ## For debug and record purposes:
        self.RecvTX_RecordCounter.append(self.counter)
        ## End
        # 2. For responder only:
        if self.ID == TWR.responderID:
            self.procTX_ResponderAdditional()
    
    
    
    def procTX_ResponderAdditional(self):
        if self.Flag_T_RX_Ready:
            T3T2 = self.T_TX - self.T_RX
            self.T3T2_Record.append(T3T2)
            self.mqttc.sendMsg(self.topic_t3t2, T3T2)
            self.Flag_T_RX_Ready = False
            # print("Send out T3-T2")
        else:
            print("WARNING: Missing T2")
        ## For debug and record purposes
        self.fulldata[self.counter_NumRanging] = self.fulldata_temp
        self.fulldata_temp = []
        self.counter_NumRanging = self.counter_NumRanging + 1
        ## End
    
    def procRX(self):
        ## For debug purpose, print out progress:
        # print("Received signal from the other device")
        ## End
        self.T_RX = self.absIndex
        self.Flag_ExpRX = False
        self.Flag_T_RX_Ready = True
        ## For debug and record purposes:
        self.RecvRX_RecordCounter.append(self.counter)
        ## End
        if self.ID == TWR.initiatorID:
            self.procRX_InitoatorAdditional()
    
    
    def procRX_InitoatorAdditional(self):
        if self.Flag_T_TX_Ready:
            self.T4T1 = self.T_RX - self.T_TX
            self.T4T1_Record.append(self.T4T1)
            self.Flag_T_TX_Ready = False
            self.Flag_T4T1Ready = True
            self.T4T1Ready_CD = self.T4T1Ready_CD_SET
        else:
            print("WARNING: Missing T1 at ",self.counter_NumRanging)
            
            
            
            
            
    def calRange_Initiator(self):
        
        if self.mqttc.checkTopicDataLength(self.topic_t3t2)>=1:
            ## For debug purpose, print out progress:
            # print("Received T3-T2")
            ## End
            self.T3T2 = self.mqttc.readTopicData(self.topic_t3t2)[-1]
            self.Flag_T4T1Ready = False
        
            Ranging = TWR.SOUNDSPEED*(self.T4T1 - self.T3T2)/2/self.RATE
            self.Ranging_Record[self.counter_NumRanging] = Ranging
            ## For debug and record purposes
            self.fulldata[self.counter_NumRanging] = self.fulldata_temp
            self.fulldata_temp = []
            ## End
            print("%d: Ranging = %.3f m" % (self.counter_NumRanging, Ranging))
            self.counter_NumRanging = self.counter_NumRanging + 1
            
        else:
            ## For debug purpose, print out progress:
            print("Waiting for T3-T2 from the other device")
            ## End
            if self.T4T1Ready_CD:
                self.T4T1Ready_CD = self.T4T1Ready_CD - 1
            else:
                self.Flag_T4T1Ready = False
                print("WARNING: Missing T3-T2 at ", self.counter_NumRanging)
                ## For debug and record purposes
                self.fulldata[self.counter_NumRanging] = self.fulldata_temp
                self.fulldata_temp = []
                ## End
                self.counter_NumRanging = self.counter_NumRanging + 1
    
    def stop(self):
        # stop all the services
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        GPIO.cleanup()
        self.mqttc.closeClient()
    
    
    def start(self):
        self.stream.start_stream()
        while True:
            data = self.stream.read(self.CHUNK)
            startTime = time.time()
            self.counter = self.counter + 1
            
            if self.counter <= self.NumIgnoredFrame:
                continue
            
            
            self.ready2recv_countDown()
            self.send_ready2recv_msg() # send ready2recv message
            self.send_singleTone() # send out single-tone signal via Pico
            self.read_ready2recv_msg() # read ready2recv message
            
            
            ndata = func.preProcessingData(data,self.FORMAT)-self.DCOffset
            ## for debug and record purposes:
            self.fulldata_temp.append(ndata)
            ## End
            
            self.frames.append(ndata)
            if len(self.frames) < self.NumReqFrames:
                continue
            
            Index1, peak1 = self.peak_detection_algorithm()
            
            if Index1.size>0: # if signal is detected
                ## For debug purpose, print out progress:
                # print("Signal Detected")
                ## End
                self.Index, self.Peak = func.peakFilter(Index1, peak1, TH = 0.8)
                if self.Index <= self.TH_MaxIndex: # claim the peak is detected
                    self.absIndex = func2.calAbsSampleIndex(self.counter,
                                                            self.Index,
                                                            self.CHUNK,
                                                            self.NumIgnoredFrame,
                                                            self.NumReqFrames)
                    
                    ## For debug and record purposes:
                    # Index_Record.append(absIndex)
                    # PeakCounter_Record.append(counter)
                    ## End
                    if self.Flag_ExpTX:
                        self.procTX()
                        
                    if self.Flag_ExpRX:
                        self.procRX()
                        
                    
            if self.Flag_T4T1Ready:
                 self.calRange_Initiator()
                 

            self.frames.pop(0)
            
            if (self.counter_NumRanging or len(self.T3T2_Record)) >= self.NumRanging:
                print("Ranging Finished!")
                break
            self.oneLoopTime.append(time.time() - startTime)
        
        self.stop()
############################################################################


    

# print(self.Ranging_Record)

# func.errorStat(self.Ranging_Record, 0.54, offset=0, bin=100)

# func.errorStat(self.Ranging_Record[(self.Ranging_Record>0) & (self.Ranging_Record<5)], 0.54)

# print(Index_Record)

# print(np.diff(np.unique(Index_Record)))

'''
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
plt.grid()
plt.show()
'''

    



