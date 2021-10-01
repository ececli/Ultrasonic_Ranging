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
        self.peak_interval = 10 # int(self.NumSigSamples/100)
        self.peak_width = int(self.NumSigSamples/20) # int(self.NumSigSamples/100)
        
        # Value changes in the loop
        self.frames = []
        self.counter = 0
        self.counter_NumRanging = 0
        self.Flag_jumpOneFrame = False

        if self.ID == TWR.initiatorID:
            self.Flag_ExpRX = False
            self.Flag_SendSig = True
        else:
            self.Flag_ExpRX = True
            self.Flag_SendSig = False

        self.Ranging_Record = np.zeros(self.NumRanging)
        self.T3T2_Record = np.zeros(self.NumRanging)
        self.T4T1_Record = np.zeros(self.NumRanging)
        
        
        self.sendOut_RecordCounter = []
        self.RecvTX_RecordCounter = []
        self.RecvRX_RecordCounter = []

        self.Index_Record = []
        self.PeakCounter_Record = []
        self.fulldata = np.frompyfunc(list, 0, 1)(np.empty((self.NumRanging), dtype=object))
        self.fulldata_temp = []
        



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
        
    
    def send_singleTone(self):
        # Send out Single-Tone Signal

        ## For debug purpose, print out progress:
        # print("Send out single-tone signal")
        ## End
        func2.sendSignal(self.pin_OUT,1e-4)
        self.Flag_SendSig = False
        ## for debug and record purposes: 
        # self.sendOut_RecordCounter.append(self.counter)
        ## End
    
    

    def peak_detection_algorithm(self):
        # Peak Detection Algorithm
        if self.Flag_ExpRX:
            TH = self.THRESHOLD
        else:
            TH = self.THRESHOLD_TX
                
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
        self.Flag_ExpRX = True
        ## For debug and record purposes:
        # self.RecvTX_RecordCounter.append(self.counter)
        ## End
        # 2. For responder only:
        if self.ID == TWR.responderID:
            self.T3T2_Record[self.counter_NumRanging] = self.T_TX - self.T_RX
            ## For debug and record purposes
            self.fulldata[self.counter_NumRanging] = self.fulldata_temp
            self.fulldata_temp = []
            ## End
            self.counter_NumRanging = self.counter_NumRanging + 1


    def procRX(self):
        ## For debug purpose, print out progress:
        # print("Received signal from the other device")
        ## End
        self.T_RX = self.absIndex
        self.Flag_ExpRX = False
        self.Flag_SendSig = True
        ## For debug and record purposes:
        # self.RecvRX_RecordCounter.append(self.counter)
        ## End
        if self.ID == TWR.initiatorID:
            self.T4T1_Record[self.counter_NumRanging] = self.T_RX - self.T_TX
            ## For debug and record purposes
            self.fulldata[self.counter_NumRanging] = self.fulldata_temp
            self.fulldata_temp = []
            ## END
            self.counter_NumRanging = self.counter_NumRanging + 1
    
    def stop(self):
        # stop all the services
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        GPIO.cleanup()
        # self.mqttc.closeClient()
    
    
    def start(self):
        self.stream.start_stream()
        while True:
            data = self.stream.read(self.CHUNK)
            self.counter = self.counter + 1
            
            if self.counter <= self.NumIgnoredFrame:
                continue
            if self.counter == self.NumIgnoredFrame+1:
                print("Processing Data Now")

            
            ndata = func.preProcessingData(data,self.FORMAT)-self.DCOffset
            ## for debug and record purposes:
            self.fulldata_temp.append(ndata)
            ## End
            
            if self.Flag_jumpOneFrame:
                self.Flag_jumpOneFrame = False
                continue
            
            self.frames.append(ndata)
            if len(self.frames) < self.NumReqFrames:
                continue
            
            if self.Flag_SendSig:
                self.send_singleTone() # send out single-tone signal via Pico
            
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
                        
                    if self.Flag_ExpRX:
                        self.procRX()
                    else:
                        self.procTX()
                    
                    self.frames = []
                    self.Flag_jumpOneFrame = True
            else: # No signal detected
                self.frames.pop(0)
            ## Debug:
            if len(self.frames)>=2:
                print("BUG APPEAR!")
                print(self.counter_NumRanging,self.counter,len(self.frames))
                print(self.Flag_ExpRX, self.Flag_SendSig, self.Flag_jumpOneFrame)
                print(len(ndata))
                break
            ## END
            if self.counter_NumRanging >= self.NumRanging:
                print("Ranging Finished!")
                break

        self.stop()
############################################################################
    def sendT3T2(self):
        # self.init_communications()
        # time.sleep(3)

        self.mqttc.sendMsg(self.topic_t3t2, self.T3T2_Record)
        print("Sending T3-T2 Status: Done")
        # self.mqttc.closeClient()
        
    def recvT3T2(self):
        # self.init_communications()
        # time.sleep(1)
        while True:
            if self.mqttc.checkTopicDataLength(self.topic_t3t2)>=self.NumRanging:
                break
        ## For debug purpose, print out progress:
        # print("Received T3-T2")
        ## End
        self.T3T2 = self.mqttc.readTopicData(self.topic_t3t2)
        print("Read all T3-T2")
        # print(self.T3T2)            
        self.Ranging_Record = TWR.SOUNDSPEED*(self.T4T1_Record - self.T3T2)/2/self.RATE
        # print(self.Ranging_Record)
        # self.mqttc.closeClient()


    




    





