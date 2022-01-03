import time
import zmq
import sys
import numpy as np
import RPi.GPIO as GPIO
from scipy import signal
from myMQTT_Class import myMQTT
from numba import jit

dt = np.dtype([('counter', 'i4'),
               ('status', 'i4'),
               ('timestamp', 'f8')
               ])

dt_settings = np.dtype([('lag', 'i4'),
                        ('width', 'i4'), 
                        ('threshold', 'f8'),
                        ('influence', 'f8'),
                        ('mph', 'f8')
                        ])

dt_state = np.dtype([('avgFilter', 'f8'),
                     ('std2', 'f8'),
                     ('write_addr', 'i4'),
                     ('time_between', 'i4'),
                     ('length', 'i4'),
                     ('pk_idx', 'i4'),
                     ('pk', 'f8'),
                     ('pk_time', 'i4'),
                     ])

def sendSignal(PIN,Duration):
    GPIO.output(PIN,True)
    time.sleep(Duration)
    GPIO.output(PIN,False)
    return

@jit
def sg_block_v2(x, offset, z, Pxx, c, block_size, window_size):
    z1, z2 = z
    for idx in range(block_size):
        z0 = x[offset + idx] - x[offset + idx-window_size] + c*z1 -z2;
        z2 = z1;
        z1 = z0;
        Pxx[idx] = np.sqrt((z2*z2 + z1*z1 - c * z1*z2)/2)
    z[0] = z1
    z[1] = z2
    return Pxx, z


@jit
def peak_marking_block(y, yLen, filteredY, settings, state): #, signals): #, filteredY):
    lag = settings.lag

    avgFilter = state.avgFilter
    std2 = state.std2
    stdFilter = state.std2**0.5
    pks = []
    # start, length, pk, pk_idx = -1, 0, -1, -1
    write_addr = state.write_addr
    time_between = state.time_between
    length = state.length
    pk = state.pk
    pk_idx = state.pk_idx
    pk_time = state.pk_time
    #print(write_addr, length, pk, pk_idx)
    for i in range(yLen):
        # if (i%64) == 0:
        #    print(avgFilter, stdFilter)
        write_addr += 1
        time_between += 1
        
        idx = write_addr % lag
        prev_idx = (write_addr -1 + lag) % lag
        
        oldValue = filteredY[idx] # store this for use to update mean and std

        if abs(y[i] - avgFilter) > settings.threshold * stdFilter and (write_addr>=lag):
            if (y[i] > avgFilter):
                # signals[i] = 1
                #if start<0:
                if length==0:
                    start, length = i, 1
                    #print(start)
                else:
                    #print(i, start, length)
                    length += 1
                if y[i]>pk:
                    pk, pk_idx = y[i], i
                    pk_time = time_between
            else:
                # signals[i] = -1
                if (length>0) and (length>settings.width) and (pk>settings.mph):
                    # pks.append([start, length, pk_idx, pk, pk_time])
                    pks.append([pk_time, length, pk])
                    print("-1", i, start, length, pk_idx, pk)
                    start, length, pk, pk_idx = -1, 0, -1, -1
                    time_between = time_between - pk_time
            filteredY[idx] = settings.influence * y[i] + (1 - settings.influence) * filteredY[prev_idx]
        else:
            # signals[i] = 0
            filteredY[idx] = y[i]
            if (length>0) and (length>settings.width) and (pk>settings.mph):
                #pks.append([start, length, pk_idx, pk, pk_time])
                pks.append([pk_time, length, pk])
                #print("0:", i, start, length, pk, pk_idx, time_between, pk_time)
                time_between = time_between - pk_time 
            start, length, pk, pk_idx = -1, 0, -1, -1
        prevAvg = avgFilter
        avgFilter = avgFilter + (filteredY[idx] - oldValue) / lag
        #print(prevAvg, avgFilter, filteredY[i%lag], oldValue)
        # avgFilter = np.mean(filteredY)
        std2 = std2 + (filteredY[idx] - oldValue)*(filteredY[idx] + oldValue - avgFilter - prevAvg) /lag
        stdFilter = std2**0.5
        #stdFilter = np.std(filteredY)
    state.std2 = std2
    state.avgFilter = avgFilter
    state.write_addr = write_addr
    state.time_between = time_between
    state.length = length
    state.pk = pk
    state.pk_idx = pk_idx
    state.pk_time = pk_time

    return pks






if __name__ == '__main__':

    # initialize ID
    if len(sys.argv)>=2:
        role = sys.argv[1]
        print(role)
    else:
        role = 'responder'
        print('Please enter role. Otherwise, role is responder')

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
    



    # constant
    pin_OUT = 5

    CHUNK = 64 # buffer size
    RATE = 64000 # Hz
    f0 = 25000 # Hz
    duration = 0.004 # second

    NumRanging = 1

    settings_np = np.recarray(1, dtype=dt_settings)[0]
    settings_np.mph = 800
    settings_np.lag = 400
    settings_np.width = 250
    settings_np.threshold = 3
    settings_np.influence = 7e-3
    #print('settings', settings_np)

    broker_address = "192.168.68.131"
    topic_t3t2 = "ranging/delay/t3t2_delay_ms"


    initialStage_Duration = 5 # seconds
    warmUpStage_Duration = 5 # seconds


    
    # initialize ZMQ
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("ipc:///dev/shm/mic_data")
    socket.setsockopt(zmq.SUBSCRIBE, b'')




    # setup communication
    mqttc = myMQTT(broker_address)
    mqttc.registerTopic(topic_t3t2)
    # mqttc.registerTopic(topic_tell_IamReady2Recv)
    # mqttc.registerTopic(topic_check_ifOtherReady)
    # clear existing msg in topics
    if mqttc.checkTopicDataLength(topic_t3t2)>0:
        mqttc.readTopicData(topic_t3t2)
    # if mqttc.checkTopicDataLength(topic_check_ifOtherReady)>0:
    #     mqttc.readTopicData(topic_check_ifOtherReady)












    

    NumSigSamples = int(duration*RATE)
    NumIgnoredFrame = int(np.ceil(initialStage_Duration*RATE/CHUNK))
    NumWarmUpFrame = int(np.ceil(warmUpStage_Duration*RATE/CHUNK))

    # 
    k = int((f0/RATE)*NumSigSamples)

    w = 2*np.pi*k/NumSigSamples
    c = 2*np.cos(w)
    z = np.zeros(2)
    ring = np.zeros(512, np.int32)
    write_addr = 0
    Pxx = np.zeros(CHUNK)




    state = np.recarray(1, dtype=dt_state)[0]
    state.avgFilter = 0
    state.std2 = 0
    state.write_addr = -1
    state.length = 0
    state.pk_idx = -1
    state.pk = -1
    state.time_between = -1
    state.pk_time = -1


    settings = settings_np
    filteredY = np.zeros(settings.lag)
    packet_s = np.zeros(CHUNK)
    pks_blocks = []




    fulldata_temp = []
    fulldata = np.frompyfunc(list, 0, 1)(np.empty((NumRanging), dtype=object))

    counter = 0
    COUNT_PRE = 0
    counter_NumRanging = 0


    if ID == 1:
        Flag_ExpRX = False
        Flag_SendSig = True
    else:
        Flag_ExpRX = True
        Flag_SendSig = False


    Flag_abnormal = False
    Flag_initial = True
    Flag_warmUp = True
    Data_warmUp = []

    try:
        # start loop 
        while True:
            rawData = socket.recv()

            counter = counter + 1

            # Throw the first N second data
            if Flag_initial:
                if counter < NumIgnoredFrame:
                    continue
                if counter == NumIgnoredFrame:
                    Flag_initial = False
                    counter = 0
                    print('Beginning Data has been thrown away')
                    continue


            header = np.frombuffer(rawData[:16], dtype=dt)
            mic_data = np.frombuffer(rawData[16:], dtype=np.int32)
            mic = mic_data >>14
            COUNT = header[0][0]
            status = header[0][1]
            TS = header[0][2] # no use so far

            
            # Warm up period: Collect data to calculate DC offset
            if Flag_warmUp:
                if counter < NumWarmUpFrame:
                    Data_warmUp.append(mic)
                    continue
                if counter == NumWarmUpFrame:
                    if len(Data_warmUp)>0:
                        DCOffset = int(np.mean(np.concatenate(Data_warmUp)))
                    else:
                        DCOffset = 0
                    Flag_warmUp = False
                    counter = 0
                    COUNT_PRE = COUNT
                    print("DC Offset is ",DCOffset)
                    continue


            ## for debug and record purposes:
            fulldata_temp.append(mic - DCOffset)
            if counter == 1:
                startTime = time.time()
                print('Start Time is ',startTime)
            ## End


            ## Check abnormal input data
            # Currently haven't figure out a solution when it happens
            if status:
                Flag_abnormal = True
                print('[Abnormal Mic Data] Input Overflow: ',counter,COUNT,status)

            if (COUNT - COUNT_PRE) != 1:
                Flag_abnormal = True
                print('[Abnormal Mic Data] Missing a Buffer at Subscriber: ',counter,COUNT,COUNT_PRE)
            COUNT_PRE = COUNT
            ## End of checking abnormal input data



            if Flag_SendSig:
                sendSignal(pin_OUT,1e-4)
                Flag_SendSig = False
                print('[Signal Sent] ',counter_NumRanging,counter)


            
            ## mic data is ready to use for filtering and peak detection
            ring[(write_addr):(write_addr+CHUNK)] = mic - DCOffset
            Pxx, z = sg_block_v2(ring, write_addr, z, Pxx, c, CHUNK, NumSigSamples)
            write_addr += CHUNK
            write_addr &= (512-1)
            result = peak_marking_block(Pxx, len(Pxx), filteredY, settings, state, packet_s)
            if result:
                #debug purpose:
                pks_blocks.extend(result)

                if Flag_ExpRX:




                    ## For debug purpose, print out progress:
                    # print("Received signal from the other device")
                    ## End
                    # print("[RX Peak Detected] ",counter_NumRanging,counter,absIndex,Peak)
                    # T_RX = absIndex
                    Flag_ExpRX = False
                    Flag_SendSig = True
                    ## For debug and record purposes:
                    # RecvRX_RecordCounter.append(counter)
                    ## End

                    # jumpCount = 15

                    if ID == 1:
                        T4T1_Record[counter_NumRanging] = result[0][0]
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
                    # print("[TX Peak Detected] ",counter_NumRanging,counter,absIndex,Peak)
                    # T_TX = absIndex
                    Flag_ExpRX = True
                    ## For debug and record purposes:
                    # RecvTX_RecordCounter.append(self.counter)
                    ## End

                    # jumpCount = 10  

                    # 2. For responder only:
                    if ID == 2:
                        T3T2_Record[counter_NumRanging] = result[0][0]
                        ## For debug and record purposes
                        fulldata[counter_NumRanging] = fulldata_temp
                        fulldata_temp = []
                        ## End
                        counter_NumRanging = counter_NumRanging + 1
                    


            if counter_NumRanging>=NumRanging:
                print("Ranging Finished! Total counter=",counter)
                break
            








    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Terminated by User")

    Duration = time.time()-startTime
    GPIO.cleanup()

    if ID == 1:
        while True:
            if mqttc.checkTopicDataLength(topic_t3t2)>=NumRanging:
                break
        ## For debug purpose, print out progress:
        # print("Received T3-T2")
        ## End
        T3T2 = mqttc.readTopicData(topic_t3t2)
        print("Read all T3-T2")
        # print("T3T2:")
        # print(T3T2)            
        Ranging_Record = SOUNDSPEED*(T4T1_Record - T3T2)/2/RATE
        a = Ranging_Record[(Ranging_Record>0) & (Ranging_Record<5)]
        print("T4T1:")
        print(T4T1_Record)
        # print(a)
        if a.size>0:
            print(len(a),np.mean(a),np.std(a))
        mqttc.closeClient()
    else:
        mqttc.sendMsg(topic_t3t2, T3T2_Record)
        print(T3T2_Record)
        print("Sending T3-T2 Status: Done")
        time.sleep(5)

    print("Duration is ", Duration)

    allFullData = np.concatenate(fulldata)
    a_file = open('Fulldata_'+role+'.dat', "w")
    np.savetxt(a_file, allFullData, fmt='%d', delimiter=',')
    a_file.close()
    print('Data written to file.')
    logData = open('RawData_'+role+'.dat','wb')

    print("Length of the raw data is ",len(rawData))
    for d in rawData:
        buf = bytes(d)
        logData.write(buf)
        mic = np.frombuffer(d, dtype=np.int32)
    
    logData.close()

    print('Finished Writing Raw Data to Files')
