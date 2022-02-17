import time
import zmq
import sys
import numpy as np
import RPi.GPIO as GPIO
from scipy import signal
import socket
from numba import jit
import os
import csv
import bluetooth

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
                     ('pk_time', 'i4')
                     ])


dt_bt = np.dtype([('Valid','?'),
                ('Flag_NumSamples', '?'),
                ('NumSamples', 'i4'),
                ('NumReTransmission', "i4")
                ])


def create_csv(filename, csv_head):
    if not os.path.exists(filename):
        with open(filename, "w") as f:
            csv_write = csv.writer(f)
            csv_write.writerow(csv_head)

def write_csv(filename, csv_data):
    with open(filename, "a+") as f:
        csv_write = csv.writer(f)
        csv_write.writerow(csv_data)





def sendSignal(PIN,Duration):
    GPIO.output(PIN,True)
    time.sleep(Duration)
    GPIO.output(PIN,False)
    return

'''
def get_ip_address():
    ip_address = '';
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8",80))
    ip_address = s.getsockname()[0]
    s.close()
    return ip_address
'''

# Sliding Goertzol Filtering without any windowing
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

# Sliding Goertzol Filtering with Hanning windowing
@jit
def sg_block_win_v3(x, offset, zz, Pxx, ccc, bbb, block_size, window_size):

    z01, z02, z11, z12, z21, z22 = zz
    c0, c1, c2 = ccc
    b0, b1 = bbb
    # c0 = np.cos(2*np.pi*k/N)
    # c1 = np.cos(2*np.pi*(k-1)/N)
    # c2 = np.cos(2*np.pi*(k+1)/N)
    # b0 = np.cos(2*np.pi/N)
    # b1 = np.cos(4*np.pi/N)
    for idx in range(block_size):
        z00 = x[offset + idx] - x[offset + idx - window_size] + 2*c0*z01 -z02
        z02 = z01
        z01 = z00
        z10 = x[offset + idx] - x[offset + idx - window_size] + 2*c1*z11 -z12
        z12 = z11
        z11 = z10
        z20 = x[offset + idx] - x[offset + idx - window_size] + 2*c2*z21 -z22
        z22 = z21
        z21 = z20

        Pxx[idx] = np.sqrt(4*(z01*z01+z02*z02) + (z11*z11+z12*z12) + (z21*z21+z22*z22) - 4*(b0*z01*z11+z02*z12) - 4*(b0*z01*z21+z02*z22) + 2*(b1*z11*z21+z12*z22) - 2*(2*c0*z01-c1*z11-c2*z21)*(2*z02-z12-z22))
    zz[0] = z01
    zz[1] = z02
    zz[2] = z11
    zz[3] = z12
    zz[4] = z21
    zz[5] = z22
    return Pxx, zz


# Z-score peak finding algorithm, modified by Sae Woo Nam (NIST)
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


    if len(sys.argv)>=3:
        GT = sys.argv[2]
        print("Ground Truth is set as ", GT)
    else:
        GT = 0
        print("Ground Truth is not set")




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


    address_list = ['DC:A6:32:E1:9F:C8', 'DC:A6:32:E8:BF:E0']


    port = 1
    



    # constant

    SOUNDSPEED = 343

    pin_OUT = 5

    CHUNK = 64 # buffer size
    RATE = 64000 # Hz
    f0 = 25000 # Hz
    duration = 0.004 # second

    NumRanging = 100

    jumpCount_Set = 20


    Flag_usingWindowing = False # choose whether to use hanning windowing or not

    Flag_write2CSV = False

    ringBufferSize = 512

    max_timeoutCount = 10000


    


    settings_np_noWin = np.recarray(1, dtype=dt_settings)[0]
    settings_np_noWin.mph = 800
    settings_np_noWin.lag = 400
    settings_np_noWin.width = 250
    settings_np_noWin.threshold = 3
    settings_np_noWin.influence = 7e-3


    settings_np_usingWin = np.recarray(1, dtype=dt_settings)[0]
    settings_np_usingWin.mph = 573
    settings_np_usingWin.lag = 509
    settings_np_usingWin.width = 168
    settings_np_usingWin.threshold = 3
    settings_np_usingWin.influence = 9.8e-3




    initialStage_Duration = 5 # seconds
    warmUpStage_Duration = 5 # seconds


    csv_filename = "two_way_ranging_results.csv"
    csv_head = ["Set_Number_Ranging","Actual_Number_Ranging", "JumpCount", "Windowing", "Ground_Truth", "Mean", "Std", "Duration", "Start_Time", "Abnormal"]


    # setup bluetooth

    own_address = bluetooth.read_local_bdaddr()[0]

    for addr in address_list:
        if own_address.upper() != addr.upper():
            target_address = addr

    print("Own Address is ", own_address)
    print("Target Address is ",target_address)

    bt_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    if ID == 1: # responder will bind and serve as a server
        bt_sock.bind(("", port))   
        bt_sock.listen(1)
        print("Binded to own device with port %d." % port)

        print('Waiting data from the other device')
        client_sock, address = bt_sock.accept()  
        print("Accepted connection from ", address)
        time.sleep(1)

    else: # initiator will act as a client
        time.sleep(2)
        bt_sock.connect((target_address, port))
        print("Connected to the other device.")
        

     



    # GPIO init
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin_OUT,GPIO.OUT)
    GPIO.output(pin_OUT,False)


    
    # initialize ZMQ
    ## Subscribe mic data
    context = zmq.Context()
    mic_subscriber = context.socket(zmq.SUB)
    mic_subscriber.connect("ipc:///dev/shm/mic_data")
    mic_subscriber.setsockopt(zmq.SUBSCRIBE, b'')

    ''' 
    ## Tell BtComm the ID of this device
    control_server = context.socket(zmq.REP)
    control_server.bind("ipc:///dev/shm/control_data")
    message = control_server.recv_string()
    print("Received Request: ",message)
    time.sleep(1)
    control_server.send_pyobj(ID)

    ## 
    if ID == 2: # responder, which need to send data out
        T3T2_publisher = context.socket(zmq.PUB)
        T3T2_publisher.bind("ipc:///dev/shm/T3T2_data")
    if ID == 1:
        T3T2_subscriber = context.socket(zmq.SUB)
        T3T2_subscriber.connect("ipc:///dev/shm/T3T2_data")
        T3T2_subscriber.setsockopt(zmq.SUBSCRIBE, b'')



        # server = context.socket(zmq.REP)
        # server.bind("tcp://*:" + str(port))
        # print("Responder acts as a server. ")
    '''

    

    NumSigSamples = int(duration*RATE)
    NumIgnoredFrame = int(np.ceil(initialStage_Duration*RATE/CHUNK))
    NumWarmUpFrame = int(np.ceil(warmUpStage_Duration*RATE/CHUNK))

    # 
    k = int((f0/RATE)*NumSigSamples)

    # init for sliding goertzol filtering without hanning windowing
    c = 2*np.cos(2*np.pi*k/NumSigSamples)
    z = np.zeros(2)

    
    # init for hanning windowing approach
    c0 = np.cos(2*np.pi*k/NumSigSamples)
    c1 = np.cos(2*np.pi*(k-1)/NumSigSamples)
    c2 = np.cos(2*np.pi*(k+1)/NumSigSamples)
    b0 = np.cos(2*np.pi/NumSigSamples)
    b1 = np.cos(4*np.pi/NumSigSamples)
    ccc = np.asarray([c0,c1,c2])
    bbb = np.asarray([b0,b1])
    zz = np.zeros(6)


    ring = np.zeros(ringBufferSize, np.int32)
    write_addr = 0
    Pxx = np.zeros(CHUNK)

    # init z-score parameters
    if Flag_usingWindowing:
        settings = settings_np_usingWin
    else:
        settings = settings_np_noWin

    # for z-score peak finding algorithm
    state = np.recarray(1, dtype=dt_state)[0]
    state.avgFilter = 0
    state.std2 = 0
    state.write_addr = -1
    state.length = 0
    state.pk_idx = -1
    state.pk = -1
    state.time_between = -1
    state.pk_time = -1

    filteredY = np.zeros(settings.lag)
    


    # The purpose of this part is to speed up jit.
    # I hope there is a better way to do so. Instead of initializing state, settings and filteredY again
    sg_block_v2(np.zeros(512, np.int32), 0, np.zeros(2), np.zeros(CHUNK), c, CHUNK, NumSigSamples)
    sg_block_win_v3(np.zeros(512, np.int32), 0, np.zeros(6), np.zeros(CHUNK), ccc, bbb, CHUNK, NumSigSamples)

    peak_marking_block(np.zeros(CHUNK), len(np.zeros(CHUNK)), filteredY, settings, state)

    state = np.recarray(1, dtype=dt_state)[0]
    state.avgFilter = 0
    state.std2 = 0
    state.write_addr = -1
    state.length = 0
    state.pk_idx = -1
    state.pk = -1
    state.time_between = -1
    state.pk_time = -1

    filteredY = np.zeros(settings.lag)

    
    # debug purpose
    fulldata_temp = []
    fulldata = np.frompyfunc(list, 0, 1)(np.empty((NumRanging), dtype=object))
    pks_blocks = []

    counter = 0
    COUNT_PRE = 0
    counter_NumRanging = 0
    timeoutCount = 0


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

    Flag_jump = False
    jumpCount = 0

    Flag_speedUpJit = True

    T3T2_Record = np.zeros(NumRanging)
    T4T1_Record = np.zeros(NumRanging)
    Distance_Record = np.zeros(NumRanging)

    msgRXPD = " "
    msgSS = " "
    msgTXPD = " "

    timeoutEventCounter = 0
    Flag_waitingT3T2 = False


    bt_data_prepare = np.rec.array(np.zeros(1, dtype=dt_bt))

    try:
        # start loop 
        while True:
            rawData = mic_subscriber.recv()

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

            
            ## mic data is ready to use for filtering and peak detection
            ring[(write_addr):(write_addr+CHUNK)] = mic - DCOffset
            if Flag_usingWindowing:
                Pxx, zz = sg_block_win_v3(ring, write_addr, zz, Pxx, ccc, bbb, CHUNK, NumSigSamples)
            else:
                Pxx, z = sg_block_v2(ring, write_addr, z, Pxx, c, CHUNK, NumSigSamples)
            write_addr += CHUNK
            write_addr &= (ringBufferSize-1)
            result = peak_marking_block(Pxx, len(Pxx), filteredY, settings, state)






            if Flag_SendSig:
                sendSignal(pin_OUT,1e-4)
                Flag_SendSig = False
                msgSS = f"[Signal Sent], {counter_NumRanging}, {counter}"
                print(msgSS)



            if result:
                #debug purpose:
                # pks_blocks.extend(result)


                timeoutCount = 0




                # jumpCount = jumpCount_Set
                # Flag_jump = True

                if Flag_ExpRX:




                    ## For debug purpose, print out progress:
                    # print("Received signal from the other device")
                    # print("[RX Peak Detected] ",counter_NumRanging,counter,result)
                    msgRXPD = f"[RX Peak Detected], {counter_NumRanging}, {counter}, {result}"
                    print(msgRXPD)
                    ## End
                    # T_RX = absIndex
                    Flag_ExpRX = False
                    
                    ## For debug and record purposes:
                    # RecvRX_RecordCounter.append(counter)
                    ## End


                    T4T1 = result[0][0]
                    T4T1_Record[counter_NumRanging] = result[0][0]


                    ## For debug and record purposes
                    fulldata[counter_NumRanging] = fulldata_temp
                    fulldata_temp = []


                    # Flag_waitBluetooth = True

                    # Wait to receive Bluetooth signal. Two purposes: 
                    # 1. After receiving Bluetooth signal, send ultrasonic sound out
                    # 2. Get T3 - T2 info and calculate Distance.
                    if ID == 1:
                        raw_bt_data = client_sock.recv(255)
                    else:
                        raw_bt_data = bt_sock.recv(255)

                    bt_data = np.frombuffer(raw_bt_data, dtype=dt_bt)
                    if len(bt_data)>1: 
                        print("[Warning] More Than One Bluetooth Data Package Received")

                    if bt_data[-1][0] and bt_data[-1][1]:
                        T3T2_R = bt_data[-1][2]
                        Distance = SOUNDSPEED * (T4T1 - T3T2_R)/2/RATE 
                        Distance_Record[counter_NumRanging] = Distance
                        print("[Distance Estimate], %.3f, %d, %d" %(Distance, counter_NumRanging,counter))
                        counter_NumRanging = counter_NumRanging + 1


                    Flag_SendSig = True

                        '''
                        ## END
                        # Flag_waitingT3T2 = True
                        print("Waiting T3-T2")
                        checkTime_start = time.time()
                        T3T2_R = client_sock.recv(255).decode()
                        print("T3-T2 is ",T3T2_R)
                        T3T2_R = int(float(T3T2_R))
                        
                        checkTime = time.time() - checkTime_start
                        Distance = SOUNDSPEED * (T4T1 - T3T2_R)/2/RATE 
                        Distance_Record[counter_NumRanging] = Distance
                        print("[Distance Estimate], %.3f, %d, %d, %f" %(Distance, counter_NumRanging,counter,checkTime))

                        counter_NumRanging = counter_NumRanging + 1
                        '''





                else:
                    ## For debug purpose, print out progress:
                    # print("Received own signal")
                    ## End
                    # 1. General process after receiving its own signal
                    # print("[TX Peak Detected] ",counter_NumRanging,counter,result)
                    msgTXPD = f"[TX Peak Detected], {counter_NumRanging}, {counter}, {result}"
                    print(msgTXPD)
                    # T_TX = absIndex
                    Flag_ExpRX = True
                    ## For debug and record purposes:
                    # RecvTX_RecordCounter.append(self.counter)
                    ## End



                    T3T2 = result[0][0]
                    T3T2_Record[counter_NumRanging] = result[0][0]
                    ## For debug and record purposes
                    fulldata[counter_NumRanging] = fulldata_temp
                    fulldata_temp = []


                    # Send Bluetooth Data package to the other device with two purposes:
                    # 1. Let the other know that this device is ready to receive ultrasound signal from the other device
                    # 2. send T3-T2 information to the other device

                    bt_data_prepare.Valid = True
                    bt_data_prepare.Flag_NumSamples = True
                    bt_data_prepare.NumSamples = T3T2
                    bt_data_prepare.NumReTransmission = 0
                    raw_bt_data = bt_data_prepare.tobytes()

                        ## End
                        # print("Prepare to send T3-T2")
                        # time.sleep(0.1)
                    if ID == 1:
                        client_sock.send(raw_bt_data)
                    else:
                        bt_sock.send(raw_bt_data)
                    print("T3-T2 has been sent: ",T3T2)

                        # counter_NumRanging = counter_NumRanging + 1
                    






            if counter_NumRanging>=NumRanging:
                print("Ranging Finished! Total counter=",counter)
                GPIO.cleanup()
                # logFile.close()
                break
            

            
            if ID == 1:
                timeoutCount = timeoutCount + 1
                if timeoutCount >= max_timeoutCount:
                    print("Timeout")
                    timeoutEventCounter = timeoutEventCounter + 1
                    Flag_SendSig = True
                    Flag_ExpRX = False
                    timeoutCount = 0

            

            



    except KeyboardInterrupt:
        GPIO.cleanup()
        # logFile.close()
        print("Terminated by User")

    Duration = time.time()-startTime
    

    if ID == 1:
        # print("T4T1:")
        # print(T4T1_Record)
        # This operation only works on my home network since two devices' IP is 130 and 131


        # print("T3T2:")
        # print(T3T2)            
        
        a = Distance_Record[(Distance_Record>0) & (Distance_Record<5)]
        
        # print(a)
        if a.size>0:
            print(len(a),np.mean(a),np.std(a))
            if Flag_write2CSV:
                create_csv(csv_filename, csv_head)
                csv_data = [NumRanging,len(a),jumpCount_Set,int(Flag_usingWindowing),GT,np.mean(a),np.std(a),Duration,startTime, int(Flag_abnormal)]
                write_csv(csv_filename, csv_data)
        print("Numbers of Timeout happens ",timeoutEventCounter)
        # mqttc.closeClient()
    # else:
        # mqttc.sendMsg(topic_t3t2, T3T2_Record)
        # while True:

        # message = server.recv_string()
        # print("Received Request: ",message)
        # time.sleep(1)
        # server.send_pyobj(T3T2_Record)
        
        # publisher.send_pyobj(T3T2_Record)
        # print("T3T2:")
        # print(T3T2_Record)
        # print("Sending T3-T2 Status: Done")
        # time.sleep(5)

    print("Duration is ", Duration)
    # print("FullData: ")
    # print(fulldata)

    allFullData = np.concatenate(fulldata[0:counter_NumRanging])

    if Flag_usingWindowing:
        winStr = "_win_"
    else:
        winStr = "_noWin_"


    filename = 'Fulldata_'+role+time.strftime("_%Y%m%d_%H%M_")+winStr+'jump_'+str(jumpCount_Set)+'.dat'

    a_file = open(filename, "w")
    if len(fulldata_temp)>0:
        np.savetxt(a_file, np.concatenate((allFullData,fulldata_temp)), fmt='%d', delimiter=',')
    else:
        np.savetxt(a_file, allFullData, fmt='%d', delimiter=',')
    a_file.close()
    print('Full Data written to file.')

    '''
    if len(fulldata_temp)>0:
        b_file = open('Lastdata_'+role+'.dat', "w")
        np.savetxt(b_file, fulldata_temp, fmt='%d', delimiter=',')
        a_file.close()
        print('Last Data written to file.')
    '''
    '''
    logData = open('RawData_'+role+'.dat','wb')

    print("Length of the raw data is ",len(rawData))
    for d in rawData:
        buf = bytes(d)
        logData.write(buf)
        mic = np.frombuffer(d, dtype=np.int32)
    
    logData.close()

    print('Finished Writing Raw Data to Files')
    '''

