import zmq
import time
import numpy as np


dt_bt = np.dtype([('Flag_Valid','?'),
                ('Flag_NumSamples', '?'),
                ('Flag_LastReq','?'),
                ('NumSamples', 'i4'),
                ('NumReTransmission', "i4")
                ])

bt_data = np.rec.array(np.zeros(1, dtype=dt_bt))



# setup ZMQ
context = zmq.Context()

publisher = context.socket(zmq.PUB)
publisher.bind("ipc:///dev/shm/Data2Blue")

subscriber = context.socket(zmq.SUB)
subscriber.connect("ipc:///dev/shm/Blue2Data")
subscriber.setsockopt(zmq.SUBSCRIBE, b'')








counter = 0
startTime = time.time()

bt_data.Flag_NumSamples = True
bt_data.Flag_Valid = True 
bt_data.NumSamples = counter
bt_data.NumReTransmission = 0
bt_data.Flag_LastReq = False
raw_bt_data = bt_data.tobytes()
publisher.send(raw_bt_data)


while True:
    '''
    bt_data.Flag_NumSamples = True
    bt_data.Flag_Valid = True 
    bt_data.NumSamples = counter
    bt_data.NumReTransmission = 0
    bt_data.Flag_LastReq = False
    raw_bt_data = bt_data.tobytes()
    publisher.send(raw_bt_data)
    '''
    # print("Sent ",counter)
    # Received = bt_sock.recv(255)
    # print(Received)
    # bt_data_R = np.frombuffer(Received, dtype=dt_bt)
    # print(Received)

    try:
        blue2data = subscriber.recv(flags=zmq.NOBLOCK)
        counter = counter + 1

        bt_data.NumSamples = counter
        raw_bt_data = bt_data.tobytes()
        publisher.send(raw_bt_data)

    except zmq.Again as e:
        pass

    # counter = counter + 1
    # time.sleep(0.5)

    if counter == 1000:
        duration = time.time() - startTime
        break



time.sleep(10)
print("Duration is ", duration)


