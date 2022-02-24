import zmq
import time
import numpy as np

'''
dt_bt = np.dtype([('id','U17'),
                ('status', 'i4'),
                ('NumSamples', 'i4')
                ])
'''
dt_bt = np.dtype([('Flag_Valid','?'),
                ('Flag_NumSamples', '?'),
                ('Flag_LastReq','?'),
                ('NumSamples', 'i4'),
                ('NumReTransmission', "i4")
                ])

# setup ZMQ
context = zmq.Context()

publisher = context.socket(zmq.PUB)
publisher.bind("ipc:///dev/shm/Data2Blue")

subscriber = context.socket(zmq.SUB)
subscriber.connect("ipc:///dev/shm/Blue2Data")
subscriber.setsockopt(zmq.SUBSCRIBE, b'')








while True:
    # T3T2 = client_sock.recv(255).decode()
    # print(counter, time.time())
    try:
        blue2data = subscriber.recv(flags=zmq.NOBLOCK)
        bt_data = np.frombuffer(blue2data, dtype=dt_bt)
        print("length of data is ", len(bt_data), bt_data)

        publisher.send(blue2data)

    except zmq.Again as e:
        pass



    
    # bt_data = np.frombuffer(raw_bt_data, dtype=dt_bt)
    # print(counter)
    
    # print(bt_data[-1][3])
    
    # T3T2 = bt_data[-1][3]
    # client_sock.send(str(T3T2).encode())
    # counter = counter + 1
    # time.sleep(1)



