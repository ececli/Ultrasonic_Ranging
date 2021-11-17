import time
import zmq
import sys
import numpy as np



if __name__ == '__main__':


    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("ipc:///dev/shm/mic_data")
    socket.setsockopt(zmq.SUBSCRIBE, b'')

    CHUNK = 64

    counter = []
    abnornalStatus = []
    while True:
        COUNT,status,TS,b_data = socket.recv_multipart()
        # print(int(counter),int(status),float(TS))
        mic_data = np.frombuffer(b_data, dtype=np.int32)
        # print(mic_data)
        counter.append(COUNT)
        if int(status):
            print([COUNT,status])
            abnornalStatus.append([COUNT,status])
        print(int(COUNT))
    print(np.unique(np.diff(counter)))



