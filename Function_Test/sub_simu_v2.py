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

    dt = np.dtype([('counter', 'i4'), ('status', 'i4'), ('timestamp', 'f8')])

    counter = []
    abnornalStatus = []
    TS_pre = 0
    while True:
        rawData = socket.recv()

        header = np.frombuffer(rawData[:16], dtype=dt)
        mic_data = np.frombuffer(rawData[16:], dtype=np.int32)
        mic_data = mic_data >>14
        counter = header[0][0]
        status = header[0][1]
        TS = header[0][2]

        
        print(counter,status,TS,TS - TS_pre)
        TS_pre = TS
        print(mic_data)





