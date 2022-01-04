import zmq
import numpy as np
import time


dt = np.dtype([('counter', 'i4'),
               ('status', 'i4'),
               ('timestamp', 'f8')
               ])


# initialize ZMQ
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("ipc:///dev/shm/mic_data")
socket.setsockopt(zmq.SUBSCRIBE, b'')



counter = 0
while True:
    rawData = socket.recv()
    counter = counter + 1
    header = np.frombuffer(rawData[:16], dtype=dt)
    mic_data = np.frombuffer(rawData[16:], dtype=np.int32)
    mic = mic_data >>14
    COUNT = header[0][0]
    status = header[0][1]
    TS = header[0][2] # no use so far
    print(COUNT,mic)
    if counter == 10:
        break






# Prepare our context and publisher
string = "tcp://192.168.68.130:5556"
client = context.socket(zmq.REQ)
client.connect(string)

client.send_string("Chang")
a = client.recv_pyobj()
print(a)
