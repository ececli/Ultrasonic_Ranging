import zmq
import numpy as np
import time

'''
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

'''

a = np.int32(np.floor(np.random.rand(10)*10000))
print(a)
string = "tcp://192.168.68.131:5563"

# Prepare our context and publisher
context2 = zmq.Context()
subscriber = context2.socket(zmq.SUB)
subscriber.connect(string)

subscriber.setsockopt(zmq.SUBSCRIBE, b'')
print("Waiting data from the other device now")
while True:
    a = subscriber.recv_pyobj()
    if a:
        print(a)
        break

# subscriber.setsockopt(zmq.SUBSCRIBE, b"")

# while True:
    # Read envelope with address
    # [address, contents] = subscriber.recv_multipart()
    # print("[%s] %s" % (address, contents))

time.sleep(5)