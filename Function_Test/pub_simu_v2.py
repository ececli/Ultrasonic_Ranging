import time
import zmq
import sys
import numpy as np

# Simulate pub. Reading data from file and pub it out. 


if __name__ == '__main__':
    if len(sys.argv)>=2:
        filename = sys.argv[1]
    else:
        print('Missing data filename. Please try again')
        exit()

    with open(filename, 'rb') as fd:
        raw = fd.read()
    print('Data read from file.')


    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    # socket.bind("tcp://*:%s" % port)
    socket.bind("ipc:///dev/shm/mic_data")
    time.sleep(1)

    CHUNK = 64
    RATE = 64000
    status = 0

    dt = np.dtype([('counter', 'i4'), ('status', 'i4'), ('timestamp', 'f8')])
    header = np.rec.array(np.zeros(1, dtype=dt))

    for counter in range(int(len(raw)/4/CHUNK)):
    # for counter in range(10):

        b_data = bytes(raw[(counter*4*CHUNK):((counter+1)*4*CHUNK)])
        header.counter  = counter
        header.status = status
        header.timestamp = time.time()
        raw_header = header.tobytes()
        socket.send(raw_header+b_data)
        time.sleep(CHUNK/RATE)

    print("Publish Done.")
    time.sleep(3)
