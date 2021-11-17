import time
import zmq
import sys
# import numpy as np

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
    status = 0


    for counter in range(int(len(raw)/4/CHUNK)):
        TS = time.time()
        b_data = bytes(raw[(counter*32):((counter+1)*32)])
        socket.send_multipart([b'%d' % counter,
                                b'%d' % status,
                                b'%f' % TS,
                                b_data])
        # block = np.frombuffer(b_data, dtype=np.int32)

    print("Publish Done.")
    time.sleep(3)
