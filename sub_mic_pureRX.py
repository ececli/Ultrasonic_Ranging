import sys
import zmq
import time


# sub_mic_pureRX.py is the code that the mic record the data all the time without sending any signal out. 


# Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect ("ipc:///dev/shm/mic_data")
socket.setsockopt(zmq.SUBSCRIBE, b"")


rawData_all = []

counter = 0
maxCounter = 20000

while True:
    rawData = socket.recv()
    rawData_all.append(rawData)

    counter = counter + 1

    if counter>=maxCounter:
        print("Finished")
        break




logData = open('RawData_pureRX.dat','wb')
print("Length of the raw data is ",len(rawData_all))
for d in rawData_all:
    buf = bytes(d)
    logData.write(buf)

logData.close()

print('Finished Writing Raw Data to Files')


