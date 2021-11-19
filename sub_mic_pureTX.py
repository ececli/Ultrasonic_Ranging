import sys
import zmq
import time
import RPi.GPIO as GPIO


# sub_mic_pureTX.py is the code that the mic record the data all the time while the buzzer sends a single tone periodically. 


def sendSignal(PIN,Duration):
    GPIO.output(PIN,True)
    time.sleep(Duration)
    GPIO.output(PIN,False)
    return



# Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect ("ipc:///dev/shm/mic_data")
socket.setsockopt(zmq.SUBSCRIBE, b"")



pin_OUT = 5

# GPIO init
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_OUT,GPIO.OUT)
GPIO.output(pin_OUT,False)



rawData_all = []

counter = 0
Flag_SendSig = False
maxCounter = 11000

while True:
    rawData = socket.recv()
    rawData_all.append(rawData)

    counter = counter + 1


    if Flag_SendSig:
        sendSignal(pin_OUT,1e-4)
        Flag_SendSig = False
        print('[Signal Sent at counter ] ',counter)
        # continue
    
    
    if counter%500 == 0:
        Flag_SendSig = True


    if counter>=maxCounter:
        print("Finished")
        break



GPIO.cleanup()


logData = open('RawData_pureTX.dat','wb')
print("Length of the raw data is ",len(rawData_all))
for d in rawData_all:
    buf = bytes(d)
    logData.write(buf)

logData.close()

print('Finished Writing Raw Data to Files')


