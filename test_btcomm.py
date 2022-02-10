import time
import zmq
import sys
import numpy as np
import RPi.GPIO as GPIO
from scipy import signal
import socket
from numba import jit
import os
import csv







if __name__ == '__main__':

    # initialize ID
    if len(sys.argv)>=2:
        role = sys.argv[1]
        print(role)
    else:
        role = 'responder'
        print('Please enter role. Otherwise, role is responder')




    if role == 'initiator':
        ID = 1
        theOtherID = 2
    elif role == 'responder':
        ID = 2
        theOtherID = 1
    else:
        ID = 2
        theOtherID = 1
        print("role options: 1. initiator, 2. responder (default)")
    

    
    # initialize ZMQ
    ## Subscribe mic data
    context = zmq.Context()
    mic_subscriber = context.socket(zmq.SUB)
    mic_subscriber.connect("ipc:///dev/shm/mic_data")
    mic_subscriber.setsockopt(zmq.SUBSCRIBE, b'')

    ## Tell BtComm the ID of this device
    control_server = context.socket(zmq.REP)
    control_server.bind("ipc:///dev/shm/control_data")
    message = control_server.recv_string()
    print("Received Request: ",message)
    time.sleep(1)
    control_server.send_pyobj(ID)

    ## 
    if ID == 2: # responder, which need to send data out
        T3T2_publisher = context.socket(zmq.PUB)
        T3T2_publisher.bind("ipc:///dev/shm/T3T2_data")
    if ID == 1:
        T3T2_subscriber = context.socket(zmq.SUB)
        T3T2_subscriber.connect("ipc:///dev/shm/T3T2_data")
        T3T2_subscriber.setsockopt(zmq.SUBSCRIBE, b'')

    counter = 0
    while True:
        if ID == 2:
            T3T2_publisher.send_pyobj(counter)
            time.sleep(1)
            counter = counter + 1
            if counter == 10:
                break

        if ID == 1:
            T3T2_R = T3T2_subscriber.recv_pyobj()
            print(T3T2_R,type(T3T2_R))






    

