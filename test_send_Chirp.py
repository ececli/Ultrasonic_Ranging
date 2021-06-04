# send out a single tone via a buzzer.
# The frequency of the tone is f0.
# The duration of the tone is duration.
# The code records the timestamps of each tone sent
import configparser
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time
import os
import pigpio
import twoWayRangingLib as func


# Load Parameters
confFile = "UR_pyConfig.conf"
cp = configparser.ConfigParser()
cp.read(confFile)


RATE = cp.getint("MIC","RATE")

f1 = cp.getint("SIGNAL","f1")
f0 = cp.getint("SIGNAL","f0") 
duration = cp.getint("SIGNAL","duration") # microseconds
TIMEOUTCOUNTS = cp.getint("SIGNAL","TimeoutCounts")

ratio = cp.getint("SPEAKER","ratio")
pin_OUT = cp.getint("SPEAKER","pin_OUT")


numSending = 10
intervalSending = 0.4 # seconds

pi = pigpio.pi()
pi.set_mode(pin_OUT,pigpio.OUTPUT)
pi.hardware_PWM(pin_OUT,0,0)

counter = 0
while True:
    func.sendChirp(pi,pin_OUT,f0,f1,duration,RATE,ratio)
    counter = counter + 1
    time.sleep(intervalSending)
    if counter >=numSending:
        break

pi.stop()

