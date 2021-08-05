import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import twoWayRangingLib as func
import time
import pigpio




# init functions
pin1 = 12
pin2 = 4
f0=25000
f1=30000
duration = 0.004

numSending = 1
intervalSending = 1 # seconds

pi_IO = pigpio.pi()
pi_IO.set_mode(pin1,pigpio.OUTPUT)
pi_IO.set_mode(pin2,pigpio.OUTPUT)

# generate wave form
wf = func.genChirpWaveForm_2pin(f0, f1, duration, pin1, pin2)
wid = func.createWave(pi_IO, wf)


for k in range(numSending):
    TS = func.sendWave(pi_IO, wid)
    print(TS)
    time.sleep(intervalSending)



func.deleteWave(pi_IO, wid)
pi_IO.stop()

