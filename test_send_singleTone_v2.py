# send out a single tone via a buzzer.
# The frequency of the tone is f0.
# The duration of the tone is duration.
# The code records the timestamps of each tone sent
import configparser
import pigpio
import time
import matplotlib.pyplot as plt
import numpy as np
import twoWayRangingLib as func

confFile = "UR_pyConfig.conf"

cp = configparser.ConfigParser()
cp.read(confFile)


f0 = cp.getint("SIGNAL","f0")
duration = cp.getint("SIGNAL","duration")
ratio = cp.getint("SPEAKER","ratio")
pin_OUT = cp.getint("SPEAKER","pin_OUT")

# pin_OUT = 12
# ratio = 500000
# f0 = 30000
# duration = 2000 # microseconds
numSending = 1
intervalSending = 1 # seconds

pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.hardware_PWM(pin_OUT,0,0)

counter = 0
while True:
    T1 = func.sendSingleTone(pi_IO,pin_OUT,f0,duration,ratio)
    counter = counter + 1
    print(counter,T1)
    time.sleep(intervalSending)
    if counter >=numSending:
        break

pi_IO.stop()

