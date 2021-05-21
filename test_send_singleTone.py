# send out a single tone via a buzzer.
# The frequency of the tone is f0.
# The duration of the tone is duration.
# The code records the timestamps of each tone sent

import pigpio
import time
import matplotlib.pyplot as plt
import numpy as np

def sendSingleTone(pi,pin,f0,duration,ratio):
    pi.hardware_PWM(pin,f0,ratio)
    startTS = pi.get_current_tick()
    while (pi.get_current_tick() - startTS) < duration:
        pass
    pi.hardware_PWM(pin,0,0)


def cb_raisingEdgeTS(gpio, level, tick):
    global signal_TS_OUT
    signal_TS_OUT.append(tick)


global signal_TS_OUT
signal_TS_OUT = []
pin_OUT = 12
ratio = 500000
f0 = 30000
duration = 2000 # microseconds
numSending = 10
intervalSending = 0.4 # seconds

pi = pigpio.pi()
pi.set_mode(pin_OUT,pigpio.OUTPUT)
pi.hardware_PWM(pin_OUT,0,0)
cb_OUT = pi.callback(pin_OUT, pigpio.RISING_EDGE, cb_raisingEdgeTS)
counter = 0
while True:
    sendSingleTone(pi,pin_OUT,f0,duration,ratio)
    counter = counter + 1
    time.sleep(intervalSending)
    if counter >=numSending:
        break

pi.stop()
time.sleep(0.5)
plt.figure()
plt.plot(np.asarray(signal_TS_OUT)-signal_TS_OUT[0],'r.')
plt.show()

plt.figure()
plt.plot(np.diff(np.asarray(signal_TS_OUT)),'r.')
plt.show()