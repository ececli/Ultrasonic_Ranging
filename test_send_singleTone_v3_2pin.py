import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time
import pigpio

def genWaveForm(f0, duration, pin):
    # Example: 
    # duration = 2000 # microsecond
    # f0 = 25000 # 10khz signal
    # pin_OUT = 12

    # Note: with f0=31250 Hz, duration can be 1600, 1920 or 3200 microseconds

    period = 1.0/f0*1e6 # period of the signal
    NumPeriod_Signal = int(duration/period)
    bit_time = int(period/2) # bit time in microsecond

    actual_f0 =  1/(bit_time*2)*1e6 # in Hz
    actual_duration = bit_time*2*NumPeriod_Signal # in microsecond

    if actual_f0 != f0:
        print("Warning: Actual f0 is ",actual_f0)
        
    if actual_duration != duration:
        print("Warning: Actual duration is ",actual_duration)

    PIN_MASK = 1<<pin

    # generate the wave form for one period
    pulse_1_period = [(PIN_MASK,0,bit_time),(0,PIN_MASK,bit_time)]
    # repeat
    pulses = pulse_1_period * NumPeriod_Signal
    wf = []
    for p in pulses:
        wf.append(pigpio.pulse(p[0], p[1], p[2]))
    
    return wf



def genWaveForm_2pin(f0, duration, pin1, pin2):
    # Example: 
    # duration = 2000 # microsecond
    # f0 = 25000 # 10khz signal
    # pin_OUT = 12

    # Note: with f0=31250 Hz, duration can be 1600, 1920 or 3200 microseconds

    period = 1.0/f0*1e6 # period of the signal
    NumPeriod_Signal = int(duration/period)
    bit_time = int(period/2) # bit time in microsecond

    actual_f0 =  1/(bit_time*2)*1e6 # in Hz
    actual_duration = bit_time*2*NumPeriod_Signal # in microsecond

    if actual_f0 != f0:
        print("Warning: Actual f0 is ",actual_f0)
        
    if actual_duration != duration:
        print("Warning: Actual duration is ",actual_duration)

    PIN1_MASK = 1<<pin1
    PIN2_MASK = 1<<pin2

    # generate the wave form for one period
    pulse_1_period = [(PIN1_MASK,PIN2_MASK,bit_time),(PIN2_MASK,PIN1_MASK,bit_time)]
    pulse_end_period = (0,PIN2_MASK,bit_time)
    # repeat
    pulses = pulse_1_period * NumPeriod_Signal
    pulses.append(pulse_end_period)
    
    wf = []
    for p in pulses:
        wf.append(pigpio.pulse(p[0], p[1], p[2]))
    
    return wf


def createWave(pi_IO, wf):
    pi_IO.wave_clear()
    pi_IO.wave_add_generic(wf)
    wid = pi_IO.wave_create()
    return wid

def deleteWave(pi_IO, wid):
    pi_IO.wave_delete(wid)

def sendWave(pi_IO, wid):
    # duration is in microsecond
    # this function uses wave function in pigpio
    # pro: controling sending time accurately.
    # con: cannot use arbitrary combination of f0 and duration
    # suggestion: f0=31250 Hz with duration =1600 or 1920 or 3200 microseconds
    # OR: f0=25000 with duration 2000
    # To use this function, use createWave and genWaveForm first.
    pi_IO.wave_send_once(wid)
    startTS = pi_IO.get_current_tick() # version 1
    return startTS



# init functions
pin1 = 12
pin2 = 4
f0=25000
duration = 4000

numSending = 5
intervalSending = 3 # seconds

pi_IO = pigpio.pi()
pi_IO.set_mode(pin1,pigpio.OUTPUT)
pi_IO.set_mode(pin2,pigpio.OUTPUT)

# generate wave form
wf = genWaveForm_2pin(f0, duration, pin1, pin2)
wid = createWave(pi_IO, wf)


for k in range(numSending):
    TS = sendWave(pi_IO, wid)
    print(TS)
    time.sleep(intervalSending)



deleteWave(pi_IO, wid)
pi_IO.stop()
