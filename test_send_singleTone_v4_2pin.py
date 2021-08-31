import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time
import pigpio

def genTestWaveForm_2pin(f0, sig_duration, int_duration, numSignal, pin1, pin2):
    # Example:
    # f0 = 25000 # 10khz signal
    # sig_duration = 4000 # microsecond
    # int_duration = 80000 # microsecond
    # numSignal = 10
    # pin1 = 12
    # pin2 = 4

    period = 1.0/f0*1e6 # period of the signal (microsecond)
    NumPeriod_Signal = int(sig_duration/period)
    bit_time = int(period/2) # bit time in microsecond

    actual_f0 =  1/(bit_time*2)*1e6 # in Hz
    actual_duration = bit_time*2*NumPeriod_Signal # in microsecond

    if actual_f0 != f0:
        print("Warning: Actual f0 is ",actual_f0)
        
    if actual_duration != sig_duration:
        print("Warning: Actual duration is ",actual_duration)

    PIN1_MASK = 1<<pin1
    PIN2_MASK = 1<<pin2

    # generate the signal wave-form for one period
    pulse_1_period = [(PIN1_MASK,PIN2_MASK,bit_time),(PIN2_MASK,PIN1_MASK,bit_time)]
    # pulse_end_period = (0,PIN2_MASK,bit_time)
    # generate the no-signal interval for int_duration period
    pulse_end_period = (0,PIN2_MASK,int_duration)
    # repeat
    pulses = pulse_1_period * NumPeriod_Signal
    pulses.append(pulse_end_period)
    
    pulses_all = pulses * numSignal
    wf = []
    for p in pulses_all:
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
sig_duration = 4000
int_duration = 80000
numSignal = 10


pi_IO = pigpio.pi()
pi_IO.set_mode(pin1,pigpio.OUTPUT)
pi_IO.set_mode(pin2,pigpio.OUTPUT)

# generate wave form
wf = genTestWaveForm_2pin(f0, sig_duration, int_duration, numSignal, pin1, pin2)
wid = createWave(pi_IO, wf)
TS = sendWave(pi_IO, wid)
print(TS)



deleteWave(pi_IO, wid)
pi_IO.stop()

