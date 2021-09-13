import rp2
from machine import Pin
import time

# Use two pins to generate 25 kHz signal with 2 ms or 4 ms. Two pins with reversed signal
# to achieve differential signals.
@rp2.asm_pio(set_init=(rp2.PIO.OUT_LOW,rp2.PIO.OUT_LOW))
def sendSignal_2pin():

    wait(1, pin, 0)
    set(x,24) # set 2 as 24 so that it will run 25 times
    label("FourPeriod")
    set(pins, 1) [1] # set pins to 0 (all low), 1 (one low one high), 2 (one high one low)
    set(pins, 2) [1]
    set(pins, 1) [1]
    set(pins, 2) [1]
    set(pins, 1) [1]
    set(pins, 2) [1]
    set(pins, 1) [1]
    set(pins, 2)
    jmp(x_dec,"FourPeriod")
    set(pins, 0)
    wait(0, pin, 0)



pin28 = Pin(28, Pin.IN, Pin.PULL_DOWN)

# Instantiate a state machine at 100 kHz, with set bound to Pin(26), 
# or Pin 26 and 27 if 2 pins are used
sm = rp2.StateMachine(0, sendSignal_2pin, freq=100000, set_base=Pin(26), in_base = pin28)

# Activate the state machine
sm.active(1)



