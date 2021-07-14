import time
import pigpio
 
duration = 2000 # microsecond
f0 = 25000 # 10khz signal
pin_OUT = 12

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

PIN_OUT_MASK = 1<<pin_OUT

# generate the wave form for one period
pulse_1_period = [(PIN_OUT_MASK,0,bit_time),(0,PIN_OUT_MASK,bit_time)]
# repeat
pulses = pulse_1_period * NumPeriod_Signal
wf = []
for p in pulses:
    wf.append(pigpio.pulse(p[0], p[1], p[2]))

pi_IO = pigpio.pi()
if not pi_IO.connected:
    exit()

pi_IO.set_mode(pin_OUT, pigpio.OUTPUT)

pi_IO.wave_clear()
pi_IO.wave_add_generic(wf)
wid = pi_IO.wave_create()

if wid >= 0:
    beforeTS = pi_IO.get_current_tick()
    pi_IO.wave_send_once(wid)
    afterTS = pi_IO.get_current_tick()

    pi_IO.wave_delete(wid)
 
pi_IO.stop()
print(beforeTS,afterTS,afterTS-beforeTS)

"""
fields = '10101010101010101'
gpio = 12
levels = int(fields, 2)
edges= [1<<gpio, levels]
num_bits = len(fields)

 
for bit in range(num_bits-1, -1, -1):
    print(bit)
    on = 0
    off = 0
    if (1<<bit) & edges[1]:
        on |= edges[0]
    else:
        off |= edges[0]
    pulses.append((on, off, bit_time))
 
pi = pigpio.pi()
if not pi.connected:
    exit()

pi.set_mode(gpio, pigpio.OUTPUT)

 
wf = []
for p in pulses:
    wf.append(pigpio.pulse(p[0], p[1], p[2]))
 
pi.wave_clear()
pi.wave_add_generic(wf)
 
wid = pi.wave_create()
if wid >= 0:
    pi.wave_send_once(wid)

    pi.wave_delete(wid)
 
pi.stop()
"""