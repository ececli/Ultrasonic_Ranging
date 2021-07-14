import pigpio
import twoWayRangingLib as func
import time

f0 = 25000
duration = 2000
pin_OUT = 12

numSending = 5
intervalSending = 5 # seconds



pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)

wf = func.genWaveForm(f0, duration, pin_OUT)
wid = func.createWave(pi_IO, wf)

for k in range(numSending):
    TS = func.sendWave(pi_IO, wid)
    print(TS)
    time.sleep(intervalSending)

func.deleteWave(pi_IO, wid)
pi_IO.stop()