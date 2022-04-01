import pigpio
import time

ratio = 500000
pin_OUT = 12

f0 = 30000
plus = -500
stop_f0 = 20000
duration = 10

pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.hardware_PWM(pin_OUT,0,0)

print("f0 = ",f0)
while True:
    pi_IO.hardware_PWM(pin_OUT,f0,ratio)

    startTS = time.time()
    while (time.time() - startTS) < duration:
        pass
    f0 = f0 + plus
    print("f0 = ",f0)
    if f0 < stop_f0:
        break

pi_IO.hardware_PWM(pin_OUT,0,0)
