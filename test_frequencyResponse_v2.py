import pigpio
import time

ratio = 500000
pin_OUT = 12

f0 = 30000


pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.hardware_PWM(pin_OUT,0,0)


try:
    
    pi_IO.hardware_PWM(pin_OUT,f0,ratio)
    print("Start sending ultrasound at frequency ",f0)
    while True:
        pass
except KeyboardInterrupt:
    print("Terminated by user")

pi_IO.hardware_PWM(pin_OUT,0,0)

