import pigpio
import matplotlib.pyplot as plt
import numpy as np
import configparser

# Load Parameters
confFile = "UR_pyConfig.conf"
cp = configparser.ConfigParser()
cp.read(confFile)

pin_OUT = cp.getint("SPEAKER","pin_OUT")
pi_IO = pigpio.pi()
pi_IO.set_mode(pin_OUT,pigpio.OUTPUT)
pi_IO.hardware_PWM(pin_OUT,0,0)

NumRealization = 10000
t_record = np.zeros(NumRealization)

# pi_IO.hardware_PWM(pin,f0,ratio)
for count in range(NumRealization):
    TS = pi_IO.get_current_tick()
    t_record[count] = TS

durations = np.diff(t_record)
plt.figure()
plt.plot(durations,'.')
plt.show()