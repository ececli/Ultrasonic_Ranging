import numpy as np
import matplotlib.pyplot as plt


Ranging_Record1= np.loadtxt('Ranging_LPF.csv', delimiter=',')
Ranging_Record2= np.loadtxt('Ranging_sincos.csv', delimiter=',')

plt.figure()
plt.hist(Ranging_Record1,bins=100,label='LPF')
plt.hist(Ranging_Record2,bins=100,label='sin-cos')
plt.legend()
plt.show()
