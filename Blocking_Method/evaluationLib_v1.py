import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq



def errorStat(data, GT, offset=0, bin=100):
    fixedData = data - offset
    meanData = np.mean(fixedData)
    stdData = np.std(fixedData)
    
    Error = fixedData - GT
    meanAbsError = np.mean(np.abs(Error))
    meanError = np.mean(Error)
    
    sortedMAE = np.sort(np.abs(Error))
    RE95 = sortedMAE[int(len(fixedData)*0.95)-1]
    
    
    print("--------------------------------")
    print("Number of Valid Data is ", len(fixedData))
    print("Mean Distance = %.1f cm" % np.multiply(meanData,100))
    print("Std of Distance = %.1f cm" % np.multiply(stdData,100))
    print("Mean Error = %.1f cm" % np.multiply(meanError,100))
    print("Mean Absolute Error = %.1f cm" % np.multiply(meanAbsError,100))
    print("RE95 = %.1f cm" % np.multiply(RE95,100))
    print("--------------------------------")
    plt.figure()
    plt.plot(fixedData,'b.')
    plt.xlabel('Index of Samples')
    plt.ylabel("Distance (m)")
    plt.grid()
    plt.show()
    
    count, bins_count = np.histogram(np.abs(Error), bins=bin)
    pdf = count / np.sum(count)
    cdf = np.cumsum(pdf)
    plt.figure()
    axes = plt.gca()
    axes.set_ylim([0,1])
    plt.plot(bins_count[1:], cdf)
    plt.axhline(y=0.95,color="r")
    plt.axvline(x=RE95,color="r")
    plt.grid()
    plt.xlabel("Absolute Error (m)")
    plt.ylabel("Probability")
    plt.show()
    return meanError,meanAbsError