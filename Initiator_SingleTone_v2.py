from twoWayRangingClass_v1 import TWR
# import matplotlib.pyplot as plt
import time
import twoWayRangingLib_v2 as func
import numpy as np


initiator = TWR('initiator','UR_pyConfig_v2.conf')
initiator.initialize()
startTime = time.time()
initiator.start()
duration = time.time() - startTime
print("Running Time is ", duration)
# initiator.recvT3T2()


a = initiator.Ranging_Record[(initiator.Ranging_Record>0) & (initiator.Ranging_Record<5)]
print(len(a),np.mean(a),np.std(a))


'''
data_to_check = initiator.fulldata[0]

func.checkFFT(data_to_check,initiator.sos,initiator.RATE)

func.checkBPFilteredData(data_to_check,initiator.sos)

func.getOutputFig_IQMethod2(data_to_check,
                            initiator.RefSignal,
                            initiator.RefSignal2,
                            initiator.THRESHOLD,
                            initiator.peak_interval,
                            initiator.peak_width,
                            recordedPeak=0,
                            preBPFilter = True,
                            sos = initiator.sos)
    

func.errorStat(initiator.Ranging_Record[(initiator.Ranging_Record>0) & (initiator.Ranging_Record<5)],
               GT = 1.48)
'''