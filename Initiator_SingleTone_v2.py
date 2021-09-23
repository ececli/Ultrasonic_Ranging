from twoWayRangingClass_v1 import TWR
# import matplotlib.pyplot as plt
import time
import twoWayRangingLib_v2 as func


initiator = TWR('initiator','UR_pyConfig_v2.conf')
initiator.initialize()
startTime = time.time()
initiator.start()
duration = time.time() - startTime
print(duration)



func.checkFFT(initiator.fulldata[0],initiator.sos,initiator.RATE)

func.checkBPFilteredData(initiator.fulldata[0],initiator.sos)

func.getOutputFig_IQMethod2(initiator.fulldata[0],
                            initiator.RefSignal,
                            initiator.RefSignal2,
                            initiator.THRESHOLD,
                            initiator.peak_interval,
                            initiator.peak_width,
                            recordedPeak=0,
                            preBPFilter = True,
                            sos = initiator.sos)
    

func.errorStat(initiator.Ranging_Record, GT = 0.53)