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


data_to_check = initiator.fulldata[16]

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
    

func.errorStat(initiator.Ranging_Record, GT = 0.715)