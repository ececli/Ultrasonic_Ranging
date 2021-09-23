from twoWayRangingClass_v1 import TWR
import time
import twoWayRangingLib_v2 as func

responder = TWR('responder','UR_pyConfig_v2.conf')
responder.initialize()
startTime = time.time()
responder.start()
duration = time.time() - startTime
print(duration)

data_to_check = responder.fulldata[16]

func.checkFFT(data_to_check,responder.sos,responder.RATE)

func.checkBPFilteredData(data_to_check,responder.sos)

func.getOutputFig_IQMethod2(data_to_check,
                            responder.RefSignal,
                            responder.RefSignal2,
                            responder.THRESHOLD,
                            responder.peak_interval,
                            responder.peak_width,
                            recordedPeak=0,
                            preBPFilter = True,
                            sos = responder.sos)