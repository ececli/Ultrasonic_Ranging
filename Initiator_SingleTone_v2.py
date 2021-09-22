from twoWayRangingClass_v1 import TWR
import matplotlib.pyplot as plt
import evaluationLib_v1 as evalu
import time


initiator = TWR('initiator','UR_pyConfig_v2.conf')
initiator.initialize()
startTime = time.time()
initiator.start()
duration = time.time() - startTime
print(duration)


# plt.figure()
# plt.plot(initiator.Ranging_Record,'b.')
# plt.grid()
# plt.show()

evalu.errorStat(initiator.Ranging_Record, GT = 0.53)