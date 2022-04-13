import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq, rfft, rfftfreq
import glob

RATE = 64000
numBins = 100
filename = 'Fulldata_2022041*.dat'
a = {}
fn = {}
for file in glob.glob(filename):

    fulldata = np.loadtxt(file, delimiter = ",")
    print(file, len(fulldata))

    DCoffset = np.mean(fulldata)
    normalAllData = fulldata-DCoffset


    yf = rfft(normalAllData)
    xf = rfftfreq(len(normalAllData), 1/RATE)


    
    lenEvaData = int(len(normalAllData)/numBins)
    xf_eva = rfftfreq(lenEvaData, 1/RATE)
    k=0
    max_yf = []
    cor_xf = []
    while True:
        yf_eva = rfft(normalAllData[k:k+lenEvaData])
        k=k+lenEvaData
        ymax = np.max(np.abs(yf_eva[xf_eva > 19000]))
        corx = xf_eva[np.abs(yf_eva) == ymax]
        if len(corx)>1:
            print(k,corx)
        max_yf.append(ymax)
        cor_xf.append(corx[0])
        # max_yf.append(np.max(np.abs(yf_eva)))
        # cor_xf.append(xf_eva[np.argmax(np.abs(yf_eva))])
        
        if k >= len(normalAllData):
            break

    print("Mean = ", np.mean(max_yf))
    print("Std = ", np.std(max_yf))
    print("Corresponding frequencies are ", np.unique(cor_xf))
    
    if np.unique(cor_xf)[0] in a.keys():
        # a[np.unique(cor_xf)[0]].append(max_yf)
        # a[np.unique(cor_xf)[0]].append(np.asarray(max_yf))
        a[np.unique(cor_xf)[0]] = np.concatenate((a[np.unique(cor_xf)[0]],np.asarray(max_yf)))
        fn[np.unique(cor_xf)[0]].append(int(file[15:17]+file[18:22]))
    else:
        # a[np.unique(cor_xf)[0]] = max_yf
        a[np.unique(cor_xf)[0]] = np.asarray(max_yf)
        fn[np.unique(cor_xf)[0]] = [int(file[15:17]+file[18:22])]
    
    
bb = sorted(a.keys())
plt.figure()
for b in bb:
    plt.plot(b,np.mean(a[b]),'.')
plt.show()
X = []
Y = []
ERR = []
for b in bb:
    X.append(b)
    Y.append(np.mean(a[b]))
    ERR.append(np.std(a[b]))
plt.figure()
plt.plot(X,Y)
plt.errorbar(X,Y, yerr = ERR, fmt = 'o', color = 'r')
plt.show()

# just 28000Hz
freq = 27998.0
c = a[freq]
recTime = np.sort(fn[freq])
arg_recTime = np.argsort(fn[freq])

plt.figure()
counter = 0
for k in arg_recTime:
    plt.plot(np.arange(counter*100,counter*100+100),c[k*100:(k+1)*100],'.')
    counter = counter + 1
plt.xlabel('Index')
plt.ylabel('Magnitude')
plt.grid()
plt.show()

################################################################################

# compare data with different days
day1 = {}
day2 = {}
for b in bb:
    k = 0
    day1_empty = True
    day2_empty = True
    while True:
        if fn[b][k]<130000:
            if day1_empty:
                day1[b] = a[b][k*100:k*100+100]
                print('day1 ',b,fn[b][k])
                day1_empty = False
        else:
            if day2_empty:
                day2[b] = a[b][k*100:k*100+100]
                day2_empty = False
                print('day2 ',b,fn[b][k])
        k = k + 1
        if (not day1_empty) and (not day2_empty):
            break
                
            
X = []
Y1 = []
ERR1 = []
Y2 = []
ERR2 = []
for b in bb:
    X.append(b)
    Y1.append(np.mean(day1[b]))
    ERR1.append(np.std(day1[b]))
    Y2.append(np.mean(day2[b]))
    ERR2.append(np.std(day2[b]))
plt.figure()
plt.plot(X,Y1,'r',label='day 1')
plt.errorbar(X,Y1, yerr = ERR1, fmt = 'o', color = 'r')
plt.plot(X,Y2,'b',label='day 2')
plt.errorbar(X,Y2, yerr = ERR2, fmt = 's', color = 'b')
plt.xlabel("Frequency")
plt.ylabel("Magnitude")
plt.legend()
plt.grid()
plt.show()