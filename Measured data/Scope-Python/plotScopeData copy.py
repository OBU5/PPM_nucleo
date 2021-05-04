import csv
import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib import gridspec


filePath = 'Polarization - turn off/BSS138/TimeConst-4.7ms/5ms between S3 and S2/BPfilter/'

C1_label = 'C1'
C2_label = 'C2'
C3_label = 'C3'
C4_label = 'C4'

startTime = 10
stopTime = 50

C1_corrector = 1
C2_corrector = 1
C3_corrector = 1
C4_corrector = 1.4705

time = []
C1_y = []
C2_y = []
C3_y = []
C4_y = []


zoomedTime = []
zoomedC1_y = []
zoomedC2_y = []
zoomedC3_y = []
zoomedC4_y = []

#get C1
with open(filePath + 'C1.dat', 'r') as file:
    reader = csv.reader(file, delimiter = ' ')
    for row in reader:
        time.append(float(row[0]))
        C1_y.append(float(row[1]))

#get C2
with open(filePath + 'C2.dat', 'r') as file:
    reader = csv.reader(file, delimiter = ' ')
    for row in reader:
        C2_y.append(float(row[1]))

#get C3
with open(filePath + 'C3.dat', 'r') as file:
    reader = csv.reader(file, delimiter = ' ')
    for row in reader:
        C3_y.append(float(row[1]))

#get C4
with open(filePath + 'C4.dat', 'r') as file:
    reader = csv.reader(file, delimiter = ' ')
    for row in reader:
        C4_y.append(float(row[1]))

# modify signal in order to start from 0 and to have tim units in [ms]. Also zoom signal by desired parameters
offset = 0
if(time[0] != 0):
    offset = - time[0]
    for i in range(len(time)):
        time[i] = (time[i] + offset) * 1000
       
    
for i in range(len(time)):
    if ((time[i] >= startTime) and (time[i] <= stopTime)):
            zoomedTime.append(time[i] - startTime)
            zoomedC1_y.append(C1_y[i] * C1_corrector)
            zoomedC2_y.append(C2_y[i] * C2_corrector)
            zoomedC3_y.append(C3_y[i] * C3_corrector)
            zoomedC4_y.append(C4_y[i] * C4_corrector)

fig = plt.figure(1)

gs = gridspec.GridSpec(4, 1, height_ratios=[2, 2, 1, 1]) 

#plot C1
plt.subplot(gs[0])
plt.plot(zoomedTime,zoomedC1_y, color = "tab:blue", label=C1_label)  
plt.legend(loc="upper right")
plt.grid(True)
plt.xlim(0, stopTime - startTime)

ax = plt.gca() 

#plot C2
plt.subplot(gs[1])
plt.plot(zoomedTime,zoomedC2_y, color = "tab:orange", label=C2_label)  
plt.legend(loc="upper right")
plt.grid(True)
plt.xlim(0, stopTime - startTime)
ax = plt.gca() 
ax.set_xticklabels([])  # remove x axis

#plot C3
plt.subplot(gs[2])
plt.plot(zoomedTime,zoomedC3_y, color = "tab:green", label=C3_label)  
plt.legend(loc="upper right")
plt.grid(True)
plt.xlim(0, stopTime - startTime)
plt.yticks(np.arange(0, 6, 1.0))
ax = plt.gca() 
ax.set_xticklabels([])  # remove x axis

#plot C4
plt.subplot(gs[3])
plt.plot(zoomedTime,zoomedC4_y, color = "tab:red", label=C4_label)  
plt.legend(loc="upper right")
plt.grid(True)
plt.xlabel('Čas [ms]') 
plt.xlim(0, stopTime - startTime)
plt.yticks(np.arange(0, 6, 1.0))
fig.subplots_adjust(wspace=0, hspace=0.1)

                
plt.show()