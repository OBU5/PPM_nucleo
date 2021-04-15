import csv
import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib import gridspec
import math



filePath = 'Multisim/Filter/'

C1_label = 'StageA'
C2_label = 'StageB'
C3_label = 'StageC'
C4_label = 'StageD'

startTime = 0
stopTime = 10000000

C1_corrector = 1
C2_corrector = 1
C3_corrector = 1
C4_corrector = 1

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

       
    
for i in range(len(time)):
    if ((time[i] >= startTime) and (time[i] <= stopTime)):
            zoomedTime.append(time[i])
            zoomedC1_y.append(20*math.log((C1_y[i] * C1_corrector),10))
            zoomedC2_y.append(20*math.log((C2_y[i] * C2_corrector),10))
            zoomedC3_y.append(20*math.log((C3_y[i] * C3_corrector),10))
            zoomedC4_y.append(20*math.log((C4_y[i] * C4_corrector),10))

fig = plt.figure(1)


#plot C1
plt.plot(zoomedTime,zoomedC1_y,linewidth=2, color = "tab:orange", label=C1_label)  
plt.legend(loc="upper right")

#plot C2
plt.plot(zoomedTime,zoomedC2_y,linewidth=2, color = "tab:red", label=C2_label)  
plt.legend(loc="upper right")

#plot C3
plt.plot(zoomedTime,zoomedC3_y, linewidth=2, color = "tab:green", label=C3_label)  
plt.legend(loc="upper right")

#plot C4
plt.plot(zoomedTime,zoomedC4_y, linewidth=2, color = "tab:blue", label=C4_label)  
plt.legend(loc="upper right")
plt.grid(True)
plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
plt.xlabel('Frekvence [Hz]') 
plt.ylabel('Zisk [dB]') 
plt.xscale('log')
plt.xlim(100, 20000)
plt.ylim(-20, 60)



                
plt.show()