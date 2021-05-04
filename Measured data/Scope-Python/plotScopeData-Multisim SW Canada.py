import csv
import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib import gridspec
from matplotlib.ticker import ScalarFormatter


filePath = 'Multisim/SwitchingCircuit-Canada/'

C1_label = 'Gate1, Gate4'
C2_label = 'Gate5'
C3_label = 'Sensor_A'
C4_label = 'Sensor_B'
C5_label = 'Proud'

startTime = 4.009920435
stopTime = 4.010361508

C1_corrector = 1
C2_corrector = 1
C3_corrector = 1
C4_corrector = 1
C5_corrector = 1000

time = []
time5 = []
zoomedTime5= []
C1_y = []
C2_y = []
C3_y = []
C4_y = []
C5_y = []


zoomedTime = []
zoomedC1_y = []
zoomedC2_y = []
zoomedC3_y = []
zoomedC4_y = []
zoomedC5_y = []

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
#get C4
with open(filePath + 'C5.dat', 'r') as file:
    reader = csv.reader(file, delimiter = ' ')
    for row in reader:
        C5_y.append(float(row[1]))        
        time5.append(float(row[0]))



# modify signal in order to start from 0 and to have tim units in [ms]. Also zoom signal by desired parameters

       
    
for i in range(len(time)):
    if ((time[i] >= startTime) and (time[i] <= stopTime)):
            zoomedTime.append(time[i])
            zoomedC1_y.append(C1_y[i] * C1_corrector)
            zoomedC2_y.append(C2_y[i] * C2_corrector)
            zoomedC3_y.append(C3_y[i] * C3_corrector)
            zoomedC4_y.append(C4_y[i] * C4_corrector)

            
for i in range(len(time5)):
    if ((time5[i] >= startTime) and (time5[i] <= stopTime)):
            zoomedTime5.append(time5[i])
            zoomedC5_y.append(C5_y[i] * C5_corrector)



fig = plt.figure(1)

gs = gridspec.GridSpec(5, 1, height_ratios=[2, 2, 2, 1, 1]) 

#plot C1
plt.subplot(gs[3])
plt.plot(zoomedTime,zoomedC1_y,linewidth=2, color = "tab:green", label=C1_label)  
plt.legend(loc="upper right")
plt.grid(True)
plt.xlim(startTime, stopTime)
plt.yticks(np.arange(0, 6, 2.5))
plt.ylabel('Napětí [V]') 
ax = plt.gca() 
ax.set_xticklabels([])  # remove x axis

#plot C1
plt.subplot(gs[4])
plt.plot(zoomedTime,zoomedC2_y,linewidth=2, color = "tab:green", label=C2_label)  
plt.legend(loc="upper right")
plt.grid(True)
plt.xlim(startTime, stopTime)
plt.ylabel('Napětí [V]') 
plt.xlabel('Čas [s]') 
fig.subplots_adjust(wspace=0, hspace=0.1)
x_formatter = ScalarFormatter(useOffset=False)
plt.yticks(np.arange(0, 6, 2.5))
ax = plt.gca() 
ax.xaxis.set_major_formatter(x_formatter)

#plot C2
plt.subplot(gs[0])
plt.plot(zoomedTime,zoomedC3_y,linewidth=2, color = "tab:blue", label=C3_label)  
plt.legend(loc="upper right")
plt.grid(True)
plt.yticks(np.arange(0, 13, 2.0))
plt.ylabel('Napětí [V]') 
plt.xlim(startTime, stopTime)
ax = plt.gca() 
ax.set_xticklabels([])  # remove x axis


#plot C4
plt.subplot(gs[1])
plt.plot(zoomedTime,zoomedC4_y, linewidth=2, color = "tab:orange", label=C4_label)  
plt.legend(loc="upper right")
plt.grid(True)
plt.xlim(startTime, stopTime)
plt.yticks(np.arange(0, 250, 50))
plt.ylabel('Napětí [V]') 
ax = plt.gca() 
ax.set_xticklabels([])  # remove x axis

#plot C2
plt.subplot(gs[2])
plt.plot(zoomedTime5,zoomedC5_y,linewidth=2, color = "tab:red", label=C5_label)  
plt.legend(loc="upper right")
plt.grid(True)
plt.ylabel('Proud [mA]') 
plt.xlim(startTime, stopTime)
ax = plt.gca()
plt.yticks(np.arange(0, 600, 100))

ax.set_xticklabels([])  # remove x axis

                
plt.show()