import numpy as np
import matplotlib.pyplot as plt

# reads data from txt file
#data : time,Temp,DAC,current
contents = []

file1 = "data_1743192645.txt"
file2 = "data_1742934512.txt"

with open(file1, "rt") as cont:
    for line in cont:
       element = line.split(",") # splits up into values
       element[3] = element[3][:-2] # cuts off the '\n' from last value
       contents.append(element)

start = 0 #starting time for the timer
ambient = 0

#calculates the ambient temperature 
index = 0

while(int(contents[index][2]) == 0):
    ambient += float(contents[index][1])
    start = float(contents[index][0])
    index += 1

ambient /= index # takes average

datapointsx = []
datapointsy = []

for i in range(index, len(contents)): #loops through all the values with a nonzero DAC number
    datapointsx.append(float(contents[i][0])-start) #[time, change in temp / DAC number]
    datapointsy.append((float(contents[i][1]) - ambient)/int(contents[i][2]))

slope = []
n = 11
halfval = int((n - 1)/2) # integer division

dataysmooth = []
dataxsmooth = []

#moving average to smooth out data
for i in range(halfval, len(datapointsx) - 1 - halfval):
    sum = 0
    for j in range(i - halfval, i + halfval + 1):
        sum += datapointsy[j]
    
    dataysmooth.append(sum / n)
    dataxsmooth.append(datapointsx[i])
    
#for i in range(halfval, len(dataxsmooth) - 1 - halfval):
#    width = 1
#    m, b = np.polyfit(dataxsmooth[i - width:i + 1 + width], dataysmooth[i - width:i + 1 + width], 1)
#    slope.append(m)

for i in range(0, len(dataxsmooth) - 1):
    slope.append((dataysmooth[i+1]-dataysmooth[i])/(dataxsmooth[i+1]-dataxsmooth[i]))

min = 0
mini = 0
for i in range(0, len(slope)):
    if (slope[i] < min):
        min = slope[i]
        mini = i

mintime = dataxsmooth[mini]

Kp = dataysmooth[-1]
ssFrac = 0.63*Kp # 63% Kp
LplusT = 0.0

for i in range(0, len(dataxsmooth)):
    if dataysmooth[i] < ssFrac:
        LplusT = dataxsmooth[i]
        break

L = mintime - dataysmooth[mini] / min
T = LplusT - L

print(Kp)
print(L)
print(T)

K = 0.15 / Kp + (0.35 - L*T / ((L + T)*(L + T)))*(T / (Kp * L))
Ti = 0.35 * L + 6.7*L*T*T / (T*T + 2*L*T + 10*L*L)

print(K)
print(K / Ti)

#plt.plot(dataxsmooth[:1000], slope[:1000])
#plt.plot(dataxsmooth[:100], dataysmooth[:100])
#plt.plot([L, mintime], [0, dataysmooth[mini]])
#plt.show()