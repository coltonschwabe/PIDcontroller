import numpy as np
import matplotlib.pyplot as plt

# reads data from txt file
#data : time,Temp,DAC,current
contents1 = []
contents2 = []

file1 = "data_1747755577.txt" #start at 87.7
file2 = "data_1747754741.txt" #start at 

with open(file1, "rt") as cont:
    for line in cont:
       element = line.split(",") # splits up into values
       element[3] = element[3][:-2] # cuts off the '\n' from last value
       contents1.append(element)

with open(file2, "rt") as cont:
    for line in cont:
       element = line.split(",") # splits up into values
       element[3] = element[3][:-2] # cuts off the '\n' from last value
       contents2.append(element)

start1 = float(contents1[0][0]) # staring time
datapointsx1 = []
datapointsy1 = []

start2 = float(contents2[0][0]) # staring time
datapointsx2 = []
datapointsy2 = []

for i in range(0, len(contents1)):
    datapointsx1.append(float(contents1[i][0])-start1)
    datapointsy1.append((float(contents1[i][1])))

for i in range(0, len(contents1)):
    datapointsx2.append(float(contents2[i][0])-start2)
    datapointsy2.append((float(contents2[i][1])))

plt.plot(datapointsx1, datapointsy1, label = "Old Algorithm")
plt.plot(datapointsx2, datapointsy2, label = "New Algorithm")
plt.grid(True, color='gray', linewidth=0.5, linestyle='--')
plt.xlabel("Time (s)")
plt.ylabel("Temperature (C)")
plt.legend()
plt.show()
