import sys
from math import sin, cos, pi
import numpy as np
import matplotlib.pyplot as plt
import math


with open(sys.argv[1], 'r') as r:
    axr = []
    ayr= []
    lines = r.readlines()
    for l in lines:
        parts = l.split()
        x = float(parts[0])
        y = float(parts [1])
        axr.append(x)
        ayr.append(y)
plt.plot(axr, ayr, 'r-')

with open(sys.argv[2], 'r') as f:
    person=[]
    lines = f.readlines()
    for l in lines:
        parts = l.split()
        x = float(parts[0])
        y = float(parts [1])

        person.append([x,y])

dist=[10000,10000,10000,10000,10000,10000]
for j, p in enumerate(person):

    for i in range (len(axr)):

        d = math.sqrt((axr[i]-p[0])**2 + (ayr[i]-p[1])**2)
        if ((d < dist[j])):
            dist.pop(j)
            dist.insert(j,d)



print ("Distancias del recorrido a las personas", dist)


plt.xlim([0,10])
plt.ylim([0,10])
plt.axis('equal')
plt.show()
