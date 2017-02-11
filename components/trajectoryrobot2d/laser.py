import sys
from math import sin, cos, pi
import numpy as np
import matplotlib.pyplot as plt

with open(sys.argv[1], 'r') as f:
	ax = []
	ay = []
	lines = f.readlines()
	for l in lines:
		parts = l.split()
		angle = (pi/2.)-float(parts[0])
		dist  = float(parts[1])
		x = dist*cos(angle)
		y = dist*sin(angle)
		ax.append(x)
		ay.append(y)
plt.plot(ax, ay, 'ro')

with open(sys.argv[2], 'r') as f:
	bx = []
	by = []
	lines = f.readlines()
	for l in lines:
		parts = l.split()
		angle = (pi/2.)-float(parts[0])
		dist  = float(parts[1])
		x = dist*cos(angle)
		y = dist*sin(angle)
		bx.append(x)
		by.append(y)
plt.plot(bx, by, 'go')

with open(sys.argv[3], 'r') as f:
	px = []
	py = []
	lines = f.readlines()
	for l in lines:
		parts = l.split()
		x = float(parts[0])
		y = float(parts[1])
		px.append(x)
		py.append(y)
plt.plot(px, py, '*k')

plt.show()
