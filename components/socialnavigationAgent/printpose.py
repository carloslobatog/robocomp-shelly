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
		x = parts[0]
		y = parts [1]
		ax.append(x)
		ay.append(y)
plt.plot(ax, ay, 'ro')

plt.show()
