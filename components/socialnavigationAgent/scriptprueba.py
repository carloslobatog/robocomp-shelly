import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from math import *
from mpl_toolkits.mplot3d import axes3d


class Numeros:
    def __init__(self, num1, num2):
        self.num1=num1
        self.num2=num2
    def suma(self):
        print( self.num1, self.num2)
        return self.num1+self.num2
