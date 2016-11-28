#
# Copyright (C) 2016 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, traceback, time

from PySide import *
from genericworker import *
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from math import *
from mpl_toolkits.mplot3d import axes3d


class Person(object):
    x = 0
    y = 0
    th = 0
    puntos = []
    xdot = 0
    ydot = 0

    _radius = 0.30

    """ Public Methods """

    def __init__(self, x=0, y=0, th=0):
        self.x = x
        self.y = y
        self.th = th


    def draw(self, drawPersonalSpace=False):

        if (drawPersonalSpace):
            # define grid.
            npts = 50
            x = np.linspace(self.x - 4, self.x + 4, npts)
            y = np.linspace(self.y - 4, self.y + 4, npts)

            X, Y = np.meshgrid(x, y)
            # plt.plot(X, Y, '*')

            Z = self._calculatePersonalSpace(X, Y)
            # print(Z)
            # http://www.python-course.eu/matplotlib_contour_plot.php
            # https://es.mathworks.com/matlabcentral/answers/230934-how-to-extract-x-and-y-position-of-contour-line
            CS = plt.contour(X, Y, Z, 10)
            dat0 = CS.allsegs[5][0]
            self.puntos = dat0
           # print(dat0)
            plt.plot(dat0[:, 0], dat0[:, 1],'*')



            #CS = plt.contour(X, Y, Z, 10)
            # surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)

        # Corpo
        body = plt.Circle((self.x, self.y), radius=self._radius, fill=False)
        # plt.gca().add_patch(body)
        x_aux = self.x + self._radius * cos(self.th)
        y_aux = self.y + self._radius * sin(self.th)
        heading = plt.Line2D((self.x, x_aux), (self.y, y_aux), lw=3, color='k')
        # plt.gca().add_line(heading)

        # plt.axis('equal')
        # plt.show()

    """ Private Methods """

    def _calculatePersonalSpace(self, x, y):

        sigma_h = 2.0
        sigma_r = 1.0
        sigma_s = 4 / 3
        rot = pi/2 - self.th

        alpha = np.arctan2(y - self.y, x - self.x) - rot - pi/2
        nalpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Normalizando no intervalo [-pi, pi)

        sigma = np.copy(nalpha)
        for i in range(nalpha.shape[0]):
            for j in range(nalpha.shape[1]):
                sigma[i, j] = sigma_r if nalpha[i, j] <= 0 else sigma_h

        a = cos(rot) ** 2 / 2 * sigma ** 2 + sin(rot) ** 2 / 2 * sigma_s ** 2
        b = sin(2 * rot) / 4 * sigma ** 2 - sin(2 * rot) / 4 * sigma_s ** 2
        c = sin(rot) ** 2 / 2 * sigma ** 2 + cos(rot) ** 2 / 2 * sigma_s ** 2

        z = np.exp(-(a * (x - self.x) ** 2 + 2 * b * (x - self.x) * (y - self.y) + c * (y - self.y) ** 2))

        return z


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        self.timer.start(self.Period)

    def setParams(self, params):
        # try:
        #	par = params["InnerModelPath"]
        #	innermodel_path=par.value
        #	innermodel = InnerModel(innermodel_path)
        # except:
        #	traceback.print_exc()
        #	print "Error reading config params"
        return True

    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')
        # computeCODE
        # try:
        #	self.differentialrobot_proxy.setSpeedBase(100, 0)
        # except Ice.Exception, e:
        #	traceback.print_exc()
        #	print e
        return True

    #
    # getPolyline
    #
    def getPolylines (self, persons, v, d):
        polylines = []

        plt.close('all')

        #fig = plt.figure()
       # ax = fig.add_subplot(111, projection='3d')

        fig, ax = plt.subplots()
        ax.grid(True)

        # x = y = np.arange(-3.0, 3.0, 0.05)
        # X, Y = np.meshgrid(x, y)
        # zs = np.array([fun(x,y) for x,y in zip(np.ravel(X), np.ravel(Y))])
        # Z = zs.reshape(X.shape)

        p1 = Person(persons[0].x, persons[0].z, persons[0].angle)
        print('Pose x', p1.x, 'Pose z', p1.y, 'Rotacion', p1.th)

        p2 = Person(persons[1].x, persons[1].z, persons[1].angle)
        print('Pose x', p2.x, 'Pose z', p2.y, 'Rotacion', p2.th)

        p1.draw(drawPersonalSpace=True)
        p2.draw(drawPersonalSpace=True)

        polyline1 = []
        for punto in p1.puntos:
            point = SNGPoint2D()
            point.x = punto[0]
            point.z = punto[1]
            polyline1.append(point)
        polylines.append(polyline1)

        polyline2 = []
        for punto in p2.puntos:
            point = SNGPoint2D()
            point.x = punto[0]
            point.z = punto[1]
            polyline2.append(point)
        polylines.append(polyline2)
        
        plt.xlabel('X')
        plt.ylabel('Y')

        plt.xlim(-6, 6)
        plt.ylim(-6, 6)

        # Z = z1 + z2

        plt.axis('equal')
        plt.show()

        return polylines
