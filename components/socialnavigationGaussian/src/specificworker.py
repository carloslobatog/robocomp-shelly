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
import pickle

import matplotlib.patches as patches
from PySide import *
from genericworker import *
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from math import *
from mpl_toolkits.mplot3d import axes3d
from scipy.spatial import ConvexHull
from normal import Normal
import GaussianMix as GM
import checkboundaries as ck
from PySide.QtCore import QRect, QRectF, Qt, QSize, QSizeF, QPointF
from PySide.QtGui import QTransform, QPainter, QPolygonF


import math


def getPolyline(grid, resolution, lx_inf, ly_inf):

    totalpuntos = []
    for j in range(grid.shape[1]):
        for i in range(grid.shape[0]):
            if grid[j, i] > 0:
                mismocluster, pos = ck.checkboundaries(grid, i, j, totalpuntos)
                if (mismocluster == True):
                    totalpuntos[pos].append([i, j])
                else:
                    puntos = []
                    puntos.append([i, j])
                    totalpuntos.append(puntos)

    ret = []
    for lista in totalpuntos:
        #los puntos en el grid sufren una traslacion, con esto los devolvemos a su posicion original
        for puntos in lista:
            puntos[0] = puntos[0] * resolution + lx_inf
            puntos[1] = puntos[1] * resolution + ly_inf


        points = np.asarray(lista)
        hull = ConvexHull(points)
        ret.append(points[hull.vertices])
        """""
        # interpolar los puntos
        threshold = 0.1
        #print '-----------------', threshold
        v = []
        prev = points[hull.vertices][-1]
        for curr in points[hull.vertices]:
            dx = curr[0] - prev[0]
            dy = curr[1] - prev[1]
            dist = math.sqrt(dx * dx + dy * dy)
         #   print prev, curr, dist

            if dist > threshold:
                iters = dist / threshold
          #      print 'dentro', iters, 'dx', dx, 'dy', dy
                for iter in xrange(int(iters)):
                    wx = prev[0] + iter * dx / iters
                    wy = prev[1] + iter * dy / iters
           #         print '  ', wx, wy
                    #dx2 = wx - curr[0]
                    #dy2 = wy - curr[1]
                    #dist2 = math.sqrt(dx2 * dx2 + dy2 * dy2)

                    #if dist2 > threshold + 0.001:
                    #v.append([wx, wy])

                   # else:
                    #    break

                    v.append([wx, wy])
            v.append(curr)
            prev = curr

        ret.append(v)
        """

    return ret


class Person(object):
    x = 0
    y = 0
    th = 0
<<<<<<< HEAD
    polyline = []
    xdot = 0
    ydot = 0
    vel=0
=======
    vel = 0

>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033

<<<<<<< HEAD
    _radius = 0.30

<<<<<<< HEAD
=======
    _radius = 0.35
>>>>>>> parent of 22b88f9... changes in trajectory de mierda
=======
>>>>>>> 28535e950b9c0a725a163618df62901f2683c6eb
    """ Public Methods """

<<<<<<< HEAD
    def __init__(self, x=0, y=0, th=0,vel=0):
=======
    def __init__(self, x=0, y=0, th=0, vel=0):
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033
        self.x = x
        self.y = y
        self.th = th
        self.vel = vel


    def draw(self, v,sigma_h,sigma_r,sigma_s,rot, drawPersonalSpace=False):
        #numero de curvas de contorno
        nc = 10
        if v <= 20:
            aprox = 0
        else:
            if v == 100:
                aprox = nc - 2
            else:
                aprox = int(v/10) - 1


            #numero de curvas de contorno que se van a dibujar

            # define grid.
        npts = 50
        x = np.linspace(self.x - 4, self.x + 4, npts)
        y = np.linspace(self.y - 4, self.y + 4, npts)

        X, Y = np.meshgrid(x, y)
        # plt.plot(X, Y, '*')

        Z = self._calculatePersonalSpace(X, Y,sigma_h,sigma_r,sigma_s,rot)

        if (drawPersonalSpace):
            # print(Z)
            # http://www.python-course.eu/matplotlib_contour_plot.php
            # https://es.mathworks.com/matlabcentral/answers/230934-how-to-extract-x-and-y-position-of-contour-line
            #surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)

            ##PROBLEMICA -> a partir de nc> 4 dibuja una linea de menos. Por eso en el caso de que v==100 he puesto que aprox=nc-2

            CS = plt.contour(X, Y, Z, nc)


            dat0 = CS.allsegs[aprox][0]

            #print(dat0)

            ##Dibujar la polilinea
           # plt.plot(dat0[:, 0], dat0[:, 1], '*b-')


            # CS = plt.contour(X, Y, Z, 10)
            #surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)

            # Corpo
            body = plt.Circle((self.x, self.y), radius=self._radius, fill=False)
            plt.gca().add_patch(body)

            # Orientacion
           # print("alpha:")
            x_aux = self.x + self._radius * cos(pi/2 - self.th);
            #print(self.x)
            #print(x_aux);
            y_aux = self.y + self._radius * sin(pi/2 - self.th);
            #print(self.y)

            #print(y_aux)
            heading = plt.Line2D((self.x, x_aux), (self.y, y_aux), lw=1, color='k')
            plt.gca().add_line(heading)

            plt.axis('equal')


        return Z



    """ Private Methods """

    def _calculatePersonalSpace(self, x, y,sigma_h,sigma_r,sigma_s,rot):
        """"",sigma_h
        sigma_h = 2.0
        sigma_r = 1.0
        sigma_s = 4/3
        """
        ##he cambiado el valor de las sigmas porque la gaussiana que dibujaba con las anteriores era muy grande

<<<<<<< HEAD



=======
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033
        alpha = np.arctan2(y - self.y, x - self.x) - rot - pi / 2
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


class Object():
    def __init__(self, x=0, y=0, th=0, sp=0):
        self.x = x
        self.y = y
        self.th = th
        self.sp = sp


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        self.timer.start(self.Period)
       # plt.ion()

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
       # print('SpecificWorker.compute...')
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
    def getPersonalSpace(self, persons, v, dibujar):

        plt.close('all')
       ##DESCOMENTAR EL FIGUREEE
       # plt.figure()

        #ax = fig.add_subplot(111, projection='3d')

        #fig, ax = plt.subplots()
        #ax.grid(True)
      #  x = y = np.arange(-3.0, 3.0, 0.05)
      #  X, Y = np.meshgrid(x, y)


        ##Limites de la representacion

        lx_inf = -6
        lx_sup = 8
        ly_inf = -6
        ly_sup = 8
        """""
        ##cambio los limites para los otros valores de sigma
        lx_inf = 0
        lx_sup = 10
        ly_inf = 0
        ly_sup = 10
        """""
        # zs = np.array([fun(x,y) for x,y in zip(np.ravel(X), np.ravel(Y))])
        # Z = zs.reshape(X.shape)



        ##########################################CLUSTERING##################################################

        normals = []

        for p in persons:
            pn = Person(p.x, p.z, p.angle)
            #print('Pose x', pn.x, 'Pose z', pn.y, 'Rotacion', pn.th)
            pn.draw(v,4,2,2*4/3,pi/2 - pn.th, drawPersonalSpace=dibujar)
            #normals.append(Normal(mu=[[pn.x], [pn.y]], sigma=[-pn.th - pi/2, 2.0, 2.0, 2.0], elliptical=True))
            normals.append(Normal(mu=[[pn.x], [pn.y]], sigma=[-pn.th - pi/2, 4, 2, 2*4/3], elliptical=True))
        #print ("numero de gaussianas",len(normals))

        h = 0.9
        resolution = 0.1
        limits = [[lx_inf, lx_sup], [ly_inf, ly_sup]]
        _, z = Normal.makeGrid(normals, h, 2, limits=limits, resolution=resolution)
        grid = GM.filterEdges(z, h)


        #plt.figure()
        #plt.imshow(z, shape=grid.shape, interpolation='none', aspect='equal', origin='lower', cmap='Greys', vmin=0, vmax=2)

        #plt.figure()
        #plt.imshow(grid, shape=grid.shape, interpolation='none', aspect='equal', origin='lower', cmap='Greys', vmin=0, vmax=2)

        if (dibujar):
           # plt.figure()
            plt.imshow(grid, extent=[lx_inf, lx_sup, ly_inf, ly_sup], shape=grid.shape, interpolation='none', aspect='equal', origin='lower', cmap='Greys', vmin=0, vmax=2)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.axis('equal')

        np.savetxt('log.txt', grid, fmt='%i')

        ###########################LEO EL GRID Y SEPARO LAS POLILINEAS, DESPUES SE HACE CONVEXHULL####################################
        ##el convex hull se hace para obtener los puntos de la polilinea ordenados ya que los necesitamos asi para el laser

        polylines = []
        totalpuntosorden = getPolyline(grid, resolution, lx_inf, ly_inf)

        for pol in totalpuntosorden:
            polyline = []
            for pnt in pol:
                punto = SNGPoint2D()
                punto.x = pnt[0]
                punto.z = pnt[1]
                polyline.append(punto)
            polylines.append(polyline)

        """""
        if (dibujar):
            for ps in polylines:
              #  plt.figure()
                for p in ps:
                    plt.plot(p.x, p.z, "*r-")
                    plt.axis('equal')
                    plt.xlabel('X')
                    plt.ylabel('Y')
            plt.show()

        """
        plt.show()

        return polylines

    def getPassOnRight(self, persons, v, dibujar):

<<<<<<< HEAD
        plt.close('all')
=======
        plt.close("all")
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033


        lx_inf = -6
        lx_sup = 8
        ly_inf = -6
        ly_sup = 8

        normals = []

        for p in persons:
            pn = Person(p.x, p.z, p.angle, p.vel)

            pn.draw(v,(50/((7*pn.vel/50)+43)*4), (50/((7*pn.vel/50)+43)*4)/2, 2*(50/((7*pn.vel/50)+43)*4)/3,pi/2-pn.th, drawPersonalSpace=dibujar)
            pn.draw(v,4, 1.5, 10/3, pi - pn.th, drawPersonalSpace=dibujar)

            normals.append(Normal(mu=[[pn.x], [pn.y]], sigma=[-pn.th - pi/2, (50/((7*pn.vel/50)+43)*4), (50/((7*pn.vel/50)+43)*4)/2, 2*(50/((7*pn.vel/50)+43)*4)/3], elliptical=True))
            normals.append(Normal(mu=[[pn.x], [pn.y]], sigma=[-pn.th , 4, 1.5, 10/3], elliptical=True))


        h = 0.4
        resolution = 0.1
        limits = [[lx_inf, lx_sup], [ly_inf, ly_sup]]
        _, z = Normal.makeGrid(normals, h, 2, limits=limits, resolution=resolution)
        grid = GM.filterEdges(z, h)

        if (dibujar):
            plt.figure()
            plt.imshow(grid, extent=[lx_inf, lx_sup, ly_inf, ly_sup], shape=grid.shape, interpolation='none', aspect='equal', origin='lower', cmap='Greys', vmin=0, vmax=2)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.axis('equal')

        np.savetxt('log.txt', grid, fmt='%i')

        polylines = []
        totalpuntosorden = getPolyline(grid, resolution, lx_inf, ly_inf)

        for pol in totalpuntosorden:
            polyline = []
            for pnt in pol:
                punto = SNGPoint2D()
                punto.x = pnt[0]
                punto.z = pnt[1]
                polyline.append(punto)
            polylines.append(polyline)
        plt.show()
        return polylines



    #
    # getObjectInteraction
    #
    def getObjectInteraction(self, persons, objects, d):
<<<<<<< HEAD
=======

>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033
        print("getObjectInteration")
        plt.close('all')

        polylines = []
<<<<<<< HEAD
        for o in objects:
            print ("OBJETO")

            obj = Person(o.x, o.z, o.angle)

=======

        for o in objects:
            obj = Object(o.x, o.z, o.angle, o.space)
            print("OBJETO")
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033
            ##para dibujarlo
            if (d):
                plt.figure('ObjectSpace')
                rect = plt.Rectangle((obj.x-0.25,obj.y-0.25),0.5,0.5,fill=False)

                plt.gca().add_patch(rect)
                x_aux = obj.x + 0.25 * cos(pi / 2 - obj.th);
                y_aux = obj.y + 0.25 * sin(pi / 2 - obj.th);
                heading = plt.Line2D((obj.x, x_aux), (obj.y, y_aux), lw=1, color='k')
                plt.gca().add_line(heading)

            w = 1.0
<<<<<<< HEAD
            h = 1.5
            print (obj.x,obj.y)
            ##para calcular el rectangulo
            s = QRectF(QPointF(0, 0), QSize(w, h))
=======

            print (obj.x,obj.y)
            ##para calcular el rectangulo
            s = QRectF(QPointF(0, 0), QSizeF(w, obj.sp))
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033

            # if (d):
            #     plt.plot (s.bottomLeft().x(),s.bottomLeft().y(),"go")
            #     plt.plot(s.bottomRight().x(), s.bottomRight().y(), "ro")
            #     plt.plot(s.topRight().x(), s.topRight().y(), "yo")
            #     plt.plot(s.topLeft().x(), s.topLeft().y(), "bo")

            space = QPolygonF()
            space.append(s.topLeft())
            space.append(s.topRight())
<<<<<<< HEAD
            space.append(QPointF(s.bottomRight().x()+ w/4, s.bottomRight().y()))
            space.append(QPointF(s.bottomLeft().x()-w/4,s.bottomLeft().y()))


            #space = QPolygonF(space.bottomLeft(),space.bottomRight(),space.topRight()+w/2,space.topLeft()+w/2)
            #space = QPolygonF(space)
             #space.moveCenter(QPointF(obj.x,obj.y))
=======
            space.append(QPointF(s.bottomRight().x()+ obj.sp/6, s.bottomRight().y()))
            space.append(QPointF(s.bottomLeft().x()-obj.sp/6,s.bottomLeft().y()))
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033


            t = QTransform()
            t.translate(-w/2, 0)
            space = t.map(space)
            t = QTransform()
            t.rotateRadians(-obj.th)
            space = t.map(space)

            t = QTransform()
            t.translate(obj.x,obj.y)
            space = t.map(space)

            # points = []
            # for x in xrange(space.count()-1):
            #     point = space.value(x)
            #     print ("valor", point)
            #     points.append([point.x(),point.y()])
            #     plt.plot(point.x(),point.y(),"go")


            polyline = []

            for x in xrange(space.count() ):
                point = space.value(x)
<<<<<<< HEAD
                print("valor", point)

=======
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033
                if (d):
                    plt.plot(point.x(), point.y(), "go")

                p = SNGPoint2D()
                p.x = point.x()
                p.z = point.y()
                polyline.append(p)


            for p in persons:
<<<<<<< HEAD
                print("PERSONA")
                pn = Person (p.x, p.z, p.angle)
                print ("Pose persona", pn.x, pn.y)
                if (d):
=======
                pn = Person(p.x, p.z, p.angle)
                print("PERSONA")
                if d:
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033
                    body = plt.Circle((pn.x, pn.y), radius=0.3, fill=False)
                    plt.gca().add_patch(body)

                    x_aux = pn.x + 0.30 * cos(pi / 2 - pn.th);
                    y_aux = pn.y + 0.30 * sin(pi / 2 - pn.th);
                    heading = plt.Line2D((pn.x, x_aux), (pn.y, y_aux), lw=1, color='k')
                    plt.gca().add_line(heading)
                    plt.axis('equal')

<<<<<<< HEAD
                if (space.containsPoint(QPointF(pn.x,pn.y),Qt.OddEvenFill)):
                    print("DENTROOOOO")
=======

                ##CHECKING THE ORIENTATION
                a = abs(obj.th - abs(pn.th-math.pi))
                if a < math.radians(45):
                    checkangle = True
                else:
                    checkangle = False


                # ang = abs(pn.th - math.pi)
                # print(thrinf, thrsup, ang)
                # if thrinf < ang and ang < thrsup:
                #     checkangle = True
                #     print ("MIRANDOOOOOO")

                # else:
                #     print ("NO MIRAAAAA")
                #     checkangle = False

                ##CHECKING IF THE PERSON IS INSIDE THE POLYGON
                if space.containsPoint(QPointF(pn.x,pn.y),Qt.OddEvenFill) and checkangle:
                    print("DENTROOOOO Y MIRANDO")
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033
                    polylines.append(polyline)

                    break

                else:
<<<<<<< HEAD
                    print("FUERAAAAAAA")
=======
                    print("FUERA O NO MIRANDO")
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033


        if (d):
            for ps in polylines:
                #  plt.figure()
                for p in ps:
                    plt.plot(p.x, p.z, "ro")
                    plt.axis('equal')
                    plt.xlabel('X')
                    plt.ylabel('Y')
            plt.show()
        plt.show()



        return polylines
