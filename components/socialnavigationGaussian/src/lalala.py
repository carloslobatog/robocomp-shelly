import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import checkboundaries as ck


grid = np.loadtxt('log.txt')

resolution = 0.1
##Limites de la representacion
lx_inf = -6
lx_sup = 8
ly_inf = -6
ly_sup = 8


def getPolyline(grid, resolution, lx_inf, lx_sup, ly_inf, ly_sup):
    ret = []
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
    for lista in totalpuntos:
        for puntos in lista:
            puntos[0] = puntos[0] * resolution + lx_inf
            puntos[1] = puntos[1] * resolution + ly_inf
        points = np.asarray(lista)
        hull = ConvexHull(points)
        ret.append((points[hull.vertices, 0], points[hull.vertices, 1]))
    return ret

print getPolyline(grid, resolution, lx_inf, lx_sup, ly_inf, ly_sup)

