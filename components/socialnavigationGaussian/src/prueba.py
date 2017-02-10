"""""
listas =[1,2,3,4]
listita = [[1,3],[2,5]]
listas.append(listita)

listas[4].append([8, 8])
"""

entornopunto = [1,2,3,4,5,6]

finpoly = False
while (finpoly == False):

    print("Estamos en el while")

    for e in entornopunto:
        print ("Estamos en el for")
        if (e<5):
            print ("e es menor que 5")

            break
        else:
            finpoly =True
            print("FINNNNN")

##########################################OBTENCION DE POLILINEAS#####################
###Leemos el grid y nos quedamos con los puntos negros, despues pasamos a convexhull

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

##Convexhull de cada una de las polilineas
chlist = []

for lista in totalpuntos:
    p = np.asarray(lista)
    hull = ConvexHull(p)

    chlist.append(np.asarray(hull.points))

polylines = []

for p in chlist:
    polyline = []
    for pnt in p:
        punto = SNGPoint2D()
        punto.x = pnt[0]
        punto.z = pnt[1]
        polyline.append(punto)
    polylines.append(polyline)

if (dibujar):
    for ps in polylines:
        plt.figure()
        for p in ps:
            plt.plot(p.x, p.z, "*r-")
            plt.axis('equal')
            plt.xlabel('X')
            plt.ylabel('Y')
        plt.show()


        ####INTENTO DE LEER POLILINEA EN ORDEN


        totalpuntos = []
        matrizbool = np.ones([grid.shape[0], grid.shape[1]], dtype=bool)


        for j in range(grid.shape[1]):
            for i in range(grid.shape[0]):
                if (matrizbool[i,j] == True):
                    if (grid[j, i] > 0 ):
                        print("Punto negro", [i, j])
                        cp = [i, j]
                        print("Creamos nueva lista de puntos")
                        puntos = []
                        finpoly = False

                        while (finpoly == False):
                            #print("Anadimos el punto a la lista")

                            puntos.append(cp)
                            entorno = []
                            ##########CAMBIAR EL ENTORNO PARA QUE VAYA EN ORDEN
                            for ix in range(-1, 2):
                                for iy in range(-1, 2):
                                    entorno.append([cp[0] + ix, cp[1] + iy])
                            index = entorno.index(cp)
                            entorno.pop(index)

                            #print ("punto",cp,"entorno",entorno)

                            boolentorno =[]
                            ###Comprobar que asi el punto no esta en el entorno!!!!!!!!!!!!!!
                            for e in entorno:
                                for ix in range(-1, 2,2):
                                    for iy in range(-1, 2,2):
                                        boolentorno.append(matrizbool[[e[0] + ix, e[1] + iy]])

                                if np.mean(boolentorno != 0):
                                    #  QUIERE DECIR QUE ALGUN PUNTO DEL ENTORNO NO SE HA COMPROBADO AUN
                                    if (matrizbool[e[0], e[1]]==True):
                                        if (grid[e[1], e[0]] > 0):
                                            print("Hay un punto negro en el entorno", e)
                                            matrizbool[e[0], e[1]] = False
                                            cp = e
                                            break
                                        else:
                                            matrizbool[[e[0], e[1]]]=False


                                else:
                                    finpoly = True

                        totalpuntos.append(puntos)


                    matrizbool[i,j]= False
