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
