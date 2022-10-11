#--------------------------------------
#Método para calcular o produto escalar de um
#quatérnio
#Parâmetros: 
#s: escalar
#q: quatérnio
#Retorno:
#qr: quatérnio resultante
#--------------------------------------
import numpy as np 
def quatScale(s,q0):
    #q = a + bi + cj + dk
    qr = np.zeros((4,1))
    qr[0,0] = s*q0[0,0] #parte real
    qr[1,0] = s*q0[1,0] #imaginário i
    qr[2,0] = s*q0[2,0] #imaginário j
    qr[3,0] = s*q0[3,0] #imaginário k
    return qr