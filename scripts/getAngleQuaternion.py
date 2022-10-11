#------------------------------------------
#Método para retornar o valor do angulo de rotação de
#um quatérnio
#------------------------------------------
import numpy as np 
import math as mt
def getAngleQuaternion(q):
    theta = mt.acos(q[0,0])
    n1 = np.zeros((3,1))
    n2 = np.zeros((3,1))
  
    for i in range(1,4,1):
        n1[i,0] = q[i,0]*(1/mt.sin(theta))
        n2[i,0] = q[i,0]*(1/mt.sin(-theta))
    
    q1 = np.array([[q[0,0]]])
    q1 = np.concatenate((q1,n1),axis=0)
    #q1 = [q[0,0];n1]  
    if np.linalg.norm(q-q1) != 0:
        theta = -theta
    
    return theta