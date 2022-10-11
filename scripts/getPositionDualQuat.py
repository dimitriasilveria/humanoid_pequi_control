#--------------------------------------
#Método para obter a posição x y z
#de um quatérnio dual
#Parâmetros: 
#q: quatérnio dual
#Retorno:
#p: posição x y z
#--------------------------------------
from getRotationDualQuat import getRotationDualQuat
from quatMult import quatMult
from quatConj import quatConj
import numpy as np
def getPositionDualQuat(q):
     r = getRotationDualQuat(q)
     qd = np.array([q[4,0],q[5,0], q[6,0], q[7,0]]).reshape((4,1))
     p = 2*quatMult(qd,quatConj(r))
     p = np.array([p[1,0],p[2,0],p[3,0]]).reshape((3,1))  #eu não sei se tá certo
     return p