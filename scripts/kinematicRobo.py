import numpy as np 
import math
from dualQuatMult import dualQuatMult
from dualQuatDH import dualQuatDH
from kinematicModel import KinematicModel
from globalVariables import GlobalVariables
from dualQuatConj import dualQuatConj
from quatNorm import quatNorm
from quatScale import quatScale

def kinematicRobo(theta,hOrg,hP,tipo,CoM):
    #--------------------------------------
    #Método para calcular o a cinemática direta
    #do robô utilizando quatérnio dual
    #height é a altura do robô
    #L1 é o tamanho do link da bacia
    #L2 é a altura da bacia até o primeiro motor
    #--------------------------------------
    #variaveis globais-----------------------------------------------------------------------
    glob = GlobalVariables()
    #global hpi, L1, L2, height, MDH #se eu chamar essas variáveis em outro lugar do código, ele vai pegar esses valores?
    hpi = glob.getHpi()
    L1 = glob.getL1()
    L2 = glob.getL2()
    #height = glob.getHeight()
    MDH = glob.getMDH()
    #------------------------------------------------------------------------------------------------
    # l = np.size(theta[:,0],0)
    # r = np.size(theta[:,1],0)
    
    # thetar = np.zeros((l,1))
    # thetal = np.zeros((r,1))
    thetar = theta[:,0].reshape((6,1))
    thetal = theta[:,1].reshape((6,1))
    # for i in range(r):
    #     thetar[i,0] = theta[i,0]
    # for i in range(l):
    #     thetal[i,0] = theta[i,1]
    
    # a3 = 6
    # a4 = a3
    # a5 = 2
    
    #MDH = np.array([[0,      0,   0,    np.pi/2],  # do fixo pro 0           (sobre o motthetar1)
    #               [-np.pi/2,   0,   0,   -np.pi/2],  # motthetar1 do frame 0 pro 1(sobre o motthetar2)
    #               [0,       0,  a3,       0],  # motthetar2 do frame 1 pro 2(sobre o motthetar3)
    #               [0,       0,  a4,       0],  # motthetar3 do frame 2 pro 3(sobre o motthetar4)
    #               [0,       0,   0,    np.pi/2],  # motthetar4 do frame 3 pro 4(sobre o motthetar5)
    #               [0,       0,  a5,       0]])  # motthetar5 do frame 4 pro 5(sobre o motthetar6)
   
     
    #transformações da origem para a origem
    #da configuração inicial das 2 pernas
    hCoM_O0_rightLeg = dualQuatDH(hpi,-L2,-L1,0,0) #transformação do sist de coordenadas do centro de massa para a origem 0 da perna direita
    hCoM_O0_leftLeg = dualQuatDH(hpi,-L2, L1,0,0)  #transformação do sist de coordenadas do centro de massa para a origem 0 da perna esquerda
    
    hB_O6a = dualQuatMult(hOrg,hP) #transformação para auxiliar na localização do pé em contato com o chão
    if tipo == 1: #tipo = 1 significa que a perna direita está apoiada
        hO6_O0 = KinematicModel(MDH,thetar,1,0)  #transformação do sistema de coordenadas do link O6 par ao link O0 (do início da perna para o pé)
    else:
        hO6_O0 = KinematicModel(MDH,thetal,1,0)

    
    hB_O0 = dualQuatMult(hB_O6a,hO6_O0) #representa a base ou sistema global (hOrg + hp), ou seja, do sistema base para o sistema O0
    #hB_O0 = hO6_O0
    #transformação da base global para a base O0, depois de O0 para o CoM
    if tipo == 1:
        hB_CoM = dualQuatMult(hB_O0,dualQuatConj(hCoM_O0_rightLeg))
    else:
        hB_CoM = dualQuatMult(hB_O0,dualQuatConj(hCoM_O0_leftLeg))
    
    hr = hB_CoM #a função retornará a orientação do CoM (em relação à base global)

    if CoM == 0:
        #transformação da base O0 para O6
        if tipo == 1:
            hB_O0 = dualQuatMult(hB_CoM,hCoM_O0_leftLeg) 
            hO0_O6 = KinematicModel(MDH,thetal,6,1) #transformação da base 0 até a base 6
            hr = dualQuatMult(hB_O0,hO0_O6) #posição do pé suspenso (nesse caso, o esquerdo)
        else:
            hB_O0 = dualQuatMult(hB_CoM,hCoM_O0_rightLeg)
            hO0_O6 = KinematicModel(MDH,thetar,6,1)
            hr = dualQuatMult(hB_O0,hO0_O6)


    return hr    
