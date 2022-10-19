import numpy as np 
import math as mt 
from dualQuatMult import dualQuatMult
from kinematicRobo import kinematicRobo
from transformacao import transformacao
from globalVariables import GlobalVariables
from dualQuatConj import dualQuatConj
from dualHamiltonOp import dualHamiltonOp
from kinematicRobo import kinematicRobo
from jacobianoCoM import jacobiano2
from jacobianoPes import jacobianoPes
from kinematicModel import KinematicModel
import pandas as pd
from std_msgs.msg import Int16MultiArray
#-----------------------------------
#Método para executar o  passo com a perna esquerda como suporte da 
#caminhada e a perna direita em movimento
#-----------------------------------
def fase2(ha,ha2,trajCoM,ind,trajPA,theta,vecGanho,pub):
    angles = Int16MultiArray()
    # print('aqui começa a fase2')
    glob = GlobalVariables()
    hEdo = glob.getHEDO()
    L1 = glob.getL1()
    #L2 = glob.getL2()
    #L3 = glob.getL3()
    #L4 = glob.getL4()
    #L5 = glob.getL5()
    height = glob.getHeight()
    MDH = glob.getMDH()
    hpi = glob.getHpi()
    #global hpi, L1, L2, L3, L4, L5, height, MDH, hEdo

    dt = hEdo #dt é o tempo da solução da equação Edo
    n = np.array([0, 1, 0]) # n é o vetor diretor do quaternio
    thetab = hpi #parametro da função de caminhada que é igual a pi/2
    realRb = np.array([np.cos(thetab/2)])
    hOrg = np.array([1, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1)) #posição da base 
    rb =  np.concatenate((realRb,np.sin(thetab/2)*n), axis = 0).reshape((4,1))
    pb = np.array([[0],[0], [0], [0]])

    #base B para a base O6 (perna em movimento)
    hB_O6 = transformacao(pb,rb)
    hP = dualQuatMult(hOrg,hB_O6)
    T = np.size(trajCoM,0)
    #t = 1:1:T;
    #matrizes auxiliares
    Mhd = np.zeros((8,T))
    Mha = np.zeros((8,T))
    Mdhd = np.zeros((8,T))

    Mhd2 = np.zeros((8,T))
    Mdhd2= np.zeros((8,T))

    #calculo de Mhd - matriz de hd
    r = np.array([1, 0, 0, 0]).reshape((4,1))
    #p = [0 0 0 0]';
    p = np.array([0, 0, 0, 0]).reshape((4,1))
    hB1 = transformacao(p,r) #transformação base robô
    for i in range (0,T,1):
        p = np.array([0, trajCoM[i,0],trajCoM[i,1],trajCoM[i,2]]).reshape((4,1))
        r = np.array([1, 0, 0, 0]).reshape((4,1))
        hd = transformacao(p,r) #posição desejada do CoM
        mhd = dualQuatMult(hB1,hd)
        #for j in range(8):
        Mhd[:,i]  = mhd[:,0]
         
        
        if i <ind:
            p = np.array([0, trajPA[i,0], trajPA[i,1], trajPA[i,2]]).reshape((4,1))
            n = np.array([0, 1, 0])
            angulo = mt.pi/2 #é isso mesmo?????????????????????????????????????????????????????????????
            realR = np.array([mt.cos(angulo/2)])
            imagR = mt.sin(angulo/2)*n
            rb = np.concatenate((realR,imagR), axis = 0).reshape((4,1))
            hd = transformacao(p,rb) #posição desejada do pé
            mhd2 = dualQuatMult(hB1,hd)
            #for j in range(8):
            Mhd2[:,i] = mhd2[:,0] #transformação da base até o pé
        else:
            #for j in range(8):
                #Mhd2[j,i] = Mhd2[j,ind-1]
            Mhd2[:,i] = Mhd2[:,ind-1]
    
    ha2 = kinematicRobo(theta,hOrg,hP,0,0)
    ha = kinematicRobo(theta,hOrg,hP,0,1)
    # print(getPositionDualQuat(ha),'ha2 fase2')
    #hP = ha2
    #ha2 = kinematicRobo(theta,hOrg,hP,0,0) #posição da perna direita

    #Mdhd[:,0]  = (Mhd[:,0]  - Mhd[:,0])*(1/dt)
    #Mdhd2[:,0]  = (Mhd2[:,0]  - Mhd2[:,0])*(1/dt)

    for i in range(1,T,1):
        #for j in range(8):
        Mdhd[:,i] = (Mhd[:,i] - Mhd[:,i-1])*(1/dt)

        Mdhd2[:,i] = (Mhd2[:,i] - Mhd2[:,i-1])*(1/dt)

    ##################################
    #inicio do codigo
    #LQR
    ganhoS = vecGanho[0,0]
    ganhoQ = vecGanho[1,0]
    ganhoR = vecGanho[2,0]
    #controlador proporcional
   
    ganhoK2 = vecGanho[3,0]
    K2 = ganhoK2*np.eye(8)

    #ganho P-FF
    S = ganhoS*np.eye(8)
    Q = ganhoQ*np.eye(8)
    R = ganhoR*np.eye(8)
    Rinv = np.linalg.inv(R)
    C8 = np.diag([1, -1, -1, -1, 1, -1, -1, -1])
    #iniciar condições finais esperadas para P e E
    Pf = S
    Ef = np.array([0, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1))

    P = Pf
    MP2 = np.zeros((8,8,T))
    for j in range(8):
        for k in range(8):
            MP2[j,k,T-1] = P[j,k]
    E = Ef
    ME2 = np.zeros((8,T))
    #for j in range(8):
    ME2[:,T-1] =E[:,0]


    mhd = np.zeros((8,1))
    mdhd = np.zeros((8,1))

    for i in range(T-2,-1,-1):   
        #for j in range(8):
        #mhdPlus[:,0] = Mhd[:,i+1]
        #mdhdPlus[:,0] = Mdhd[:,i+1]
        aux = dualQuatMult(dualQuatConj(Mhd[:,i+1].reshape((8,1))),Mdhd[:,i+1].reshape((8,1)))
        A  = dualHamiltonOp(aux,0)
        c = -aux
        #prod2 = np.dot(P,Rinv)
        P = P -(-P@A -A.T@P + P@Rinv@P - Q)*dt
        for j in range(8):
            for k in range(8):
                MP2[j,k,i] = P[j,k]
        E = E - (-1*(A.T)@E + P@Rinv@E - P@c)*dt
        #for j in range(8):
        ME2[:,i] = E[:,0]
    
    
    for i in range(0,T,1): 
        #tic
        #Controlador LQR para O CoM
        #calculo de A e c
        aux = dualQuatMult(dualQuatConj(Mhd[:,i].reshape((8,1))),Mdhd[:,i].reshape((8,1)))
        A  = dualHamiltonOp(aux,0)
        c = -aux
        #inicio do controlador  
        #Ja = jacobianoCinematica(theta,hOrg,hP,0,1)
        xe = KinematicModel(MDH,theta,6,0)
        Ja = jacobiano2(theta,hOrg,hP,xe,1) 

        #calculo de P e E
        #calculo de N   
        Hd  = dualHamiltonOp(Mhd[:,i].reshape((8,1)),0)
        # prod3 = np.dot(Hd,C8)
        N  = Hd@C8@Ja
        #pseudo inversa de N
        Np  = np.linalg.pinv(N)
        #######################################################paramos aqui
        #calculo do erro
        e  = np.array([1, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1)) - dualQuatMult(dualQuatConj(ha),Mhd[:,i].reshape((8,1)))
        #calculo de P e E
        #for j in range(8):
        E[:,0] = ME2[:,i]
        #P = np.reshape(MP2[:,i],np.shape(A))
        P[:,:] = MP2[:,:,i].reshape((8,8))
        do = Np@Rinv@(P@e + E)
        #calculo do o deseja
        od  = dt*do/2    
        #for j in range(6):
        theta[:,1] = theta[:,1] + od[:,0]
        # if np.sign(theta[3,1])>0:
        #     theta[3,1] = -theta[3,1]
        #     print('foi útil')
        # for j in range(1,6,1):
        #     if abs(theta[j,1]) > hpi:
        #         theta[j,1] = np.sign(theta[j,1])*hpi
        ha  = kinematicRobo(theta,hOrg,hP,0,1) #posição do CoM com perna esquerda apoiada

        #controlador 2
        #calculo de A e c
        aux2 = dualQuatMult(dualQuatConj(Mhd2[:,i].reshape((8,1))),Mdhd2[:,i].reshape((8,1)))
        #A2  = dualHamiltonOp(aux2,0)
        c = -aux2
        #inicio do controlador  
        #Ja2 = jacobianoCinematica(theta,hOrg,hP,0,0)
        xe2 = kinematicRobo(theta,hOrg,hP,1,0)
        Ja2 = jacobianoPes(theta,ha,xe2,0)
        #calculo de P e E
        #calculo de N   
        Hd2  = dualHamiltonOp(Mhd2[:,i].reshape((8,1)),0)
        # prod1= np.dot(Hd2,C8)
        N2  = Hd2@C8@Ja2
        
        #pseudo inversa de N
        Np2  = np.linalg.pinv(N2)

        #calculo do erro
        e2  = np.array([1, 0, 0, 0, 0, 0, 0, 0]).reshape((8,1)) -  dualQuatMult(dualQuatConj(ha2),Mhd2[:,i].reshape((8,1)))

        vec2 = dualQuatMult(dualQuatConj(ha2),Mdhd2[:,i].reshape((8,1)))
        #do2 = np.zeros(20,20)
        do2 = Np2@(K2@e2-vec2)
        #od2 = np.zeros(100)
        od2 = (do2*dt)/2
        #for j in range(6):
        theta[:,0] = theta[:,0] + od2[:,0]

        t1 = theta[:,0]*1800/np.pi
        t2 = theta[:,1]*1800/np.pi
        t1 = t1.astype('int16').tolist()
        t2 = t2.astype('int16').tolist()

        inc = t1+t2+[0,0,0,0,0,0]
        angles.data = inc
        pub.publish(angles)
        # if np.sign(theta[3,0])>0:
        #     theta[3,0] = -theta[3,0]
        #     print('foi útil')
        ha2  = kinematicRobo(theta,hOrg,hP,0,0) #posição da perna direita

    
    return ha,ha2,theta
