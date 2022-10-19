#-----------------------------------
#Método para executar o passo inicial
#trajCoM1 é a trajetória do CoM
#trajPB1 trajetória do pé
#theta vetor com os thetas medidos
#vecGanho são os ganhos do controlador
#ind indica em qual ponto da trajetória ocorre o TD
#-----------------------------------
#from DualQuaternionClass import DualQuaternion, Quaternion
#!/usr/bin/env python

from kinematicRobo import kinematicRobo
from transformacao import transformacao
import numpy as np 
import math as mt
import time as tm
from globalVariables import GlobalVariables
from dualQuatMult import dualQuatMult
from dualQuatConj import dualQuatConj
from dualHamiltonOp import dualHamiltonOp
from jacobianoCoM import jacobiano2
from kinematicModel import KinematicModel
from jacobianoPes import jacobianoPes
import time
import pandas as pd 
import rospy
from std_msgs.msg import Int16MultiArray


def fase1(trajCoM1,ind,trajPB1,theta,vecGanho,pub):
    angles = Int16MultiArray()

    global p,r 

    #global hpi, L1, L2, L3, L4, L5, height, MDH, hEdo
    glob = GlobalVariables()
    hEdo = glob.getHEDO()
    L1 = glob.getL1()
    height = glob.getHeight()
    MDH = glob.getMDH()
    hpi = 0.5*mt.pi
    hOrg = np.array([[1],[0], [0], [0], [0], [0], [0], [0]]) #posição da base
    dt = hEdo 
    #cacular posição atual do´pé
    n = np.array([0, 1, 0]) # n é o vetor diretor do quaternio
    thetab = hpi #parametro da função de caminhada que é igual a pi/2
    realRb = np.array([np.cos(thetab/2)])
    rb =  np.concatenate((realRb,np.sin(thetab/2)*n), axis = 0).reshape((4,1))
    pb = np.array([[0],[0], [0], [0]])

    #base B para a base O6 (B é a perna em movimento)
    hB_O6 = transformacao(pb,rb)
    hP = dualQuatMult(hOrg,hB_O6)
	#dt é o tempo da solução da equação Edo e, ao mesmo tempo, o passo
    T = np.size(trajCoM1,0) #o tamanho de trajCoM = ind

    #matrizes e vetores auxiliares
    Mhd = np.zeros((8,T))
    Mdhd = np.zeros((8,T))
    mhd = np.zeros((8,1))
    mdhd = np.zeros((8,1))
    mhd2 = np.zeros((8,1))
    mdhd2 = np.zeros((8,1))

    Mhd2 = np.zeros((8,T))
    Mdhd2= np.zeros((8,T))
    
    #calculo de Mhd - matriz de hd
    r = np.array([1, 0, 0, 0]).reshape((4,1))
    p = np.array([0, 0, 0, 0]).reshape((4,1))  #height = L2 + L3 + L4 + L5
    hB1 = transformacao(p,r)#transformação base robô
    for i in range(0,T,1): 
        p = np.array([0, trajCoM1[i,0],trajCoM1[i,1],trajCoM1[i,2]]).reshape((4,1))
        r = np.array([1, 0, 0, 0]).reshape((4,1))
        hd = transformacao(p,r)
        mhd = dualQuatMult(hB1,hd)
        for j in range(8):
            Mhd[j,i] = mhd[j,0]
        #transformação da base até o centro de massa
        #se i<ind, o robô ainda não atingiu a posição de td, então a transformação é calculada em relação ao pé
	    #quando o robô chega na fase de TD, a transformação é calculada em relação ao CoM
        if i <ind:
            p = np.array([0, trajPB1[i,0], trajPB1[i,1],trajPB1[i,2]]).reshape((4,1))
            n = np.array([0, 1, 0])
            #angulo = mt.pi/2.0 #graus ou radianos????????????????????????????????????????????????????????????/
            realRb = np.array([np.cos(thetab/2)])
            rb =  np.concatenate((realRb,np.sin(thetab/2)*n), axis = 0).reshape((4,1))  
            hd = transformacao(p,rb) #posição desejada
            mhd2 = dualQuatMult(hB1,hd)#transformação da base até o pé
            for j in range(8):
                Mhd2[j,i] = mhd2[j,0]
        else:
            Mhd2[:,i] = Mhd2[:,ind-1]
  
    ha = kinematicRobo(theta,hOrg,hP,1,1) #cinemática do pé direito até o CoM
    ha2 = kinematicRobo(theta,hOrg,hP,1,0) #cinemática de um pé até o outro

    #Mdhd[:,0]  = (Mhd[:,0]  - Mhd[:,0])*(1/dt) #não deveria ser i-1, no segundo Mhd???????????????????????????????????
    #Mdhd2[:,0]  = (Mhd2[:,0]  - Mhd2[:,0])*(1/dt)

    for i in range (1,T,1):
        Mdhd[:,i] = (Mhd[:,i] - Mhd[:,i-1])*(1/dt) #por que ele fazer isso????????????????????????????????????????????????????
        Mdhd2[:,i]  =  (Mhd2[:,i] - Mhd2[:,i-1])*(1/dt) #derivada de hd, que é a posição desejada             
    ##################################
    #inicio do codigo
    #LQR 
    #calculo dos parâmetros
    ganhoS = vecGanho[0,0]
    ganhoQ = vecGanho[1,0]
    ganhoR = vecGanho[2,0]
    #controlador proporcional
    ganhoK2 = vecGanho[3,0]
    K2 = ganhoK2*np.eye(8)
    Ki = 0*np.eye(8)
    Kd = 0*np.eye(8)
    S = ganhoS*np.eye(8)
    Q = ganhoQ*np.eye(8)
    R = ganhoR*np.eye(8)
    Rinv = np.linalg.inv(R)
    # print('R::',Rinv)
    # return
    ab = np.array([1, -1,-1, -1, 1, -1, -1,-1])
    C8 = np.diag(ab)

    #iniciar condições finais esperadas para P e E
    Pf = S
    Ef = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((8,1))
    P = Pf
    MP2 = np.zeros((8,8,T))
    for j in range(8):
        for k in range(8):
            MP2[j,k,T-1] = P[j,k]
    E = Ef
    ME2 = np.zeros((8,T))
    for j in range(8):
        ME2[j,T-1] = E[j,0]

    #calcular matrizes de ganho
    for i in range(T-2,-1,-1):   

        for j in range(8):
            mhd[j,0] = Mhd[j,i+1] 
            mdhd[j,0] = Mdhd[j,i+1]
        aux = dualQuatMult(dualQuatConj(mhd),mdhd)
        A  = dualHamiltonOp(aux,0)
        c = -aux
        # prod2 = np.dot(P,Rinv)
        P = P -(-P@A -A.T@P + P@Rinv@P - Q)*dt
        for j in range(8):
            for k in range(8):
                MP2[j,k,i] = P[j,k]
        E = E - ((-1)*(A.T)@E + P@Rinv@E - P@c)*dt
        for j in range(8):
            ME2[j,i] = E[j,0]

  
    for i in range(0,T,1):
        #tic = tm.time()
        #Controlador LQR para o CoM
        #calculo de A e c
        for j in range(8):
            mhd[j,0] = Mhd[j,i] 
            mdhd[j,0] = Mdhd[j,i]
        aux = dualQuatMult(dualQuatConj(mhd),mdhd) #calculo de hd conjugado * hd derivada
        A  = dualHamiltonOp(aux,0)
        c = -aux
        #inicio do controlador
        #hB_O6a = dualQuatMult(hOrg,hP)
        xe = KinematicModel(MDH,theta,6,0)
        Ja = jacobiano2(theta,hOrg,hP,xe,0) #jacobiano para a perna direita
        #Ja = jacobianoCinematica(theta,hOrg,hP,1,1)
        #calculo de P e E
        #calculo de N   
        Hd  = dualHamiltonOp(mhd,0)
        # prod3 = np.dot(Hd,C8)
        N  = Hd@C8@Ja
        #pseudo inversa de N
        Np  = np.linalg.pinv(N)
        #Np = (1/np.linalg.norm(Np))*Np

        #calculo do erro
        e  = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((8,1)) - dualQuatMult(dualQuatConj(ha),mhd)
        #calculo de P e E

        E[:,0] = ME2[:,i]
        P[:,:] = MP2[:,:,i].reshape((8,8))
        #Pxe= np.dot(P,e)
        #do = Np@Rinv@(P@e + E) 
        do = Np@Rinv@(P@e + E) #equação final para theta ponto
        #calculo do theta deseja
        od  = (do*dt)/2
        theta[:,0] = theta[:,0] + od[:,0]
		#o movimento dos motores é limitado entre pi/2 e -pi/2, então, se theta estiver
		#fora do intervalo, esse for faz theta = limite do intervalo
        for j in range(0,6,1):
            if abs(theta[j,0]) > hpi:
                theta[j,0] = np.sign(theta[j,0])*hpi

        ha  = kinematicRobo(theta,hOrg,hP,1,1)  #não deveria ser hd?????????????????????????????????????????


        #hB_O6a = dualQuatMult(hOrg,hP)
        xe2 = kinematicRobo(theta,hOrg,hP,1,0)
        Ja2 = jacobianoPes(theta,ha,xe2,1)
        #Ja2 = jacobianoCinematica(theta,hOrg,hP,1,0)
        #calculo de P e E
        #calculo de N  
        mhd2 = np.zeros((8,1))
        mdhd2 = np.zeros((8,1))
        for j in range(8):
            mhd2[j,0] = Mhd2[j,i] 
            mdhd2[j,0] = Mdhd2[j,i]
        
        Hd2  = dualHamiltonOp(mhd2,0)
        N2  = Hd2@C8@Ja2
        #pseudo inversa de N
        Np2  = np.linalg.pinv(N2)

        #calculo do erro
        e2  = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((8,1)) - dualQuatMult(dualQuatConj(ha2),mhd2)
        vec2 = dualQuatMult(dualQuatConj(ha2),mdhd2)
        #
        # K2xe2 = np.dot(K2,e2)
        # do2 = np.dot(Np2,(K2xe2-vec2))
        # integral = integral + e2*dt
        # do2 = Np2@(K2@e2 + Kd@(e2 - e_previous) + Ki@integral - vec2)
        do2 = Np2@(K2@e2-vec2)
        od2  = (do2*dt)/2  
        theta[:,1] = theta[:,1] + od2[:,0] 

        t1 = theta[:,0]*1800/np.pi
        t2 = theta[:,1]*1800/np.pi
        t1 = t1.astype('int16').tolist()
        t2 = t2.astype('int16').tolist()

        inc = t1+t2+[0,0,0,0,0,0]
        angles.data = inc
        pub.publish(angles)
        # for j in range(0,6,1):
        #     if abs(theta[j,1]) > hpi:
        #         theta[j,1] = np.sign(theta[j,1])*hpi
        ha2  = kinematicRobo(theta,hOrg,hP,1,0)


    # print('ThetaRight, ',Mtheta)

    # plotGraficosControle(t1,dt,T,Pos,Posd,angle,angled,Mha,Mhd,Mtheta,Pos2,Posd2,angle2,angled2,Mha2,Mhd2,Mtheta2,'b','r')
    return ha, ha2, theta
