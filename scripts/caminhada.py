from trajetoria import trajetoria
from trajetoriaPes import trajetoriaPes
from fase3 import fase3
from fase1 import fase1
from fase2 import fase2
from globalVariables import GlobalVariables
from trajetoriaPesInicio import trajetoriaPesInicio
import numpy as np
import math
import time
#-----------------------------------------
#Executar a caminhada
#Parâmetros:
#U:Vetor com as variáveis de controle
#X:Vetor com a condição inicial
#vecGanho1:vetor com os ganhos do controlador para a condição da perna
#          direita em contato com chão
#vecGanho2:vetor com os ganhos do controlador para a condição da perna
#          esquerda em contato com o chão
#------------------------------------------

 


def footPlacement(a,b,zod,vf,xd,vd):
    glob = GlobalVariables()
    g = glob.getG()
    ts = glob.ts
    tdb = glob.tdbl
    Tc = np.sqrt(zod/g)
    Ct = math.cosh(ts/Tc)
    St = math.sinh(ts/Tc)
    Dt = a*(Ct**2)+b*(St/Tc)**2
    xi = (a*Ct*(xd-St*Tc*vf)+b*St/Tc*(vd-Ct*vf))/Dt

    return xi

def caminhada(pub_theta):
    #-----------------------------------------------------------
    #Obter todas as trajeotrias do CoM
    #-----------------------------------------------------------
    glob = GlobalVariables()
    g = glob.getG()
    L1 = glob.L1

    passoLargura     = 2*L1 #Largura do passo
    passoAltura      = 0.02  #altura de cada passo
    altura_y          = 0.007
    thetaR = glob.getOr()
    thetaL = glob.getOl()
    theta = np.concatenate((thetaR,thetaL),axis=1) # parametros variaveis

    #Darwin
    xod  = 0.0
    yod  = glob.L1
    zod  = 0.25
    dxod = 0.0
    dyod = -0.1325
    dzod = 0.0

    #Ganhos LQR (perna direita) e proporcional (perna esquerda)
    ganhoS1 = 0
    ganhoQ1 = 500
    ganhoR1 = 0.001
    ganhoK1 = 900
    #Ganhos LQR (perna esquerda) e proporcional (perna direita)
    ganhoS2 = 0
    ganhoQ2 = 500
    ganhoR2 = 0.001
    ganhoK2 = 900

    X0 = np.array([[xod],[yod],[zod],[dxod],[dyod],[dzod]]) #condição inicial

    vecGanho1 = np.array([ganhoS1, ganhoQ1, ganhoR1, ganhoK1]).reshape((4,1))
    vecGanho2 = np.array([ganhoS2, ganhoQ2, ganhoR2, ganhoK2]).reshape((4,1))
    #Passo 1 ##############################################

    #Passo 1 ##############################################
    [trajCoM1,vel1,ind_s1,ind_dbl1,X0] = trajetoria(X0) #trajetória para o CoM
    a = 30
    b = 0
    zod = X0[4,0]
    xd = -0.025
    vd = 0
    vf = X0[1,0]
    xi2 = footPlacement(a,b,zod,vf,xd,vd)
    tamTrajPeB1 = np.array([ind_s1,ind_dbl1])
    passoComprimento1 = xi2
    trajPB1 = trajetoriaPesInicio(np.array([[0.0],[passoLargura],[0.0]]),passoComprimento1,
    passoAltura,altura_y,passoLargura,tamTrajPeB1)

    [ha,ha2,theta] = fase1(trajCoM1,np.size(trajPB1,0),trajPB1,theta,vecGanho1,pub_theta)
    ################################################################
    # Passo 2    
    X02 = np.array([[-xi2],[-(passoLargura-X0[2,0])],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM2,vel2,ind_s2,ind_dbl2,X0] = trajetoria(X02)
    a = 36.5
    b = 3
    xd = 0.025
    vd = 0.05
    vf = X0[1,0]
    xi3 = footPlacement(a,b,zod,vf,xd,vd)
    passoComprimentoA = xi2 + X0[0,0]+xi3 
    tamTrajPa2 = np.array([ind_s1,ind_dbl1])
    trajPA2 = trajetoriaPes(np.array([[0-passoComprimento1],[-passoLargura],[0]]),passoComprimentoA,
    passoAltura,altura_y,-passoLargura,0,tamTrajPa2)
    [ha,ha2,theta] = fase2(ha,ha2,trajCoM2,
    np.size(trajPA2,0),trajPA2,theta,vecGanho2,pub_theta)
    ##########################################################
    #Passo 3
    X03 = np.array([[-xi3],[passoLargura+X0[2,0]],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM3,vel3,ind_s3,ind_dbl3,X0] = trajetoria(X03)
    a = 20
    b = 3.1
    xd = 0.025
    vd = 0.1
    vf = X0[1,0]
    xi2_1 = footPlacement(a,b,zod,vf,xd,vd)
    posInicialPe = +passoComprimento1-passoComprimentoA
    passoComprimentoB = -posInicialPe + X0[0,0] +xi2_1  
    tamTrajPb3 = np.array([ind_s3,ind_dbl3])
    trajPB3 = trajetoriaPes(np.array([[posInicialPe],[passoLargura],[0]]),
    passoComprimentoB,passoAltura,-altura_y,passoLargura,0,tamTrajPb3)
    [ha,ha2,theta] = fase3(ha,ha2,trajCoM3,np.size(trajPB3,0),trajPB3,theta,
    vecGanho1,pub_theta)


    #################################################################################

    #Passo 2_1
    X02_1 = np.array([[(-xi2_1)],[-(passoLargura-X0[2,0])],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM2_1,vel2_1,ind_s2,ind_dbl2,X0] = trajetoria(X02_1)
    a = 100
    b = 0
    xd = 0.015
    vd = 0.15
    vf = X0[1,0]
    xi3_1 = footPlacement(a,b,zod,vf,xd,vd)
    posInicialPe = -passoComprimentoB - posInicialPe 
    passoComprimentoA = -posInicialPe + X0[0,0] +xi3_1  #- 0.01256
    tamTrajPa2_1 = np.array([ind_s2,ind_dbl2])
    trajPA2_1 = trajetoriaPes(np.array([[posInicialPe],[-passoLargura],[0]]),
    passoComprimentoA,passoAltura,altura_y,-passoLargura,0,tamTrajPa2_1)
    [ha,ha2,theta] = fase2(ha,ha2,trajCoM2_1,np.size(trajPA2_1,0),
    trajPA2_1,theta,vecGanho2,pub_theta)


    #############################################################################################3

    #Passo 3_1
    X03_1 = np.array([[-xi3_1],[passoLargura+X0[2,0]],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM3_1,vel3_1,ind_s3,ind_dbl3,X0] = trajetoria(X03_1)
    a =10
    b = 1
    xd = 0.025
    vd = 0.15
    vf = X0[1,0]
    xi2_2 = footPlacement(a,b,zod,vf,xd,vd)
    posInicialPe = -passoComprimentoA - posInicialPe
    passoComprimentoB = -posInicialPe + X0[0,0] + xi2_2
    tamTrajPb3_1 = np.array([ind_s3,ind_dbl3])
    trajPB3_1 = trajetoriaPes(np.array([[posInicialPe],[passoLargura],[0]]),
    passoComprimentoB,passoAltura,-altura_y,passoLargura,0,tamTrajPb3_1)
    [ha,ha2,theta] = fase3(ha,ha2,trajCoM3_1,np.size(trajPB3_1,0),
    trajPB3_1,theta,vecGanho1,pub_theta)

    #############################################################################################

    # passo 2_2
    X02_2 = np.array([[-xi2_2],[X0[2,0]-passoLargura],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM2_2,vel2_2,ind_s2,ind_dbl2,X0] = trajetoria(X02_2)
    a = 2000
    b = 0
    xd = 0.0155
    vd = 0.00
    vf = X0[1,0]
    xi3_2 = footPlacement(a,b,zod,vf,xd,vd)
    posInicialPe = -passoComprimentoB - posInicialPe
    passoComprimentoA = -posInicialPe + X0[0,0] + xi3_2
    tamTrajPa2_2 = np.array([ind_s2,ind_dbl2])
    trajPA2_2 = trajetoriaPes(np.array([[posInicialPe],[-passoLargura],[0]]),
    passoComprimentoA,passoAltura,altura_y,-passoLargura,0,tamTrajPa2_2)
    # print(getPositionDualQuat(ha),'ha2 fase2 inicial')
    [ha,ha2,theta] = fase2(ha,ha2,trajCoM2_2,np.size(trajPA2_2,0),
    trajPA2_2,theta,vecGanho2,pub_theta)

    # #####passo3_2#########################################################################
    X03_2 = np.array([[-xi3_2],[passoLargura+X0[2,0]],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM3_2,vel3_2,ind_s3,ind_dbl3,X0] = trajetoria(X03_2)
    a = 6.8
    b = 0
    xd = 0.014
    vd = 0.0
    # vf = X0[1,0]
    xi2_3 = footPlacement(a,b,zod,vf,xd,vd)

    posInicialPe = -passoComprimentoA - posInicialPe
    passoComprimentoB = -posInicialPe + X0[0,0] + xi2_3 
    tamTrajPb3_2 = np.array([ind_s3,ind_dbl3])
    trajPB3_2 = trajetoriaPes(np.array([[posInicialPe],[passoLargura],[0]]),
    passoComprimentoB,passoAltura,-altura_y,passoLargura,0,tamTrajPb3_1)
    [ha,ha2,theta] = fase3(ha,ha2,trajCoM3_2,np.size(trajPB3_2,0),
    trajPB3_2,theta,vecGanho1,pub_theta)

    # ##################################################################################3
    # #stopping
    X02_3 = np.array([[(-xi2_3)],[-(passoLargura-X0[2,0])],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM2_3,vel2_3,ind_s2,ind_dbl2,X0] = trajetoria(X02_3)

    a = 42000
    b = 0
    xd = 0.008
    vd = 0.0
    vf = X0[1,0]
    xi3_3 = footPlacement(a,b,zod,vf,xd,vd)

    posInicialPe = -passoComprimentoB - posInicialPe

    passoComprimentoA = -posInicialPe + X0[0,0] +xi3_3  #- 0.01256

    tamTrajPa2_3 = np.array([ind_s2,ind_dbl2])
    trajPA2_3 = trajetoriaPes(np.array([[posInicialPe],[-passoLargura],[0]]),
    passoComprimentoA,passoAltura,altura_y,-passoLargura,0,tamTrajPa2_3)

    [ha,ha2,theta] = fase2(ha,ha2,trajCoM2_3,np.size(trajPA2_3,0),
    trajPA2_3,theta,vecGanho2,pub_theta)  
    # ##########################################################
    # #Passo 3_3
    X03_3 = np.array([[-xi3_3],[passoLargura+X0[2,0]],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM3_3,vel3_3,ind_s3,ind_dbl3,X0] = trajetoria(X03_3)
    # a = 200
    # b = 0
    # xd = 0.013
    # vd = 0.0
    # vf = X0[1,0]
    # [xi2_4,d2_1] = footPlacement(a,b,zod,vf,xd,vd)
    xi2_4 = 0.0
    posInicialPe = -passoComprimentoA-posInicialPe
    passoComprimentoB = -posInicialPe + X0[0,0] +xi2_4

    tamTrajPb3 = np.array([ind_s3,ind_dbl3])
    trajPB3 = trajetoriaPes(np.array([[posInicialPe],[passoLargura],[0]]),
    passoComprimentoB,passoAltura,-altura_y,passoLargura,0,tamTrajPb3)
    [ha,ha2,theta] = fase3(ha,ha2,trajCoM3_3,np.size(trajPB3,0),trajPB3,
    theta,vecGanho1,pub_theta)
    return

    # #############from turtle import pos, position



    

    
    