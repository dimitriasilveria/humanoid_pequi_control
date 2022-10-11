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
def right_step(X0,xi,passoLar1,passoLar2,vd,xd,passoCompAnt,passoAlt,posIniPeAnt,altura_y,a,b,ha,ha2,ganho,theta,pub):
    x = X0[0,0]
    v = X0[1,0]
    zod = X0[4,0]
    X0 = np.array([[-xi],[-(passoLar1-X0[2,0])],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM,vel,ind_s,ind_dbl,X0] = trajetoria(X0)
    vf = X0[1,0]
    xi1 = footPlacement(a,b,zod,vf,xd,vd)

    posInicialPe = -passoCompAnt - posIniPeAnt
    passoComprimento =  + X0[0,0]+xi1 - posInicialPe 

    tamTrajP = np.array([ind_s,ind_dbl])
    trajP = trajetoriaPes(np.array([[posInicialPe],[-passoLar1],[0]]),passoComprimento,passoAlt,
    altura_y,-passoLar2,0,tamTrajP)
    [ha,ha2,theta] = fase2(ha,ha2,trajCoM,np.size(trajP,0),trajP,theta,ganho,pub)
    return X0,vel, xi1,passoComprimento,posInicialPe,trajCoM, ha, ha2,theta

def left_step(X0,xi,passoLar1,passoLar2,vd,xd,passoCompAnt,passoAlt,posIniPeAnt,altura_y,a,b,ha,ha2,ganho,theta,pub):
    x = X0[0,0]
    v = X0[1,0]
    zod = X0[4,0]
    X0 = np.array([[-xi],[passoLar1+X0[2,0]],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM,vel,ind_s,ind_dbl,X0] = trajetoria(X0)
    vf = X0[1,0]
    xi1 = footPlacement(a,b,zod,vf,xd,vd)

    posInicialPe = -passoCompAnt - posIniPeAnt

    passoComprimento =  + X0[0,0]+xi1 - posInicialPe 

    tamTrajP = np.array([ind_s,ind_dbl])
    trajP = trajetoriaPes(np.array([[posInicialPe],[passoLar1],[0]]),
    passoComprimento,passoAlt,-altura_y,passoLar2,0,tamTrajP)
    [ha,ha2,theta] = fase3(ha,ha2,trajCoM,np.size(trajP,0),trajP,theta,ganho,pub)
    return X0, vel, xi1,passoComprimento,posInicialPe,trajCoM,ha,ha2,theta 

def caminhada(X0,vecGanho1,vecGanho2,pub):
    #-----------------------------------------------------------
    #Obter todas as trajeotrias do CoM
    #-----------------------------------------------------------
    glob = GlobalVariables()
    g = glob.getG()
    # passoComprimento = 0.08 #tamanho do passo
    passoLargura     = 0.07 #Largura do passo
    passoAltura      = 0.02  #altura de cada passo
    altura_y          = 0.007
    thetaR = glob.getOr()
    thetaL = glob.getOl()
    theta = np.concatenate((thetaR,thetaL),axis=1) # parametros variaveis

    #Passo 1 ##############################################

    [trajCoM1,vel1,ind_s,ind_dbl,X0] = trajetoria(X0) #trajetória para o CoM
    a = 30
    b = 1
    zod = X0[4,0]
    xd = -0.025
    vd = 0
    vf = X0[1,0]
    xi2 = footPlacement(a,b,zod,vf,xd,vd)
    #trajetoria 2
    CoM = trajCoM1
    # print(np.size(trajCoM1))
    ind = np.size(CoM,0) #pegar a última posição do vetor de pontos
    trajCoM2 = np.zeros((ind,3))
    trajCoM3 = np.zeros((ind,3))
    tamTrajPeB1 = np.array([ind_s,ind_dbl])
    passoComprimento1 = xi2
    #trajPB1 = trajetoriaPes(np.array([[passoComprimento],[passoLargura],[0]]),passoComprimento,passoAltura,1,tamTrajPeB1)
    trajPB1 = trajetoriaPesInicio(np.array([[0.0],[passoLargura],[0.0]]),passoComprimento1,passoAltura,0.0649,tamTrajPeB1)
    [ha,ha2,theta] = fase1(trajCoM1,np.size(trajPB1,0),trajPB1,theta,vecGanho1,pub)
    ################################################################
    # Passo 2    
    X02 = np.array([[-xi2],[-(0.0649-X0[2,0])],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM2,vel2,ind_s,ind_dbl,X0] = trajetoria(X02)
    a = 36.5
    b = 5
    xd = 0.025
    vd = 0.05
    vf = X0[1,0]
    xi = footPlacement(a,b,zod,vf,xd,vd)
    passoComprimento = xi2 + X0[0,0]+xi
    tamTrajPa = np.array([ind_s,ind_dbl])
    trajPA = trajetoriaPes(np.array([[0-passoComprimento1],[-0.0649],[0]]),passoComprimento,
    passoAltura,altura_y,-passoLargura,0,tamTrajPa)
    [ha,ha2,theta] = fase2(ha,ha2,trajCoM2,np.size(trajPA,0),trajPA,theta,vecGanho2,pub)

    #passo 3
    X03 = np.array([[-xi],[passoLargura+X0[2,0]],[X0[4,0]],[X0[1,0]],[X0[3,0]],[X0[5,0]]])
    [trajCoM3,vel3,ind_s,ind_dbl,X0] = trajetoria(X03)
    a = 20
    b = 50
    xd = 0.025
    vd = 0.1
    vf = X0[1,0]
    xi2_1 = footPlacement(a,b,zod,vf,xd,vd)
    posInicialPe = +passoComprimento1-passoComprimento
    passoComprimento = -posInicialPe + X0[0,0] +xi2_1
    tamTrajPb = np.array([ind_s,ind_dbl])
    trajPB = trajetoriaPes(np.array([[posInicialPe],[passoLargura],[0]]),passoComprimento,
    passoAltura,-altura_y,passoLargura,0,tamTrajPb)
    [ha,ha2,theta] = fase3(ha,ha2,trajCoM3,np.size(trajPB,0),trajPB,theta,vecGanho1,pub)
    #################################################################################

    #Passo 2_1
    [X0,vel2_1, xi,passoComprimento,posInicialPe,trajCoM2_1,ha,ha2,theta] = right_step(X0,xi2_1,
    passoLargura,passoLargura,0.15,0.015,passoComprimento,passoAltura,posInicialPe,altura_y,2,3,ha,ha2,
    vecGanho2,theta,pub)
   
    #############################################################################################3

    #Passo 3_1
    [X0,vel3_1, xi,passoComprimento,posInicialPe,trajCoM3_1,ha,ha2,theta] = left_step(X0,xi,
    passoLargura,passoLargura,0.15,0.015,passoComprimento,passoAltura,posInicialPe,altura_y,0.001,5,ha,ha2,
    vecGanho1,theta,pub)
    velConc = np.concatenate((vel2_1,vel3_1))
    trajCoMConc = np.concatenate((trajCoM2_1,trajCoM3_1))

    #Passo 2_1
    [X0,vel2_1, xi,passoComprimento,posInicialPe,trajCoM2_1,ha,ha2,theta] = right_step(X0,xi,
    passoLargura,passoLargura,0.15,0.015,passoComprimento,passoAltura,posInicialPe,altura_y,200,3,ha,ha2,
    vecGanho2,theta,pub)
   
    #############################################################################################3

    #Passo 3_1
    [X0,vel3_1, xi,passoComprimento,posInicialPe,trajCoM3_1,ha,ha2,theta] = left_step(X0,xi,
    passoLargura,0.068,0.15,0.015,passoComprimento,passoAltura,posInicialPe,altura_y,2000,1,ha,ha2,
    vecGanho1,theta,pub)
    velConc = np.concatenate((velConc,vel2_1,vel3_1))
    trajCoMConc = np.concatenate((trajCoMConc,trajCoM2_1,trajCoM3_1))

    #Passo 2_1
    [X0,vel2_1, xi,passoComprimento,posInicialPe,trajCoM2_1,ha,ha2,theta] = right_step(X0,xi,
    0.068,passoLargura,0.15,0.017,passoComprimento,passoAltura,posInicialPe,altura_y,2000,1,ha,ha2,
    vecGanho2,theta,pub)
    # velConc = np.concatenate((velConc,vel2_1))
    # trajCoMConc = np.concatenate((trajCoMConc,trajCoM2_1)) 
    #############################################################################################3

    #Passo 3_1
    [X0,vel3_1, xi,passoComprimento,posInicialPe,trajCoM3_1,ha,ha2,theta] = left_step(X0,xi,
    passoLargura,passoLargura,0.15,0.025,passoComprimento,passoAltura,posInicialPe,altura_y,8000,0,ha,ha2,
    vecGanho1,theta,pub)
    velConc = np.concatenate((velConc,vel2_1,vel3_1))
    trajCoMConc = np.concatenate((trajCoMConc,trajCoM2_1,trajCoM3_1))

    #Passo 2_1

    [X0,vel2_1, xi,passoComprimento,posInicialPe,trajCoM2_1,ha,ha2,theta] = right_step(X0,xi,
    passoLargura,passoLargura,0.13,0.027,passoComprimento,passoAltura,posInicialPe,altura_y,1000,1,ha,ha2,
    vecGanho2,theta,pub)
    # velConc = np.concatenate((velConc,vel2_1))
    # trajCoMConc = np.concatenate((trajCoMConc,trajCoM2_1)) 
    # #############################################################################################3

    #Passo 3_1
    [X0,vel3_1, xi,passoComprimento,posInicialPe,trajCoM3_1,ha,ha2,theta] = left_step(X0,xi,
    passoLargura,passoLargura,0.08,0.026,passoComprimento,passoAltura,posInicialPe,altura_y,1000,0,ha,ha2,
    vecGanho1,theta,pub)
    velConc = np.concatenate((velConc,vel2_1,vel3_1))
    trajCoMConc = np.concatenate((trajCoMConc,trajCoM2_1,trajCoM3_1))

    #Passo 2_1
    # print(xi)
    [X0,vel2_1,xi,passoComprimento,posInicialPe,trajCoM2_1,ha,ha2,theta] = right_step(X0,xi,
    passoLargura,passoLargura,0.05,0.02,passoComprimento,passoAltura,posInicialPe,altura_y,2000,3,ha,ha2,
    vecGanho2,theta,pub)

    #passo 3
    [X0,vel3_1, xi,passoComprimento,posInicialPe,trajCoM3_1,ha,ha2,theta] = left_step(X0,xi,
    passoLargura,passoLargura,0.08,0.026,passoComprimento,passoAltura,posInicialPe,altura_y,1000,0,ha,ha2,
    vecGanho1,theta,pub)
    velConc = np.concatenate((velConc,vel2_1,vel3_1))
    trajCoMConc = np.concatenate((trajCoMConc,trajCoM2_1,trajCoM3_1))

    #Passo 2_1
    # print(xi)
    [X0,vel2_1,xi,passoComprimento,posInicialPe,trajCoM2_1,ha,ha2,theta] = right_step(X0,xi,
    passoLargura,passoLargura,0.05,0.02,passoComprimento,passoAltura,posInicialPe,altura_y,2000,3,ha,ha2,
    vecGanho2,theta,pub)
    return

    # #############from turtle import pos, position



    

    
    