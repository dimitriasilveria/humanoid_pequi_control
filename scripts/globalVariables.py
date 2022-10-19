import numpy as np
import math as mt
class GlobalVariables:
    thetaR = np.zeros((6,1))
    thetaL = np.zeros((6,1))
    hpi = (np.pi)/2.0
    #hubo's parameters
    # L1 = 0.085
    # L2 = 0.0
    # L3 = 0.3
    # L4 = 0.3
    # L5 = 0.0663
    # #Darwin's parameters
    L1 = 0.065
    L2 = 0.053
    L3 = 0.1003
    L4 = 0.100
    L5 = 0.0421

    #HRP's parameters
    # L1 = 0.035
    # L2 = 0.345
    # L3 = 0.3
    # L4 = 0.3
    # L5 = 0.0
    
    # L1 = 1.0
    # L2 = 0.0
    # L3 = 1.0
    # L4 = 1.0
    # L5 = 0.0

    height = L2 + L3 + L4 + L5

    #tabela DH 1
    oi = np.array([0.0, -hpi, 0.0, 0.0, 0.0,0.0]).reshape((6,1))
    di = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((6,1))
    ai = np.array([0.0, 0.0, L3, L4, 0.0, L5]).reshape((6,1))
    si = np.array([hpi, -hpi, 0.0, 0.0, hpi, 0.0]).reshape((6,1))

    # oi2 = np.array([[0.0], [0.0], [0.0], [-np.pi], [-hpi],[0.0]])
    # di2 = np.array([[0.0], [0.0], [0.0], [0.0], [0.0],[0.0]])
    # ai2 = np.array([[0.0], [-L4], [-L3], [0.0], [0.0],[0.0]])
    # si2 = np.array([[-hpi], [0.0], [0.0], [-hpi], [-hpi],[0.0]])

    # ori = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((6,1)) #perna direita
    # ol = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((6,1)) #perna esquerda
    ori = np.array([0.0,0.0,-0.686966,1.3732868,-0.6863206,0.0]).reshape((6,1)) #perna direita
    ol  = np.array([0.0,0.0,-0.686966,1.3732868,-0.6863206,0.0]).reshape((6,1)) #perna esquerda  
    # ol = np.array([0.34761034,-0.43843274,0.3868615 ,-0.46848793,0.78814092,0.55240586]).reshape((6,1)) #perna direita
    # ori = np.array([-0.03602698,0.04953561,-0.51070792,1.02498516,0.11493388,-0.06124262]).reshape((6,1)) #perna esquerda
    MDH = np.zeros((6,4))
    for j in range(6):
        MDH[j,0] = oi[j,0]
        MDH[j,1] = di[j,0]
        MDH[j,2] = ai[j,0]
        MDH[j,3] = si[j,0]
    

    # MDH2 = np.zeros((6,4))
    # for j in range(6):
    #     MDH2[j,0] = oi2[j,0]
    #     MDH2[j,1] = di2[j,0]
    #     MDH2[j,2] = ai2[j,0]
    #     MDH2[j,3] = si2[j,0]
    #     #MDH[j,4] = ori[j,0]
    #     #MDH[j,5] = ol[j,0]
    #print(MDH)
    hEdo = 10**(-3) #passo para o cálculo da EDO
    #m = 38.1 #hubo
    # m = 3.5 #Darwin
    m = 58.2 #HRP
    #L = 0.63 #tamanho da perna em relação ao CoM Hubo
    #L = 0.2425 #darwin's leg size
    L = 0.204
    ts = 0.1
    tdbl = 0.04
    #L = 0.22276
    g = 9.8 
    h = 10**(-5) #passo para o calculo das derivadas
    maxNGrad = 10**6 #número máximo de iterações método
    ganhoAlpha = 10**(-1) #ganho do fator de ganho para cada passo
    gamma = 0.2 #ganho para os método gradiente(momento)
    thetaM = 0.5
    phiM = 0.5
    KM = 2.0
    BSSM = 0.2
    #lstep =
    pfa = np.array([0.0, 0.0, 0.0]).reshape((3,1)) #posição do pé de suporte em MS
    #expK = 10000.0 #ordem de grandeza da constante massa-mola
    expK = 1000.0 #ordem de grandeza da constante massa-mola Darwin

    #Hubo dynamic model parameters
    # phi = 0.3408040779
    # theta = 0.4052906627
    # k = 19196.8680322965
    # Bss = 0.0415640571

    #DArwin dynamic model parameters
    phi= 0.6482570031
    theta = 0.2823507428
    k = 153.9300628927
    Bss = 0.0414743461
    
    def getHpi(self):
        return self.hpi

    def getL1(self):
        return self.L1

    def getL2(self):
        return self.L2
    
    def getL3(self):
        return self.L3

    def getL4(self):
        return self.L4
    
    def getL5(self):
        return self.L5

    def getHeight(self):
        return self.height

    def getMDH(self):
        return self.MDH
    
    # def getMDH2(self):
    #     return self.MDH2

    def getHEDO(self):
        return self.hEdo

    def getH(self):
        return self.h

    def getMaxNGrad(self):
        return self.maxNGrad
    
    def getGanhoAlpha(self):
        return self.ganhoAlpha

    def getGamma(self):
        return self.gamma

    def getThetaM(self):
        return self.thetaM

    def getPhiM(self):
        return self.phiM

    def getKM(self):
        return self.KM

    def getBSSM(self):
        return self.BSSM
    
    def getPfa(self):
        return self.pfa

    def getExpK(self):
        return self.expK 
    
    def getM(self):
        return self.m

    def getL(self):
        return self.L

    def getG(self):
        return self.g 

    def getOr(self):
        return self.ori

    def getOl(self):
        return self.ol

    def getPhi(self):
        return self.phi
    
    def getTheta(self):
        return self.theta
    
    def getK(self):
        return self.k

    def getBss(self):
        return self.Bss

    def setPhi(self,phi):
        self.phi = phi

    def setTheta(self,theta):
        self.theta = theta

    def setK(self,k):
        self.k = k

    def setBss(self,Bss):
        self.Bss = Bss

    def setThetaR(self, thetaR):
        self.thetaR = thetaR
		
    def setThetaL(self, thetaL):
        self.thetaL = thetaL
    
    def getThetaR(self):
        return self.thetaR
		
    def getThetaL(self):
        return self.thetaL
    
