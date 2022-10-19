import numpy as np
import math as mt 
#-----------------------------------------------------------
#Calcula o valor das derivadas do modelo dinâmico 1
#Parâmetros
#t   - tempo
#Y0  - Vetor com a condição inicial do sistema
#params: parâmetros do sistema
#m   - massa
#L   - comprimento da perna do modelo
#g   - gravidade
#k   - constante da mola
#Bss - coeficiente angular do modelo
#Retorno:
#dydt - vetor com primeira e segunda derivada do sistema
#-----------------------------------------------------------
def LIPM2(t,Y0,params):

#-----------------------------------------------------------
#constantes
#-----------------------------------------------------------
    g    = params[2,0]
#-----------------------------------------------------------
#condição inicial
#----------------------------------------------------------- 
    #Y0 = np.array((6,1))
    x  = Y0[0,0]
    dx = Y0[1,0]
    y  = Y0[2,0]
    dy = Y0[3,0]
    z  = Y0[4,0]
    dz = Y0[5,0]
#-----------------------------------------------------------
#norma de L
#-----------------------------------------------------------      
#-----------------------------------------------------------
#derivadas
#-----------------------------------------------------------   
    f1 = dx
    f2 = 0
    f3 = dy
    f4 = 0
    f5 = Y0[4,0]
    f6 = Y0[5,0]
 #-----------------------------------------------------------
#solução
#-----------------------------------------------------------    
    dydt = np.array([[f1],[f2],[f3],[f4],[f5],[f6]])
    return dydt