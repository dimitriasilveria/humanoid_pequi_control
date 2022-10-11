import numpy as np 
import math
from derivadaFX import DerivadaFX
from derivadaFU import DerivadaFU
#-----------------------------------------------------------
#Cálculo do Jacobiano Analítico
#Parâmetros:
#theta - vetor com as variáveis de juntas 
#hOrg  - quatérnio dual com a origem do sistema de referência
#hP    - quatérnio dual com a posição do pé de apoio
#tipo  - flag que indica se a cinemática é da perna direita ou esquerda
#CoM   - flag para indicar se é para computar a cinemática até o centro de 
#		 massa ou até o outro pé.
#----------------------------------------------------------   
def jacobianoF(u,x,t,model,pfb,A):
#-----------------------------------------------------------
#cálculo das derivadas para cada variável de controle
#----------------------------------------------------------    
    Jx = np.zeros((5,5))
    J1= DerivadaFX(x,0,t,model,pfb,A)
    J2= DerivadaFX(x,1,t,model,pfb,A)
    J3= DerivadaFX(x,2,t,model,pfb,A)
    J4= DerivadaFX(x,3,t,model,pfb,A)
    J5= DerivadaFX(x,4,t,model,pfb,A)
        
    for j in range(5):
        Jx[j,0] = J1[j,0]
        Jx[j,1] = J2[j,0]
        Jx[j,2] = J3[j,0]
        Jx[j,3] = J4[j,0]
        Jx[j,4] = J5[j,0]
    Jx = np.dot(A,Jx)

    Ju = np.zeros((5,5))
    J1= DerivadaFU(u,x,0,t,model,pfb,A)
    J2= DerivadaFU(u,x,1,t,model,pfb,A)
    J3= DerivadaFU(u,x,2,t,model,pfb,A)
    J4= DerivadaFU(u,x,3,t,model,pfb,A)
    J5= DerivadaFU(u,x,4,t,model,pfb,A)
        
    for j in range(5):
        Ju[j,0] = J1[j,0]
        Ju[j,1] = J2[j,0]
        Ju[j,2] = J3[j,0]
        Ju[j,3] = J4[j,0]
        Ju[j,4] = J5[j,0]

    Ju = np.dot(A,Ju)

    
        
    return np.array([Jx, Ju])