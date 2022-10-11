import numpy as np
#---------------------------------------
#Calcular a trajetória dos pés
#Função de Bézier de 4ª ordem
#---------------------------------------
def trajetoriaPesInicio(posP,passo,altura,largura,tam):
    
    p0 = np.array([[posP[0,0]],[posP[2,0]]]) 
    p1 = np.array([[passo/2],[altura]])
    p2 = np.array([[passo],[0]])
    # p0 = np.array([[posP[0,0]],[posP[2,0]]]) + np.array([[-passo],[0]])
    # p1 = np.array([[posP[0,0]],[posP[2,0]]]) + np.array([[-passo/2],[altura/3]])
    # p2 = np.array([[posP[0,0]],[posP[2,0]]]) 
    dt = 1/(tam[0]-1)
    t = np.array(np.arange(0,1+dt,dt)) 
    t = t.reshape(1,-1)
    ind = np.size(t,axis=1)
    total = int(tam[0]+tam[1])
    B = np.zeros((total,3))
    for i in range(0,ind,1):
        B[i,0] = (1 - t[0,i])**2 * p0[0,0] + 2*t[0,i]*(1 - t[0,i])*p1[0,0] + t[0,i]*t[0,i]*p2[0,0]
        B[i,2] = (1 - t[0,i])**2 * p0[1,0] + 2*t[0,i]*(1 - t[0,i])*p1[1,0] + t[0,i]*t[0,i]*p2[1,0]
        B[i,1] = ((largura-posP[1,0])/(t[0,ind-1]-t[0,0]))*(t[0,i]) + posP[1,0]
    

        
        #trajPes = np.transpose(trajPes)

    trajPes = np.ones((ind,3))
    for j in range(ind):
        trajPes[j,0] = B[j,0]
        trajPes[j,1] = B[j,1] 
        trajPes[j,2] = B[j,2]  
    #trajPes = np.transpose(trajPes)
    return trajPes

