import numpy as np
#---------------------------------------
#Calcular a trajetória dos pés
#Função de Bézier de 4ª ordem
#---------------------------------------
def trajetoriaPes(posP,passo,alturaZ,alturaY,largura,metade,tam):
    
    p0 =  np.array([[posP[0,0]],[posP[2,0]]]) 
    p1 =  np.array([[posP[0,0]+(passo)/2],[alturaZ]]) 
    p2 =  np.array([[(passo+posP[0,0])],[0]]) 

    p0y =  np.array([[posP[0,0]],[posP[1,0]]]) 
    p1y =  np.array([[posP[0,0]+(passo)/2],[alturaY]]) 
    p2y =  np.array([[(passo+posP[0,0])],[posP[1,0]]]) 

    dt = 1/(tam[0]-1)
    t = np.array(np.arange(0,1+dt,dt)) 
    t = t.reshape(1,-1)
    ind = np.size(t,axis=1)
    k = ind-tam[0]
    total = int(tam[0]+tam[1])
    B = np.zeros((total,3))
    for i in range(0,ind,1):
        B[i,0] = ((p2[0,0]-p0[0,0])/(t[0,ind-1]-t[0,0]))*(t[0,i]) + p0[0,0]
        #B[i,0] = (1 - t[0,i])**2 * p0[0,0] + 2*t[0,i]*(1 - t[0,i])*p1[0,0] + t[0,i]*t[0,i]*p2[0,0]
        B[i,2] = (1 - t[0,i])**2 * p0[1,0] + 2*t[0,i]*(1 - t[0,i])*p1[1,0] + t[0,i]*t[0,i]*p2[1,0]
        if posP[1,0] != largura:
            B[i,1] = ((largura-posP[1,0])/(t[0,ind-1]-t[0,0]))*(t[0,i]) + posP[1,0]
        else:
            B[i,1] = alturaY*np.sin(np.pi*t[0,i]) + largura
            # if i <=ind/2:
            #     B[i,1] = 2*alturaY*t[0,i] + largura
            # else:
            #     B[i,1] = -2*alturaY*t[0,i] + 2*alturaY + largura
            #B[i,1] = (1 - t[0,i])**2 * p0y[1,0] + 2*t[0,i]*(1 - t[0,i])*p1y[1,0] + t[0,i]*t[0,i]*p2y[1,0]
    if metade ==1:
        trajPes = np.ones((k,3))
        for j in range(k):
            trajPes[j,0] = B[j+tam,0]
            trajPes[j,1] = B[j+tam,1]
            trajPes[j,2] = B[j+tam,2]
        
        #trajPes = np.transpose(trajPes)
    else:
        trajPes = np.ones((ind,3))
        for j in range(ind):
            trajPes[j,0] = B[j,0]
            trajPes[j,1] = B[j,1]
            trajPes[j,2] = B[j,2]  
        #trajPes = np.transpose(trajPes)
    return trajPes

