import numpy as np 
import math
import cmath
from globalVariables import GlobalVariables
from rungekuttaLIPM import rungeKuttaLIPM
#-----------------------------------------------------------
#Método para calcular a trajetória do modelo dinâmico
#otimização de um quarto de passo, resolvendo 2 EDOs
#para implementar o modelo dinâmico completo, é necessário resolver mais 4 EDOs
#retornando os pontos dos pés, CoM e a trajetória
#Parâmetros:
#U - variáveis de controle
#X - vetor com as condições iniciais
#Retorno:
#pa - ponto do pé A
#pb - ponto do pé B
#pc - ponto do centro de massa
#M  - trajetória completa vetores x,y e z
#-----------------------------------------------------------
def trajetoria(X):
#-----------------------------------------------------------
#variáveis globais
#-----------------------------------------------------------
    glob = GlobalVariables()
    m    = glob.getM()    #massa
    L    = glob.getL()    #tamanho da perna
    g    = glob.getG()     #gravidade
    h    = glob.getH()   #passo para o calculo das derivadas
    hEdo = glob.getHEDO() #passo para calculo - EDO
    ts = glob.ts
    tdb = glob.tdbl
#-----------------------------------------------------------
#condições iniciais para MS (referentes ao CM)
#----------------------------------------------------------- 
    xod  = X[0,0]
    yod  = X[1,0]
    zod  = X[2,0]
    dxod = X[3,0]
    dyod = X[4,0]
    dzod = X[5,0]
#-----------------------------------------------------------
#valores para a otimização valores de u
#-----------------------------------------------------------  
    #u = [theta phi k Bss]

#-----------------------------------------------------------
#vetor com as condições iniciais MS
#-----------------------------------------------------------         
    y0 = np.array([[xod],[dxod],[yod],[dyod],[zod],[dzod]])

#-----------------------------------------------------------
#vetor com os parâmetros constantes
#-----------------------------------------------------------         
    params = np.array([[m],[L],[g]])
#-----------------------------------------------------------
#Parâmetros para os métodos 
#-----------------------------------------------------------           
    t = 0   #inicio do tempo  t = 0
    h = hEdo #passo do método rungekuttaLIMP inicial
    N = 10000 #número máximo de iterações
    #primeiro metodo
    sh = h #tamanho do passo para o método rungekuttaLIMP atualizando durante a execução do método
    
    ind_s = 0 #contador

    #traj = ind + 1 #tamanho do vetor com os pontos da trajetória
#-----------------------------------------------------------
#vetores auxiliares para guardar a trajetória
#-----------------------------------------------------------        
    px = [y0[0,0]]
    py = [y0[2,0]]
    pz = [y0[4,0]]
    vx = [y0[1,0]]
    vy = [y0[3,0]]
    vz = [y0[5,0]]
#y = np.zeros((6,1))
#-----------------------------------------------------------
#inicio do método 1 MS para TD
#----------------------------------------------------------- 
    for i in range(N): #for de 0 até N*h, com passo h
#-----------------------------------------------------------
#vetor de parâmetros
#-----------------------------------------------------------         
        var = np.array([[t],[h],[1]])
#-----------------------------------------------------------
#método numérico para solucionar as equações diferenciais
#passo a passo
#----------------------------------------------------------- 
        y = rungeKuttaLIPM(var,y0,params)
#-----------------------------------------------------------
#atualizando a condição inicial
#-----------------------------------------------------------         
        y0  = y
#-----------------------------------------------------------
#atualizando o instante t
#-----------------------------------------------------------                 
        t = t+sh
#-----------------------------------------------------------
#verificando a condição de parada posição Z < que Z de touchdown
#Z de touchdown = L*cos(theta)
#-----------------------------------------------------------                     
        if t >= ts:

            break 
        
#-----------------------------------------------------------
#colocando os valores nos vetores auxiliares
#-----------------------------------------------------------       
        else:    
            px.append(y0[0,0])
            py.append(y0[2,0])
            pz.append(y0[4,0])
            vx.append(y0[1,0])
            vy.append(y0[3,0])
            vz.append(y0[5,0])
#-----------------------------------------------------------
#atualizando o contador
#-----------------------------------------------------------   
            ind_s = ind_s + 1   
    
#-----------------------------------------------------------
#atualizando o contador - tratando o valor
#-----------------------------------------------------------   
    #if ind > 1:  #não preciso desse if, porque iniciei o contador no 0
       #ind = ind -1 #o Juan havia iniciado em 1
    #end
    
#-----------------------------------------------------------
#Posição do centro de massa no momento de  Touchdown (TD)
#-----------------------------------------------------------        
   
#-----------------------------------------------------------
#iniciando o segundo contador
#-----------------------------------------------------------
    ind_dbl = 0
    #traj2 = ind2 + 1
    sh = h #tamanho do passo para o método rungekuttaLIMP atualizando durante a execução do método
#-----------------------------------------------------------
#vetores auxiliares para guardar a trajetória
#-----------------------------------------------------------    
    px2 = [y0[0,0]]
    py2 = [y0[2,0]]
    pz2 = [y0[4,0]]
    vx2 = [y0[1,0]]
    vy2 = [y0[3,0]]
    vz2 = [y0[5,0]]

#-----------------------------------------------------------
#inicio do método 2  TD para LH
#-----------------------------------------------------------     
    for x in range(N):

#-----------------------------------------------------------
#vetor de parâmetros
#-----------------------------------------------------------    
        var = np.array([[t],[h],[2]])
#-----------------------------------------------------------
#método numérico para solucionar as equações diferenciais
#passo a passo
#-----------------------------------------------------------
        y = rungeKuttaLIPM(var,y0,params)
#-----------------------------------------------------------
#atualizando nova condição inicial
#-----------------------------------------------------------
        y0  = y
#-----------------------------------------------------------
#atualizando o instante t
#-----------------------------------------------------------
        t = t+sh
#-----------------------------------------------------------
#verificando a condição de parada posição dZ > 0
#-----------------------------------------------------------       
        #if v.all():
        if t >= tdb+ts:
            break
#-----------------------------------------------------------
#atualizando os vetores auxiliares da trajetória
#-----------------------------------------------------------
        else:
            px2.append(y0[0,0])
            py2.append(y0[2,0])
            pz2.append(y0[4,0])
            vx2.append(y0[1,0])
            vy2.append(y0[3,0])
            vz2.append(y0[5,0])
#-----------------------------------------------------------
#atualizando o contador
#-----------------------------------------------------------
            ind_dbl = ind_dbl+1
    
#-----------------------------------------------------------
#atualizando o contador - tratando o valor
#-----------------------------------------------------------
    #if ind2 > 1
     #   ind2 = ind2 -1;
    
#-----------------------------------------------------------
#trajetória do centro de massa CoM M = [x y z]
#-----------------------------------------------------------
# concatenando as listas, para preencher o vetor M
    pxTot = px + px2
    pyTot = py + py2
    pzTot = pz + pz2   
    pxTot = np.asarray(pxTot)
    pxTot = pxTot.reshape(-1,1)
    pyTot = np.asarray(pyTot)
    pyTot = pyTot.reshape(-1,1)
    pzTot = np.asarray(pzTot)
    pzTot = pzTot.reshape(-1,1)

    vxTot = vx + vx2
    vyTot = vy + vy2
    vzTot = vz + vz2   
    vxTot = np.asarray(vxTot)
    vxTot = vxTot.reshape(-1,1)
    vyTot = np.asarray(vyTot)
    vyTot = vyTot.reshape(-1,1)
    vzTot = np.asarray(vzTot)
    vzTot = vzTot.reshape(-1,1)
 
    M = np.concatenate((pxTot, pyTot, pzTot),axis = 1)
    M = M.reshape(-1,3)

    V = np.concatenate((vxTot, vyTot, vzTot),axis = 1)
    V = V.reshape(-1,3)


    
    return M, V, ind_s, ind_dbl, y0
