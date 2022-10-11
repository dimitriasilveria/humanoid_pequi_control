#--------------------------------------
#Método para calcular o produto escalar 
#entre dois quatérnio dual
#Parâmetros: 
#q: dual quatérnio
#p: dual quatérnio
#Retorno:
#mag: produto escalar entre os quatérnios
#--------------------------------------
def dualQuatDot(q,p):
    mag = q[0,0]*p[0,0] + q[1,0]*p[1,0] + q[2,0]*p[2,0] + q[3,0]*p[3,0] 
    return mag