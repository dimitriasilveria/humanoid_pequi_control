import numpy as np
from caminhada import caminhada
from globalVariables import GlobalVariables
from std_msgs.msg import Float64MultiArray
import rospy
glob = GlobalVariables()

rospy.init_node('walking_module')
m = glob.getM()
L = glob.getL()
g = glob.getG()
h = glob.getH()
hEdo = glob.getHEDO()
#global  m, L, g,  h, hEdo

U0 = np.zeros((5,1))

pub = rospy.Publisher('legs/angles',Float64MultiArray,queue_size=10)
rate = rospy.Rate(100)
#Darwin
xod  = 0.0
yod  = 0.035
zod  = 0.208
# dxod = 0.17
dxod = 0.0
dyod = -0.1
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



X01 = np.array([[xod],[yod],[zod],[dxod],[dyod],[dzod]]) #condição inicial

vecGanho1 = np.array([ganhoS1, ganhoQ1, ganhoR1, ganhoK1]).reshape((4,1))
vecGanho2 = np.array([ganhoS2, ganhoQ2, ganhoR2, ganhoK2]).reshape((4,1))
while not rospy.is_shutdown():
    caminhada(X01,vecGanho1,vecGanho2,pub)
    rate.sleep()
