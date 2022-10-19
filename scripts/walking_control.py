#! /usr/bin/env python
import queue
import rospy
import numpy as np
from caminhada import caminhada
from globalVariables import GlobalVariables
from std_msgs.msg import Int16MultiArray, Float32
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

pub_theta = rospy.Publisher('arm/cmd_position',Int16MultiArray,queue_size=10)

rate = rospy.Rate(50)
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



X01 = np.array([[xod],[yod],[zod],[dxod],[dyod],[dzod]]) #condição inicial

vecGanho1 = np.array([ganhoS1, ganhoQ1, ganhoR1, ganhoK1]).reshape((4,1))
vecGanho2 = np.array([ganhoS2, ganhoQ2, ganhoR2, ganhoK2]).reshape((4,1))
# a = np.arange(-np.pi,np.pi,0.01)
# vec = 600*np.sin(a)
# print(np.size(a))
# i = 0
while not rospy.is_shutdown():
    caminhada(pub_theta)
    # angle.data = [int(vec[i]),int(vec[i])]
    # i = i + 1
    # print(vec[i])
    # pub.publish(angle)
    # if (i == np.size(vec)-1):
    #     i = 0
    rate.sleep()
