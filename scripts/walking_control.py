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
