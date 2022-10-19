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
published_topic = rospy.get_param('~published_topic',default = 'arm/cmd_position')

pub_theta = rospy.Publisher(published_topic,Int16MultiArray,queue_size=10)

rate = rospy.Rate(50)
#Darwin

# a = np.arange(-np.pi,np.pi,0.01)
# vec = 600*np.sin(a)
# print(np.size(a))
# i = 0
while not rospy.is_shutdown():
    caminhada(pub_theta)

    rate.sleep()
