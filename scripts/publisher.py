#! /usr/bin/env python

import rospy
import numpy as np 
from std_msgs.msg import Float64
from globalVariables import GlobalVariables
import pandas as pd
import asyncio
import pandas as pd
from gazebo_msgs.msg import ModelStates

theta= Float64()
a = GlobalVariables()
  

rospy.init_node('angles') #iniciando o nó
#ang 1-6 perna direita e ang7-12 perna esquerda

Vl = np.array([0.0, 0.0, -0.61649654, 2*0.61649654, 0.61649654, 0.0]).reshape((6,1))
Vr = np.array([0.0, 0.0, 0.61649654, -2*0.61649654, -0.61649654, 0.0]).reshape((6,1))
# Vr = np.array([0.0, 0.0, -0.5147, 1.056, 0.0, -0.5147]).reshape((6,1)) #perna direita
# Vl = np.array([0.0, 0.0, -0.5147, 1.056, 0.0, -0.5147]).reshape((6,1)) #perna esquerda
# Vr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((6,1)) #perna direita
# Vl = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((6,1)) #perna esquerda
p1 = rospy.Publisher('/robotis_op3/r_hip_yaw_position/command',Float64,queue_size=200) # '/robotis_op3/r_hip_yaw_position/command' é o tópico onde a mensagem será publicada"
p2 = rospy.Publisher('/robotis_op3/r_hip_roll_position/command',Float64,queue_size=200)
p3 = rospy.Publisher('/robotis_op3/r_hip_pitch_position/command',Float64,queue_size=200)
p4 = rospy.Publisher('/robotis_op3/r_knee_position/command',Float64,queue_size=200)
p5 = rospy.Publisher('/robotis_op3/r_ank_pitch_position/command',Float64,queue_size=200)
p6 = rospy.Publisher('/robotis_op3/r_ank_roll_position/command',Float64,queue_size=200)
p7 = rospy.Publisher('/robotis_op3/l_hip_yaw_position/command',Float64,queue_size=200)
p8 = rospy.Publisher('/robotis_op3/l_hip_roll_position/command',Float64,queue_size=200)
p9 = rospy.Publisher('/robotis_op3/l_hip_pitch_position/command',Float64,queue_size=200)
p10 = rospy.Publisher('/robotis_op3/l_knee_position/command',Float64,queue_size=200)
p11 = rospy.Publisher('/robotis_op3/l_ank_pitch_position/command',Float64,queue_size=200)
p12= rospy.Publisher('/robotis_op3/l_ank_roll_position/command',Float64,queue_size=200)

rate = rospy.Rate(50)
right = pd.read_csv('thetaRight1.csv')
left = pd.read_csv('thetaLeft1.csv')
# Vr = right.iloc[1].to_numpy()
# Vl = left.iloc[1].to_numpy()
print(np.shape(right))
print(Vl)
k = 0


async def publish_right(Vr,k):

     theta.data = -Vr[0] #escrevendo Vr[0] em theta
     p1.publish(theta) #publicando theta
     theta.data = -Vr[1]
     p2.publish(theta)
     theta.data = -Vr[2]
     p3.publish(theta)
     theta.data = -Vr[3]
     p4.publish(theta)
     theta.data = Vr[4]
     p5.publish(theta)
     theta.data = Vr[5]
     p6.publish(theta)


async def publish_left(Vl,k):
     
     theta.data = -Vl[0]
     p7.publish(theta)
     theta.data = -Vl[1]
     p8.publish(theta)
     theta.data = Vl[2]
     p9.publish(theta)
     theta.data = Vl[3]
     p10.publish(theta)
     theta.data = -Vl[4]
     p11.publish(theta)
     theta.data = Vl[5]
     p12.publish(theta)

while not rospy.is_shutdown():
     # if k == np.size(right,axis=0):
     #      print('here')
     #      k = np.size(right,axis=0)-280
     # Vr = a.getThetaR()   #vetor perna direira p1-6
     # Vl = a.getThetaL()   #vetor perna esquerda p7-12
     # if k >5:
     #     Vl = np.zeros(6)
     #     Vr = np.zeros(6)
     Vr = right.iloc[k].to_numpy()
     Vl = left.iloc[k].to_numpy()
     loop = asyncio.get_event_loop()
     loop.run_until_complete(asyncio.gather(publish_left(Vl,k),publish_right(Vr,k)))

     k = k+1

     rate.sleep()
     # print(Vr,'vr')
     # print(Vl,'vl')
     # # for j in range(20,0,-1):
     # j=1
     # theta.data = Vr[0]/j #escrevendo Vr[0] em theta
     # p1.publish(theta) #publicando theta
     # theta.data = Vr[1]/j
     # p2.publish(theta)
     # theta.data = Vr[2]/j
     # p3.publish(theta)
     # theta.data = Vr[3]/j
     # print(theta)
     # p4.publish(theta)
     # theta.data = -Vr[4]/j
     # p5.publish(theta)
     # theta.data = Vr[5]/j
     # p6.publish(theta)
     # theta.data = Vl[0]/j
     # p7.publish(theta)
     # theta.data = -Vl[1]/j
     # p8.publish(theta)
     # theta.data = -Vl[2]/j
     # p9.publish(theta)
     # theta.data = -Vl[3]/j
     # p10.publish(theta)
     # print(theta,'left')
     # theta.data = Vl[4]/j
     # p11.publish(theta)
     # theta.data = Vl[5]/j
     # p12.publish(theta)


         #escrevendo Vr[0] em theta
    # for j in range(np.size(right)):
    #     theta.data = right[j,0]
    #     p1.publish(theta) #publicando theta
    #     print("publicando", right)
    #     theta.data = right[j,0]
    #     p2.publish(theta)

    #     theta.data = right[j,1]
    #     p3.publish(theta)

    #     theta.data = right[j,2]
    #     p4.publish(theta)

    #     theta.data = right[j,3]
    #     p5.publish(theta)

    #     theta.data = right[j,4]
    #     p6.publish(theta)

    #     theta.data = left[j,5]
    #     p7.publish(theta)

    #     theta.data = left[j,0]
    #     p8.publish(theta)

    #     theta.data = left[j,1]
    #     p9.publish(theta)

    #     theta.data = right[j,2]
    #     p10.publish(theta)

    #     theta.data = left[j,3]
    #     p11.publish(theta)

    #     theta.data = left[j,5]
    #     p12.publish(theta)
    
    




