#! /usr/bin/env python

from humanoid_pequi_control.scripts.caminhada import caminhada
import rospy

from std_msg.msg import Float64
from caminhada import caminhada

import actionlib

import humanoid_pequi_control.msg

global current_distance

class WalkingAction(object):
    # create messages that are used to publish feedback/result
    _feedback = humanoid_pequi_control.msg.WalkingFeedback()
    _result = humanoid_pequi_control.msg.WalkingResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, humanoid_pequi_control.msg.WalkingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      

    def __callback__(self,data):
        global current_distance
        current_distance = data.data

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating Walking sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        rospy.Subscriber("/control/routine", Float64, self.callback)

        diff= current_distance - goal.initial_position

        while diff < goal.distance:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            caminhada(X0=0,vecGanho1=0,vecGanho2=0,pub=0)
        
        success = True
        
        

        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
       


if __name__ == '__main__':
    rospy.init_node('Walking')
    server = WalkingAction(rospy.get_name())
    rospy.spin()
