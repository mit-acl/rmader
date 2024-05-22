#!/usr/bin/env python

import math
import os
import time
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped
from snapstack_msgs.msg import State
import numpy as np
from random import * 

class TermGoalSender:

    def __init__(self):

        # params
        self.dist_limit = 0.3

        # term_goal init
        self.term_goal=PoseStamped()
        self.term_goal.header.frame_id='world'
        self.pubTermGoal = rospy.Publisher('term_goal', PoseStamped, queue_size=1, latch=True)
        
        # state_pos init
        self.state_pos=np.array([0.0, 0.0, 0.0])

        # define waypoints in highbay
        self.wps = np.array([[-3.8+2.7*0, -3.3],
                            [-3.8+2.7*0, +3.6],
                            [-3.8+2.7*1, +3.6],
                            [-3.8+2.7*1, -3.3],
                            [-3.8+2.7*2, -3.3],
                            [-3.8+2.7*2, +3.6],
                            [4.3, +3.6],
                            [4.3, +3.0],
                            [4.0, -3.3],
                            [-3.8+2.7*3, +3.6],
                            [-3.8+2.7*3, -3.3]])
        self.wps_idx = 0

        # every 0.01 sec timerCB is called back
        self.is_change_goal = True
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timerCB)

        # send goal
        self.sendGoal()

        # init_pos
        self.is_init_pos = False

    def timerCB(self, event):

        # term_goal in array form
        self.term_goal_pos=np.array([self.term_goal.pose.position.x,self.term_goal.pose.position.y,self.term_goal.pose.position.z])

        # distance
        dist=np.linalg.norm(self.term_goal_pos-self.state_pos)

        # check distance and if it's close enough publish new term_goal
        if dist < self.dist_limit:
            self.sendGoal()

    def sendGoal(self):

        self.term_goal.pose.position.x = self.wps[self.wps_idx][0]
        self.term_goal.pose.position.y = self.wps[self.wps_idx][1]
        self.term_goal.pose.position.z = 2.0
        self.wps_idx = (self.wps_idx + 1) % len(self.wps)
        self.pubTermGoal.publish(self.term_goal)

        return

    def stateCB(self, data):
        if not self.is_init_pos:
            self.init_pos = np.array([data.pos.x, data.pos.y, data.pos.z])
            self.is_init_pos = True

        self.state_pos = np.array([data.pos.x, data.pos.y, data.pos.z])

def startNode():
    c = TermGoalSender()
    rospy.Subscriber("state", State, c.stateCB)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('TermGoalSender')
    startNode()
