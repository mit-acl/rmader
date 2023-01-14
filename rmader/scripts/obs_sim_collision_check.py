#!/usr/bin/env python

#Author: Kota Kondo
#Date: June 3. 2022

#you can get transformation btwn two agents and if they are too close (violating bbox) then it prints out warning
#the reason why we use tf instead of snapstack_msgs/State is two agents publish their states asynchrnonously and therefore comparing
#these states (with slightly different timestamp) is not accurate position comparison. Whereas tf always compares two states with the same time stamp

import math
import os
import sys
import time
import rospy
import rosgraph

from geometry_msgs.msg import PoseStamped
from snapstack_msgs.msg import State
from rmader_msgs.msg import Collision

import numpy as np
from random import *
import tf2_ros
from numpy import linalg as LA

class CollisionDetector:

    def __init__(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.sleep(3) #Important, if not it won't work

        # numerical tolerance
        # state is not synchronized and time difference could be up to 0.01[s] so we need tolerance
        # if max vel is 2.0m/s -> there should be 0.04m tolerance
        self.tol = 0.0 #[m] 

        # bbox size
        self.bbox_x = 0.05
        self.bbox_y = 0.05
        self.bbox_z = 0.05

        self.initialized = True

        # publisher init
        self.collision=Collision()
        self.pubIsCollided = rospy.Publisher('is_collided', Collision, queue_size=1, latch=True)

        # agent and obs
        self.agent_obs_list = []
        for i in range(1,11):
            if i<=9:
                agent = "SQ0" + str(i) + "s" 
            else:
                agent = "SQ" + str(i) + "s"
            self.agent_obs_list.append(agent)
        for j in range(4000, 4010):
            self.agent_obs_list.append(f"obs{j}")

    # collision detection
    def collisionDetect(self, timer):

        if self.initialized:
            for agent1 in self.agent_obs_list:
                for agent2 in self.agent_obs_list:
                    if agent1 == agent2 or (agent1[:3] == "obs" and agent2[:3] == "obs"):
                        break

                    trans = self.get_transformation(agent1, agent2)
                    # if agent1 == "SQ06s" or agent2 == "SQ06s":
                    #     print(agent1 + " and " + agent2)
                    #     print(trans.transform.translation.x)
                    #     print(trans.transform.translation.y)
                    #     print(trans.transform.translation.z)

                    if trans is not None:

                        if (abs(trans.transform.translation.x) < self.bbox_x
                            and abs(trans.transform.translation.y) < self.bbox_y
                            and abs(trans.transform.translation.z) < self.bbox_z):
                        
                            print("collision btwn " + agent1 + " and " + agent2)

                            x_diff = abs(trans.transform.translation.x)
                            y_diff = abs(trans.transform.translation.y)
                            z_diff = abs(trans.transform.translation.z)

                            # print("difference in x is " + str(x_diff))
                            # print("difference in y is " + str(y_diff))
                            # print("difference in z is " + str(z_diff))

                            # max_dist = max(abs(trans.transform.translation.x), abs(trans.transform.translation.y), abs(trans.transform.translation.z))
                            max_dist = max(x_diff, y_diff, z_diff)

                            self.collision.is_collided = True
                            self.collision.agent1 = agent1
                            self.collision.agent2 = agent2

                            print("violation dist is " + str(max_dist))
                            print("\n")

                            self.collision.dist = max_dist 
                            self.pubIsCollided.publish(self.collision)

    def get_transformation(self, source_frame, target_frame):
        try:
            transformation = self.tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(0.01))
            return transformation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

def startNode():
    c = CollisionDetector()
    rospy.Timer(rospy.Duration(0.001), c.collisionDetect)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('CollisionDetector')
    startNode()