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

import csv 
import pandas as pd
import matplotlib.pyplot as plt

class DistVel:

    def __init__(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.sleep(3) #Important, if not it won't work

        self.num_of_agents = 6
        self.initialized = True
        # self.initialized_mat = [False for i in range(self.num_of_agents)]

        self.state_pos = np.empty([self.num_of_agents,3])
        self.state_vel = np.empty([self.num_of_agents,3])

        self.t2 = 0
        self.t3 = 0
        self.t4 = 0
        self.t5 = 0

        self.rel_pos_2to3 = []
        self.rel_vel_2to3 = []
        self.rel_pos_4to5 = []
        self.rel_vel_4to5 = []
        self.rel_pos_4to6 = []
        self.rel_vel_4to6 = []
        self.rel_pos_5to6 = []
        self.rel_vel_5to6 = []

        self.t2_list = []
        self.t3_list = []
        self.t4_list = []
        self.t5_list = []

    def myhook(self):

        fig = plt.figure()
        ax = fig.add_subplot()
        plt.plot(self.t2_list, self.rel_pos_2to3, label='pos NX04 to 05', color='red')
        plt.plot(self.t2_list, self.rel_vel_2to3, label='vel NX04 to 05', color='green')
        # plt.plot(self.t4_list, self.rel_pos_4to6, label='pos NX04 to 06', color='green')
        # plt.plot(self.t5_list, self.rel_pos_5to6, label='pos NX05 to 06', color='blue')
        ax.legend()
        ax.set_xticks(np.arange(20,75,5))
        plt.xlim([20,75])
        plt.grid(axis='y', color='black', alpha=0.2)
        plt.ylabel('Relative Position')
        plt.xlabel('Time [s]')
        plt.savefig("/home/kota/data/rmader/rmader_decentr/comm_delay_test2and3/rel_pos_vel.png", bbox_inches="tight")
        plt.close()

        # fig = plt.figure()
        # plt.plot(self.t2_list, self.rel_vel_2to3, label='vel NX04 to 05', color='red')
        # # plt.plot(self.t4_list, self.rel_vel_4to6, label='vel NX04 to 06', color='green')
        # # plt.plot(self.t5_list, self.rel_vel_5to6, label='vel NX05 to 06', color='blue')
        # ax.legend()
        # plt.grid(axis='y', color='black', alpha=0.2)
        # plt.ylabel('Relative Velocity')
        # plt.xlabel('Time [s]')
        # plt.savefig("/home/kota/data/rmader/rmader_decentr/rel_vel.png", bbox_inches="tight")
        # plt.close()

    # relative dist and vel
    def relativeDistVel(self, timer):
    
        # print(LA.norm(self.state_pos[4-1,0:3] - self.state_pos[5-1,0:3]))
        self.rel_pos_2to3.append(LA.norm(self.state_pos[2-1,0:3] - self.state_pos[3-1,0:3]))
        self.rel_vel_2to3.append(LA.norm(self.state_vel[2-1,0:3] - self.state_vel[3-1,0:3]))

        self.rel_pos_4to5.append(LA.norm(self.state_pos[4-1,0:3] - self.state_pos[5-1,0:3]))
        self.rel_vel_4to5.append(LA.norm(self.state_vel[4-1,0:3] - self.state_vel[5-1,0:3]))

        self.rel_pos_4to6.append(LA.norm(self.state_pos[4-1,0:3] - self.state_pos[6-1,0:3]))
        self.rel_vel_4to6.append(LA.norm(self.state_vel[4-1,0:3] - self.state_vel[6-1,0:3]))
        
        self.rel_pos_5to6.append(LA.norm(self.state_pos[5-1,0:3] - self.state_pos[6-1,0:3]))
        self.rel_vel_5to6.append(LA.norm(self.state_vel[5-1,0:3] - self.state_vel[6-1,0:3]))
        
        self.t2_list.append(self.t2 - 1666380391.85826376)
        self.t3_list.append(self.t3)
        self.t4_list.append(self.t4)
        self.t5_list.append(self.t5)


    # def NX01stateCB(self, data):
    #     self.state_pos[0,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    #     self.state_vel[0,0:3] = np.array([data.vel.x, data.vel.y, data.vel.z])
    #     if self.initialized_mat[0] == False and LA.norm(self.state_pos[0,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[0] = True
    def NX02stateCB(self, data):
        self.state_pos[1,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        self.state_vel[1,0:3] = np.array([data.vel.x, data.vel.y, data.vel.z])
        self.t2 = data.header.stamp.secs + data.header.stamp.nsecs/10**9
        # if self.initialized_mat[1] == False and LA.norm(self.state_pos[1,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
        #     self.initialized_mat[1] = True
    def NX03stateCB(self, data):
        self.state_pos[2,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        self.state_vel[2,0:3] = np.array([data.vel.x, data.vel.y, data.vel.z])
        self.t3 = data.header.stamp.secs + data.header.stamp.nsecs/10**9
        # if self.initialized_mat[2] == False and LA.norm(self.state_pos[2,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
        #     self.initialized_mat[2] = True
    # def NX04stateCB(self, data):
    #     self.state_pos[3,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    #     self.state_vel[3,0:3] = np.array([data.vel.x, data.vel.y, data.vel.z])
    #     self.t4 = data.header.stamp.secs + data.header.stamp.nsecs/10**9
    #     # if self.initialized_mat[3] == False and LA.norm(self.state_pos[3,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #     #     self.initialized_mat[3] = True
    # def NX05stateCB(self, data):
    #     self.state_pos[4,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    #     self.state_vel[4,0:3] = np.array([data.vel.x, data.vel.y, data.vel.z])
    #     self.t5 = data.header.stamp.secs + data.header.stamp.nsecs/10**9
    #     # if self.initialized_mat[4] == False and LA.norm(self.state_pos[4,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #     #     self.initialized_mat[4] = True
    # def NX06stateCB(self, data):
    #     self.state_pos[5,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    #     self.state_vel[5,0:3] = np.array([data.vel.x, data.vel.y, data.vel.z])
    #     # if self.initialized_mat[5] == False and LA.norm(self.state_pos[5,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #     #     self.initialized_mat[5] = True
    # # def NX07stateCB(self, data):
    #     self.state_pos[6,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    #     if self.initialized_mat[6] == False and LA.norm(self.state_pos[6,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[6] = True
    # def NX08stateCB(self, data):
    #     self.state_pos[7,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    #     if self.initialized_mat[7] == False and LA.norm(self.state_pos[7,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[7] = True
    # def NX09stateCB(self, data):
    #     self.state_pos[8,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    #     if self.initialized_mat[8] == False and LA.norm(self.state_pos[8,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[8] = True
    # def NX10stateCB(self, data):
    #     self.state_pos[9,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    #     if self.initialized_mat[9] == False and LA.norm(self.state_pos[9,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[9] = True
    

    def get_transformation(self, source_frame, target_frame):

        # get the tf at first available time
        try:
            transformation = self.tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(0.01))
            return transformation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            # rospy.logerr("Unable to find the transformation")

def startNode():
    c = DistVel()
    # rospy.Subscriber("/NX01/state", State, c.NX01stateCB)
    rospy.Subscriber("/NX02/state", State, c.NX02stateCB)
    rospy.Subscriber("/NX03/state", State, c.NX03stateCB)
    # rospy.Subscriber("/NX04/state", State, c.NX04stateCB)
    # rospy.Subscriber("/NX05/state", State, c.NX05stateCB)
    # rospy.Subscriber("/NX06/state", State, c.NX06stateCB)
    # rospy.Subscriber("/NX07/state", State, c.NX07stateCB)
    # rospy.Subscriber("/NX08/state", State, c.NX08stateCB)
    # rospy.Subscriber("/NX09/state", State, c.NX09stateCB)
    # rospy.Subscriber("/NX10/state", State, c.NX10stateCB)
    rospy.Timer(rospy.Duration(0.01), c.relativeDistVel)
    rospy.spin()
    rospy.on_shutdown(c.myhook)

if __name__ == '__main__':
    rospy.init_node('RelativeDistVel')
    startNode()