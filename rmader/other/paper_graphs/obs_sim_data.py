#!/usr/bin/env python  
# Kota Kondo

import bagpy
from bagpy import bagreader
import pandas as pd
import rosbag
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import tf
import tf2_ros
import tf2_py
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
import statistics
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
import sys

if __name__ == '__main__':

    # initialization
    home_dir = "/media/kota/T7/rmader_ral/mader/rmader_obs"
    num_of_agents = 10
    source_dir = home_dir+"/bags/cd50ms/dc125ms"           
    source_dir_len = len(source_dir)
    source_dir = home_dir+"/bags/cd50ms/dc125ms/*.bag"           
    rosbag_list = glob.glob(source_dir)
    rosbag_list.sort() #alphabetically order
    rosbags = []

    for bag in rosbag_list:
        rosbags.append(bag)
    
    # collision when not stopped
    collision_cnt_not_stopped = 0
    # collision when stopped
    collision_cnt_stopped = 0
    # total dist
    total_dist = 0.0
    # travel time
    travel_time = 0.0
    # stop count
    stop_cnt = 0.0

    for bag in rosbags:
        print('rosbag ' + str(bag))
        tf_buffer = tf2_py.BufferCore(rospy.Duration(1000))
        print('hi')
        bag = rosbag.Bag(bag)
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            print(topic)
            print(msg)
            
            for msg_tf in msg.transforms:
                tf_buffer.set_transform("SQ01s", "SQ02s")
            print(tf_buffer)

    ave_dist = total_dist / num_of_agents

    os.system(f'echo "{test}" >> '+home_dir+'/ave_dist.txt')
    os.system('echo "ave dist '+str(round(ave_dist,3))+'m" >> '+home_dir+'/ave_dist.txt')
    os.system('echo "------------------------------------------------------------" >> '+home_dir+'/ave_dist.txt')

    os.system(f'echo "{test}" >> '+home_dir+'/max_vel.txt')
    os.system('echo "max vel '+str(max_vel)+'" >> '+home_dir+'/max_vel.txt')
    os.system('echo "------------------------------------------------------------" >> '+home_dir+'/max_vel.txt')