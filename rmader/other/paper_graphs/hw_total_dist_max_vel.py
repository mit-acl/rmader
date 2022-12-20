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
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
import statistics
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager

if __name__ == '__main__':

    # initialization
    num_of_agents = 6
    stop_cnt_tol = 1e-2 # stop count torelance
    home_dir = "/media/kota/T7/rmader_ral/hw/rmader_mesh_6agents"
    rmader_ave_dist = []
    rmader_stop_cnt = []
    tests = [2, 3, 4, 5, 6] 
    # tests = [2] 

    for test in tests:
        source_dir = home_dir + f"/test{test}"           
        source_dir_len = len(source_dir)
        source_dir = home_dir + f"/test{test}/*.bag"           
        rosbag_list = glob.glob(source_dir)
        rosbag_list.sort() #alphabetically order
        rosbags = []

        for bag in rosbag_list:
            rosbags.append(bag)
        total_dist = 0.0
        max_vel_list = []
        max_vel = 0.0

        for i in range(len(rosbags)):
            print('rosbags ' + str(rosbags[i]))
            b = bagreader(rosbags[i], verbose=False)
            # introduced goal_reached topic so no need to check actual_traj
            agent = rosbags[i][source_dir_len+1:source_dir_len+5]
            log_data = b.message_by_topic(f'/{agent}/state')
            log = pd.read_csv(log_data, usecols=["pos.x", "pos.y", "pos.z", "vel.x", "vel.y", "vel.z"])
            log.columns = ["px", "py", "pz", "vx", "vy", "vz"]  
            
            ###### difference from the previous pos to the current pos
            # print(log.diff().to_numpy())
            diff_matrix = log.diff().to_numpy()

            # since the first row is NaN, starts 
            for i in range(1, len(log.diff())):
                total_dist += np.linalg.norm(log.diff().to_numpy()[i,0:2])

            ###### max vel
            for i in range(len(log.to_numpy())):
                max_vel = max(max_vel, np.linalg.norm(log.to_numpy()[i,3:5]))

        os.system(f'echo "test{test}" >> '+home_dir+'/total_dist.txt')
        os.system('echo "total travel dist '+str(round(total_dist,3))+'m" >> '+home_dir+'/total_dist.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/total_dist.txt')

        os.system(f'echo "test{test}" >> '+home_dir+'/max_vel.txt')
        os.system('echo "max vel '+str(max_vel)+'" >> '+home_dir+'/max_vel.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/max_vel.txt')