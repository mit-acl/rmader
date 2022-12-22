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
import sys

if __name__ == '__main__':

    # initialization
    stop_cnt_tol = 1e-2 # stop count torelance
    home_dir = "/media/kota/T7/rmader_ral/hw/rmader_obs"
    rmader_ave_dist = []
    rmader_stop_cnt = []
    tests = []
    tests.append("4agent2obs/test4")
    tests.append("4agent2obs/test5")
    tests.append("4agent2obs/test7")
    tests.append("full_space_6_agents/test10")
    tests.append("full_space_6_agents/test11") 
    # tests = [2] 

    for k, test in enumerate(tests):
        if k <= 2:
            num_of_agents = 4
        else:
            num_of_agents = 6 
        source_dir = home_dir + f"/{test}"           
        source_dir_len = len(source_dir)
        source_dir = home_dir + f"/{test}/*.bag"           
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
            for j in range(1, len(log.diff())):
                total_dist += np.linalg.norm(log.diff().to_numpy()[j,0:2])

            ###### max vel
            for j in range(len(log.to_numpy())):
                max_vel = max(max_vel, np.linalg.norm(log.to_numpy()[j,3:5]))
                if max_vel > 10:
                    print(max_vel)
                    print('rosbags ' + str(rosbags[i]))
                    sys.exit(1)
        ave_dist = total_dist / num_of_agents

        os.system(f'echo "{test}" >> '+home_dir+'/ave_dist.txt')
        os.system('echo "ave dist '+str(round(ave_dist,3))+'m" >> '+home_dir+'/ave_dist.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/ave_dist.txt')

        os.system(f'echo "{test}" >> '+home_dir+'/max_vel.txt')
        os.system('echo "max vel '+str(max_vel)+'" >> '+home_dir+'/max_vel.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/max_vel.txt')