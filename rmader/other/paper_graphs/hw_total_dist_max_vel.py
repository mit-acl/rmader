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
    # tests_num = [2, 3, 4, 5, 6] 
    # for test_num in tests_num:
    #     tests.append(f"test{test_num}")
    # print(tests)
    # tests.append("4agent2obs/test4")
    # tests.append("4agent2obs/test5")
    # tests.append("4agent2obs/test7")
    tests.append("6agent2obs/test10")
    tests.append("6agent2obs/test11") 
    tests.append("6agent2obs/test3") 
    # tests = [2] 

    for k, test in enumerate(tests):
        num_of_agents = 6 
        source_dir = home_dir + f"/{test}"           
        source_dir_len = len(source_dir)
        source_dir = home_dir + f"/{test}/*.bag"           
        rosbag_list = glob.glob(source_dir)
        rosbag_list.sort() #alphabetically order
        print(rosbag_list)
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
            try:
                log_data_twist = b.message_by_topic(f'/{agent}/mocap/twist')
                log_twist = pd.read_csv(log_data_twist, usecols=["twist.linear.x", "twist.linear.y", "twist.linear.z"])
                log_twist.columns = ["vx", "vy", "vz"]  
            except:
                log_data_twist = b.message_by_topic(f'/{agent}/state')
                log_twist = pd.read_csv(log_data_twist, usecols=["vel.x", "vel.y", "vel.z"])
                log_twist.columns = ["vx", "vy", "vz"]

            log_data_world = b.message_by_topic(f'/{agent}/world')
            log_world = pd.read_csv(log_data_world, usecols=["pose.position.x", "pose.position.y", "pose.position.z"])
            log_world.columns = ["px", "py", "pz"]

            ###### difference from the previous pos to the current pos
            # print(log.diff().to_numpy())
            diff_matrix = log_world.diff().to_numpy()

            # since the first row is NaN, starts 
            for j in range(1, len(log_world.diff())):
                total_dist += np.linalg.norm(log_world.diff().to_numpy()[j,0:2])

            ###### max vel
            for j in range(len(log_twist.to_numpy())):
                max_vel = max(max_vel, np.linalg.norm(log_twist.to_numpy()[j,0:2]))
                if max_vel > 3.46:
                    print("max velocity is too large: ", max_vel, "abort")
                    print('rosbags ' + str(rosbags[i]))
                    # sys.exit(1)
        ave_dist = total_dist / num_of_agents

        os.system(f'echo "{test}" >> '+home_dir+'/ave_dist.txt')
        os.system('echo "ave dist '+str(round(ave_dist,3))+'m" >> '+home_dir+'/ave_dist.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/ave_dist.txt')

        os.system(f'echo "{test}" >> '+home_dir+'/max_vel.txt')
        os.system('echo "max vel '+str(max_vel)+'" >> '+home_dir+'/max_vel.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/max_vel.txt')