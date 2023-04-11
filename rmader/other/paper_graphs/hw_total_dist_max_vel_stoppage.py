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
    stop_cnt_tol = 0.1 # stop count torelance
    home_dir = "/media/kota/T7/rmader_ral/hw/rmader_obs"
    rmader_ave_dist = []
    rmader_stop_cnt = []
    tests = []
    # # tests_num = [2, 17, 19, 24, 30] 
    # tests_num = [2, 3, 4, 5, 6] 
    # for test_num in tests_num:
    #     tests.append(f"test{test_num}")

    tests.append("2agent1obs/test1") 
    # tests.append("2agent1obs/test2") 
    # tests.append("4agent2obs/test4")
    # tests.append("4agent2obs/test5")
    # tests.append("4agent2obs/test7")
    # tests.append("6agent2obs/test10")
    # tests.append("6agent2obs/test11") 
    # tests.append("6agent2obs/test3")
    print(tests)

    for k, test in enumerate(tests):
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
        total_stop_duration = 0.0

        for i in range(len(rosbags)):
            num_of_agents = int(rosbags[i][len(home_dir)+1:len(home_dir)+2]) # for rmader_obs
            # num_of_agents = 6
            print('rosbag ', str(rosbags[i]))
            b = bagreader(rosbags[i], verbose=False)
            # introduced goal_reached topic so no need to check actual_traj
            agent = rosbags[i][source_dir_len+1:source_dir_len+5]
            # try:
            #     log_data_twist = b.message_by_topic(f'/{agent}/mocap/twist')
            #     log_twist = pd.read_csv(log_data_twist, usecols=["header.stamp.secs", "header.stamp.nsecs", "twist.linear.x", "twist.linear.y", "twist.linear.z"])
            # except:
            log_data_twist = b.message_by_topic(f'/{agent}/state')
            log_twist = pd.read_csv(log_data_twist, usecols=["header.stamp.secs", "header.stamp.nsecs", "vel.x", "vel.y", "vel.z"])
            log_twist.columns = ["secs", "nsecs", "vx", "vy", "vz"]

            log_data_world = b.message_by_topic(f'/{agent}/world')
            log_world = pd.read_csv(log_data_world, usecols=["pose.position.x", "pose.position.y", "pose.position.z"])
            log_world.columns = ["px", "py", "pz"]

            log_data_setpoint = b.message_by_topic(f'/{agent}/setpoint')
            log_setpoint = pd.read_csv(log_data_setpoint, usecols=["header.stamp.secs", "header.stamp.nsecs"])
            log_setpoint.columns = ["secs", "nsecs"]

            log_data_trajs = b.message_by_topic(f'/{agent}/rmader/trajs')
            log_trajs = pd.read_csv(log_data_trajs, usecols=["header.stamp.secs", "header.stamp.nsecs"])
            log_trajs.columns = ["secs", "nsecs"]

            ###### difference from the previous pos to the current pos
            # print(log.diff().to_numpy())
            diff_matrix = log_world.diff().to_numpy()

            # since the first row is NaN, starts at 1
            for j in range(1, len(log_world.diff())):
                total_dist += np.linalg.norm(log_world.diff().to_numpy()[j,0:2])

            ###### goal reached time for stop count (more like the last committed traj timestamp but it's accurate representation for stop count)
            goal_reached_time = log_trajs.to_numpy()[-1,0] + log_trajs.to_numpy()[-1,1] / 1e9
            print("goal_reached_time", goal_reached_time)
            ###### stop count
            start_time = log_setpoint.to_numpy()[0,0] + log_setpoint.to_numpy()[0,1] / 1e9
            stop_duration = 0.0
            initialized = False
            stopped = False

            for j in range(len(log_twist.to_numpy())):
                current_time = log_twist.to_numpy()[j,0] + log_twist.to_numpy()[j,1] / 1e9
                vel_norm = np.linalg.norm(log_twist.to_numpy()[j,2:5])
                
                if current_time > goal_reached_time:
                    print('goal reached!')
                    break
                # if current_time > start_time + 60:
                #     break

                if not initialized and current_time > start_time:
                    print('initialized!')
                    initialized = True
                if initialized:
                    if vel_norm > stop_cnt_tol and stopped:
                        stop_duration += current_time - stop_duration_start
                        stopped = False
                    elif vel_norm < stop_cnt_tol and not stopped:
                        stop_duration_start = log_twist.to_numpy()[j,0] + log_twist.to_numpy()[j,1] / 1e9
                        stopped = True

            total_stop_duration += stop_duration

            ###### max vel
            for j in range(len(log_twist.to_numpy())):
                max_vel = max(max_vel, np.linalg.norm(log_twist.to_numpy()[j,2:5]))
                if max_vel > 8.0:
                    print("max velocity is too large: ", max_vel, "abort")
                    print('rosbags ' + str(rosbags[i]))
                    # sys.exit(1)

        ave_dist = total_dist / num_of_agents
        ave_stop_duration = total_stop_duration / num_of_agents

        os.system(f'echo "{test}" >> '+home_dir+'/ave_dist.txt')
        os.system('echo "ave dist '+str(round(ave_dist,3))+'m" >> '+home_dir+'/ave_dist.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/ave_dist.txt')

        os.system(f'echo "{test}" >> '+home_dir+'/max_vel.txt')
        os.system('echo "max vel '+str(max_vel)+'" >> '+home_dir+'/max_vel.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/max_vel.txt')

        os.system(f'echo "{test}" >> '+home_dir+'/stop_duration.txt')
        os.system('echo "stop duration '+str(round(ave_stop_duration,3))+'" >> '+home_dir+'/stop_duration.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/stop_duration.txt')