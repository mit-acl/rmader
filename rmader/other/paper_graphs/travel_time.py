#!/usr/bin/env python  
# Kota Kondo

#  python collision_check.py

import bagpy
from bagpy import bagreader
import pandas as pd
import sys
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
    num_of_agents = 10
    stop_cnt_tol = 1e-3 # stop count torelance
    home_dir = "/media/kota/T7/rmader_ral"
    methods = ["oldmader", "rmader", "wo_check_rmader", "ego_swarm", "edg_team"]
    oldmader_ave_tt = [7.81, 7.74, 7.77, 7.74, 7.67] #travel time = tt
    oldmader_max_tt = [9.29, 8.83, 8.85, 9.35, 8.62]
    oldmader_min_tt = [7.29, 7.22, 7.22, 7.19, 7.22]
    rmader_ave_tt = [9.65, 10.54, 10.91, 9.88, 10.8]
    rmader_max_tt = [14.73, 15.47, 15.84, 14.99, 16.12]
    rmader_min_tt = [7.26, 7.8, 8.23, 7.93, 8.92]
    wo_check_rmader_ave_tt = [10.47, 14.54, 11.44, 13.04, 16.02]
    wo_check_rmader_max_tt = [14.84, 26.47, 19.06, 19.62, 8.9]
    wo_check_rmader_min_tt = [7.44, 7.71, 8.1, 8.9, 10.49]
    ego_swarm_ave_tt = [5.64, 5.56, 5.39, 5.31, 5.31]
    ego_swarm_max_tt = [8.73, 8.83, 8.72, 8.12, 9.02]
    ego_swarm_min_tt = [4.94, 4.94, 5.02, 5.03, 5.01]
    edg_team_ave_tt = [13.61, 13.25, 13.76, 15.99, 15.69]
    edg_team_max_tt = [21.29, 27.77, 25.68, 37.06, 35.89]
    edg_team_min_tt = [10.87, 10.74, 11.27, 11.71, 12.64]

    # for method in methods: 
    #     cd_list = [0, 50, 100, 200, 300]
    #     for cd in cd_list:
    #         if cd == 0:
    #             dc = 75 
    #         elif cd == 50:
    #             dc = 125 
    #         elif cd == 100:
    #             dc = 175
    #         elif cd == 200:
    #             dc = 250
    #         elif cd == 300:
    #             dc = 350

    #         if method == "oldmader":
    #             # old mader
    #             source_dir = f"/media/kota/T7/rmader_ral/mader/oldmader/bags/cd{cd}ms/*.bag"
    #             first_agent_idx = 1
    #             dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
    #         elif method == "rmader":
    #             # rmader
    #             source_dir = f"/media/kota/T7/rmader_ral/mader/rmader/bags/cd{cd}ms/dc{dc}ms/*.bag"
    #             first_agent_idx = 1
    #             dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
    #         elif method == "wo_check_rmader":
    #             # rmader without check
    #             source_dir = f"/media/kota/T7/rmader_ral/mader/wo_check_rmader/bags/cd{cd}ms/dc{dc}ms/*.bag"
    #             first_agent_idx = 1
    #             dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
    #         elif method == "ego_swarm":
    #             # EGO-Swarm
    #             source_dir = f"/media/kota/T7/rmader_ral/ego_swarm/bags/cd{cd}ms/*.bag"
    #             first_agent_idx = 0
    #             dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
    #         elif method == "edg_team":
    #             # EDG-Team
    #             source_dir = f"/media/kota/T7/rmader_ral/edg_team/bags/cd{cd}ms/*.bag"
    #             first_agent_idx = 0
    #             dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms

    #         rosbag_list = glob.glob(source_dir)
    #         rosbag_list.sort() #alphabetically order
    #         rosbags = []

    #         for bag in rosbag_list:
    #             rosbags.append(bag)
    #         print("rosbag", rosbag)
    #         # read ros bags
    #         completion_time_per_sim_list = []
    #         for i in range(len(rosbags)):
    #             print('rosbag ' + str(rosbags[i]))
    #             start_time_initialized = False
    #             finish_time_initialized = False
    #             start_time = 0.0
    #             finish_time = 0.0
    #             for topic, msg, t in  rosbag.Bag(rosbags[i]).read_messages():
    #                 if method == "ego_swarm" or method == "edg_team":
    #                     if topic == '/drone_0_planning/pos_cmd': # for rmader, i didn't record goal
    #                         if not start_time_initialized:
    #                             start_time = t.secs + t.nsecs/1e9
    #                             print("start_time", start_time)
    #                             start_time_initialized = True
    #                     if topic == '/goal_reached':
    #                         if not finish_time_initialized:
    #                             finish_time = t.secs + t.nsecs/1e9
    #                             print("finish_time", finish_time)
    #                             finish_time_initialized = True
    #                 elif method == "rmader" or method == "wo_check_rmader" or method == "oldmader":
    #                     if topic == '/SQ01s/rmader/actual_traj': # for rmader, i didn't record goal
    #                         if not start_time_initialized:
    #                             start_time = t.secs + t.nsecs/1e9
    #                             print("start_time", start_time)
    #                             start_time_initialized = True
    #                     if topic == '/goal_reached':
    #                         if not finish_time_initialized:
    #                             finish_time = t.secs + t.nsecs/1e9
    #                             print("finish_time", finish_time)
    #                             finish_time_initialized = True

    #             completion_time = finish_time - start_time
    #             print("completion_time", completion_time)
    #             completion_time_per_sim_list.append(completion_time)

    #         os.system('echo "'+source_dir+'" >> '+home_dir+'/completion_time.txt')
    #         os.system('echo "ave is '+str(round(statistics.mean(completion_time_per_sim_list),2))+'s" >> '+home_dir+'/completion_time.txt')
    #         os.system('echo "min is '+str(round(min(completion_time_per_sim_list),2))+'s" >> '+home_dir+'/completion_time.txt')
    #         os.system('echo "max is '+str(round(max(completion_time_per_sim_list),2))+'s" >> '+home_dir+'/completion_time.txt')
    #         os.system('echo "------------------------------------------------------------" >> '+home_dir+'/completion_time.txt')

    #         if method == "oldmader":
    #             # old mader
    #             oldmader_ave_tt.append(statistics.mean(completion_time_per_sim_list))
    #             oldmader_min_tt.append(min(completion_time_per_sim_list))
    #             oldmader_max_tt.append(max(completion_time_per_sim_list))
    #         elif method == "rmader":
    #             # rmader
    #             rmader_ave_tt.append(statistics.mean(completion_time_per_sim_list))
    #             rmader_min_tt.append(min(completion_time_per_sim_list))
    #             rmader_max_tt.append(max(completion_time_per_sim_list))
    #         elif method == "wo_check_rmader":
    #             # rmader without check
    #             wo_check_rmader_ave_tt.append(statistics.mean(completion_time_per_sim_list))
    #             wo_check_rmader_min_tt.append(min(completion_time_per_sim_list))
    #             wo_check_rmader_max_tt.append(max(completion_time_per_sim_list))
    #         elif method == "ego_swarm":
    #             # EGO-Swarm
    #             ego_swarm_ave_tt.append(statistics.mean(completion_time_per_sim_list))
    #             ego_swarm_min_tt.append(min(completion_time_per_sim_list))
    #             ego_swarm_max_tt.append(max(completion_time_per_sim_list))
    #         elif method == "edg_team":
    #             # EDG-Team
    #             edg_team_ave_tt.append(statistics.mean(completion_time_per_sim_list))
    #             edg_team_min_tt.append(min(completion_time_per_sim_list))
    #             edg_team_max_tt.append(max(completion_time_per_sim_list))
    
    # x axis
    x = [0, 50, 100, 200, 300]

    # font
    font = font_manager.FontProperties()
    font.set_family('serif')
    plt.rcParams.update({"text.usetex": True})
    plt.rcParams["font.family"] = "Times New Roman"
    # font.set_name('Times New Roman')
    font.set_size(16)
                
    # completion time
    fig, ax = plt.subplots()
    ax.plot(x, oldmader_ave_tt, color='b', linewidth=1.5, marker = 'o', label='MADER')
    ax.plot(x, rmader_ave_tt, color='r', linewidth=1.5, marker = 'o', label='RMADER')
    ax.plot(x, wo_check_rmader_ave_tt, color='purple', linewidth=1.5, marker = 'o', label='RMADER w/o check')
    ax.plot(x, ego_swarm_ave_tt, color='g', linewidth=1.5, marker = 'o', label='EGO-Swarm')
    ax.plot(x, edg_team_ave_tt, color='orange', linewidth=1.5, marker = 'o', label='EDG-Team')
    plt.fill_between(x, oldmader_max_tt, oldmader_min_tt, color='b', alpha=0.2)
    plt.fill_between(x, rmader_max_tt, rmader_min_tt, color='r', alpha=0.2)
    plt.fill_between(x, wo_check_rmader_max_tt, wo_check_rmader_min_tt, color='purple', alpha=0.2)
    plt.fill_between(x, ego_swarm_max_tt, ego_swarm_min_tt, color='g', alpha=0.2)
    plt.fill_between(x, edg_team_max_tt, edg_team_min_tt, color='orange', alpha=0.2)
    ax.set_xticks(np.arange(0,300+50,50))
    ax.set_xticklabels(np.arange(0,300+50,50), fontproperties=font)
    # ax.set_yticks(np.arange(6,19,1))
    # ax.set_yticklabels(np.arange(6,19,1), fontproperties=font)
    ax.set_ylabel("Travel Time [s]", fontproperties=font)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    plt.legend(prop=font)
    plt.grid(color='black', alpha=0.2)
    # plt.title('Collision-free Trajectory Generation', fontproperties=font)
    plt.xlabel(r"$\delta_\mathrm{introd}$ [ms]", fontproperties=font)
    plt.savefig('/media/kota/T7/rmader_ral/travel_time.pdf', bbox_inches = "tight")
    # plt.show()
    plt.close('all')         