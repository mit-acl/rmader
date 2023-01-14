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
    num_of_agents = 10
    stop_cnt_tol = 1e-2 # stop count torelance
    home_dir = "/media/kota/T7/rmader_ral/mader/rmader_obs"
    rmader_ave_dist = []
    rmader_stop_cnt = []

    cd_list = [50]
    for cd in cd_list:
        dc = 80 
        source_dir = f"/media/kota/T7/rmader_ral/mader/rmader_obs/bags/cd{cd}ms/dc{dc}ms/*.bag"
        first_agent_idx = 1
        dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms

        rosbag_list = glob.glob(source_dir)
        rosbag_list.sort() #alphabetically order
        rosbag = []

        for bag in rosbag_list:
            rosbag.append(bag)
        total_dist_list = []
        stop_cnt_list = []
        for i in range(len(rosbag)):
            print('rosbag ' + str(rosbag[i]))
            b = bagreader(rosbag[i], verbose=False)
            # introduced goal_reached topic so no need to check actual_traj
            dist = 0
            stop_cnt = 0
            for i in range(first_agent_idx,num_of_agents+first_agent_idx):
                stopped = True # in the beginning it is stopped
                if i <= 9:
                    num = f"0{i}"
                else:
                    num = str(i)
                
                log_data = b.message_by_topic(f'/SQ{num}s/state')
                log = pd.read_csv(log_data, usecols=["pos.x", "pos.y", "pos.z", "vel.x", "vel.y", "vel.z"])
                log.columns = ["px", "py", "pz", "vx", "vy", "vz"]  
                
                ###### difference from the previous pos to the current pos
                # print(log.diff().to_numpy())
                diff_matrix = log.diff().to_numpy()

                # since the first row is NaN, starts 
                for i in range(1, len(log.diff())):
                    dist += np.linalg.norm(log.diff().to_numpy()[i,0:2])

                ###### stop count
                for i in range(len(log.to_numpy())):
                    if np.linalg.norm(log.to_numpy()[i,3:5]) > stop_cnt_tol:
                        stopped = False
                    elif np.linalg.norm(log.to_numpy()[i,3:5]) < stop_cnt_tol and not stopped:
                        stop_cnt = stop_cnt + 1
                        stopped = True

                stop_cnt = stop_cnt - 1 # for the last stop

                # in case of collision, stop_cnt won't work, so need to skip the bag
                if (stop_cnt < 0):
                    is_skip_bag = True
                    print("skip the bag")
                    break

            dist /= num_of_agents
            stop_cnt /= num_of_agents
            total_dist_list.append(dist)
            stop_cnt_list.append(stop_cnt)

        ave_total_dist = sum(total_dist_list)/len(total_dist_list)
        ave_stop_cnt = sum(stop_cnt_list)/len(stop_cnt_list)
                        
        os.system('echo "'+source_dir+'" >> '+home_dir+'/total_dist.txt')
        os.system('echo "ave travel dist '+str(round(ave_total_dist,3))+'m" >> '+home_dir+'/total_dist.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/total_dist.txt')

        os.system('echo "'+source_dir+'" >> '+home_dir+'/stop_cnt.txt')
        os.system('echo "ave stop count '+str(round(ave_stop_cnt,3))+'" >> '+home_dir+'/stop_cnt.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/stop_cnt.txt')

        rmader_ave_dist.append(ave_total_dist)
        rmader_stop_cnt.append(ave_stop_cnt)
    
    # # plot
    # # x axis
    # x = [0, 50, 100, 200, 300]

    # # font
    # font = font_manager.FontProperties()
    # font.set_family('serif')
    # plt.rcParams.update({"text.usetex": True})
    # plt.rcParams["font.family"] = "Times New Roman"
    # # font.set_name('Times New Roman')
    # font.set_size(16)

    # # number of stops
    # fig, ax = plt.subplots()
    # ax.plot(x, oldmader_stop_cnt, color='b', linewidth=1.5, marker = 'o', label='MADER')
    # ax.plot(x, rmader_stop_cnt, color='r', linewidth=1.5, marker = 'o', label='RMADER')
    # ax.plot(x, wo_check_rmader_stop_cnt, color='purple', linewidth=1.5, marker = 'o', label='RMADER w/o check')
    # ax.plot(x, ego_swarm_stop_cnt, color='g', linewidth=1.5, marker = 'o', label='EGO-Swarm')
    # ax.plot(x, edg_team_stop_cnt, color='orange', linewidth=1.5, marker = 'o', label='EDG-Team')
    # ax.set_xticks(np.arange(0,300+50,50))
    # ax.set_xticklabels(np.arange(0,300+50,50), fontproperties=font)
    # # ax.set_yticks(np.arange(0,10,1))
    # # ax.set_yticklabels(np.arange(0,10,1), fontproperties=font)
    # ax.set_ylabel("Ave. Number of Stops", fontproperties=font)
    # ax.spines['top'].set_visible(False)
    # ax.spines['right'].set_visible(False)
    # plt.legend(prop=font)
    # plt.grid(color='black', alpha=0.2)
    # # plt.title('Collision-free Trajectory Generation', fontproperties=font)
    # plt.xlabel(r"$\delta_\mathrm{introd}$ [ms]", fontproperties=font)
    # plt.savefig(home_dir+'/stop.pdf', bbox_inches = "tight")
    # # plt.show()
    # plt.close('all')    

    # # total travel distance
    # fig, ax = plt.subplots()
    # ax.plot(x, oldmader_ave_dist, color='b', linewidth=1.5, marker = 'o', label='MADER')
    # ax.plot(x, rmader_ave_dist, color='r', linewidth=1.5, marker = 'o', label='RMADER')
    # ax.plot(x, wo_check_rmader_ave_dist, color='purple', linewidth=1.5, marker = 'o', label='RMADER w/o check')
    # ax.plot(x, ego_swarm_ave_dist, color='g', linewidth=1.5, marker = 'o', label='EGO-Swarm')
    # ax.plot(x, edg_team_ave_dist, color='orange', linewidth=1.5, marker = 'o', label='EDG-Team')
    # ax.set_xticks(np.arange(0,300+50,50))
    # ax.set_xticklabels(np.arange(0,300+50,50), fontproperties=font)
    # # ax.set_yticks(np.arange(0,10,1))
    # # ax.set_yticklabels(np.arange(0,10,1), fontproperties=font)
    # ax.set_ylabel("Total Travel Distance", fontproperties=font)
    # ax.spines['top'].set_visible(False)
    # ax.spines['right'].set_visible(False)
    # plt.legend(prop=font)
    # plt.grid(color='black', alpha=0.2)
    # # plt.title('Collision-free Trajectory Generation', fontproperties=font)
    # plt.xlabel(r"$\delta_\mathrm{introd}$ [ms]", fontproperties=font)
    # plt.savefig(home_dir+'/total_dist.pdf', bbox_inches = "tight")
    # # plt.show()
    # plt.close('all')   