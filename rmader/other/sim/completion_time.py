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

if __name__ == '__main__':

    ##### parameters
    cd_list = [0, 50, 100, 200, 300]

    for cd in cd_list:
        is_oldmader=True
        if cd == 0: 
            dc_list = [0, 100] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 50:
            dc_list = [0, 150] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 100:
            dc_list = [0, 200] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 200:
            dc_list = [0, 300]
        elif cd == 300:
            dc_list = [0, 400]

        # this gives you 2d array, row gives you each sims data in corresponding dc
        box_plot_list = [] 
        for dc in dc_list:
            str_dc = str(dc)
            parent_source_dir = sys.argv[1]

            # source directory
            if is_oldmader:
                source_dir = parent_source_dir + "/oldmader/bags/cd"+str(cd)+"ms" # change the source dir accordingly #10 agents
                is_oldmader = False
            else:
                source_dir = parent_source_dir + "/rmader/bags/cd"+str(cd)+"ms/dc"+str_dc+"ms" # change the source dir accordingly #10 agents
            
            source_len = len(source_dir)
            source_bags = source_dir + "/*.bag" # change the source dir accordingly
            rosbag_list = glob.glob(source_bags)
            rosbag_list.sort() #alphabetically order
            rosbag = []

            for bag in rosbag_list:
                rosbag.append(bag)

            print("rosbag", rosbag)

            # read ros bags
            completion_time_per_sim_list = []
            completion_time_per_sim = 0.0
            for i in range(len(rosbag)):
                print('rosbag ' + str(rosbag[i]))
                b = bagreader(rosbag[i], verbose=False)
                sim_id = rosbag[i][source_len+5:source_len+7]
                
                # introduced goal_reached topic so no need to check actual_traj

                try:
                    log_data = b.message_by_topic("/goal_reached")
                    log = pd.read_csv(log_data, usecols=["completion_time", "is_goal_reached"])
                    completion_time = log.completion_time.iloc[0]
                    # print('completion time ' + str(completion_time_agent))
                    completion_time_per_sim_list.append(completion_time)
                    box_plot_list.append(completion_time_per_sim_list)

                except:
                    print("agents didn't reach goals")

            os.system('echo "'+source_dir+'" >> '+parent_source_dir+'/completion_time.txt')
            try:
                os.system('echo "ave is '+str(round(statistics.mean(completion_time_per_sim_list),2))+'s" >> '+parent_source_dir+'/completion_time.txt')
                os.system('echo "max is '+str(round(max(completion_time_per_sim_list),2))+'s" >> '+parent_source_dir+'/completion_time.txt')
                os.system('echo "min is '+str(round(min(completion_time_per_sim_list),2))+'s" >> '+parent_source_dir+'/completion_time.txt')
            except:
                pass
            os.system('echo "------------------------------------------------------------" >> '+parent_source_dir+'/completion_time.txt')

