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
    # cd_list = [0, 50, 100, 200, 300]
    cd_list = [50]

    for cd in cd_list:
        is_oldmader=False
        if cd == 0: 
            dc_list = [75] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 50:
            dc_list = [80] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 100:
            dc_list = [175] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 200:
            dc_list = [250]
        elif cd == 300:
            dc_list = [350]
        # this gives you 2d array, row gives you each sims data in corresponding dc
        for dc in dc_list:
            str_dc = str(dc)
            parent_source_dir = sys.argv[1]

            # source directory
            if is_oldmader:
                source_dir = parent_source_dir + "/oldmader/bags/cd"+str(cd)+"ms" # change the source dir accordingly #10 agents
                # is_oldmader = False
            else:
                source_dir = parent_source_dir + "/rmader_obs/bags/cd"+str(cd)+"ms/dc"+str_dc+"ms" # change the source dir accordingly #10 agents
            
            source_len = len(source_dir)
            source_bags = source_dir + "/*.bag" # change the source dir accordingly
            rosbag_list = glob.glob(source_bags)
            rosbag_list.sort() #alphabetically order
            rosbags = []

            for bag in rosbag_list:
                rosbags.append(bag)

            # print("rosbag", rosbag)

            # read ros bags
            completion_time_per_sim_list = []
            for i in range(len(rosbags)):
                print('rosbag ' + str(rosbags[i]))
                start_time_initialized = False
                finish_time_initialized = False
                start_time = 0.0
                finish_time = 0.0
                for topic, msg, t in  rosbag.Bag(rosbags[i]).read_messages():
                    # if topic == '/SQ01s/goal':
                    if topic == '/SQ01s/rmader/actual_traj': # for rmader, i didn't record goal
                        if not start_time_initialized:
                            start_time = t.secs + t.nsecs/1e9
                            print("start_time", start_time)
                            start_time_initialized = True
                        
                    if topic == '/goal_reached':
                        if not finish_time_initialized:
                            finish_time = t.secs + t.nsecs/1e9
                            print("finish_time", finish_time)
                            finish_time_initialized = True

                completion_time = finish_time - start_time
                print("completion_time", completion_time)
                completion_time_per_sim_list.append(completion_time)

            os.system('echo "'+source_dir+'" >> '+parent_source_dir+'/completion_time.txt')
            try:
                os.system('echo "ave is '+str(round(statistics.mean(completion_time_per_sim_list),2))+'s" >> '+parent_source_dir+'/completion_time.txt')
                os.system('echo "max is '+str(round(max(completion_time_per_sim_list),2))+'s" >> '+parent_source_dir+'/completion_time.txt')
                os.system('echo "min is '+str(round(min(completion_time_per_sim_list),2))+'s" >> '+parent_source_dir+'/completion_time.txt')
            except:
                pass
            os.system('echo "------------------------------------------------------------" >> '+parent_source_dir+'/completion_time.txt')