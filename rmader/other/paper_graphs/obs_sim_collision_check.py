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
from tf_bag import BagTfTransformer


if __name__ == '__main__':

    # initialization
    home_dir = "/media/kota/T7/rmader_ral/mader/rmader_obs"
    num_of_agents = 10
    source_dir = home_dir+"/bags/cd50ms/dc80ms"           
    source_len = len(source_dir)
    rosbag_list = glob.glob(source_dir+"/*.bag")
    rosbag_list.sort() #alphabetically order
    rosbags = []
    bbox_limit = 0.05

    for bag in rosbag_list:
        rosbags.append(bag)
    
    # collision count
    collision_cnt = 0
    # total dist
    total_dist = 0.0
    # travel time
    travel_time = 0.0
    # stop count
    stop_cnt = 0.0

    agent_obs_list = []
    for i in range(1,11):
        if i<=9:
            agent = "SQ0" + str(i) + "s" 
        else:
            agent = "SQ" + str(i) + "s"
        agent_obs_list.append(agent)
    for j in range(4000, 4010):
        agent_obs_list.append(f"obs{j}")

    for rosbag_name in rosbags:
        print('rosbag ' + str(rosbag_name))
        sim_id = rosbag_name[source_len+5:source_len+8]
        bag = rosbag.Bag(rosbag_name)
        bag_transformer = BagTfTransformer(bag)
        
        b = bagreader(rosbag_name, verbose=False)
        log_data = b.message_by_topic("/SQ01s/goal")
        log = pd.read_csv(log_data, usecols=["header.stamp.secs", "header.stamp.nsecs"])
        log.columns = ["secs", "nsecs"]
        t_go_bag = log.secs[0] + log.nsecs[0] / 1e+9
        final_time_bag=bag.get_end_time()
        total_time_bag=final_time_bag-t_go_bag;
        times_discret = np.linspace(t_go_bag, final_time_bag, 2000)

        for i_agent1 in range(len(agent_obs_list)):
            for i_agent2 in range(i_agent1+1, len(agent_obs_list)): 
                agent1 = agent_obs_list[i_agent1]
                agent2 = agent_obs_list[i_agent2]
                if agent1 == agent2 or (agent1[:3] == "obs" and agent2[:3] == "obs"):
                    break
                print("agent1", agent1)
                print("agent2", agent2)

                for ii in range(len(times_discret)-1):
                    time=times_discret[ii];
                    
                    if agent1[:2] == "SQ":
                        parent_frame1 = "vicon"
                    else:
                        parent_frame1 = "world"

                    if agent2[:2] == "SQ":
                        parent_frame2 = "vicon"
                    else:
                        parent_frame2 = "world"

                    translation1, quaternion = bag_transformer.lookupTransform(parent_frame1, agent1, rospy.Time.from_sec(time))
                    translation2, quaternion = bag_transformer.lookupTransform(parent_frame2, agent2, rospy.Time.from_sec(time))
                    dist=np.linalg.norm(np.array(translation1)-np.array(translation2), ord=np.inf)

                    if (dist < 0.05):
                        collision_cnt += 1
                        print("sim " + sim_id + ": ******collision******" )
                        os.system(f'echo "simulation '+sim_id+': ***collision*** in {time} with {agent1} and {agent2}" >> '+source_dir+'/collision_status.txt')

        print("sim " + sim_id + ": no collision" )
        os.system('echo "simulation '+sim_id+': no collision" >> '+source_dir+'/collision_status.txt')    
    
    collision_per = collision_cnt
    os.system('echo "'+source_dir+'" >> '+home_dir+'/collision_count.txt')
    os.system('echo "'+str(collision_cnt)+'/'+str(len(rosbag))+' - '+str(round(collision_per,2))+'%" >> '+home_dir+'/collision_count.txt')
    os.system('echo "------------------------------------------------------------" >> '+home_dir+'/collision_count.txt')
