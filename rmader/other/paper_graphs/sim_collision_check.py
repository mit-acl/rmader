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
    home_dir = "/media/kota/T7/rmader_ral"
    num_of_agents = 10
    methods = ["rmader"]

    for method in methods: 
        cd_list = [50, 100]
        for cd in cd_list:
            if cd == 0:
                dc = 75 
            elif cd == 50:
                dc = 125 
            elif cd == 100:
                dc = 175
            elif cd == 200:
                dc = 250
            elif cd == 300:
                dc = 350

            if method == "oldmader":
                # old mader
                source_dir = f"/media/kota/T7/rmader_ral/mader/oldmader/bags/cd{cd}ms"
                first_agent_idx = 1
                dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
            elif method == "rmader":
                # rmader
                source_dir = f"/media/kota/T7/rmader_ral/mader/rmader/bags/cd{cd}ms/dc{dc}ms"
                first_agent_idx = 1
                dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
            elif method == "wo_check_rmader":
                # rmader without check
                source_dir = f"/media/kota/T7/rmader_ral/mader/wo_check_rmader/bags/cd{cd}ms/dc{dc}ms"
                first_agent_idx = 1
                dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
            elif method == "ego_swarm":
                # EGO-Swarm
                source_dir = f"/media/kota/T7/rmader_ral/ego_swarm/bags/cd{cd}ms"
                first_agent_idx = 0
                dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
            elif method == "edg_team":
                # EDG-Team
                source_dir = f"/media/kota/T7/rmader_ral/edg_team/bags/cd{cd}ms"
                first_agent_idx = 0
                dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms

            source_len = len(source_dir)
            rosbag_list = glob.glob(source_dir+"/*.bag")
            rosbag_list.sort() #alphabetically order
            rosbags = []
            bbox_limit = 0.25 - 0.001

            for bag in rosbag_list:
                rosbags.append(bag)
            
            # collision count
            collision_cnt = 0

            agent_list = []
            for i in range(first_agent_idx,first_agent_idx+num_of_agents):

                if method == "rmader" or method == "wo_check_rmader" or method == "oldmader":
                    if i<=9:
                        agent = "SQ0" + str(i) + "s" 
                    else:
                        agent = "SQ" + str(i) + "s"
                    
                agent_list.append(agent)

            for rosbag_name in rosbags:
                print('rosbag ' + str(rosbag_name))
                sim_id = rosbag_name[source_len+5:source_len+8]
                bag = rosbag.Bag(rosbag_name)
                bag_transformer = BagTfTransformer(bag)
                
                b = bagreader(rosbag_name, verbose=False)
                log_data = b.message_by_topic("/SQ01s/rmader/actual_traj")
                log = pd.read_csv(log_data, usecols=["header.stamp.secs", "header.stamp.nsecs"])
                log.columns = ["secs", "nsecs"]
                t_go_bag = log.secs[3] + log.nsecs[3] / 1e+9
                final_time_bag=bag.get_end_time()
                total_time_bag=final_time_bag-t_go_bag
                times_discret = np.linspace(t_go_bag, final_time_bag, 2000)

                for i_agent1 in range(len(agent_list)):
                    for i_agent2 in range(i_agent1+1, len(agent_list)): 
                        agent1 = agent_list[i_agent1]
                        agent2 = agent_list[i_agent2]
                        if agent1 == agent2:
                            break

                        for ii in range(len(times_discret)-1):
                            time=times_discret[ii];                    
                            parent_frame1 = "vicon"
                            parent_frame2 = "vicon"
                            translation1, quaternion = bag_transformer.lookupTransform(parent_frame1, agent1, rospy.Time.from_sec(time))
                            translation2, quaternion = bag_transformer.lookupTransform(parent_frame2, agent2, rospy.Time.from_sec(time))
                            dist=np.linalg.norm(np.array(translation1)-np.array(translation2), ord=np.inf)

                            if (dist < bbox_limit):
                                collision_cnt += 1
                                print("sim " + sim_id + ": ******collision******" )
                                os.system(f'echo "simulation '+sim_id+': ***collision*** in '+str(time)+' with '+str(agent1)+' and '+str(agent2)+'" >> '+source_dir+'/collision_status.txt')
                                break

                print("sim " + sim_id + ": no collision" )
                os.system('echo "simulation '+sim_id+': no collision" >> '+source_dir+'/collision_status.txt')    
            
            collision_per = collision_cnt
            os.system('echo "'+source_dir+'" >> '+home_dir+'/collision_count.txt')
            os.system('echo "'+str(collision_cnt)+'/'+str(100)+' - '+str(round(collision_per,2))+'%" >> '+home_dir+'/collision_count.txt')
            os.system('echo "------------------------------------------------------------" >> '+home_dir+'/collision_count.txt')
