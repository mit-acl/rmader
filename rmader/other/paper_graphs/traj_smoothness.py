#!/usr/bin/python

# Kota Kondo, kkondo@mit.edu, Aug 19, 2022

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import rospy
import numpy as np
import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import os
import sys
import numpy as np
from colorama import init, Fore, Back, Style
from termcolor import colored
import re
import glob
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager

# initialization
home_dir = "/media/kota/T7/rmader_ral/"
methods = ["oldmader", "rmader", "wo_check_rmader", "ego_swarm", "edg_team"]
num_of_agents=10
oldmader_traj_smoothness_acc = []
oldmader_traj_smoothness_jer = []
rmader_traj_smoothness_acc = []
rmader_traj_smoothness_jer = []
wo_check_rmader_traj_smoothness_acc = []
wo_check_rmader_traj_smoothness_jer = []
ego_swarm_traj_smoothness_acc = []
ego_swarm_traj_smoothness_jer = []
edg_team_traj_smoothness_acc = []
edg_team_traj_smoothness_jer = []

for method in methods: 
    cd_list = [0, 50, 100, 200, 300]
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
            source_dir = f"/media/kota/T7/rmader_ral/mader/oldmader/bags/cd{cd}ms/*.bag"
            first_agent_idx = 1
            dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
        elif method == "rmader":
            # rmader
            source_dir = f"/media/kota/T7/rmader_ral/mader/rmader/bags/cd{cd}ms/dc{dc}ms/*.bag"
            first_agent_idx = 1
            dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
        elif method == "wo_check_rmader":
            # rmader without check
            source_dir = f"/media/kota/T7/rmader_ral/mader/wo_check_rmader/bags/cd{cd}ms/dc{dc}ms/*.bag"
            first_agent_idx = 1
            dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
        elif method == "ego_swarm":
            # EGO-Swarm
            source_dir = f"/media/kota/T7/rmader_ral/ego_swarm/bags/cd{cd}ms/*.bag"
            first_agent_idx = 0
            dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
        elif method == "edg_team":
            # EDG-Team
            source_dir = f"/media/kota/T7/rmader_ral/edg_team/bags/cd{cd}ms/*.bag"
            first_agent_idx = 0
            dt = 0.01 #[s] MADERm EGO-Swarm, EDG-Team's state/pos_cmd is published every 10ms
        
        # get the bags
        list_of_bags = glob.glob(source_dir)

        ave_smoothness_acc = 0.0
        ave_smoothness_jer = 0.0

        for name_bag in list_of_bags:
            bag = rosbag.Bag(name_bag)
            print('rosbag ' + str(list_of_bags[i]))
            topics=[]
            for i in range(0+first_agent_idx,num_of_agents+first_agent_idx): #if you use MADER you need to make it range(1, num_of_agents+1)
                if i <= 9:
                    num = f"0{i}"
                else:
                    num = str(i)
                if method == "ego_swarm" or method == "edg_team":
                    topics.append('/drone_'+str(i)+'_planning/pos_cmd')
                elif method == "rmader" or method == "wo_check_rmader" or method == "oldmader":
                    topics.append(f'/SQ{num}s/state')
                else: 
                    topics.append(f'/SQ{num}s/goal')

            print(topics)
            for index_agent in range(0, num_of_agents):
                prev_vel = 0.0
                prev_acc = 0.0
                for topic, msg, t in bag.read_messages(topics=topics[index_agent]):
                    if method == "ego_swarm" or method == "edg_team":
                        acc=np.linalg.norm(np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z]));
                        jerk = (acc - prev_acc) / dt 
                        prev_acc = acc
                    elif method == "rmader" or method == "wo_check_rmader" or method == "oldmader":
                        vel = np.linalg.norm(np.array([msg.vel.x, msg.vel.y, msg.vel.z]))
                        acc = (vel - prev_vel) / dt
                        jerk = (acc - prev_acc) / dt 
                        prev_vel = vel
                        prev_acc = acc
                    else:
                        acc = np.linalg.norm(np.array([msg.a.x, msg.a.y, msg.a.z]))
                        jerk = np.linalg.norm(np.array([msg.j.x, msg.j.y, msg.j.z]))
                    ave_smoothness_acc = ave_smoothness_acc + acc**2
                    ave_smoothness_jer = ave_smoothness_jer + jerk**2

            # print( "ave smoothness (acc): ", round(ave_smoothness_acc,2))
            # print( "ave smoothness (jer): ", round(ave_smoothness_jer,2))

            rospy.sleep(4.0)
            bag.close()

        # output txt file
        ave_smoothness_acc /= num_of_agents
        ave_smoothness_jer /= num_of_agents
        ave_smoothness_acc /= len(list_of_bags)
        ave_smoothness_jer /= len(list_of_bags)
        ave_smoothness_acc *= dt
        ave_smoothness_jer *= dt

        os.system('echo "----------------------------------------------------------------------------------" >> '+home_dir+'smoothness.txt')
        os.system('echo "'+str(source_dir)+'" >> '+home_dir+'smoothness.txt')
        os.system('echo "----------------------------------------------------------------------------------" >> '+home_dir+'smoothness.txt')
        os.system('echo " smoothness (acc)'+str(round(ave_smoothness_acc,2))+'" >> '+home_dir+'smoothness.txt')
        os.system('echo " smoothness (jer)'+str(round(ave_smoothness_jer,2))+'" >> '+home_dir+'smoothness.txt')

        if method == "oldmader":
            # old mader
            oldmader_traj_smoothness_acc.append(ave_smoothness_acc)
            oldmader_traj_smoothness_jer.append(ave_smoothness_jer)
        elif method == "rmader":
            # rmader
            rmader_traj_smoothness_acc.append(ave_smoothness_acc)
            rmader_traj_smoothness_jer.append(ave_smoothness_jer)
        elif method == "wo_check_rmader":
            # rmader without check
            wo_check_rmader_traj_smoothness_acc.append(ave_smoothness_acc)
            wo_check_rmader_traj_smoothness_jer.append(ave_smoothness_jer)
        elif method == "ego_swarm":
            # EGO-Swarm
            ego_swarm_traj_smoothness_acc.append(ave_smoothness_acc)
            ego_swarm_traj_smoothness_jer.append(ave_smoothness_jer)
        elif method == "edg_team":
            # EDG-Team
            edg_team_traj_smoothness_acc.append(ave_smoothness_acc)
            edg_team_traj_smoothness_jer.append(ave_smoothness_jer)

# plot
# x axis
x = [0, 50, 100, 200, 300]

# font
font = font_manager.FontProperties()
font.set_family('serif')
plt.rcParams.update({"text.usetex": True})
plt.rcParams["font.family"] = "Times New Roman"
# font.set_name('Times New Roman')
font.set_size(16)

# traj. smoothness (acc) time
fig, ax = plt.subplots()
ax.plot(x, oldmader_traj_smoothness_acc, color='b', linewidth=1.5, marker = 'o', label='MADER')
ax.plot(x, rmader_traj_smoothness_acc, color='r', linewidth=1.5, marker = 'o', label='RMADER')
ax.plot(x, wo_check_rmader_traj_smoothness_acc, color='purple', linewidth=1.5, marker = 'o', label='RMADER w/o check')
ax.plot(x, ego_swarm_traj_smoothness_acc, color='g', linewidth=1.5, marker = 'o', label='EGO-Swarm')
ax.plot(x, edg_team_traj_smoothness_acc, color='orange', linewidth=1.5, marker = 'o', label='EDG-Team')
ax.set_xticks(np.arange(0,300+50,50))
ax.set_xticklabels(np.arange(0,300+50,50), fontproperties=font)
# ax.set_yticks(np.arange(6,19,1))
# ax.set_yticklabels(np.arange(6,19,1), fontproperties=font)
ax.set_ylabel(r"$\int\left\Vert\mathbf{a}\right\Vert^2dt \ [m^2/3^3]$", fontproperties=font)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
plt.legend(prop=font)
plt.grid(color='black', alpha=0.2)
# plt.title('Collision-free Trajectory Generation', fontproperties=font)
plt.xlabel(r"$\delta_\mathrm{introd}$ [ms]", fontproperties=font)
plt.savefig(home_dir+'/traj_smoothness_acc.pdf', bbox_inches = "tight")
# plt.show()
plt.close('all') 

# traj. smoothness (jerk) time
fig, ax = plt.subplots()
ax.plot(x, rmader_traj_smoothness_jer, color='r', linewidth=1.5, marker = 'o', label='RMADER (proposed)')
ax.plot(x, wo_check_rmader_traj_smoothness_jer, color='purple', linewidth=1.5, marker = 'o', label='RMADER w/o check')
ax.plot(x, oldmader_traj_smoothness_jer, color='b', linewidth=1.5, marker = 'o', label='MADER')
ax.plot(x, ego_swarm_traj_smoothness_jer, color='g', linewidth=1.5, marker = 'o', label='EGO-Swarm')
ax.plot(x, edg_team_traj_smoothness_jer, color='orange', linewidth=1.5, marker = 'o', label='EDG-Team')
ax.set_xticks(np.arange(0,300+50,50))
ax.set_xticklabels(np.arange(0,300+50,50), fontproperties=font)
# ax.set_yticks(np.arange(6,19,1))
# ax.set_yticklabels(np.arange(6,19,1), fontproperties=font)
ax.set_ylabel(r"$\int\left\Vert\mathbf{j}\right\Vert^2dt \ [m^2/3^5]$", fontproperties=font)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
plt.legend(prop=font)
plt.grid(color='black', alpha=0.2)
# plt.title('Collision-free Trajectory Generation', fontproperties=font)
plt.xlabel(r"$\delta_\mathrm{introd}$ [ms]", fontproperties=font)
plt.savefig(home_dir+'/traj_smoothness_jer.pdf', bbox_inches = "tight")
# plt.show()
plt.close('all')      

