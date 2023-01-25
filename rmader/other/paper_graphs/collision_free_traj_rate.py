#!/usr/bin/python

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
import sys
import scipy
import numpy
import matplotlib.font_manager as font_manager

if __name__ == '__main__':

    ##### collision-free trajectory percentage

    # collision-free rate 
    oldmader = [95, 92, 86, 72, 70] 
    rmader = [100, 100, 100, 100, 100]
    wo_check_rmader = [100, 100, 100, 100, 100]
    ego_swarm = [34, 27, 27, 48, 29]
    edg_team = [100, 100, 99, 99, 99]
    
    # x axis
    x = [0, 50, 100, 200, 300]

    # font
    font = font_manager.FontProperties()
    font.set_family('serif')
    plt.rcParams.update({"text.usetex": True})
    plt.rcParams["font.family"] = "Times New Roman"
    # font.set_name('Times New Roman')
    font.set_size(16)

    # plot
    fig = plt.figure()
    ax = fig.add_subplot()
    plt.plot(x, oldmader, color='b', linewidth=1.5, marker = 'o', label='MADER')
    plt.plot(x, rmader, color='r', linewidth=1.5, marker = 'o', label='RMADER')
    plt.plot(x, wo_check_rmader, color='purple', linewidth=1.5, marker = 'o', label='RMADER w/o check')
    plt.plot(x, ego_swarm, color='g', linewidth=1.5, marker = 'o', label='EGO-Swarm')
    plt.plot(x, edg_team, color='orange', linewidth=1.5, marker = 'o', label='EDG-Team')
    ax.set_xticks(np.arange(0,300+50,50))
    ax.set_xticklabels(np.arange(0,300+50,50), fontproperties=font)
    ax.set_yticks(np.arange(0,100+20,20))
    ax.set_yticklabels([0, 20, 40, 60, 80, 100], fontproperties=font)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    plt.grid(color='black', alpha=0.2)
    # plt.title('Collision-free Trajectory Generation', fontproperties=font)
    plt.xlabel(r"$\delta_\mathrm{introd}$ [ms]", fontproperties=font)
    plt.ylabel(r"Collision-free Traj. Rate [\%]", fontproperties=font)
    plt.legend(prop=font)
    plt.savefig('/media/kota/T7/rmader_ral/collision_free_traj.pdf', bbox_inches = "tight")
    # plt.show()
    plt.close('all')
