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
import matplotlib.pyplot as pl
import os
import glob
import sys
import scipy
import numpy
import matplotlib.font_manager as font_manager
import matplotlib.patches as mpl_patches
from matplotlib.ticker import PercentFormatter

if __name__ == '__main__':

    # you wanna get histogram or know the value at q-th percentile
    num_of_agents = 10
    home_dir = "/media/kota/T7/rmader_ral/hw/rmader_mesh_6agents"
    figname = '/mesh_comm_hist.pdf'

    # font
    font = font_manager.FontProperties()
    font.set_family('serif')
    plt.rcParams.update({"text.usetex": True})
    plt.rcParams["font.family"] = "Times New Roman"
    # font.set_name('Times New Roman')
    font.set_size(16)

    # go through tests
    comm_delay_arr_all_rmader = numpy.array([])

    # which tests we are using?
    tests = [2, 3, 4, 5, 6] 
    # tests = [6] 

    for test in tests:
        source_dir = home_dir + "/test"+str(test) # change the source dir accordingly #10 agents 
        source_dir_len = len(source_dir)
        source_bags = source_dir + "/*.bag" # change the source dir accordingly #10 agents
        rosbag_list = glob.glob(source_bags)
        rosbag_list.sort() #alphabetically order
        rosbag = []
        comm_delay = []

        for bag in rosbag_list:
            rosbag.append(bag)

        print(rosbag)

        for i in range(len(rosbag)):

            agent = rosbag[i][source_dir_len+1:source_dir_len+5]
            print(agent)
            b = bagreader(rosbag[i], verbose=False);
            log_data = b.message_by_topic("/" + agent + "/mader/comm_delay")
            try:
                log = pd.read_csv(log_data)
            except:
                pass

            for j in range(len(log.comm_delay)):
                comm_delay.append(log.comm_delay[j])

        # print percentile
        comm_delay_arr_all_rmader = numpy.append(comm_delay_arr_all_rmader, numpy.array(comm_delay))

    textstr_rmader = []
    textstr_rmader.append('RMADER Percentile')
    # q-th percentile
    for q in range(0,105,5):
        # in case you wanna calculate the value of q-th percentile
        print(str(q) + "-th percentile value is " + str(numpy.percentile(comm_delay_arr_all_rmader, q)))
        if (q==25 or q==50 or q==75 or q==95):
            textstr_rmader.append(str(q)+'th: '+str(round(numpy.percentile(comm_delay_arr_all_rmader, q),2)*100)+'ms')

    handles = [mpl_patches.Rectangle((0, 0), 1, 1, fc="white", ec="white", lw=0, alpha=0)] * 5

    fig = plt.figure()
    ax = fig.add_subplot()
    bins = np.arange(-0.5,0.25,0.01)
    n, bins, patches = plt.hist(x=comm_delay_arr_all_rmader, weights=np.ones(len(comm_delay_arr_all_rmader))/len(comm_delay_arr_all_rmader), bins=bins, color="red", edgecolor='black', alpha=0.5, label='RMADER')
    # plt.axvline(x=dc/1000, color="red")
    # ax.set_xticks(np.arange(0,0.25,0.01), fontproperties=font)
    # ax.set_xticklabels(np.arange(0,250,10), fontproperties=font)
    # ax.set_yticks(np.arange(0,len(comm,0.2), fontproperties=font)
    # ax.set_yticklabels(np.arange(0,120,20), fontproperties=font)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    # plt.rcParams["font.family"] = "Times New Roman"
    plt.grid(axis='y', color='black', alpha=0.2)
    # plt.title('Comm delay histogram \n max comm_delay is '+str(round(max_comm_delay*1000))+' [ms]')
    l2 = ax.legend(handles, textstr_rmader, loc='best', fontsize=16, fancybox=True, framealpha=0.7, handlelength=0, handletextpad=0, prop=font)
    l3 = ax.legend(prop=font)
    plt.xlabel(r"$\delta_\mathrm{actual}$ [ms]", fontproperties=font)
    plt.yticks(fontproperties=font)
    # plt.ylabel("Number of Messages", fontproperties=font)
    plt.gca().yaxis.set_major_formatter(PercentFormatter(1))
    plt.ylabel("Probability", fontproperties=font)
    plt.savefig(home_dir+figname, bbox_inches="tight")
    plt.close('all')
    sys.exit()

    