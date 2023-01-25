#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python comm_delay_histogram_percentile.py
#  Ex. python comm_delay_histogram_percentile.py

# change cd and dc

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
import matplotlib.patches as mpl_patches


if __name__ == '__main__':

    # you wanna get histogram or know the value at q-th percentile
    is_histogram = False
    num_of_agents = 10

    # font
    font = font_manager.FontProperties()
    font.set_family('serif')
    font.set_name('Times New Roman')

    # go through tests
    comm_delay_arr_all = numpy.array([])

    # which tests we are using?
    tests = [10, 11, 3] 

    for test in tests:

        figname = '/rmader_obs_cd_histogram.png'
        source_dir = f"/media/kota/T7/rmader_ral/hw/rmader_obs/6agent2obs/test{test}" # change the source dir accordingly #10 agents 
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
            
            log_data = b.message_by_topic("/" + agent + "/rmader/comm_delay")
            try:
                log = pd.read_csv(log_data)
            except:
                pass

            for j in range(len(log.comm_delay)):
                comm_delay.append(log.comm_delay[j])

        # print percentile
        comm_delay_arr_all = numpy.append(comm_delay_arr_all, numpy.array(comm_delay))

        # print('here'+str(max(comm_delay)))
        # percentile = scipy.stats.percentileofscore(comm_delay_arr, input_comm_delay, kind='mean')
        # os.system('echo "cd='+str(cd)+', dc='+str(dc)+':   '+str(input_comm_delay) + ' is ' + str(percentile) + '-th percentile" >> '+source_dir+'/comm_delay_percentile.txt')
        # print(comm_delay)
        # max_comm_delay = max(comm_delay)


    max_comm_delay = numpy.amax(comm_delay_arr_all)

    textstr = []
    textstr.append('Percentile')
    # q-th percentile
    for q in range(0,105,5):
        # in case you wanna calculate the value of q-th percentile
        print(str(q) + "-th percentile value is " + str(numpy.percentile(comm_delay_arr_all, q)))
        if (q==25 or q==50 or q==75 or q==95):
            textstr.append(str(q)+'th: '+str(round(numpy.percentile(comm_delay_arr_all, q),2)*100)+'ms')

    handles = [mpl_patches.Rectangle((0, 0), 1, 1, fc="white", ec="white", lw=0, alpha=0)] * 5

    fig = plt.figure()
    ax = fig.add_subplot()
    # bins = np.arange(0,0.750,0.01)
    n, bins, patches = plt.hist(x=comm_delay_arr_all, bins=bins, color="blue", edgecolor='black')
    # plt.axvline(x=dc/1000, color="red")
    # ax.set_xticks(np.arange(0,0.750,0.05), fontproperties=font)
    # ax.set_xticklabels(np.arange(0,750,50), fontproperties=font)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    # plt.rcParams["font.family"] = "Times New Roman"
    plt.grid(axis='y', color='black', alpha=0.2)
    # plt.title('Comm delay histogram \n max comm_delay is '+str(round(max_comm_delay*1000))+' [ms]')
    ax.legend(handles, textstr, loc='best', fontsize=16, fancybox=True, framealpha=0.7, handlelength=0, handletextpad=0, prop=font)
    plt.xlabel("Communication Delay [ms]", fontproperties=font)
    plt.ylabel("Number of Messages", fontproperties=font)
    plt.yticks(fontproperties=font)
    plt.savefig(source_bags)
    plt.close('all')

    print('max comm delay is: '+str(max_comm_delay))

    # q-th percentile
    for q in range(0,105,5):
        # in case you wanna calculate the value of q-th percentile
        print(str(q) + "-th percentile value is " + str(numpy.percentile(comm_delay_arr_all, q)))
    
    sys.exit()