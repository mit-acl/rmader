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
from matplotlib.ticker import PercentFormatter

if __name__ == '__main__':

    # font
    font = font_manager.FontProperties()
    font.set_family('serif')
    font.set_name('Times New Roman')
    font.set_size(16)
    # plt.rcParams.update({"text.usetex": True})

    is_oldmader = False # always False bc oldmader doesn't have comm_delay
    num_of_agents = 10

    cd_list = [0, 50, 100, 200, 300]

    comm_delay_arr_all_list = [] #store lists
    for cd in cd_list:

        if cd == 0:
            dc_list = [75] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 50:
            dc_list = [125] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 100:
            dc_list = [175] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 200:
            dc_list = [275]
        elif cd == 300:
            dc_list = [375]
        
        comm_delay_arr_all = numpy.array([])
        for dc in dc_list:
            
            # comm_delay you use
            input_comm_delay = dc/1000
            str_dc = str(dc)

            figname = 'rmader_comm_delay_histogram_all_included.pdf'
            source_bags = "/media/kota/T7/rmader_ral/rmader/bags/cd"+str(cd)+"ms/dc"+str_dc+"ms/*.bag" # change the source dir accordingly #10 agents

            rosbag_list = glob.glob(source_bags)
            rosbag_list.sort() #alphabetically order
            rosbag = []
            comm_delay = []


            for bag in rosbag_list:
                rosbag.append(bag)

            print(rosbag)

            for i in range(len(rosbag)):
                b = bagreader(rosbag[i], verbose=False)
                
                for i in range(1,num_of_agents+1):
                    if i < 10:
                        log_data = b.message_by_topic("/SQ0" + str(i) + "s/rmader/comm_delay")
                    else:
                        log_data = b.message_by_topic("/SQ" + str(i) + "s/rmader/comm_delay")
                    try:
                        log = pd.read_csv(log_data)

                        for j in range(len(log.comm_delay)):
                            comm_delay.append(log.comm_delay[j])
                    except:
                        pass

            # print percentile
            comm_delay_arr_all = numpy.append(comm_delay_arr_all, numpy.array(comm_delay))

        # save three list
        comm_delay_arr_all_list.append(comm_delay_arr_all)

    fig = plt.figure()
    ax = fig.add_subplot()
    n, bins, patches = plt.hist(x=comm_delay_arr_all_list[0], bins=np.arange(0+cd_list[0]/1000,0.1+0.01+cd_list[0]/1000,0.0025), weights=np.ones(len(comm_delay_arr_all_list[0]))/len(comm_delay_arr_all_list[0]), color="blue", edgecolor = 'black', alpha=0.5, label=r"$\delta_\mathrm{introd.}$=0ms")
    n, bins, patches = plt.hist(x=comm_delay_arr_all_list[1], bins=np.arange(0+cd_list[1]/1000,0.1+0.01+cd_list[1]/1000,0.0025), weights=np.ones(len(comm_delay_arr_all_list[1]))/len(comm_delay_arr_all_list[1]), color="limegreen", edgecolor = 'black', alpha=0.5, label=r"$\delta_\mathrm{introd.}$=50ms")
    n, bins, patches = plt.hist(x=comm_delay_arr_all_list[2], bins=np.arange(0+cd_list[2]/1000,0.1+0.01+cd_list[2]/1000,0.0025), weights=np.ones(len(comm_delay_arr_all_list[2]))/len(comm_delay_arr_all_list[2]), color="sienna", edgecolor = 'black', alpha=0.5, label=r"$\delta_\mathrm{introd.}$=100ms")
    n, bins, patches = plt.hist(x=comm_delay_arr_all_list[3], bins=np.arange(0+cd_list[3]/1000,0.1+0.01+cd_list[3]/1000,0.0025), weights=np.ones(len(comm_delay_arr_all_list[3]))/len(comm_delay_arr_all_list[3]), color="magenta", edgecolor = 'black', alpha=0.5, label=r"$\delta_\mathrm{introd.}$=200ms")
    n, bins, patches = plt.hist(x=comm_delay_arr_all_list[4], bins=np.arange(0+cd_list[4]/1000,0.1+0.01+cd_list[4]/1000,0.0025), weights=np.ones(len(comm_delay_arr_all_list[4]))/len(comm_delay_arr_all_list[4]), color="orange", edgecolor = 'black', alpha=0.5, label=r"$\delta_\mathrm{introd.}$=300ms")
    ax.set_xticks(np.arange(0,0.400,0.05))
    ax.set_xticklabels(np.arange(0,400,50), fontproperties=font)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    plt.grid(axis='y', color='black', alpha=0.2)
    plt.legend(prop=font)
    plt.xlabel(r"$\delta_\mathrm{actual}$ [ms]", fontproperties=font)
    plt.yticks(fontproperties=font)
    plt.gca().yaxis.set_major_formatter(PercentFormatter(1))
    plt.ylabel("Probability", fontproperties=font)
    plt.savefig('/media/kota/T7/rmader_ral/'+figname, bbox_inches="tight")
    plt.close('all')