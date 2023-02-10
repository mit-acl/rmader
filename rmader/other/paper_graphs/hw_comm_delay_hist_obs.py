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
import sys
import scipy
import numpy
import matplotlib.font_manager as font_manager
import matplotlib.patches as mpl_patches
from matplotlib.ticker import PercentFormatter

if __name__ == '__main__':

    # basic params
    figname = "/comm_delay.pdf"

    # file path?
    home_dir = "/media/kota/T7/rmader_ral/hw/"
    # rmader_obs
    tests = ["rmader_obs/2agent1obs/test1", "rmader_obs/2agent1obs/test2", "rmader_obs/4agent2obs/test4", \
                    "rmader_obs/4agent2obs/test7", "rmader_obs/6agent2obs/test3", "rmader_obs/6agent2obs/test10", \
                    "rmader_obs/6agent2obs/test11"]

    # font settings
    font = font_manager.FontProperties()
    font.set_family('serif')
    plt.rcParams.update({"text.usetex": True})
    plt.rcParams["font.family"] = "Times New Roman"
    # font.set_name('Times New Roman')
    font.set_size(16)

    # go through tests
    cd_mesh = []
    for test in tests:
        source_dir = home_dir + test # change the source dir accordingly #10 agents 
        source_dir_len = len(source_dir)
        source_bags = source_dir + "/*.bag" # change the source dir accordingly #10 agents
        rosbag_list = glob.glob(source_bags)
        rosbag_list.sort() #alphabetical order
        comm_delay = []

        # agents
        agents = []
        for rb in rosbag_list:
            agents.append(rb[source_dir_len+4:source_dir_len+5])

        for i in range(len(rosbag_list)):
            agent_name = rosbag_list[i][source_dir_len+1:source_dir_len+5]
            b = bagreader(rosbag_list[i], verbose=False);
            log_data = b.message_by_topic("/" + agent_name + "/rmader/comm_delay")
            log = pd.read_csv(log_data)

            # initialize lists for each agent
            cdlists = [[] for i in range(len(agents))]

            # save offset for each agent
            offsets = {}
            for ag in agents:
                offsets[ag] = 0

            # sort comm_delays into list of comm delays for each agent
            for logcd, logid in zip(log.comm_delay, log.id):
                if str(logid) in agents:
                    cdlists[agents.index(str(logid))].append(logcd)

            # get offset
            for ag, cdlist in zip(agents, cdlists):
                if cdlist == []:
                    print('agent itself')
                else:
                    offsets[ag] = min(cdlist)

            # correct offsets
            for ag, cdlist in zip(agents, cdlists):
                cd_mesh.extend(list(map(lambda x : x - offsets[ag], cdlist)))

            # print(len(cd_mesh))
            print(offsets)

    textstr_rmader = []
    textstr_rmader.append('RMADER Percentile')
    # q-th percentile
    for q in range(0,105,5):
        # in case you wanna calculate the value of q-th percentile
        print(str(q) + "-th percentile value is " + str(numpy.percentile(cd_mesh, q)))
        if (q==25 or q==50 or q==75 or q==95):
            textstr_rmader.append(str(q)+'th: '+str(round(numpy.percentile(cd_mesh, q),2)*100)+'ms')

    handles = [mpl_patches.Rectangle((0, 0), 1, 1, fc="white", ec="white", lw=0, alpha=0)] * 5

    fig = plt.figure()
    ax = fig.add_subplot()
    bins = np.arange(0,0.1,0.001)
    n, bins, patches = plt.hist(x=cd_mesh, weights=np.ones(len(cd_mesh))/len(cd_mesh), bins=bins, color="red", edgecolor='black', alpha=0.5, label='RMADER')
    # plt.axvline(x=dc/1000, color="red")
    ax.set_xticks(np.arange(0,0.1,0.001), fontproperties=font)
    # ax.set_xticklabels(np.arange(0,100,10), fontproperties=font)
    ax.set_yticks(np.arange(0,0.8,0.2), fontproperties=font)
    ax.set_yticklabels(np.arange(0,80,20), fontproperties=font)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    # plt.rcParams["font.family"] = "Times New Roman"
    plt.grid(axis='y', color='black', alpha=0.2)
    # plt.title('Comm delay histogram \n max comm_delay is '+str(round(max_comm_delay*1000))+' [ms]')
    # l2 = ax.legend(handles, textstr_rmader, loc='best', fontsize=16, fancybox=True, framealpha=0.7, handlelength=0, handletextpad=0, prop=font)
    # l3 = ax.legend(prop=font)
    plt.xlabel(r"$\delta_\mathrm{actual}$ [ms]", fontproperties=font)
    plt.yticks(fontproperties=font)
    # plt.ylabel("Number of Messages", fontproperties=font)
    plt.gca().yaxis.set_major_formatter(PercentFormatter(1))
    plt.ylabel("Probability", fontproperties=font)
    plt.savefig(home_dir+figname, bbox_inches="tight")
    plt.close('all')
    print('done!')
    sys.exit()

    