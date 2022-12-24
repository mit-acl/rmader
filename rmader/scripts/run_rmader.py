#!/usr/bin/env python
# coding=utf-8

# /* ----------------------------------------------------------------------------
#  * Copyright 2022, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import math
import os
import sys
import time
import rospy
from snapstack_msgs.msg import State
import subprocess
import numpy as np
import time
from numpy import linalg as LA
import struct
from rmader_msgs.msg import GoalReached

if __name__ == '__main__':

    ##### parameters
    is_docker = True
    num_of_agents=10
    radius=10 #[m]
    how_long_to_wait=60 #[s]
    comm_delay = 50 #[ms]
    delay_check = 125 #[ms]
    sim_id = "0"
    kill_all="tmux kill-server & killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roscore & killall -9 rosmaster & pkill rmader_node & pkill -f dynamic_obstacles & pkill -f rosout & pkill -f behavior_selector_node & pkill -f rviz & pkill -f rqt_gui & pkill -f perfect_tracker & pkill -f rmader_commands"
    folder_txts = 'a' #this is not used

    ##### make sure ROS (and related stuff) is not running
    os.system(kill_all)

    ##### add commands to the commands list
    commands = []
    commands.append("roscore")

    for num in range(1,num_of_agents+1):
        if num <= 9:
            agent_id = "0"+str(num)
        else:
            agent_id = str(num)

    # start controller for each UAV
    commands.append("sleep 5.0 && roscd rmader && cd scripts && python launch_many_drones.py controller "+sim_id+" "+folder_txts+" "+str(num_of_agents)+" "+str(radius))
    # start rmader for each UAV
    commands.append("sleep 5.0 && roscd rmader && cd scripts && python launch_many_drones.py rmader "+sim_id+" "+folder_txts+" "+str(num_of_agents)+" "+str(radius))
    # start rviz
    commands.append("sleep 5.0 && roscd rmader && cd rviz_cfgs && rosrun rviz rviz -d rmader.rviz")

    # publish the goal command
    commands.append("sleep 10.0 && roscd rmader && cd scripts && python launch_many_drones.py send_goal "+sim_id+" "+folder_txts+" "+str(num_of_agents)+" "+str(radius))
    commands.append("sleep 10.0 && tmux detach")

    # tmux session housekeeping
    session_name="run_sim_multiagent_session"
    os.system("tmux kill-session -t" + session_name)
    os.system("tmux new-session -d -s "+str(session_name)+" -x 300 -y 300")

    # tmux splitting
    for i in range(len(commands)):
        os.system('tmux new-window -t ' + str(session_name))
    for i in range(len(commands)):
        os.system('tmux send-keys -t '+str(session_name)+':'+str(i) +'.0 "'+ commands[i]+'" '+' C-m')

    os.system("tmux attach")
    print("Commands sent")

    print("If you want to kill the simulation, hit RETURN.")
    input("")

    # os.system('tmux new-window -t ' + str(session_name))
    time.sleep(1.0)
    os.system(kill_all)