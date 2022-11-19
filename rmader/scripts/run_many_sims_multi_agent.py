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

def checkGoalReached(num_of_agents):
    try:
        is_goal_reached = subprocess.check_output(['rostopic', 'echo', '/goal_reached', '-n', '1'], timeout=2).decode()
        print("True")
        return True 
    except:
        print("False")
        return False        

def myhook():
  print("shutdown time!")

if __name__ == '__main__':

    ##### parameters
    is_docker = True
    num_of_sims=5
    num_of_agents=15
    radius=15
    how_long_to_wait=60 #[s]
    cd_list = [0, 50, 100, 200, 300]

    ##### loop
    for cd in cd_list:
        is_oldmader=True
        if cd == 0: 
            dc_list = [0, 100] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 50:
            dc_list = [0, 150] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 100:
            dc_list = [0, 200] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 200:
            dc_list = [0, 300]
        elif cd == 300:
            dc_list = [0, 400]

        for dc in dc_list:
            dc_in_ms = dc/1000;
            cd_in_ms = cd/1000;
            str_dc = str(dc)

            #### rmader.yaml modification. comment out delay_check param and is_delaycheck param
            os.system("sed -i '/is_delaycheck/s/^/#/g' $(rospack find rmader)/param/rmader.yaml")
            os.system("sed -i '/delay_check_sec/s/^/#/g' $(rospack find rmader)/param/rmader.yaml")
            os.system("sed -i '/simulated_comm_delay_sec/s/^/#/g' $(rospack find rmader)/param/rmader.yaml")

            #### folder name
            if is_docker: ## using Docker on lambda machine
                source_dir = "/home/data"
                if is_oldmader:
                    folder_bags=source_dir + "/oldmader/bags/cd"+str(cd)+"ms"
                    folder_txts=source_dir + "/oldmader/txt_files/cd"+str(cd)+"ms"
                    folder_csv=source_dir + "/oldmader/csv/cd"+str(cd)+"ms"
                else:
                    folder_bags=source_dir +"/rmader/bags/cd"+str(cd)+"ms/dc"+str_dc+"ms"
                    folder_txts=source_dir +"/rmader/txt_files/cd"+str(cd)+"ms/dc"+str_dc+"ms"
                    folder_csv=source_dir +"/rmader/csv/cd"+str(cd)+"ms/dc"+str_dc+"ms"
            else: ## on my desktop
                source_dir = "/home/kota/test/data"
                if is_oldmader:
                    folder_bags=source_dir + "/oldmader/bags/cd"+str(cd)+"ms"
                    folder_txts=source_dir + "/oldmader/txt_files/cd"+str(cd)+"ms"
                    folder_csv=source_dir + "/oldmader/csv/cd"+str(cd)+"ms"
                else:
                    folder_bags=source_dir + "/rmader/bags/cd"+str(cd)+"ms/dc"+str_dc+"ms"
                    folder_txts=source_dir + "/rmader/txt_files/cd"+str(cd)+"ms/dc"+str_dc+"ms"
                    folder_csv=source_dir + "/rmader/csv/cd"+str(cd)+"ms/dc"+str_dc+"ms"

            # create directy if not exists
            if (not os.path.exists(folder_bags)):
                os.makedirs(folder_bags)

            # create directy if not exists
            # if (not os.path.exists(folder_txts)):
            #     os.makedirs(folder_txts)

             # create directy if not exists
            # if (not os.path.exists(folder_csv)):
            #     os.makedirs(folder_csv)        

            # name_node_record="bag_recorder"
            kill_all="tmux kill-server & killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roscore & killall -9 rosmaster & pkill rmader_node & pkill -f dynamic_obstacles & pkill -f rosout & pkill -f behavior_selector_node & pkill -f rviz & pkill -f rqt_gui & pkill -f perfect_tracker & pkill -f rmader_commands"

            #make sure ROS (and related stuff) is not running
            os.system(kill_all)

            for k in range(num_of_sims):

                if k <= 9:
                    sim_id = "0"+str(k)
                else:
                    sim_id = str(k)

                commands = []
                name_node_record="bag_recorder"
                commands.append("roscore")

                for num in range(1,num_of_agents+1):
                    if num <= 9:
                        agent_id = "0"+str(num)
                    else:
                        agent_id = str(num)

                    commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/rmader/tuning_param/delay_check_sec "+str(dc_in_ms))
                    commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/rmader/tuning_param/simulated_comm_delay_sec "+str(cd_in_ms))
                    if is_oldmader:
                        commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/rmader/is_delaycheck false")
                    else:
                        commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/rmader/is_delaycheck true")

                commands.append("sleep 4.0 && roscd rmader && cd scripts && python launch_many_drones.py controller "+sim_id+" "+folder_txts+" "+str(num_of_agents)+" "+str(radius))
                commands.append("sleep 4.0 && roscd rmader && cd scripts && python launch_many_drones.py rmader "+sim_id+" "+folder_txts+" "+str(num_of_agents)+" "+str(radius))
                # commands.append("sleep 4.0 && cd "+folder_bags+" && rosbag record -a -o sim_" + sim_id + " __name:="+name_node_record)
                commands.append("sleep 4.0 && cd "+folder_bags+" && rosbag record -e '(.*)state(.*)' '(.*)drone_marker(.*)' '(.*)actual_traj(.*)' '(.*)traj_safe_colored(.*)' '(.*)traj_safe_colored_bef_commit(.*)' -o sim_" + sim_id + " __name:="+name_node_record)
                commands.append("sleep 4.0 && roslaunch --wait rmader collision_detector.launch num_of_agents:=" + str(num_of_agents))
                # commands.append("sleep 4.0 && roslaunch --wait rmader ave_distance.launch num_of_agents:="+str(num_of_agents)+" folder_loc:="+folder_csv+" sim:="+sim_id)
                # commands.append("sleep 4.0 && rvmd")

                #publishing the goal should be the last command
                commands.append("sleep 15.0 && roscd rmader && cd scripts && python launch_many_drones.py send_goal "+sim_id+" "+folder_txts+" "+str(num_of_agents)+" "+str(radius))
                commands.append("sleep 15.0 && roslaunch --wait rmader goal_reached.launch") #we are calculating completion time here so sleep time needs to be the same as send_goal
                commands.append("sleep 15.0 && tmux detach")

                # print("len(commands)= " , len(commands))
                session_name="run_many_sims_multi_agent_session"
                os.system("tmux kill-session -t" + session_name)
                os.system("tmux new-session -d -s "+str(session_name)+" -x 300 -y 300")

                # tmux splitting
                for i in range(len(commands)):
                    # print('splitting ',i)
                    os.system('tmux new-window -t ' + str(session_name))
               
                for i in range(len(commands)):
                    os.system('tmux send-keys -t '+str(session_name)+':'+str(i) +'.0 "'+ commands[i]+'" '+' C-m')

                os.system("tmux attach")
                print("Commands sent")

                # rospy.init_node('goalReachedCheck_in_sims', anonymous=True)
                # c = GoalReachedCheck_sim()
                # rospy.Subscriber("goal_reached", GoalReached, c.goal_reachedCB)
                # rospy.on_shutdown(myhook)

                # check if all the agents reached the goal
                is_goal_reached = False
                tic = time.perf_counter()
                toc = time.perf_counter()
                os.system('tmux new-window -t ' + str(session_name))

                while (toc - tic < how_long_to_wait and not is_goal_reached):
                    toc = time.perf_counter()
                    if(checkGoalReached(num_of_agents)):
                        print('all the agents reached the goal')
                        time.sleep(2) # gives us time to write csv file for ave distance
                        is_goal_reached = True
                    time.sleep(0.1)

                if (not is_goal_reached):
                    os.system('echo "simulation '+sim_id+': not goal reached" >> '+folder_bags+'/status.txt')
                else:
                    os.system('echo "simulation '+sim_id+': goal reached" >> '+folder_bags+'/status.txt')

                os.system("rosnode kill "+name_node_record);
                # os.system("rosnode kill goalReachedCheck_in_sims")
                # os.system("rosnode kill -a")
                time.sleep(1.0)
                os.system(kill_all)
                time.sleep(1.0)

            # uncomment delay_check param
            os.system("sed -i '/delay_check/s/^#//g' $(rospack find rmader)/param/rmader.yaml")
            os.system("sed -i '/is_delaycheck/s/^#//g' $(rospack find rmader)/param/rmader.yaml")

            # use old mader only once
            if is_oldmader:
                is_oldmader=False
            time.sleep(3.0)

    # After the simulations
    session_name="data"
    os.system("tmux kill-session -t" + session_name)
    os.system("tmux new-session -d -s "+str(session_name)+" -x 300 -y 300")

    commands = []
    commands.append("sleep 3.0 && roscd rmader && cd other/sim && python collision_check.py "+source_dir)
    commands.append("sleep 3.0 && roscd rmader && cd other/sim && python completion_time.py "+source_dir)
    commands.append("sleep 3.0 && roscd rmader && cd other/sim && python comm_delay_histogram_percentile.py")
    # commands.append("sleep 3.0 && roscd rmader && cd other/sim && python ave_distance_csv2txt.py")
    # commands.append("sleep 3.0 && roscd rmader && cd other/sim && python missed_msgs_count.py")

    # tmux splitting
    for i in range(len(commands)):
        # print('splitting ',i)
        os.system('tmux new-window -t ' + str(session_name))
   
    time.sleep(3.0)

    for i in range(len(commands)):
        os.system('tmux send-keys -t '+str(session_name)+':'+str(i) +'.0 "'+ commands[i]+'" '+' C-m')

    print("Commands sent")