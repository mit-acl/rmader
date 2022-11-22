#!/usr/bin/env python
# coding=utf-8

# /* ----------------------------------------------------------------------------
#  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Jesus Tordesillas, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import rospy
import math
import os
import sys
import time
from random import *
# import numpy as np
# from pyquaternion import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def create_session(session_name, commands):

    os.system("tmux new-session -d -s "+str(session_name)+" -x 300 -y 300")

    for i in range(len(commands)):
        # print('splitting ',i)
        os.system('tmux new-window -t ' + str(session_name))
   
    for i in range(len(commands)):
        # os.system('tmux send-keys -t '+str(session_name)+':1.'+str(i+1) +' "'+ commands[i]+'" '+' C-m')
        os.system('tmux send-keys -t '+str(session_name)+':'+str(i) +'.0 "'+ commands[i]+'" '+' C-m')
    
    # os.system('tmux send-keys -t '+str(session_name)+':1.'+str(i+1) +' "tmux kill-server" ')
    # os.system('tmux send-keys -t '+str(session_name)+':0.'+str(i+1) +' "tmux kill-server" ')

    print("Commands sent")


def convertToStringCommand(action,sim_id,folder,veh,num,x,y,z,goal_x,goal_y,goal_z,yaw, wait_time):
    # if(action=="base_station"):
    #     return "roslaunch rmader base_station.launch type_of_environment:=dynamic_forest";
    if(action=="controller"):
        quad = veh + num + "s"
        return "roslaunch --wait rmader perfect_tracker_and_sim.launch quad:=" + quad + " x:=" + str(x) + " y:=" + str(y)
    if(action=="send_goal"):
        quad = veh + num + "s";
        # print("sleep "+wait_time+" && rostopic pub /"+quad+"/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: "+str(goal_x)+", y: "+str(goal_y)+", z: "+str(goal_z)+"}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'")
        return "sleep "+wait_time+" && rostopic pub /"+quad+"/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: "+str(goal_x)+", y: "+str(goal_y)+", z: "+str(goal_z)+"}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'"
        # return "rostopic pub /"+quad+"/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: "+str(goal_x)+", y: "+str(goal_y)+", z: "+str(goal_z)+"}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'"
    if(action=="rmader"):
        # print(str(folder))
        # return "roslaunch --wait rmader onboard.launch veh:="+veh+" num:="+num+" 2>&1 | tee "+str(folder)+"/"+str(sim_id)+"_"+veh+num+"_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt"
        return "roslaunch --wait rmader onboard.launch veh:="+veh+" num:="+num
        # return "roslaunch rmader onboard.launch veh:="+veh+" num:="+num #+ " >> "+quad+".txt" #Kota comment: this line launches mader.launch with the argument of quad number
        # return "script -q -c 'roslaunch rmader rmader.launch quad:="+quad + "' "+quad+".txt"
        
if __name__ == '__main__':

    # is_sequential_start_wait_time
    is_sequential_start_wait_time = True
    time_separation = 0.25 #seconds

    # formation="sphere", "square" "circle"
    formation="circle"
    commands = []
    # print("sys.argv[0]",sys.argv[0]) # reserved for file name (launch_many_drones.py)
    action = str(sys.argv[1]) # action
    sim_id = str(sys.argv[2]) # sim_id
    folder = str(sys.argv[3]) # folder location
    num_of_agents = int(sys.argv[4]) # num_of_agents
    radius = float(sys.argv[5]) # radius

    print(action)
    print(sim_id)
    print(folder)
    print(num_of_agents)
    print(radius)

    if(formation=="sphere"):
        # num_mer=int(math.sqrt(num_of_agents)); #Num of meridians
        # num_of_agents_per_mer=int(math.sqrt(num_of_agents));    #Num of agents per meridian
        if(num_of_agents%3==0):
            num_mer=max(int(num_of_agents/4.0),3); #Num of meridians
        else: #either divisible by 4 or will force it by changing num of agents
            num_mer=max(int(num_of_agents/4.0),4); #Num of meridians
        num_of_agents_per_mer=int(num_of_agents/num_mer);    #Num of agents per meridian

    if(formation=="circle" or formation=="square"):
        num_mer=num_of_agents
        num_of_agents_per_mer=1

    print("num_mer= ", num_mer)
    print("num_of_agents_per_mer= ", num_of_agents_per_mer)

    id_number=1
    shift_z=radius;
    shift_z=1.0 # should always above 0. look at terminal goal CB in mader_ros.cpp

    #TODO: Implement the square as well for other number_of_agents
    square_starts=[[4.0, 0.0, 1.0], 
                    [4.0, 4.0, 1.0], 
                    [0.0, 4.0, 1.0], 
                    [-4.0, 4.0, 1.0],
                    [-4.0, 0.0, 1.0],
                    [-4.0, -4.0, 1.0],
                    [0.0, -4.0, 1.0],
                    [4.0, -4.0, 1.0] ]

    square_goals=  [[-4.0, 0.0, 1.0],
                    [-4.0, -4.0, 1.0],
                    [0.0, -4.0, 1.0],
                    [4.0, -4.0, 1.0],
                    [4.0, 0.0, 1.0],
                    [4.0, 4.0, 1.0],
                    [0.0, 4.0, 1.0],
                    [-4.0, 4.0, 1.0]];

    square_yaws_deg=  [-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0];

    for i in range(1, num_mer+1):
        theta=0.0+i*(2*math.pi/num_mer);
        for j in range(1, num_of_agents_per_mer+1):

            phi=(-math.pi +j*(math.pi/(num_of_agents_per_mer+1)))
            x=radius*math.cos(theta)*math.sin(phi)
            y=radius*math.sin(theta)*math.sin(phi)
            z=shift_z + radius*math.cos(phi)

            pitch=0.0;
            roll=0.0;
            yaw= theta#+math.pi  

            goal_x=radius*math.cos(theta+2*math.pi)*math.sin(phi+math.pi)
            goal_y=radius*math.sin(theta+2*math.pi)*math.sin(phi+math.pi)
            # goal_z=shift_z + radius*math.cos(phi+math.pi)
            goal_z=1.0 # should always above 0. look at terminal goal CB in mader_ros.cpp
                
            # quad="SQ0" + str(id_number) + "s";
            veh="SQ";
            if i <= 9:
                num="0" + str(id_number)
            else:
                num=str(id_number)

            id_number=id_number+1

            # sequential_start_wait_time
            wait_time = 0.0
            if (is_sequential_start_wait_time):
                wait_time = time_separation * i
            wait_time = str(wait_time)

            if(formation=="square"):
                x=square_starts[i-1][0];
                y=square_starts[i-1][1];
                z=square_starts[i-1][2];

                goal_x=square_goals[i-1][0];
                goal_y=square_goals[i-1][1];
                goal_z=square_goals[i-1][2];

                yaw=square_yaws_deg[i-1]*math.pi/180;
                print("yaw= ", square_yaws_deg[i-1])

            commands.append(convertToStringCommand(action,sim_id,folder,veh,num,x,y,z,goal_x,goal_y,goal_z, yaw, wait_time));

            x_tmp="{:5.3f}".format(x);
            y_tmp="{:5.3f}".format(y);
            z_tmp="{:5.3f}".format(z);

            goal_x_tmp="{:5.3f}".format(goal_x);
            goal_y_tmp="{:5.3f}".format(goal_y);
            goal_z_tmp="{:5.3f}".format(goal_z);
 
            print (' "start": [',x_tmp,', ',y_tmp,', ',z_tmp,'], "goal": [',goal_x_tmp,', ',goal_y_tmp,', ',goal_z_tmp,']  ')

    print("len(commands)= " , len(commands))
    session_name=action+"_session"
    os.system("tmux kill-session -t" + session_name)
    create_session(session_name, commands) #Kota commented out July 16, 2021
    # if(action!="send_goal"):
    #     # os.system("tmux attach") #comment if you don't want to visualize all the terminals
    #     time.sleep(1); #Kota added to make this "if statement" works even when i comment out the above line
    # else: ##if send_goal, kill after some time
    #     time.sleep(num_of_agents); #The more agents, the more I have to wait to make sure the goal is sent correctly
    #     os.system("tmux kill-session -t" + session_name)
