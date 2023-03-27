#!/bin/bash

# this script creates multiple windows for fly and rmader
# referred to fly script used for voxl setup, which is in /extras

# session name
SESSION=rmader
WINDOW=base_station

# creates the session with a name and renames the window name
cmd="new-session -d -s $SESSION -x- -y-; rename-window $WINDOW"
tmux -2 $cmd

# window number
w=0

#split tmux into 2x2
for i in 1 2
do
	tmux split-window -h
	tmux select-layout -t $SESSION:$w.$i even-horizontal
	tmux select-pane -t $SESSION:$w.$i
done

for i in 0 2
do 
	tmux select-pane -t $SESSION:$w.$i
	tmux split-window -v
done

for i in 1 3
do
	tmux resize-pane -t $SESSION:$w.$i -y 20
done

# wait for .bashrc to load
sleep 1

# send commands to each pane
# ssh each voxl 
tmux send-keys -t $SESSION:$w.0 "ssh root@nx08.local" C-m
tmux send-keys -t $SESSION:$w.2 "ssh root@nx09.local" C-m

sleep 3

for i in 0 2 
do
	tmux send-keys -t $SESSION:$w.$i "./nuc_voxl_connection"
done

# ssh each nuc
tmux send-keys -t $SESSION:$w.1 "ssh nuc08@192.168.22.2" C-m
tmux send-keys -t $SESSION:$w.3 "ssh nuc9@192.168.23.2" C-m

sleep 1

# run rmader hw_onboard and save termial data into txt files

# ntp date
tmux send-keys -t $SESSION:$w.1 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.3 "sudo ntpdate time.nist.gov" C-m

sleep 10

# for i in 1 7 11 # RUNNING MESH WITH NETWORK MANAGER NX01, 04, 05, 07, 08
# do
# 	tmux send-keys -t $SESSION:$w.$i "cd && ./ad_hoc_without_NM.sh" C-m
# done

sleep 5

tmux send-keys -t $SESSION:$w.1 "roslaunch rmader onboard.launch quad:=NX08" C-m
tmux send-keys -t $SESSION:$w.3 "roslaunch rmader onboard.launch quad:=NX09" C-m

# run the nuc connection command and fly command (fly script record related rostopics so the below command should be sent at last)
sleep 5 

for i in 0 2
do
	# tmux send-keys -t $SESSION:$w.$i "tmux kill-server" C-m
	tmux send-keys -t $SESSION:$w.$i "fly" C-m
done

tmux -2 attach-session -t $SESSION