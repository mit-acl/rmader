#!/bin/bash

# this script creates multiple windows for fly and rmader
# referred to fly script used for voxl setup, which is in /extras

# session name
SESSION=rmader_fly
WINDOW=base_station

# creates the session with a name and renames the window name
cmd="new-session -d -s $SESSION -x- -y-; rename-window $WINDOW"
tmux -2 $cmd

# window number
w=0

#split tmux into 2x2
for i in 1 2 3
do
	tmux split-window -h
	tmux select-layout -t $SESSION:$w.$i even-horizontal
	tmux select-pane -t $SESSION:$w.$i
done

for i in 0 2 4
do 
	tmux select-pane -t $SESSION:$w.$i
	tmux split-window -v
done

for i in 1 3 5
do
	tmux resize-pane -t $SESSION:$w.$i -y 20
done

# wait for .bashrc to load
sleep 1

# send commands to each pane
# ssh each voxl 
tmux send-keys -t $SESSION:$w.0 "ssh root@nx02.local" C-m
tmux send-keys -t $SESSION:$w.2 "ssh root@nx09.local" C-m
tmux send-keys -t $SESSION:$w.4 "ssh root@nx06.local" C-m
# tmux send-keys -t $SESSION:$w.0 "ssh root@nx01.local" C-m
# tmux send-keys -t $SESSION:$w.2 "ssh root@nx03.local" C-m

sleep 3

for i in 0 2 
do
	tmux send-keys -t $SESSION:$w.$i "./nuc_voxl_connection" C-m
done

# ssh each nuc
tmux send-keys -t $SESSION:$w.1 "ssh nuc2@192.168.16.2" C-m
tmux send-keys -t $SESSION:$w.3 "ssh nuc9@192.168.23.2" C-m

# tmux send-keys -t $SESSION:$w.1 "ssh nuc1@192.168.15.2" C-m
# tmux send-keys -t $SESSION:$w.3 "ssh nuc3@192.168.17.2" C-m
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

tmux send-keys -t $SESSION:$w.1 "cd && ./ad_hoc_without_NM.sh" C-m
tmux send-keys -t $SESSION:$w.3 "cd && ./ad_hoc_without_NM.sh" C-m

sleep 8

tmux send-keys -t $SESSION:$w.1 "cd && ping 192.168.100.9" C-m
tmux send-keys -t $SESSION:$w.3 "cd && ping 192.168.100.2" C-m
# tmux send-keys -t $SESSION:$w.1 "cd && ping 192.168.100.3" C-m
# tmux send-keys -t $SESSION:$w.3 "cd && ping 192.168.100.1" C-m

tmux send-keys -t $SESSION:$w.1 "roslaunch rmader hw_onboard.launch quad:=NX02"
tmux send-keys -t $SESSION:$w.3 "roslaunch rmader hw_onboard.launch quad:=NX09"

# run the nuc connection command and fly command (fly script record related rostopics so the below command should be sent at last)
sleep 5 

for i in 0 2 4
do
	# tmux send-keys -t $SESSION:$w.$i "tmux kill-server" C-m
	tmux send-keys -t $SESSION:$w.$i "fly" C-m
done

tmux -2 attach-session -t $SESSION