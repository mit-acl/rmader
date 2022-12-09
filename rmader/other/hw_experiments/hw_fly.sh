#!/bin/bash

# this script creates multiple windows for fly and rmader
# referred to fly script used for voxl setup, which is in /extras

# if deleting old .bag and .active in voxl and .txt on nuc
ifDELETE=$1

# session name
SESSION=rmader
WINDOW=base_station

# creates the session with a name and renames the window name
cmd="new-session -d -s $SESSION -x- -y-; rename-window $WINDOW"
tmux -2 $cmd

# window number
w=0

#split tmux into 2x6
for i in {1..7}
do
	tmux split-window -h
	tmux select-layout -t $SESSION:$w.$i even-horizontal
	tmux select-pane -t $SESSION:$w.$i
done

for i in 0 2 4 6 8 10 12 14
do 
	tmux select-pane -t $SESSION:$w.$i
	tmux split-window -v
done

# for i in 0 2 4 6 8 10
# do
# 	tmux resize-pane -t $SESSION:$w.$i -y 300
# done

for i in 1 3 5 7 9 11
do
	tmux resize-pane -t $SESSION:$w.$i -y 20
done

# wait for .bashrc to load
sleep 1

# send commands to each pane
# ssh each voxl 
# tmux send-keys -t $SESSION:$w.0 "ssh root@nx01.local" C-m
tmux send-keys -t $SESSION:$w.0 "ssh root@nx01.local" C-m
tmux send-keys -t $SESSION:$w.2 "ssh root@nx02.local" C-m
tmux send-keys -t $SESSION:$w.4 "ssh root@nx05.local" C-m
tmux send-keys -t $SESSION:$w.6 "ssh root@nx07.local" C-m
tmux send-keys -t $SESSION:$w.8 "ssh root@nx08.local" C-m
tmux send-keys -t $SESSION:$w.10 "ssh root@nx09.local" C-m  
tmux send-keys -t $SESSION:$w.12 "ssh root@nx06.local" C-m
tmux send-keys -t $SESSION:$w.14 "ssh root@nx10.local" C-m
# tmux send-keys -t $SESSION:$w.6 "ssh root@nx07.local" C-m
# tmux send-keys -t $SESSION:$w.10 "ssh root@nx09.local" C-m


sleep 5

for i in 0 2 4 6 8 10 
do
	tmux send-keys -t $SESSION:$w.$i "./nuc_voxl_connection" C-m
done

sleep 3

# ssh each nuc
# tmux send-keys -t $SESSION:$w.1 "ssh nuc1@192.168.15.2" C-m
tmux send-keys -t $SESSION:$w.1 "ssh nuc1@192.168.15.2" C-m
tmux send-keys -t $SESSION:$w.3 "ssh nuc2@192.168.16.2" C-m
tmux send-keys -t $SESSION:$w.5 "ssh nuc5@192.168.19.2" C-m
tmux send-keys -t $SESSION:$w.7 "ssh nuc7@192.168.21.2" C-m
tmux send-keys -t $SESSION:$w.9 "ssh nuc08@192.168.22.2" C-m
tmux send-keys -t $SESSION:$w.11 "ssh nuc9@192.168.23.2" C-m
# tmux send-keys -t $SESSION:$w.11 "ssh nuc08@192.168.22.2" C-m
# tmux send-keys -t $SESSION:$w.7 "ssh nuc@192.168.21.2" C-m
# tmux send-keys -t $SESSION:$w.9 "ssh nuc08@192.168.22.2" C-m
# tmux send-keys -t $SESSION:$w.11 "ssh nuc9@192.168.23.2" C-m

sleep 1

# run rmader hw_onboard and save termial data into txt files

# ntp date
tmux send-keys -t $SESSION:$w.1 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.3 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.5 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.7 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.9 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.11 "sudo ntpdate time.nist.gov" C-m

sleep 15

if [[ $ifDELETE == 'true' ]]; then 
	tmux send-keys -t $SESSION:$w.1 "cd /home/nuc1/Research/bags && rm *.txt && cd" C-m
	tmux send-keys -t $SESSION:$w.3 "cd /home/nuc2/Research/bags && rm *.txt && cd" C-m
	tmux send-keys -t $SESSION:$w.5 "cd /home/nuc5/Research/bags && rm *.txt && cd" C-m
	tmux send-keys -t $SESSION:$w.7 "cd /home/nuc7/Research/bags && rm *.txt && cd" C-m
	tmux send-keys -t $SESSION:$w.9 "cd /home/nuc08/Research/bags && rm *.txt && cd" C-m
	tmux send-keys -t $SESSION:$w.11 "cd /home/nuc9/Research/bags && rm *.txt && cd" C-m
	# tmux send-keys -t $SESSION:$w.9 "cd /home/nuc5/Research/bags && rm *.txt" C-m
	# tmux send-keys -t $SESSION:$w11 "cd /home/nuc6/Research/bags && rm *.txt" C-m
	# tmux send-keys -t $SESSION:$w.7 "cd /home/nuc/Research/bags && rm *.txt" C-m
	# tmux send-keys -t $SESSION:$w.9 "cd /home/nuc08/Research/bags && rm *.txt" C-m
	# tmux send-keys -t $SESSION:$w.11 "cd /home/nuc9/Research/bags && rm *.txt" C-m
	
fi

# for i in 5 7 11 # RUNNING MESH WITH NETWORK MANAGER NX04, 05, 08
# do
# 	tmux send-keys -t $SESSION:$w.$i "cd && ./ad_hoc_without_NM.sh" C-m
# done

sleep 5

# hw_onboard
# tmux send-keys -t $SESSION:$w.1 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc1/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX01) 2>&1 | tee ~/Research/bags/nx01_rrmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m #by using /home/nuc1/ instead of ~/, we can stop record data on sikorsky when we are not using the vehicle.
tmux send-keys -t $SESSION:$w.1 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc1/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX01) 2>&1 | tee ~/Research/bags/nx01_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
tmux send-keys -t $SESSION:$w.3 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc2/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX02) 2>&1 | tee ~/Research/bags/nx02_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
tmux send-keys -t $SESSION:$w.5 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc5/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX05) 2>&1 | tee ~/Research/bags/nx05_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
tmux send-keys -t $SESSION:$w.7 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc7/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX07) 2>&1 | tee ~/Research/bags/nx07_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
tmux send-keys -t $SESSION:$w.9 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc08/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX08) 2>&1 | tee ~/Research/bags/nx08_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
tmux send-keys -t $SESSION:$w.11 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc9/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX09) 2>&1 | tee ~/Research/bags/nx09_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
# tmux send-keys -t $SESSION:$w.11 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc6/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX06) 2>&1 | tee ~/Research/bags/nx05_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
# tmux send-keys -t $SESSION:$w.7 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX07) 2>&1 | tee ~/Research/bags/nx07_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
# tmux send-keys -t $SESSION:$w.9 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc08/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX08) 2>&1 | tee ~/Research/bags/nx08_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
# tmux send-keys -t $SESSION:$w.11 "(roscd rmader && git rev-parse HEAD && git diff --color && cd /home/nuc9/Research/bags/ && roslaunch rmader hw_onboard.launch quad:=NX09) 2>&1 | tee ~/Research/bags/nx09_rmader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m

# run the nuc connection command and fly command (fly script record related rostopics so the below command should be sent at last)
sleep 5 
for i in 0 2 4 6 8 10 12 14
do
	if [[ $ifDELETE == 'true' ]]; then
		tmux send-keys -t $SESSION:$w.$i "cd /data/bags && rm *.bag && rm *.active" C-m
		sleep 1
	fi
	# tmux send-keys -t $SESSION:$w.$i "tmux kill-server" C-m
	tmux send-keys -t $SESSION:$w.$i "fly" C-m
done

tmux -2 attach-session -t $SESSION