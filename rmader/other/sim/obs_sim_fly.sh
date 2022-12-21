#!/bin/bash

# this script creates multiple windows for fly and mader
# referred to fly script used for voxl setup, which is in /extras

# if deleting old .bag and .active in voxl and .txt on nuc
snapsim_or_perfect=$1

# session name
SESSION=sim_rmader
WINDOW=base_station

# creates the session with a name and renames the window name
cmd="new-session -d -s $SESSION -x- -y-; rename-window $WINDOW"
tmux -2 $cmd

# window number
w=0

#split tmux
for i in {1..6}
do
	tmux split-window -h
	tmux select-layout -t $SESSION:$w.$i even-horizontal
	tmux select-pane -t $SESSION:$w.$i
done

for i in 0 2 4 6 8 10 12
do 
	tmux select-pane -t $SESSION:$w.$i
	tmux split-window -v
done

# for i in 0 2 4 6 8 10
# do
# 	tmux resize-pane -t $SESSION:$w.$i -y 300
# done

# wait for .bashrc to load
sleep 1

# roscore
tmux send-keys -t $SESSION:$w.0 "roscore" C-m

# run mader hw_onboard and save termial data into txt files

tmux send-keys -t $SESSION:$w.2 "roslaunch --wait rmader onboard.launch veh:=SQ num:=01 x:=-3 y:=3" C-m 
tmux send-keys -t $SESSION:$w.4 "roslaunch --wait rmader onboard.launch veh:=SQ num:=02 x:=3 y:=3" C-m
tmux send-keys -t $SESSION:$w.6 "roslaunch --wait rmader onboard.launch veh:=SQ num:=03 x:=-3 y:=-3" C-m
tmux send-keys -t $SESSION:$w.8 "roslaunch --wait rmader onboard.launch veh:=SQ num:=04 x:=3 y:=-3" C-m
tmux send-keys -t $SESSION:$w.10 "roslaunch --wait rmader hw_obstacle.launch veh:=SQ num:=05 x:=0 y:=0" C-m
# tmux send-keys -t $SESSION:$w.12 "roslaunch --wait rmader onboard.launch veh:=SQ num:=06 x:=3 y:=-3" C-m

# snap sim or perfect tracker
if [[ $snapsim_or_perfect == "snap_sim" ]]; then
	tmux send-keys -t $SESSION:$w.3 "roslaunch --wait snap_sim sim.launch veh:=SQ num:=01 x:=-3 y:=3" C-m
	tmux send-keys -t $SESSION:$w.5 "roslaunch --wait snap_sim sim.launch veh:=SQ num:=02 x:=3 y:=3" C-m
	tmux send-keys -t $SESSION:$w.7 "roslaunch --wait snap_sim sim.launch veh:=SQ num:=03 x:=-3 y:=-3" C-m
	tmux send-keys -t $SESSION:$w.9 "roslaunch --wait snap_sim sim.launch veh:=SQ num:=04 x:=3 y:=-3" C-m
	tmux send-keys -t $SESSION:$w.11 "roslaunch --wait snap_sim sim.launch veh:=SQ num:=05 x:=0 y:=0" C-m
	# tmux send-keys -t $SESSION:$w.13 "roslaunch --wait snap_sim sim.launch veh:=SQ num:=06 x:=3 y:=-3" C-m
else
	tmux send-keys -t $SESSION:$w.3 "roslaunch --wait rmader perfect_tracker_and_sim.launch quad:=SQ01s x:=-3 y:=3" C-m
	tmux send-keys -t $SESSION:$w.5 "roslaunch --wait rmader perfect_tracker_and_sim.launch quad:=SQ02s x:=3 y:=3" C-m
	tmux send-keys -t $SESSION:$w.7 "roslaunch --wait rmader perfect_tracker_and_sim.launch quad:=SQ03s x:=-3 y:=-3" C-m
	tmux send-keys -t $SESSION:$w.9 "roslaunch --wait rmader perfect_tracker_and_sim.launch quad:=SQ04s x:=3 y:=-3" C-m
	tmux send-keys -t $SESSION:$w.11 "roslaunch --wait rmader perfect_tracker_and_sim.launch quad:=SQ05s x:=0 y:=0" C-m
	# tmux send-keys -t $SESSION:$w.13 "roslaunch --wait rmader perfect_tracker_and_sim.launch quad:=SQ06s x:=3 y:=-3" C-m
fi

# base station
tmux send-keys -t $SESSION:$w.1 "roslaunch --wait rmader base_station.launch" C-m

tmux -2 attach-session -t $SESSION