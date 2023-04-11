#!/bin/bash

# this script creates multiple windows to send goals to drons
# referred to fly script used for voxl setup, which is in /extras

# one_time_exchange code
ONE_TIME_EXCHANGE=$2

# session name
SESSION=goal
WINDOW=base_station

# creates the session with a name and renames the window name
cmd="new-session -d -s $SESSION ; rename-window $WINDOW"
tmux -2 $cmd

# window number
w=0

# split tmux into 8x2
for i in {0..2}
do
	tmux select-pane -t $SESSION:$w.$i
	tmux split-window -v
	tmux select-layout -t $SESSION:$w.$i even-vertical
done

for i in 0 2 4 6
do
	tmux select-pane -t $SESSION:$w.$i	
	tmux split-window -h
done

# wait for .bashrc to load
sleep 1

# ssh each nuc
# tmux send-keys -t $SESSION:$w.0 "ssh nuc1@192.168.15.2" C-m
# tmux send-keys -t $SESSION:$w.1 "ssh nuc9@192.168.23.2" C-m
tmux send-keys -t $SESSION:$w.0 "ssh nuc2@192.168.16.2" C-m
tmux send-keys -t $SESSION:$w.1 "ssh nuc3@192.168.17.2" C-m

sleep 1

# send goals (randomly generated or position exchange)
if [ "$1" == "pos" ]; then
	tmux send-keys -t $SESSION:$w.0 "roslaunch rmader position_exchange.launch mode:=1 quad:=NX02"  
	tmux send-keys -t $SESSION:$w.1 "roslaunch rmader position_exchange.launch mode:=2 quad:=NX03"
	# tmux send-keys -t $SESSION:$w.1 "roslaunch rmader position_exchange.launch mode:=2 quad:=NX09"
	# tmux send-keys -t $SESSION:$w.0 "roslaunch rmader position_exchange.launch mode:=1 quad:=NX01"  
fi

tmux -2 attach-session -t $SESSION