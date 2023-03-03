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
tmux send-keys -t $SESSION:$w.0 "ssh nuc7@192.168.21.2" C-m
tmux send-keys -t $SESSION:$w.1 "ssh nuc2@192.168.16.2" C-m
tmux send-keys -t $SESSION:$w.2 "ssh nuc3@192.168.17.2" C-m
tmux send-keys -t $SESSION:$w.3 "ssh nuc08@192.168.22.2" C-m
tmux send-keys -t $SESSION:$w.4 "ssh nuc5@192.168.19.2" C-m
tmux send-keys -t $SESSION:$w.5 "ssh nuc9@192.168.23.2" C-m

sleep 1

# send goals (randomly generated or position exchange)
if [ "$1" == "pos" ]; then
	tmux send-keys -t $SESSION:$w.0 "roslaunch rmader position_exchange.launch mode:=1 quad:=SQ01s"  
	tmux send-keys -t $SESSION:$w.1 "roslaunch rmader position_exchange.launch mode:=2 quad:=SQ02s"  
	tmux send-keys -t $SESSION:$w.2 "roslaunch rmader position_exchange.launch mode:=3 quad:=SQ03s" 
	tmux send-keys -t $SESSION:$w.3 "roslaunch rmader position_exchange.launch mode:=4 quad:=SQ04s"  
	tmux send-keys -t $SESSION:$w.4 "roslaunch rmader position_exchange.launch mode:=5 quad:=SQ05s"  
	tmux send-keys -t $SESSION:$w.5 "roslaunch rmader position_exchange.launch mode:=6 quad:=SQ06s"

	# with obstacles
	# tmux send-keys -t $SESSION:$w.0 "roslaunch mader position_exchange.launch mode:=1 quad:=NX01"
	# tmux send-keys -t $SESSION:$w.2 "roslaunch mader position_exchange.launch mode:=3 quad:=NX03"
	# tmux send-keys -t $SESSION:$w.3 "roslaunch mader position_exchange.launch mode:=4 quad:=NX05"
	# tmux send-keys -t $SESSION:$w.3 "roslaunch mader position_exchange.launch mode:=6 quad:=NX07"
	# tmux send-keys -t $SESSION:$w.4 "roslaunch mader position_exchange.launch mode:=5 quad:=NX08" # this is obstacle
	# tmux send-keys -t $SESSION:$w.5 "roslaunch mader position_exchange.launch mode:=4 quad:=NX09" # this is obstacle
fi

# elif [ "$1" == "ran" ]; then
# 	# tmux send-keys -t $SESSION:$w.0 "roslaunch mader random_goal.launch quad:=NX01"
# 	# tmux send-keys -t $SESSION:$w.1 "roslaunch mader random_goal.launch quad:=NX02"
# 	# tmux send-keys -t $SESSION:$w.2 "roslaunch mader random_goal.launch quad:=NX03"
# 	# # tmux send-keys -t $SESSION:$w.3 "roslaunch mader random_goal.launch quad:=NX04"
# 	# tmux send-keys -t $SESSION:$w.3 "roslaunch mader random_goal.launch quad:=NX09"
# 	# # tmux send-keys -t $SESSION:$w.3 "roslaunch mader random_goal.launch quad:=NX10"
# 	# tmux send-keys -t $SESSION:$w.4 "roslaunch mader random_goal.launch quad:=NX05"
# 	# # tmux send-keys -t $SESSION:$w.5 "roslaunch mader random_goal.launch quad:=NX06"
# 	# tmux send-keys -t $SESSION:$w.5 "roslaunch mader random_goal.launch quad:=NX07"
# fi

tmux -2 attach-session -t $SESSION