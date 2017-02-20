#!/bin/sh

tmux new-window -n 'Awesomo Tower'

# launch awesomo
tmux send-keys "roslaunch awesomo_ros awesomo.launch" C-m
tmux split-window -h
tmux send-keys "cd ~/catkin_ws/src/awesomo/scripts/; vim awesomo_remote.py" C-m

# echo awesomo quadrotor pose
tmux select-pane -t 0
tmux split-window -v
tmux send-keys "sleep 10; rostopic echo /awesomo/quadrotor/pose/local" C-m

# echo dji quadrotor pose
tmux select-pane -t 1
tmux split-window -v
tmux send-keys "sleep 10; rostopic echo /dji_sdk/local_position" C-m
