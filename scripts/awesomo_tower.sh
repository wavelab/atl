#!/bin/sh

# launch atl
tmux new-window -n 'atl'
tmux send-keys -t 'atl' "cd ~/catkin_ws/src/atl/scripts/; vim atl_remote.py" C-m

# echo atl quadrotor pose
tmux split-window -t 'atl' -h
tmux send-keys 'atl' "sleep 10; rostopic echo /atl/quadrotor/pose/local" C-m

# # echo dji quadrotor pose
# tmux select-pane -t 1
# tmux split-window -v
# tmux send-keys "sleep 10; rostopic echo /dji_sdk/local_position" C-m
