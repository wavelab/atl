#!/bin/sh

# launch awesomo
tmux new-window -n 'Awesomo'
tmux send-keys -t 'Awesomo' 'tx1; roslaunch awesomo_ros awesomo.launch' C-m
tmux split-window -t 'Awesomo' -h
tmux send-keys -t 'Awesomo' "cd ~/catkin_ws/src/awesomo/scripts/; vim awesomo_remote.py" C-m

# # echo awesomo quadrotor pose
# tmux select-pane -t 0
# tmux split-window -v
# tmux send-keys "sleep 10; rostopic echo /awesomo/quadrotor/pose/local" C-m
#
# # echo dji quadrotor pose
# tmux select-pane -t 1
# tmux split-window -v
# tmux send-keys "sleep 10; rostopic echo /dji_sdk/local_position" C-m
