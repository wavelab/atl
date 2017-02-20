#!/bin/sh

tmux new-window -n 'Awesomo Rates'

# launch awesomo
tmux send-keys "rostopic hz /awesomo/apriltag/target" C-m
tmux split-window -h
