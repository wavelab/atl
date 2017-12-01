#!/bin/bash
useradd odroid

# home directory
mkdir -p /home/odroid
chown odroid:odroid /home/odroid

# set password
echo -e "odroid\nodroid" | passwd odroid

# add odroid to sudo and dialout groups
usermod -aG sudo odroid
usermod -aG dialout odroid
