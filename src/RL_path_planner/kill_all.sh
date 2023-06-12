#!/bin/bash

tmux kill-session -t simulation &> /dev/null
pkill tmux &> /dev/null
pkill -SIGINT roslaunch &> /dev/null
pkill tmux &> /dev/null
pkill ros &> /dev/null
killall -9 gazebo gzserver gzclient mavros_node px4 ros &> /dev/null
