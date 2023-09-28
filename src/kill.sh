#!/bin/bash

pkill -SIGINT roslaunch &> /dev/null
rosnode kill /uav1/mavros &> /dev/null
killall -9 gazebo gzserver gzclient mavros_node px4 &> /dev/null
