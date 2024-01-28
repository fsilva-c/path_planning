#!/bin/bash

pkill -SIGINT roslaunch &> /dev/null
rosnode kill /uav1/mavros &> /dev/null
killall mavros_node mavros px4 gazebo_ros gzserver gzclient gazebo &> /dev/null
