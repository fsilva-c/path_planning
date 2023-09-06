#!/bin/bash

pkill roslaunch > /dev/null 2>&1
pkill ros > /dev/null 2>&1
pkill roscore > /dev/null 2>&1
pkill gzserver > /dev/null 2>&1
pkill gzclient > /dev/null 2>&1
killall -9 gazebo gzserver gzclient mavros_node px4 ros python > /dev/null 2>&1
