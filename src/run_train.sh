#!/bin/bash

# pip install pandas optuna

source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.sh
source /opt/mrs/mrs_workspace/install/setup.bash
source /opt/mrs/modules_workspace/install/setup.bash
# source /opt/mrs/git/uav_core/miscellaneous/shell_additions/shell_additions.sh

echo "Iniciando treinamento..."
python /home/fs/acso_ws/mrs_singularity/user_ros_workspace/src/path_planning/src/start_rl.py
