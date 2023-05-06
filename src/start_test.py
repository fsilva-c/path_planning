#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from trajectory import load
from uav import UAV

path = [
    [0, 0, 1.5],
    [0.2, 0, 1.5],
    [0.4, 0, 1.5],
    [0.6, 0, 1.5],
    [0.8, 0, 1.5],
    [1.0, 0, 1.5],
    [1.2, 0, 1.5],
    [1.4, 0, 1.5],
    [1.6, 0, 1.5],
    [1.8, 0, 1.5],
    [2.0, 0, 1.5],
    [2.2, 0, 1.5],
    [2.4, 0, 1.5],
    [2.6, 0, 1.5],
    [2.8, 0, 1.5],
    [3.0, 0, 1.5],
    [3.2, 0, 1.5],
    [3.4, 0, 1.5],
    [3.6, 0, 1.5],
    [3.8, 0, 1.5],
    [4.0, 0, 1.5],
    [4.2, 0, 1.5],
    [4.4, 0, 1.5],
    [4.6, 0, 1.5],
    [4.8, 0, 1.5],
    [5.0, 0, 1.5],
]

def start():
    rospy.init_node('test_env', anonymous=True)
    rospy.loginfo('Iniciando os testes...')
    # rospy.sleep(5)

    uav = UAV(uav_id=1)

    uav.movements.goto([0.0, 0.0, 3.0])

    path = load('/home/fs/user_ros_workspace/src/fs_path_planning/trajectories/trajectory3.txt')
    uav.movements.goto_trajectory(path)

    rospy.loginfo('Finalizando os testes...')

start()
