#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from uav.uav import UAV
from astar.path_planner import PathPlanner
import octomap

uav = UAV(uav_id=1)
pp = PathPlanner(goal=(55, 0, 3))

def start_astar_mission():
    rospy.init_node('astar_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    pp.run()

def start_octomap_astar_mission():
    rospy.init_node('astar_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    pp.run()

def start_octomap():
    rospy.init_node('astar_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    print(uav.uav_info.get_octomap().resolution)
    resolution = uav.uav_info.get_octomap().resolution
    point_cloud_2 = uav.uav_info.get_point_cloud_2()

    # make octree...
    '''
    c_x = K[2]
    c_y = K[5]
    f_x = K[0]
    f_y = K[4]
    D: []
    K: [924.2758995009278, 0.0, 640.5, 0.0, 924.2759313476419, 360.5, 0.0, 0.0, 1.0]
    R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    P: [924.2758995009278, 0.0, 640.5, 0.0, 0.0, 924.2759313476419, 360.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    '''

    start = np.array([0.0, 0.0, 0.0])
    goal = np.array([55.0, 0.0, 3.0])

    octree = octomap.OcTree(resolution)
    print('############## points_cloud')
    for p in pc2.read_points(point_cloud_2, field_names=('x', 'y', 'z'), skip_nans=True):
        x, y, z = p
        octree.insertPointCloud(np.array([[x, y, z]]), start)
        print(p)

    occupied, empty = octree.extractPointCloud()
    print('############## occupied')
    for point in occupied:
        print(point)

    print('############## empty')
    for point in empty:
        print(point)

# start_astar_mission()
# start_octomap_astar_mission()
start_octomap()