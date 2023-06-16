#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uav.uav import UAV
from classic_path_planner.path_planner import PathPlanner

uav = UAV(uav_id=1)
pp = PathPlanner(goal=(-26, 16, 2.5))

def start():
    rospy.init_node('astar_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    for obstable in uav.map_environment.get_obstacles_realsense():
        print(obstable)
    print(uav.uav_info.get_laser_scan())
    # pp.run()

start()
