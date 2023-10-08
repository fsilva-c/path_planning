#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uav_interface.uav import UAV
from uav_interface.position_collector import PositionCollector
from classic_path_planner.path_planner import PathPlanner

uav = UAV(uav_id=1)
pp = PathPlanner()
collector = PositionCollector()

experiments = [
    [[0.0, 0.0, 2.0], [4.5, 8.0, 2.0]]
]

def start():
    rospy.init_node('astar_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    # collector.start_collecting()
    # pp.run(goal=[4.5, 8.0, 2.0])
    pp.run(goal=[9.4, 3.9, 2.5])
    # collector.stop_collecting()

start()
