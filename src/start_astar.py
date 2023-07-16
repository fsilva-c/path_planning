#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uav.uav import UAV
from uav.position_collector import PositionCollector
from classic_path_planner.path_planner import PathPlanner

uav = UAV(uav_id=1)
pp = PathPlanner(goal=(10, 1, 2.0))
collector = PositionCollector()

def start():
    rospy.init_node('astar_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    collector.start_collecting()
    uav.movements.goto([0, 1, 2.0])
    collector.stop_collecting()
    # pp.run()

start()
