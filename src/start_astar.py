#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import perf_counter
from uav_interface.uav import UAV
from uav_interface.position_collector import PositionCollector
from classic_path_planner.path_planner import PathPlanner
from scipy.spatial import KDTree

uav = UAV(uav_id=1)
pp = PathPlanner(goal=[-24.2, 5.7, 2.5])
collector = PositionCollector()

def start():
    rospy.init_node('astar_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    # collector.start_collecting()
    pp.run()
    st = perf_counter()
    obstacles = pp.obstacles().data
    kdtree = KDTree(obstacles)
    # nó invalido...
    p1 = (-2.867416906456588, 0.9352695171732172, 1.270655496518333)
    indices = kdtree.query_ball_point(p1, 0.5)
    print(not indices)

    p2 = (0.0, 0.0, 1.27)
    indices = kdtree.query_ball_point(p2, 0.5)
    print(not indices) # nó valido...
    print(f'finalizou em {perf_counter() - st}')
    # collector.stop_collecting()

    # otimizações...
    # criar um nó no ros pra publicar os obstáculos...
    

start()
