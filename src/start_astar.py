#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uav_interface.uav import UAV
from uav_interface.position_collector import PositionCollector
from classic_path_planner.path_planner import PathPlanner

uav = UAV(uav_id=1)
pp = PathPlanner()
collector = PositionCollector()

'''
Experimentos...

1 (0.0, 0.0, 1.2) N/A (-8.0, 7.0, 3.0)
2 (0.0, 0.0, 2.0) N/A (6.5, -3.0, 2.0)
3 (0.0, 0.0, 2.5) N/A (18.5, 10.0, 2.0)
4 (0.0, 0.0, 2.5) (N/A (-23.5, 5.0, 1.5)
5 (0.0, 0.0, 2.5) N/A (17.5, 10.0, 1.0)

6 (-4.0, 5.0, 2.5) N/A (-14.0, -14.0, 2.0)
7 (-4.0, 5.0, 2.5) N/A (18.5, 12.5, 1.5)
8 (-4.0, 5.0, 1.5) (-1.85, 4.5, 2.5) (11.5, -3.5, 2.0)
9 (-4.0, 5.0, 1.5) N/A (-46.0, 6.0, 3.0)
10 (-4.0, 5.0, 1.5) N/A (-16.0, 15.0, 2.0)

11 (10.5, -2.0, 2.5) N/A (18.5, 12.5, 1.5)
12 (10.5, -2.0, 2.5) (-16.0, 15.0, 2.0) (18.5, 10.0, 2.0)
13 (10.5, -2.0, 2.5) N/A (-45.0, 30.0, 2.0)
14 (10.5, -2.0, 2.5) N/A (11.5, -3.5, 2.0) # inváilido...
15 (10.5, -2.0, 2.5) N/A (17.0, 10.0, 2.0)

16 (-24.0, 5.0, 2.5) N/A (10.5, 20.0, 2.5)
17 (-24.0, 5.0, 2.5) N/A (10.5, -2.0, 2.0)
18 (-24.0, 5.0, 2.5) N/A (17.0, 10.0, 2.0)
19 (-24.0, 5.0, 2.5) N/A (11.0, 2.0, 2.5)
20 (-24.0, 5.0, 2.5) N/A (18.5, 12.5, 1.5)
'''
import numpy as np

def start():
    rospy.init_node('astar_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    # uav.movements.goto([-24.0, 5.0, 2.5]) # ponto incial...
    # uav.movements.hover()
    # rospy.sleep(3)

    '''
    ranges = uav.uav_info.get_laser_scan().ranges
    reduced_ranges = ranges[::2]
    print(len(ranges), min(ranges), len(reduced_ranges), min(reduced_ranges))
    '''
    n = 36
    uav_position = np.array(uav.uav_info.get_uav_position(tolist=True)[:2]) # somente x,y
    obstacles = np.array(list(uav.map_environment.get_obstacles_rplidar()))
    if not obstacles.any(): # laser não colidiu com nenhum obstáculo...
        obstacles = np.array([[14, 14]])
    dists = np.linalg.norm(obstacles - uav_position, axis=1)
    indices_mais_proximos = np.argsort(dists)[:n]
    if len(indices_mais_proximos) < n: # completa o array com o valor max do laser... não colidiu com algum obstáculo
        pontos_mais_proximos = np.vstack([obstacles[indices_mais_proximos], [[14, 14]] * (n - len(indices_mais_proximos))])
    else: 
        pontos_mais_proximos = obstacles[indices_mais_proximos]

    print(pontos_mais_proximos)

    # collector.start_collecting()
    # pp.run([10.5, -2.0, 2.0])
    # pp.run([18.5, 10.0, 2.0])
    # collector.stop_collecting()

start()
